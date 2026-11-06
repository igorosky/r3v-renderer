use std::ops::{Add, Index, IndexMut, Mul, Sub};

use cgmath::num_traits;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PointI {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub i: u32,
}

impl PointI {
    pub const fn voxel_bit(local: Point<usize>) -> u32 {
        (local.x | local.y << 2 | local.z << 4) as u32
    }

    pub const fn voxel_local(bit: u32) -> Point<usize> {
        let bit = bit as usize;
        Point {
            x: bit & 0b11,
            y: (bit >> 2) & 0b11,
            z: (bit >> 4) & 0b1,
        }
    }
}

/// Bits of a page coordinate inside a [`VoxelGroup`], which is 16x16x16 pages.
const VOXEL_GROUP_EDGE_BITS: u32 = 4;
// [`ZOrder`] covers 4 bits per axis
const _: () = assert!(VOXEL_GROUP_EDGE_BITS == 4);
/// Edge length of a page of a [`VoxelGroup`] in voxels.
const PAGE_EDGE: DiscreteUnit = 4;
/// Layers of a page in one 4x4x2 block: its word holds 16 bits per layer.
const BLOCK_DEPTH: DiscreteUnit = 2;
/// Voxel coordinates shifted right by it give the key of their [`VoxelGroup`].
pub(crate) const VOXEL_GROUP_SHIFT: u32 = VOXEL_GROUP_EDGE_BITS + PAGE_EDGE.trailing_zeros();
/// Words in a [`VoxelGroup`], one per 4x4x4 page.
pub const VOXEL_GROUP_PAGES: usize = 1 << (3 * VOXEL_GROUP_EDGE_BITS);

/// Constructed voxels of a cube of 64x64x64 voxels, as r3v-voxelizer's
/// `get_all_points_compressed` returns them (its `CompressedVoxelGroup`).
///
/// `point` is the key of the group: voxel coordinates shifted right by 6,
/// `point.i` is ignored. Keys outside `[-2^25, 2^25)` wrap. `values[k]` is the
/// occupancy of the 4x4x4 page whose offset in the group, bits of x, y and z
/// interleaved from the lowest one, is `k`. Bit `x + 4 * y + 16 * z` of it is
/// set when voxel (x, y, z) of the page is constructed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VoxelGroup {
    pub point: PointI,
    pub values: Box<[u64; VOXEL_GROUP_PAGES]>,
}

impl VoxelGroup {
    /// The 4x4x2 blocks of the group holding at least one voxel, as
    /// `Tesselator::update_input_geometry` takes them.
    pub fn blocks(&self) -> impl Iterator<Item = PointI> + '_ {
        group_blocks(&self.point, &self.values)
    }
}

mod zorder_mod {
    /// Bits of one coordinate in a z-order index: bit k of x goes to bit 3k,
    /// y and z sit 1 and 2 above (`AXIS_MASK << 1`, `AXIS_MASK << 2`).
    const AXIS_MASK: usize = 0x249;

    /// Spreads the 4 low bits of `v` 3 apart (bit k to bit 3k).
    const fn spread(v: u16) -> u16 {
        let mut result = 0;
        let mut bit = 0;
        while bit < 4 {
            result |= ((v >> bit) & 1) << (3 * bit);
            bit += 1;
        }
        result
    }

    const fn spread_table(shift: u32) -> [u16; 16] {
        let mut table = [0; 16];
        let mut v = 0;
        while v < 16 {
            table[v] = spread(v as u16) << shift;
            v += 1;
        }
        table
    }

    /// Inverse of [`spread`], indexed by the bits of one axis
    /// (`index & AXIS_MASK`). Only 16 entries are used.
    const fn compact_table() -> [u8; AXIS_MASK + 1] {
        let mut table = [0; AXIS_MASK + 1];
        let mut v = 0;
        while v < 16 {
            table[spread(v) as usize] = v as u8;
            v += 1;
        }
        table
    }

    static SPREAD_X: [u16; 16] = spread_table(0);
    static SPREAD_Y: [u16; 16] = spread_table(1);
    static SPREAD_Z: [u16; 16] = spread_table(2);
    static COMPACT: [u8; AXIS_MASK + 1] = compact_table();

    /// Z-order (Morton) index of 3 coordinates of up to 4 bits each: x in bit
    /// 0, y in bit 1, z in bit 2, then the next bit of each. The same layout
    /// as the words of [`super::VoxelGroup`] (r3v-voxelizer's `ZOrder`).
    ///
    /// Lookup tables instead of BMI2 `pdep`/`pext`: those are behind
    /// `#[target_feature]`, which is never inlined, and a call costs more
    /// than a few loads.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct ZOrder {
        _private: (),
    }

    impl ZOrder {
        /// Coordinates of a z-order index below 4096.
        #[inline]
        pub fn coord_of(&self, index: u64) -> [u16; 3] {
            debug_assert!(index < 4096, "z-order index {index} has more than 12 bits");
            let index = index as usize;
            [
                COMPACT[index & AXIS_MASK] as u16,
                COMPACT[(index >> 1) & AXIS_MASK] as u16,
                COMPACT[(index >> 2) & AXIS_MASK] as u16,
            ]
        }
        /// Z-order index of coordinates below 16.
        #[inline]
        pub fn index_of(&self, [x, y, z]: [u16; 3]) -> u64 {
            debug_assert!(
                x < 16 && y < 16 && z < 16,
                "z-order coordinates {x}, {y}, {z} have more than 4 bits"
            );
            (SPREAD_X[(x & 15) as usize]
                | SPREAD_Y[(y & 15) as usize]
                | SPREAD_Z[(z & 15) as usize]) as u64
        }
    }

    #[cfg(test)]
    mod test {
        use super::ZOrder;

        /// Bit by bit reference of the layout.
        fn reference([x, y, z]: [u16; 3]) -> u64 {
            (0..4).fold(0, |index, bit| {
                index
                    | (((x >> bit) & 1) as u64) << (3 * bit)
                    | (((y >> bit) & 1) as u64) << (3 * bit + 1)
                    | (((z >> bit) & 1) as u64) << (3 * bit + 2)
            })
        }

        #[test]
        fn zorder_matches_reference_on_every_input() {
            let zorder = ZOrder::default();
            for x in 0..16 {
                for y in 0..16 {
                    for z in 0..16 {
                        let index = zorder.index_of([x, y, z]);
                        assert_eq!(index, reference([x, y, z]), "index_of {x} {y} {z}");
                        assert_eq!(zorder.coord_of(index), [x, y, z], "coord_of {index}");
                    }
                }
            }
        }

        #[test]
        fn zorder_axis_order() {
            let zorder = ZOrder::default();
            assert_eq!(zorder.index_of([1, 0, 0]), 1);
            assert_eq!(zorder.index_of([0, 1, 0]), 2);
            assert_eq!(zorder.index_of([0, 0, 1]), 4);
            assert_eq!(zorder.index_of([2, 0, 0]), 8);
            assert_eq!(zorder.index_of([15, 15, 15]), 4095);
            assert_eq!(zorder.coord_of(63), [3, 3, 3]);
        }
    }
}
pub use zorder_mod::ZOrder;

/// [`VoxelGroup::blocks`] of the group with key `key` and words `values`.
pub(crate) fn group_blocks<'a>(
    key: &PointI,
    values: &'a [u64; VOXEL_GROUP_PAGES],
) -> impl Iterator<Item = PointI> + use<'a> {
    // Below 2^31 - 64 even for a wrapped key, so adding the offsets can not
    // overflow
    let origin = Point::from_array([key.x, key.y, key.z]).map(|c| c << VOXEL_GROUP_SHIFT);
    let zorder = ZOrder::default();
    values
        .iter()
        .enumerate()
        .filter(|(_, word)| **word != 0)
        .flat_map(move |(index, &word)| {
            let page = origin
                + Point::from_array(zorder.coord_of(index as u64))
                    .map(|offset| DiscreteUnit::from(offset) * PAGE_EDGE);
            [(word as u32, 0), ((word >> u32::BITS) as u32, BLOCK_DEPTH)]
                .into_iter()
                .filter(|(mask, _)| *mask != 0)
                .map(move |(mask, z)| PointI {
                    x: page.x,
                    y: page.y,
                    z: page.z + z,
                    i: mask,
                })
        })
}

pub type DiscreteUnit = i32;
pub type RealUnit = f32;

#[repr(usize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
}

impl Axis {
    pub const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

    pub const fn plane(self) -> (Axis, Axis) {
        match self {
            Axis::X => (Axis::Y, Axis::Z),
            Axis::Y => (Axis::Z, Axis::X),
            Axis::Z => (Axis::X, Axis::Y),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelNormal {
    Positive(Axis),
    Negative(Axis),
}

impl VoxelNormal {
    pub const ALL: [VoxelNormal; 6] = [
        VoxelNormal::Positive(Axis::X),
        VoxelNormal::Negative(Axis::X),
        VoxelNormal::Positive(Axis::Y),
        VoxelNormal::Negative(Axis::Y),
        VoxelNormal::Positive(Axis::Z),
        VoxelNormal::Negative(Axis::Z),
    ];

    pub const fn axis(self) -> Axis {
        match self {
            VoxelNormal::Positive(axis) | VoxelNormal::Negative(axis) => axis,
        }
    }

    pub const fn is_positive(self) -> bool {
        matches!(self, VoxelNormal::Positive(_))
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, bytemuck::Zeroable)]
pub struct Point<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> From<Vector3D<T>> for Point<T> {
    fn from(value: Vector3D<T>) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl<T> Point<T> {
    pub fn to_array(&self) -> [T; 3]
    where
        T: Copy,
    {
        [self.x, self.y, self.z]
    }

    pub fn from_array(arr: [T; 3]) -> Self
    where
        T: Copy,
    {
        Self {
            x: arr[0],
            y: arr[1],
            z: arr[2],
        }
    }

    pub fn from_fn(mut f: impl FnMut(Axis) -> T) -> Self {
        Self {
            x: f(Axis::X),
            y: f(Axis::Y),
            z: f(Axis::Z),
        }
    }

    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Point<U> {
        Point {
            x: f(self.x),
            y: f(self.y),
            z: f(self.z),
        }
    }
}

impl<U> Add for Point<U>
where
    U: Add<Output = U>,
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

macro_rules! impl_axis_access {
    ($($type:ident),*) => {$(
        impl<T: Copy> $type<T> {
            pub const fn get(&self, axis: Axis) -> T {
                match axis {
                    Axis::X => self.x,
                    Axis::Y => self.y,
                    Axis::Z => self.z,
                }
            }
        }

        impl<T> Index<Axis> for $type<T> {
            type Output = T;

            fn index(&self, axis: Axis) -> &T {
                match axis {
                    Axis::X => &self.x,
                    Axis::Y => &self.y,
                    Axis::Z => &self.z,
                }
            }
        }

        impl<T> IndexMut<Axis> for $type<T> {
            fn index_mut(&mut self, axis: Axis) -> &mut T {
                match axis {
                    Axis::X => &mut self.x,
                    Axis::Y => &mut self.y,
                    Axis::Z => &mut self.z,
                }
            }
        }
    )*};
}

impl_axis_access!(Point, Vector3D);

pub type DiscretePoint = Point<DiscreteUnit>;
pub type RealPoint = Point<RealUnit>;

pub enum Winding {
    Clockwise,
    CounterClockwise,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Triangle<T> {
    pub v0: [T; 3],
    pub v1: [T; 3],
    pub v2: [T; 3],
}

impl<T> Triangle<T> {
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Triangle<U> {
        Triangle {
            v0: self.v0.map(&mut f),
            v1: self.v1.map(&mut f),
            v2: self.v2.map(&mut f),
        }
    }

    pub fn reverse(self) -> Self {
        Self {
            v0: self.v0,
            v1: self.v2,
            v2: self.v1,
        }
    }

    pub fn with_winding(&self, winding: Winding, normal: Vector3D<T>) -> Self
    where
        T: Copy + PartialOrd + num_traits::Zero + Sub<Output = T> + Mul<Output = T>,
    {
        let is_ccw = match winding {
            Winding::Clockwise => false,
            Winding::CounterClockwise => true,
        };
        let current_is_ccw = {
            let v0: Vector3D<T> = self.v0.into();
            let v1: Vector3D<T> = self.v1.into();
            let v2: Vector3D<T> = self.v2.into();
            (v1 - v0).cross(v2 - v0).dot(normal) >= T::zero()
        };
        if is_ccw == current_is_ccw {
            *self
        } else {
            self.reverse()
        }
    }
}

pub type DiscreteTriangle = Triangle<DiscreteUnit>;
pub type RealTriangle = Triangle<RealUnit>;

impl RealTriangle {
    pub fn normal(&self) -> Vector3D<f32> {
        let v0: Vector3D<f32> = self.v0.into();
        let v1: Vector3D<f32> = self.v1.into();
        let v2: Vector3D<f32> = self.v2.into();
        let normal = (v1 - v0).cross(v2 - v0);
        let length = normal.dot(normal).sqrt();
        if length == 0.0 {
            return normal;
        }
        Vector3D {
            x: normal.x / length,
            y: normal.y / length,
            z: normal.z / length,
        }
    }

    pub fn to_vertices(&self) -> [Vertex; 3] {
        let normal = self.normal();
        [self.v0, self.v1, self.v2].map(|position| Vertex::from(position).with_normal(normal))
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Quad<T> {
    pub v0: [T; 3],
    pub v1: [T; 3],
    pub v2: [T; 3],
    pub v3: [T; 3],
}

impl<T> Quad<T> {
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Quad<U> {
        Quad {
            v0: self.v0.map(&mut f),
            v1: self.v1.map(&mut f),
            v2: self.v2.map(&mut f),
            v3: self.v3.map(&mut f),
        }
    }
}

impl<T, U> Mul<U> for Quad<T>
where
    T: Mul<U, Output = T>,
    U: Copy,
{
    type Output = Self;

    fn mul(self, rhs: U) -> Self::Output {
        self.map(|v| v * rhs)
    }
}

pub type DiscreteQuad = Quad<DiscreteUnit>;
pub type RealQuad = Quad<RealUnit>;

impl DiscreteQuad {
    pub fn voxel_face(
        normal: VoxelNormal,
        origin: DiscretePoint,
        size: Vector2D<DiscreteUnit>,
    ) -> Self {
        let axis = normal.axis();
        let (u, v) = axis.plane();
        let corner = |du: DiscreteUnit, dv: DiscreteUnit| {
            let mut corner = origin;
            corner[axis] += normal.is_positive() as DiscreteUnit;
            corner[u] += du * size.x;
            corner[v] += dv * size.y;
            corner.to_array()
        };
        let [a, b, c, d] = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)];
        if normal.is_positive() {
            Self {
                v0: a,
                v1: b,
                v2: c,
                v3: d,
            }
        } else {
            Self {
                v0: a,
                v1: d,
                v2: c,
                v3: b,
            }
        }
    }

    pub fn to_real(&self, voxel_size: RealUnit) -> RealQuad {
        self.map(|v| v as RealUnit * voxel_size)
    }
}

impl RealQuad {
    pub fn normal(&self) -> Vector3D<f32> {
        Triangle {
            v0: self.v0,
            v1: self.v1,
            v2: self.v2,
        }
        .normal()
    }

    pub fn triangles(&self) -> [RealTriangle; 2] {
        [
            Triangle {
                v0: self.v0,
                v1: self.v1,
                v2: self.v2,
            },
            Triangle {
                v0: self.v0,
                v1: self.v2,
                v2: self.v3,
            },
        ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Vector2D<T> {
    pub x: T,
    pub y: T,
}

impl<T> Vector2D<T> {
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Vector2D<U> {
        Vector2D {
            x: f(self.x),
            y: f(self.y),
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, bytemuck::Zeroable)]
pub struct Vector3D<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Vector3D<T> {
    #[inline]
    pub fn cross(self, rhs: Self) -> Self
    where
        T: Sub<Output = T> + Mul<Output = T> + Copy,
    {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    #[inline]
    pub fn dot(self, rhs: Self) -> T
    where
        T: Mul<Output = T> + std::ops::Add<Output = T> + Copy,
    {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn unit(axis: Axis) -> Self
    where
        T: num_traits::One + num_traits::Zero,
    {
        match axis {
            Axis::X => Self {
                x: T::one(),
                y: T::zero(),
                z: T::zero(),
            },
            Axis::Y => Self {
                x: T::zero(),
                y: T::one(),
                z: T::zero(),
            },
            Axis::Z => Self {
                x: T::zero(),
                y: T::zero(),
                z: T::one(),
            },
        }
    }
}

impl<T> Vector3D<T> {
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Vector3D<U> {
        Vector3D {
            x: f(self.x),
            y: f(self.y),
            z: f(self.z),
        }
    }
}

impl<T, V> From<V> for Vector3D<T>
where
    V: Into<[T; 3]>,
{
    fn from(value: V) -> Self {
        let [x, y, z] = value.into();
        Vector3D { x, y, z }
    }
}

impl<T, U> Sub<Vector3D<U>> for Vector3D<T>
where
    T: Sub<U, Output = T>,
    U: Copy,
{
    type Output = Self;

    fn sub(self, rhs: Vector3D<U>) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<T, U> Mul<U> for Vector3D<T>
where
    T: Mul<U, Output = T>,
    U: Copy,
{
    type Output = Self;

    fn mul(self, rhs: U) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, bytemuck::Zeroable)]
pub struct Quaternion<T> {
    pub x: T,
    pub y: T,
    pub z: T,
    pub w: T,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, bytemuck::Zeroable)]
pub struct Twist<T> {
    pub linear: Vector3D<T>,
    pub rotational: Quaternion<T>,
}

pub type RealTwist = Twist<RealUnit>;

impl RealTwist {
    pub const fn identity() -> Self {
        Self {
            linear: Vector3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotational: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        }
    }
}

unsafe impl bytemuck::Pod for Vector3D<f32> {}
unsafe impl bytemuck::Pod for Quaternion<f32> {}
unsafe impl bytemuck::Pod for Twist<f32> {}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

impl Vertex {
    pub fn with_normal(self, normal: Vector3D<f32>) -> Self {
        Self {
            normal: [normal.x, normal.y, normal.z],
            ..self
        }
    }
}

impl From<Point<f32>> for Vertex {
    fn from(point: Point<f32>) -> Self {
        Self {
            position: point.to_array(),
            normal: [0.0; 3],
        }
    }
}

impl From<[f32; 3]> for Vertex {
    fn from(position: [f32; 3]) -> Self {
        Self {
            position,
            normal: [0.0; 3],
        }
    }
}

pub struct TriangleChunk {
    pub index: DiscretePoint,
    pub triangles: Vec<RealTriangle>,
}
