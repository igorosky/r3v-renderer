use std::ops::{Index, IndexMut};

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
        Point { x: bit & 0b11, y: (bit >> 2) & 0b11, z: (bit >> 4) & 0b1 }
    }
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
    pub const ALL: [Axis; 3] = [
        Axis::X,
        Axis::Y,
        Axis::Z,
    ];

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

impl<T> Point<T> {
    pub fn to_array(&self) -> [T; 3] where T: Copy {
        [self.x, self.y, self.z]
    }

    pub fn from_array(arr: [T; 3]) -> Self where T: Copy {
        Self { x: arr[0], y: arr[1], z: arr[2] }
    }

    pub fn from_fn(mut f: impl FnMut(Axis) -> T) -> Self {
        Self { x: f(Axis::X), y: f(Axis::Y), z: f(Axis::Z) }
    }

    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Point<U> {
        Point { x: f(self.x), y: f(self.y), z: f(self.z) }
    }
}

/// Per-axis access for the 3D `{ x, y, z }` structs; `get` is the `const` counterpart of indexing.
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
}

pub type DiscreteTriangle = Triangle<DiscreteUnit>;
pub type RealTriangle = Triangle<RealUnit>;

#[derive(Debug, Clone, Copy)]
pub struct Vector2D<T> {
    pub x: T,
    pub y: T,
}

#[derive(Debug, Clone, Copy)]
pub struct Vector3D<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

#[derive(Copy, Clone, Debug)]
pub struct Quaternion<T> {
    pub x: T,
    pub y: T,
    pub z: T,
    pub w: T,
}

#[derive(Copy, Clone, Debug)]
pub struct Twist<T> {
    pub linear: Vector3D<T>,
    pub rotational: Quaternion<T>,
}

pub type RealTwist = Twist<RealUnit>;

#[derive(Debug, Clone, Copy)]
pub struct VoxelQuad<T> {
    pub normal: VoxelNormal,
    pub origin: Point<T>,
    pub size: Vector2D<T>,
}

impl VoxelQuad<DiscreteUnit> {
    pub fn triangles(&self) -> [DiscreteTriangle; 2] {
        let axis = self.normal.axis();
        let (u, v) = axis.plane();
        let corner = |du: DiscreteUnit, dv: DiscreteUnit| {
            let mut corner = self.origin;
            corner[axis] += self.normal.is_positive() as DiscreteUnit;
            corner[u] += du * self.size.x;
            corner[v] += dv * self.size.y;
            corner.to_array()
        };
        let [a, b, c, d] = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)];
        if self.normal.is_positive() {
            [Triangle { v0: a, v1: b, v2: c }, Triangle { v0: a, v1: c, v2: d }]
        } else {
            [Triangle { v0: a, v1: c, v2: b }, Triangle { v0: a, v1: d, v2: c }]
        }
    }
}

pub type DiscreteQuad = VoxelQuad<DiscreteUnit>;
pub type RealQuad = VoxelQuad<RealUnit>;

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
}

impl Vertex {
    pub fn with_default_color(x: f32, y: f32, z: f32) -> Self {
        Self {
            position: [x, y, z],
            color: [1.0, 1.0, 1.0],
        }
    }
}

pub struct TriangleChunk {
    pub index: DiscretePoint,
    pub triangles: Vec<RealTriangle>,
}
