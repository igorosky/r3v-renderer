use std::ops::{Index, IndexMut, Mul, Sub};

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

    pub fn to_vertices(&self) -> [Vertex; 3]
    where
        T: Into<f32> + Copy,
    {
        let triangle: RealTriangle = (*self).map(Into::into);
        let normal = triangle.normal();
        [triangle.v0, triangle.v1, triangle.v2]
            .map(|position| Vertex::from(Point::from_array(position)).with_normal(normal))
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
}

#[derive(Debug, Clone, Copy)]
pub struct Vector2D<T> {
    pub x: T,
    pub y: T,
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

impl<T> Sub for Vector3D<T>
where
    T: Sub<Output = T>,
{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<T, U> Into<Point<U>> for Vector3D<T>
where
    T: Into<U>,
{
    fn into(self) -> Point<U> {
        Point {
            x: self.x.into(),
            y: self.y.into(),
            z: self.z.into(),
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
            [
                Triangle {
                    v0: a,
                    v1: b,
                    v2: c,
                },
                Triangle {
                    v0: a,
                    v1: c,
                    v2: d,
                },
            ]
        } else {
            [
                Triangle {
                    v0: a,
                    v1: c,
                    v2: b,
                },
                Triangle {
                    v0: a,
                    v1: d,
                    v2: c,
                },
            ]
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
    pub normal: [f32; 3],
}

impl Vertex {
    pub const DEFAULT_COLOR: [f32; 3] = [1.0, 1.0, 1.0];

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
            color: Self::DEFAULT_COLOR,
            normal: [0.0; 3],
        }
    }
}

pub struct TriangleChunk {
    pub index: DiscretePoint,
    pub triangles: Vec<RealTriangle>,
}
