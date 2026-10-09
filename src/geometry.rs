#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PointI {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub i: u32,
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
}

#[derive(Copy, Clone, Debug)]
pub struct Triangle<T> {
    pub v0: [T; 3],
    pub v1: [T; 3],
    pub v2: [T; 3],
}

pub struct Vector3D<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

pub struct Quaternion<T> {
    pub x: T,
    pub y: T,
    pub z: T,
    pub w: T,
}

pub struct Twist<T> {
    pub linear: Vector3D<T>,
    pub rotational: Quaternion<T>,
}

pub type DiscreteUnit = i32;
pub type RealUnit = f32;

pub type DiscretePoint = Point<DiscreteUnit>;
pub type RealPoint = Point<RealUnit>;

pub type DiscreteTriangle = Triangle<DiscreteUnit>;
pub type RealTriangle = Triangle<RealUnit>;

pub type RealTwist = Twist<RealUnit>;

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
