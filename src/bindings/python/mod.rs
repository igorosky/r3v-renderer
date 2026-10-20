//! Python module `r3v_renderer`. Every public component maps 1:1 onto a class
//! named like its C counterpart without the `R3v` prefix. Bulk data goes in
//! and out as lists of objects, as in r3v-voxelizer; only rendered frames are
//! written to a buffer.
//!
//! Errors are exceptions: `ValueError` for invalid arguments (nothing is
//! changed then), `IndexError` for unknown instances, `RuntimeError` for a
//! component used before it is configured and for failures such as reading a
//! file or getting a GPU.

use pyo3::{
    exceptions::{PyIndexError, PyRuntimeError, PyValueError},
    prelude::*,
};

use super::checked::{self, ErrorKind};
use crate::{camera::CameraState, geometry};

mod camera;
mod renderer;
mod robot;
mod stl;
mod tesselator;

impl From<checked::Error> for PyErr {
    fn from(error: checked::Error) -> Self {
        match error.kind {
            ErrorKind::InvalidArgument => PyValueError::new_err(error.message),
            ErrorKind::OutOfRange => PyIndexError::new_err(error.message),
            ErrorKind::NotConfigured | ErrorKind::Failed => PyRuntimeError::new_err(error.message),
        }
    }
}

type Vec3 = (f32, f32, f32);
type Vec4 = (f32, f32, f32, f32);

fn vec3(v: geometry::Vector3D<f32>) -> Vec3 {
    (v.x, v.y, v.z)
}

fn vector3d((x, y, z): Vec3) -> geometry::Vector3D<f32> {
    geometry::Vector3D { x, y, z }
}

/// One 4x4x2 block of voxels (`geometry::PointI`): x, y, z are the voxel
/// origin of the block, i its occupancy mask with bit x + 4 * y + 16 * z for
/// the voxel at (x, y, z) inside the block. Same interface as
/// `r3v_voxelizer.PointI`, whose compress mode output can be passed on as it
/// is.
#[pyclass(module = "r3v_renderer", name = "PointI", from_py_object, eq)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PyPointI {
    x: i32,
    y: i32,
    z: i32,
    i: u32,
}

#[pymethods]
impl PyPointI {
    #[new]
    fn new(x: i32, y: i32, z: i32, i: u32) -> Self {
        Self { x, y, z, i }
    }
    fn x(&self) -> i32 {
        self.x
    }
    fn set_x(&mut self, x: i32) {
        self.x = x;
    }
    fn y(&self) -> i32 {
        self.y
    }
    fn set_y(&mut self, y: i32) {
        self.y = y;
    }
    fn z(&self) -> i32 {
        self.z
    }
    fn set_z(&mut self, z: i32) {
        self.z = z;
    }
    fn i(&self) -> u32 {
        self.i
    }
    fn set_i(&mut self, i: u32) {
        self.i = i;
    }
    fn __repr__(&self) -> String {
        format!("PointI({}, {}, {}, {})", self.x, self.y, self.z, self.i)
    }
}

impl From<PyPointI> for geometry::PointI {
    fn from(p: PyPointI) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
            i: p.i,
        }
    }
}

/// Rigid transform (`geometry::RealTwist`): rotation by the quaternion
/// `rotational` (x, y, z, w), then translation by `linear`.
#[pyclass(
    module = "r3v_renderer",
    name = "Twist",
    from_py_object,
    eq,
    get_all,
    set_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
struct PyTwist {
    linear: Vec3,
    rotational: Vec4,
}

#[pymethods]
impl PyTwist {
    #[new]
    #[pyo3(signature = (linear = (0.0, 0.0, 0.0), rotational = (0.0, 0.0, 0.0, 1.0)))]
    fn new(linear: Vec3, rotational: Vec4) -> Self {
        Self { linear, rotational }
    }
    fn __repr__(&self) -> String {
        format!(
            "Twist(linear={:?}, rotational={:?})",
            self.linear, self.rotational
        )
    }
}

impl From<PyTwist> for geometry::RealTwist {
    fn from(twist: PyTwist) -> Self {
        let (x, y, z, w) = twist.rotational;
        Self {
            linear: vector3d(twist.linear),
            rotational: geometry::Quaternion { x, y, z, w },
        }
    }
}

impl From<geometry::RealTwist> for PyTwist {
    fn from(twist: geometry::RealTwist) -> Self {
        let geometry::Quaternion { x, y, z, w } = twist.rotational;
        Self {
            linear: vec3(twist.linear),
            rotational: (x, y, z, w),
        }
    }
}

/// Triangle (`geometry::RealTriangle`), counter-clockwise when seen from its
/// front side.
#[pyclass(
    module = "r3v_renderer",
    name = "Triangle",
    from_py_object,
    eq,
    get_all,
    set_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
struct PyTriangle {
    v0: Vec3,
    v1: Vec3,
    v2: Vec3,
}

#[pymethods]
impl PyTriangle {
    #[new]
    fn new(v0: Vec3, v1: Vec3, v2: Vec3) -> Self {
        Self { v0, v1, v2 }
    }
    fn __repr__(&self) -> String {
        format!("Triangle({:?}, {:?}, {:?})", self.v0, self.v1, self.v2)
    }
}

impl From<PyTriangle> for geometry::RealTriangle {
    fn from(t: PyTriangle) -> Self {
        let v = |(x, y, z): Vec3| [x, y, z];
        Self {
            v0: v(t.v0),
            v1: v(t.v1),
            v2: v(t.v2),
        }
    }
}

impl From<&geometry::RealTriangle> for PyTriangle {
    fn from(t: &geometry::RealTriangle) -> Self {
        let v = |[x, y, z]: [f32; 3]| (x, y, z);
        Self {
            v0: v(t.v0),
            v1: v(t.v1),
            v2: v(t.v2),
        }
    }
}

/// What the renderer needs to know about a camera (`camera::CameraState`).
/// `view_proj` is projection times view, column-major: `view_proj[column][row]`.
#[pyclass(
    module = "r3v_renderer",
    name = "CameraState",
    from_py_object,
    eq,
    get_all,
    set_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
struct PyCameraState {
    view_proj: [[f32; 4]; 4],
    position: Vec3,
    forward: Vec3,
}

#[pymethods]
impl PyCameraState {
    #[new]
    fn new(view_proj: [[f32; 4]; 4], position: Vec3, forward: Vec3) -> Self {
        Self {
            view_proj,
            position,
            forward,
        }
    }
    fn __repr__(&self) -> String {
        format!(
            "CameraState(view_proj={:?}, position={:?}, forward={:?})",
            self.view_proj, self.position, self.forward
        )
    }
}

impl From<CameraState> for PyCameraState {
    fn from(state: CameraState) -> Self {
        let p = state.position;
        let f = state.forward;
        Self {
            view_proj: state.view_proj.into(),
            position: (p.x, p.y, p.z),
            forward: (f.x, f.y, f.z),
        }
    }
}

impl From<PyCameraState> for CameraState {
    fn from(state: PyCameraState) -> Self {
        let (px, py, pz) = state.position;
        let (fx, fy, fz) = state.forward;
        Self {
            view_proj: state.view_proj.into(),
            position: cgmath::Point3::new(px, py, pz),
            forward: cgmath::Vector3::new(fx, fy, fz),
        }
    }
}

#[pymodule]
mod r3v_renderer {
    #[pymodule_export]
    use super::{
        PyCameraState, PyPointI, PyTriangle, PyTwist,
        camera::PyOrbitCamera,
        renderer::{PyWgpuRenderer, PyWgpuRendererConfiguration},
        robot::{
            PyRobotTransforms, PyUrdfBox, PyUrdfCapsule, PyUrdfColor, PyUrdfCylinder,
            PyUrdfDescription, PyUrdfJoint, PyUrdfJointLimit, PyUrdfLink, PyUrdfMaterial,
            PyUrdfMesh, PyUrdfOrigin, PyUrdfRobot, PyUrdfSphere, PyUrdfVisual,
        },
        stl::PyStlMesh,
        tesselator::{
            PyCpuTesselator, PyCpuTesselatorConfiguration, PyGeometryDelta, PyTriangleChunk,
        },
    };
}
