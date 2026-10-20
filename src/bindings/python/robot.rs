//! `urdf::UrdfRobot` (a robot description read by `urdf::read_file` and
//! `urdf::read_str`), `robot::UrdfRobot` (trait `Robot`) and its output,
//! `robot::RobotTransforms`. The two Rust `UrdfRobot`s are `UrdfDescription`
//! and `UrdfRobot` in Python.
//!
//! The description is plain data: read only objects whose optional parts are
//! `None` when missing.

use std::{collections::HashMap, path::PathBuf};

use pyo3::prelude::*;

use crate::{
    Robot,
    bindings::checked,
    robot::{RobotTransforms, UrdfRobot},
    urdf,
};

use super::{PyTwist, Vec3, Vec4};

fn vec3(v: urdf::Vector3) -> Vec3 {
    (v.x, v.y, v.z)
}

/// Position and orientation (`urdf::UrdfOrigin`): `rpy` is roll, pitch and
/// yaw in radians.
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfOrigin",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyUrdfOrigin {
    xyz: Vec3,
    rpy: Vec3,
}

/// Mesh geometry (`urdf::UrdfMesh`). `filename` is resolved against the URDF
/// file's directory and `package://` ancestors when read with
/// `UrdfDescription.read_file`.
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfMesh",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PyUrdfMesh {
    filename: String,
    scale: Option<Vec3>,
}

/// Box geometry (`urdf::UrdfGeometry::Box`): `size` holds the edge lengths.
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfBox",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyUrdfBox {
    size: Vec3,
}

/// Sphere geometry (`urdf::UrdfGeometry::Sphere`).
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfSphere",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyUrdfSphere {
    radius: f32,
}

/// Cylinder geometry (`urdf::UrdfGeometry::Cylinder`).
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfCylinder",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyUrdfCylinder {
    radius: f32,
    length: f32,
}

/// Capsule geometry (`urdf::UrdfGeometry::Capsule`).
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfCapsule",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyUrdfCapsule {
    radius: f32,
    length: f32,
}

/// `urdf::UrdfGeometry`: one class per variant.
#[derive(Debug, Clone, PartialEq, IntoPyObject)]
enum PyUrdfGeometry {
    Mesh(PyUrdfMesh),
    Box(PyUrdfBox),
    Sphere(PyUrdfSphere),
    Cylinder(PyUrdfCylinder),
    Capsule(PyUrdfCapsule),
}

/// Color (`urdf::UrdfColor`): `rgba` from 0 to 1.
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfColor",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyUrdfColor {
    rgba: Vec4,
}

/// Material (`urdf::UrdfMaterial`).
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfMaterial",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PyUrdfMaterial {
    name: Option<String>,
    color: Option<PyUrdfColor>,
}

/// Visual of a link (`urdf::UrdfVisual`). `geometry` is a `UrdfMesh`,
/// `UrdfBox`, `UrdfSphere`, `UrdfCylinder` or `UrdfCapsule`.
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfVisual",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PyUrdfVisual {
    name: Option<String>,
    origin: Option<PyUrdfOrigin>,
    geometry: Option<PyUrdfGeometry>,
    material: Option<PyUrdfMaterial>,
}

/// Link of a robot description (`urdf::UrdfLink`).
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfLink",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PyUrdfLink {
    name: String,
    visuals: Vec<PyUrdfVisual>,
}

/// Limits of a joint (`urdf::UrdfJointLimit`).
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfJointLimit",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyUrdfJointLimit {
    lower: f32,
    upper: f32,
    effort: f32,
    velocity: f32,
    acceleration: f32,
    deceleration: f32,
    jerk: f32,
}

/// Joint of a robot description (`urdf::UrdfJoint`). `joint_type` is
/// "revolute", "continuous", "prismatic", "fixed", "floating", "planar" or
/// "spherical"; `parent` and `child` are link names.
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfJoint",
    frozen,
    skip_from_py_object,
    eq,
    get_all
)]
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PyUrdfJoint {
    name: String,
    joint_type: String,
    parent: String,
    child: String,
    origin: Option<PyUrdfOrigin>,
    axis: Option<Vec3>,
    limit: Option<PyUrdfJointLimit>,
}

impl From<urdf::UrdfOrigin> for PyUrdfOrigin {
    fn from(origin: urdf::UrdfOrigin) -> Self {
        Self {
            xyz: vec3(origin.xyz),
            rpy: vec3(origin.rpy),
        }
    }
}

impl From<&urdf::UrdfGeometry> for PyUrdfGeometry {
    fn from(geometry: &urdf::UrdfGeometry) -> Self {
        match *geometry {
            urdf::UrdfGeometry::Mesh(ref mesh) => Self::Mesh(PyUrdfMesh {
                filename: mesh.filename.clone(),
                scale: mesh.scale.map(vec3),
            }),
            urdf::UrdfGeometry::Box { size } => Self::Box(PyUrdfBox { size: vec3(size) }),
            urdf::UrdfGeometry::Sphere { radius } => Self::Sphere(PyUrdfSphere { radius }),
            urdf::UrdfGeometry::Cylinder { radius, length } => {
                Self::Cylinder(PyUrdfCylinder { radius, length })
            }
            urdf::UrdfGeometry::Capsule { radius, length } => {
                Self::Capsule(PyUrdfCapsule { radius, length })
            }
        }
    }
}

impl From<&urdf::UrdfMaterial> for PyUrdfMaterial {
    fn from(material: &urdf::UrdfMaterial) -> Self {
        let [r, g, b, a] = material.color.as_ref().map_or([0.0; 4], |c| c.rgba);
        Self {
            name: material.name.clone(),
            color: material
                .color
                .as_ref()
                .map(|_| PyUrdfColor { rgba: (r, g, b, a) }),
        }
    }
}

impl From<&urdf::UrdfVisual> for PyUrdfVisual {
    fn from(visual: &urdf::UrdfVisual) -> Self {
        Self {
            name: visual.name.clone(),
            origin: visual.origin.map(Into::into),
            geometry: visual.geometry.as_ref().map(Into::into),
            material: visual.material.as_ref().map(Into::into),
        }
    }
}

impl From<urdf::UrdfJointLimit> for PyUrdfJointLimit {
    fn from(limit: urdf::UrdfJointLimit) -> Self {
        let urdf::UrdfJointLimit {
            lower,
            upper,
            effort,
            velocity,
            acceleration,
            deceleration,
            jerk,
        } = limit;
        Self {
            lower,
            upper,
            effort,
            velocity,
            acceleration,
            deceleration,
            jerk,
        }
    }
}

impl From<&urdf::UrdfJoint> for PyUrdfJoint {
    fn from(joint: &urdf::UrdfJoint) -> Self {
        Self {
            name: joint.name.clone(),
            joint_type: joint.joint_type.clone(),
            parent: joint.parent.clone(),
            child: joint.child.clone(),
            origin: joint.origin.map(Into::into),
            axis: joint.axis.map(vec3),
            limit: joint.limit.map(Into::into),
        }
    }
}

/// Robot description read from URDF (`urdf::UrdfRobot`).
#[pyclass(
    module = "r3v_renderer",
    name = "UrdfDescription",
    frozen,
    skip_from_py_object,
    eq
)]
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PyUrdfDescription(urdf::UrdfRobot);

#[pymethods]
impl PyUrdfDescription {
    /// `urdf::read_file`: reads a URDF file. Relative mesh file names are
    /// resolved against its directory. `RuntimeError` on failure.
    #[staticmethod]
    fn read_file(path: PathBuf) -> PyResult<Self> {
        let description = urdf::read_file(&path).map_err(checked::Error::failed)?;
        Ok(Self(description))
    }

    /// `urdf::read_str`: reads URDF from a string. Mesh file names are kept as
    /// they are. `RuntimeError` on failure.
    #[staticmethod]
    fn read_str(xml: &str) -> PyResult<Self> {
        let description = urdf::read_str(xml).map_err(checked::Error::failed)?;
        Ok(Self(description))
    }

    /// Name of the robot.
    #[getter]
    fn name(&self) -> &str {
        &self.0.name
    }

    #[getter]
    fn links(&self) -> Vec<PyUrdfLink> {
        self.0
            .links
            .iter()
            .map(|link| PyUrdfLink {
                name: link.name.clone(),
                visuals: link.visuals.iter().map(Into::into).collect(),
            })
            .collect()
    }

    #[getter]
    fn joints(&self) -> Vec<PyUrdfJoint> {
        self.0.joints.iter().map(Into::into).collect()
    }

    /// Top level materials.
    #[getter]
    fn materials(&self) -> Vec<PyUrdfMaterial> {
        self.0.materials.iter().map(Into::into).collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "UrdfDescription(name={:?}, <{} links>, <{} joints>)",
            self.0.name,
            self.0.links.len(),
            self.0.joints.len()
        )
    }
}

/// Robot computing link transforms from joint positions
/// (`robot::UrdfRobot`). Create it with `from_description`.
#[pyclass(module = "r3v_renderer", name = "UrdfRobot")]
pub(super) struct PyUrdfRobot(UrdfRobot);

#[pymethods]
impl PyUrdfRobot {
    /// `Robot::from_description` with a copy of `description`.
    #[staticmethod]
    fn from_description(description: &PyUrdfDescription) -> Self {
        Self(UrdfRobot::from_description(description.0.clone()))
    }

    /// `UrdfRobot::description`, as a copy.
    fn description(&self) -> PyUrdfDescription {
        PyUrdfDescription(self.0.description().clone())
    }

    /// `Robot::feed_sensor_input`: transforms for joint positions (radians or
    /// meters) by joint name. Joints left out are at 0.
    fn feed_sensor_input(&mut self, input: HashMap<String, f32>) -> PyRobotTransforms {
        PyRobotTransforms(self.0.feed_sensor_input(&input))
    }

    /// `Robot::last_transform`: a copy of what the last `feed_sensor_input`
    /// returned, `None` before the first one.
    fn last_transform(&self) -> Option<PyRobotTransforms> {
        self.0.last_transform().cloned().map(PyRobotTransforms)
    }
}

/// Transforms of every link and visual for one set of joint positions
/// (`robot::RobotTransforms`).
#[pyclass(module = "r3v_renderer", name = "RobotTransforms", frozen)]
pub(super) struct PyRobotTransforms(RobotTransforms);

#[pymethods]
impl PyRobotTransforms {
    /// World transform of every link, by link name.
    #[getter]
    fn links(&self) -> HashMap<String, PyTwist> {
        self.0
            .links
            .iter()
            .map(|(link, &twist)| (link.clone(), twist.into()))
            .collect()
    }

    /// World transform of every visual, by (link name, visual index).
    #[getter]
    fn visuals(&self) -> HashMap<(String, usize), PyTwist> {
        self.0
            .visuals
            .iter()
            .map(|(visual, &twist)| (visual.clone(), twist.into()))
            .collect()
    }

    /// `RobotTransforms::transform_visual_point`: `point` given in the frame
    /// of visual number `visual_index` of `link`, in world space. `None` when
    /// there is no such visual.
    fn transform_visual_point(&self, link: &str, visual_index: usize, point: Vec3) -> Option<Vec3> {
        let (x, y, z) = point;
        let [x, y, z] = self
            .0
            .transform_visual_point(link, visual_index, [x, y, z])?;
        Some((x, y, z))
    }

    /// `RobotTransforms::get_link_origin`: world position of `link`, `None`
    /// when there is no such link.
    fn get_link_origin(&self, link: &str) -> Option<Vec3> {
        let origin = self.0.get_link_origin(link)?;
        Some((origin.x, origin.y, origin.z))
    }
}
