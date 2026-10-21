//! `urdf::UrdfRobot` (a robot description read by `urdf::read_file` and
//! `urdf::read_str`), `robot::UrdfRobot` (trait `Robot`) and its output,
//! `robot::RobotTransforms`. The two Rust `UrdfRobot`s are
//! `R3vUrdfDescription` and `R3vUrdfRobot` in C.

use std::{
    collections::HashMap,
    ffi::{CString, c_char},
    path::Path,
    ptr,
};

use crate::{
    Robot,
    robot::{RobotTransforms, UrdfRobot},
    urdf,
};

use super::{
    Error, R3vStatus, R3vTwist, R3vVec3, arg, arg_mut, drop_handle, new_handle, slice_arg, status,
    str_arg,
};

/// Robot description read from URDF (`urdf::UrdfRobot`). Every string it
/// hands out stays valid until it is dropped.
pub struct R3vUrdfDescription {
    description: urdf::UrdfRobot,
    // Owns the strings the C views below point into. A CString keeps its
    // bytes on the heap, so they do not move when this Vec grows.
    _strings: Vec<CString>,
    name: *const c_char,
    links: Vec<R3vUrdfLink>,
    // Per link
    visuals: Vec<Vec<R3vUrdfVisual>>,
    joints: Vec<R3vUrdfJoint>,
    materials: Vec<R3vUrdfMaterial>,
}

// SAFETY: the raw pointers only point into `_strings`, which moves along
unsafe impl Send for R3vUrdfDescription {}

/// Position and orientation (`urdf::UrdfOrigin`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct R3vUrdfOrigin {
    pub xyz: R3vVec3,
    /// Roll, pitch and yaw in radians.
    pub rpy: R3vVec3,
}

/// Link of a robot description (`urdf::UrdfLink`).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct R3vUrdfLink {
    pub name: *const c_char,
    /// Read the visuals with `r3v_urdf_description_visual`.
    pub visual_count: usize,
}

/// Which kind of `urdf::UrdfGeometry` an `R3vUrdfGeometry` is.
#[repr(C)]
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum R3vUrdfGeometryKind {
    R3V_URDF_GEOMETRY_MESH,
    R3V_URDF_GEOMETRY_BOX,
    R3V_URDF_GEOMETRY_SPHERE,
    R3V_URDF_GEOMETRY_CYLINDER,
    R3V_URDF_GEOMETRY_CAPSULE,
}

/// Shape of a visual (`urdf::UrdfGeometry`). Only the fields of its `kind`
/// are set, the others are zero.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct R3vUrdfGeometry {
    pub kind: R3vUrdfGeometryKind,
    /// Mesh: file name, resolved against the URDF file's directory and
    /// `package://` ancestors when read with `r3v_urdf_read_file`.
    pub mesh_filename: *const c_char,
    /// Mesh: whether `mesh_scale` was given.
    pub has_mesh_scale: bool,
    pub mesh_scale: R3vVec3,
    /// Box: edge lengths.
    pub box_size: R3vVec3,
    /// Sphere, cylinder and capsule.
    pub radius: f32,
    /// Cylinder and capsule.
    pub length: f32,
}

impl R3vUrdfGeometry {
    const ZERO: Self = Self {
        kind: R3vUrdfGeometryKind::R3V_URDF_GEOMETRY_MESH,
        mesh_filename: ptr::null(),
        has_mesh_scale: false,
        mesh_scale: R3vVec3::ZERO,
        box_size: R3vVec3::ZERO,
        radius: 0.0,
        length: 0.0,
    };
}

/// Material (`urdf::UrdfMaterial`).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct R3vUrdfMaterial {
    /// NULL when the material has no name.
    pub name: *const c_char,
    /// Whether `rgba` was given.
    pub has_color: bool,
    pub rgba: [f32; 4],
}

impl R3vUrdfMaterial {
    const ZERO: Self = Self {
        name: ptr::null(),
        has_color: false,
        rgba: [0.0; 4],
    };
}

/// Visual of a link (`urdf::UrdfVisual`). The `has_*` flags tell which
/// optional parts are there.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct R3vUrdfVisual {
    /// NULL when the visual has no name.
    pub name: *const c_char,
    pub has_origin: bool,
    pub origin: R3vUrdfOrigin,
    pub has_geometry: bool,
    pub geometry: R3vUrdfGeometry,
    pub has_material: bool,
    pub material: R3vUrdfMaterial,
}

/// Limits of a joint (`urdf::UrdfJointLimit`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct R3vUrdfJointLimit {
    pub lower: f32,
    pub upper: f32,
    pub effort: f32,
    pub velocity: f32,
    pub acceleration: f32,
    pub deceleration: f32,
    pub jerk: f32,
}

/// Joint of a robot description (`urdf::UrdfJoint`). The `has_*` flags tell
/// which optional parts are there.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct R3vUrdfJoint {
    pub name: *const c_char,
    /// "revolute", "continuous", "prismatic", "fixed", "floating", "planar"
    /// or "spherical".
    pub joint_type: *const c_char,
    /// Parent link name.
    pub parent: *const c_char,
    /// Child link name.
    pub child: *const c_char,
    pub has_origin: bool,
    pub origin: R3vUrdfOrigin,
    pub has_axis: bool,
    pub axis: R3vVec3,
    pub has_limit: bool,
    pub limit: R3vUrdfJointLimit,
}

impl From<urdf::Vector3> for R3vVec3 {
    fn from(v: urdf::Vector3) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<urdf::UrdfOrigin> for R3vUrdfOrigin {
    fn from(origin: urdf::UrdfOrigin) -> Self {
        Self {
            xyz: origin.xyz.into(),
            rpy: origin.rpy.into(),
        }
    }
}

impl From<urdf::UrdfJointLimit> for R3vUrdfJointLimit {
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

/// Builds the C strings of a description.
#[derive(Default)]
struct Strings(Vec<CString>);

impl Strings {
    fn add(&mut self, string: &str) -> super::Result<*const c_char> {
        let string = CString::new(string).map_err(|_| {
            Error::failed(format!("URDF string {string:?} contains a NUL character"))
        })?;
        let pointer = string.as_ptr();
        self.0.push(string);
        Ok(pointer)
    }

    fn add_optional(&mut self, string: Option<&str>) -> super::Result<*const c_char> {
        string.map_or(Ok(ptr::null()), |string| self.add(string))
    }

    fn material(&mut self, material: &urdf::UrdfMaterial) -> super::Result<R3vUrdfMaterial> {
        Ok(R3vUrdfMaterial {
            name: self.add_optional(material.name.as_deref())?,
            has_color: material.color.is_some(),
            rgba: material.color.as_ref().map_or([0.0; 4], |color| color.rgba),
        })
    }

    fn geometry(&mut self, geometry: &urdf::UrdfGeometry) -> super::Result<R3vUrdfGeometry> {
        let mut out = R3vUrdfGeometry::ZERO;
        match geometry {
            urdf::UrdfGeometry::Mesh(mesh) => {
                out.mesh_filename = self.add(&mesh.filename)?;
                out.has_mesh_scale = mesh.scale.is_some();
                out.mesh_scale = mesh.scale.map(Into::into).unwrap_or_default();
            }
            urdf::UrdfGeometry::Box { size } => {
                out.kind = R3vUrdfGeometryKind::R3V_URDF_GEOMETRY_BOX;
                out.box_size = (*size).into();
            }
            urdf::UrdfGeometry::Sphere { radius } => {
                out.kind = R3vUrdfGeometryKind::R3V_URDF_GEOMETRY_SPHERE;
                out.radius = *radius;
            }
            urdf::UrdfGeometry::Cylinder { radius, length } => {
                out.kind = R3vUrdfGeometryKind::R3V_URDF_GEOMETRY_CYLINDER;
                out.radius = *radius;
                out.length = *length;
            }
            urdf::UrdfGeometry::Capsule { radius, length } => {
                out.kind = R3vUrdfGeometryKind::R3V_URDF_GEOMETRY_CAPSULE;
                out.radius = *radius;
                out.length = *length;
            }
        }
        Ok(out)
    }

    fn visual(&mut self, visual: &urdf::UrdfVisual) -> super::Result<R3vUrdfVisual> {
        Ok(R3vUrdfVisual {
            name: self.add_optional(visual.name.as_deref())?,
            has_origin: visual.origin.is_some(),
            origin: visual.origin.map(Into::into).unwrap_or_default(),
            has_geometry: visual.geometry.is_some(),
            geometry: match &visual.geometry {
                Some(geometry) => self.geometry(geometry)?,
                None => R3vUrdfGeometry::ZERO,
            },
            has_material: visual.material.is_some(),
            material: match &visual.material {
                Some(material) => self.material(material)?,
                None => R3vUrdfMaterial::ZERO,
            },
        })
    }

    fn joint(&mut self, joint: &urdf::UrdfJoint) -> super::Result<R3vUrdfJoint> {
        Ok(R3vUrdfJoint {
            name: self.add(&joint.name)?,
            joint_type: self.add(&joint.joint_type)?,
            parent: self.add(&joint.parent)?,
            child: self.add(&joint.child)?,
            has_origin: joint.origin.is_some(),
            origin: joint.origin.map(Into::into).unwrap_or_default(),
            has_axis: joint.axis.is_some(),
            axis: joint.axis.map(Into::into).unwrap_or_default(),
            has_limit: joint.limit.is_some(),
            limit: joint.limit.map(Into::into).unwrap_or_default(),
        })
    }
}

impl R3vUrdfDescription {
    fn new(description: urdf::UrdfRobot) -> super::Result<Self> {
        let mut strings = Strings::default();
        let name = strings.add(&description.name)?;
        let links = description
            .links
            .iter()
            .map(|link| {
                Ok(R3vUrdfLink {
                    name: strings.add(&link.name)?,
                    visual_count: link.visuals.len(),
                })
            })
            .collect::<super::Result<_>>()?;
        let visuals = description
            .links
            .iter()
            .map(|link| {
                link.visuals
                    .iter()
                    .map(|visual| strings.visual(visual))
                    .collect()
            })
            .collect::<super::Result<_>>()?;
        let joints = description
            .joints
            .iter()
            .map(|joint| strings.joint(joint))
            .collect::<super::Result<_>>()?;
        let materials = description
            .materials
            .iter()
            .map(|material| strings.material(material))
            .collect::<super::Result<_>>()?;
        Ok(Self {
            description,
            _strings: strings.0,
            name,
            links,
            visuals,
            joints,
            materials,
        })
    }
}

/// Copies `items[index]` to `out`.
///
/// # Safety
/// As for [`arg_mut`].
unsafe fn copy_item<T: Copy>(
    items: &[T],
    index: usize,
    what: &str,
    out: *mut T,
) -> super::Result<()> {
    let out = unsafe { arg_mut(out, "out") }?;
    *out = *items.get(index).ok_or_else(|| {
        Error::out_of_range(format!(
            "{what} {index} does not exist, there are {}",
            items.len()
        ))
    })?;
    Ok(())
}

/// # Safety
/// As for [`arg`].
unsafe fn description_arg<'a>(
    description: *const R3vUrdfDescription,
) -> super::Result<&'a R3vUrdfDescription> {
    unsafe { arg(description, "description") }
}

/// `urdf::read_file`: reads a URDF file. Relative mesh file names are
/// resolved against its directory. NULL on failure.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_read_file(path: *const c_char) -> *mut R3vUrdfDescription {
    new_handle(|| {
        let path = unsafe { str_arg(path, "path") }?;
        R3vUrdfDescription::new(urdf::read_file(Path::new(path)).map_err(Error::failed)?)
    })
}

/// `urdf::read_str`: reads URDF from a string. Mesh file names are kept as
/// they are. NULL on failure.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_read_str(xml: *const c_char) -> *mut R3vUrdfDescription {
    new_handle(|| {
        let xml = unsafe { str_arg(xml, "xml") }?;
        R3vUrdfDescription::new(urdf::read_str(xml).map_err(Error::failed)?)
    })
}

/// Name of the robot, NULL for NULL.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_name(
    description: *const R3vUrdfDescription,
) -> *const c_char {
    unsafe { description.as_ref() }.map_or(ptr::null(), |description| description.name)
}

/// Number of links, 0 for NULL.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_link_count(description: *const R3vUrdfDescription) -> usize {
    unsafe { description.as_ref() }.map_or(0, |description| description.links.len())
}

/// Link number `link`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_link(
    description: *const R3vUrdfDescription,
    link: usize,
    out: *mut R3vUrdfLink,
) -> R3vStatus {
    status(|| {
        let description = unsafe { description_arg(description) }?;
        unsafe { copy_item(&description.links, link, "link", out) }
    })
}

/// Visual number `visual` of link number `link`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_visual(
    description: *const R3vUrdfDescription,
    link: usize,
    visual: usize,
    out: *mut R3vUrdfVisual,
) -> R3vStatus {
    status(|| {
        let description = unsafe { description_arg(description) }?;
        let visuals = description.visuals.get(link).ok_or_else(|| {
            Error::out_of_range(format!(
                "link {link} does not exist, there are {}",
                description.visuals.len()
            ))
        })?;
        unsafe { copy_item(visuals, visual, "visual", out) }
    })
}

/// Number of joints, 0 for NULL.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_joint_count(
    description: *const R3vUrdfDescription,
) -> usize {
    unsafe { description.as_ref() }.map_or(0, |description| description.joints.len())
}

/// Joint number `joint`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_joint(
    description: *const R3vUrdfDescription,
    joint: usize,
    out: *mut R3vUrdfJoint,
) -> R3vStatus {
    status(|| {
        let description = unsafe { description_arg(description) }?;
        unsafe { copy_item(&description.joints, joint, "joint", out) }
    })
}

/// Number of top level materials, 0 for NULL.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_material_count(
    description: *const R3vUrdfDescription,
) -> usize {
    unsafe { description.as_ref() }.map_or(0, |description| description.materials.len())
}

/// Top level material number `material`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_material(
    description: *const R3vUrdfDescription,
    material: usize,
    out: *mut R3vUrdfMaterial,
) -> R3vStatus {
    status(|| {
        let description = unsafe { description_arg(description) }?;
        unsafe { copy_item(&description.materials, material, "material", out) }
    })
}

/// Frees a description. NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_description_drop(description: *mut R3vUrdfDescription) {
    unsafe { drop_handle(description) }
}

/// Robot computing link transforms from joint positions
/// (`robot::UrdfRobot`).
pub struct R3vUrdfRobot(UrdfRobot);

/// Transforms of every link and visual for one set of joint positions
/// (`robot::RobotTransforms`).
pub struct R3vRobotTransforms {
    // Never instantiated: a `*R3vRobotTransforms` points to a RobotTransforms,
    // either owned by the caller or the robot's last one
    _private: [u8; 0],
}

/// # Safety
/// A non NULL `transforms` is valid for reads during `'a`.
unsafe fn transforms_arg<'a>(
    transforms: *const R3vRobotTransforms,
) -> super::Result<&'a RobotTransforms> {
    unsafe { arg(transforms.cast::<RobotTransforms>(), "transforms") }
}

/// `Robot::from_description` with a copy of `description`. NULL on failure.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_robot_from_description(
    description: *const R3vUrdfDescription,
) -> *mut R3vUrdfRobot {
    new_handle(|| {
        let description = unsafe { description_arg(description) }?;
        Ok(R3vUrdfRobot(UrdfRobot::from_description(
            description.description.clone(),
        )))
    })
}

/// `Robot::feed_sensor_input`: transforms for the joints named in
/// `joint_names` at `joint_values` (radians or meters), `count` of each.
/// Joints left out are at 0. Drop the result with
/// `r3v_robot_transforms_drop`. NULL on failure.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_robot_feed_sensor_input(
    robot: *mut R3vUrdfRobot,
    joint_names: *const *const c_char,
    joint_values: *const f32,
    count: usize,
) -> *mut R3vRobotTransforms {
    new_handle(|| {
        let robot = unsafe { arg_mut(robot, "robot") }?;
        let names = unsafe { slice_arg(joint_names, count, "joint_names") }?;
        let values = unsafe { slice_arg(joint_values, count, "joint_values") }?;
        let input = names
            .iter()
            .zip(values)
            .enumerate()
            .map(|(k, (&name, &value))| {
                let name = unsafe { str_arg(name, &format!("joint_names[{k}]")) }?;
                Ok((name.to_owned(), value))
            })
            .collect::<super::Result<HashMap<_, _>>>()?;
        Ok(robot.0.feed_sensor_input(&input))
    })
    .cast()
}

/// `Robot::last_transform`: what the last `r3v_urdf_robot_feed_sensor_input`
/// returned, owned by the robot and valid until the next feed or until the
/// robot is dropped. Do not drop it. NULL before the first feed and for NULL.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_robot_last_transform(
    robot: *const R3vUrdfRobot,
) -> *const R3vRobotTransforms {
    unsafe { robot.as_ref() }
        .and_then(|robot| robot.0.last_transform())
        .map_or(ptr::null(), |transforms| {
            ptr::from_ref(transforms).cast::<R3vRobotTransforms>()
        })
}

/// Frees a robot. NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_urdf_robot_drop(robot: *mut R3vUrdfRobot) {
    unsafe { drop_handle(robot) }
}

/// World transform of the link named `link` (`RobotTransforms::links`).
#[unsafe(no_mangle)]
pub extern "C" fn r3v_robot_transforms_link(
    transforms: *const R3vRobotTransforms,
    link: *const c_char,
    out: *mut R3vTwist,
) -> R3vStatus {
    status(|| {
        let transforms = unsafe { transforms_arg(transforms) }?;
        let link = unsafe { str_arg(link, "link") }?;
        let out = unsafe { arg_mut(out, "out") }?;
        let twist = transforms
            .links
            .get(link)
            .ok_or_else(|| Error::not_found(format!("no link named {link:?}")))?;
        *out = (*twist).into();
        Ok(())
    })
}

/// World transform of visual number `visual_index` of the link named `link`
/// (`RobotTransforms::visuals`).
#[unsafe(no_mangle)]
pub extern "C" fn r3v_robot_transforms_visual(
    transforms: *const R3vRobotTransforms,
    link: *const c_char,
    visual_index: usize,
    out: *mut R3vTwist,
) -> R3vStatus {
    status(|| {
        let transforms = unsafe { transforms_arg(transforms) }?;
        let link = unsafe { str_arg(link, "link") }?;
        let out = unsafe { arg_mut(out, "out") }?;
        let twist = transforms
            .visuals
            .get(&(link.to_owned(), visual_index))
            .ok_or_else(|| {
                Error::not_found(format!("no visual {visual_index} on a link named {link:?}"))
            })?;
        *out = (*twist).into();
        Ok(())
    })
}

/// `RobotTransforms::get_link_origin`: world position of the link named
/// `link`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_robot_transforms_get_link_origin(
    transforms: *const R3vRobotTransforms,
    link: *const c_char,
    out: *mut R3vVec3,
) -> R3vStatus {
    status(|| {
        let transforms = unsafe { transforms_arg(transforms) }?;
        let link = unsafe { str_arg(link, "link") }?;
        let out = unsafe { arg_mut(out, "out") }?;
        let origin = transforms
            .get_link_origin(link)
            .ok_or_else(|| Error::not_found(format!("no link named {link:?}")))?;
        *out = origin.into();
        Ok(())
    })
}

/// `RobotTransforms::transform_visual_point`: `point` given in the frame of
/// visual number `visual_index` of the link named `link`, in world space.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_robot_transforms_transform_visual_point(
    transforms: *const R3vRobotTransforms,
    link: *const c_char,
    visual_index: usize,
    point: R3vVec3,
    out: *mut R3vVec3,
) -> R3vStatus {
    status(|| {
        let transforms = unsafe { transforms_arg(transforms) }?;
        let link = unsafe { str_arg(link, "link") }?;
        let out = unsafe { arg_mut(out, "out") }?;
        let [x, y, z] = transforms
            .transform_visual_point(link, visual_index, [point.x, point.y, point.z])
            .ok_or_else(|| {
                Error::not_found(format!("no visual {visual_index} on a link named {link:?}"))
            })?;
        *out = R3vVec3 { x, y, z };
        Ok(())
    })
}

/// Frees transforms returned by `r3v_urdf_robot_feed_sensor_input`, never the
/// ones from `r3v_urdf_robot_last_transform`. NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_robot_transforms_drop(transforms: *mut R3vRobotTransforms) {
    unsafe { drop_handle(transforms.cast::<RobotTransforms>()) }
}
