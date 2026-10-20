//! `urdf::UrdfRobot` (a robot description read by `urdf::read_str`),
//! `robot::UrdfRobot` (trait `Robot`) and its output,
//! `robot::RobotTransforms`. The two Rust `UrdfRobot`s are `UrdfDescription`
//! and `UrdfRobot` in JS.
//!
//! What the description and the transforms hold is plain data: plain objects
//! and arrays, made anew on every read, whose optional parts are `null` when
//! missing.

use std::collections::{BTreeMap, HashMap};

use js_sys::{Array, Map, Object};
use wasm_bindgen::prelude::*;

use crate::{
    Robot,
    robot::{RobotTransforms, UrdfRobot},
    urdf,
};

use super::{
    Result, describe, dictionary, error, index, nullable, number, numbers, object, string,
    twist_object, type_error, vector,
};

fn vec3(v: urdf::Vector3) -> JsValue {
    numbers(&[v.x, v.y, v.z]).into()
}

fn origin(origin: urdf::UrdfOrigin) -> JsValue {
    object(&[("xyz", vec3(origin.xyz)), ("rpy", vec3(origin.rpy))]).into()
}

fn geometry(geometry: &urdf::UrdfGeometry) -> JsValue {
    let kind = |name: &str| ("type", JsValue::from_str(name));
    match *geometry {
        urdf::UrdfGeometry::Mesh(ref mesh) => object(&[
            kind("mesh"),
            ("filename", JsValue::from_str(&mesh.filename)),
            ("scale", nullable(mesh.scale, vec3)),
        ]),
        urdf::UrdfGeometry::Box { size } => object(&[kind("box"), ("size", vec3(size))]),
        urdf::UrdfGeometry::Sphere { radius } => {
            object(&[kind("sphere"), ("radius", radius.into())])
        }
        urdf::UrdfGeometry::Cylinder { radius, length } => object(&[
            kind("cylinder"),
            ("radius", radius.into()),
            ("length", length.into()),
        ]),
        urdf::UrdfGeometry::Capsule { radius, length } => object(&[
            kind("capsule"),
            ("radius", radius.into()),
            ("length", length.into()),
        ]),
    }
    .into()
}

fn optional_string(value: &Option<String>) -> JsValue {
    nullable(value.as_deref(), JsValue::from_str)
}

fn material(material: &urdf::UrdfMaterial) -> JsValue {
    let color = |color: &urdf::UrdfColor| -> JsValue {
        object(&[("rgba", numbers(&color.rgba).into())]).into()
    };
    object(&[
        ("name", optional_string(&material.name)),
        ("color", nullable(material.color.as_ref(), color)),
    ])
    .into()
}

fn visual(visual: &urdf::UrdfVisual) -> JsValue {
    object(&[
        ("name", optional_string(&visual.name)),
        ("origin", nullable(visual.origin, origin)),
        ("geometry", nullable(visual.geometry.as_ref(), geometry)),
        ("material", nullable(visual.material.as_ref(), material)),
    ])
    .into()
}

fn joint_limit(limit: urdf::UrdfJointLimit) -> JsValue {
    let urdf::UrdfJointLimit {
        lower,
        upper,
        effort,
        velocity,
        acceleration,
        deceleration,
        jerk,
    } = limit;
    object(&[
        ("lower", lower.into()),
        ("upper", upper.into()),
        ("effort", effort.into()),
        ("velocity", velocity.into()),
        ("acceleration", acceleration.into()),
        ("deceleration", deceleration.into()),
        ("jerk", jerk.into()),
    ])
    .into()
}

fn joint(joint: &urdf::UrdfJoint) -> JsValue {
    object(&[
        ("name", JsValue::from_str(&joint.name)),
        ("joint_type", JsValue::from_str(&joint.joint_type)),
        ("parent", JsValue::from_str(&joint.parent)),
        ("child", JsValue::from_str(&joint.child)),
        ("origin", nullable(joint.origin, origin)),
        ("axis", nullable(joint.axis, vec3)),
        ("limit", nullable(joint.limit, joint_limit)),
    ])
    .into()
}

/// Robot description read from URDF (`urdf::UrdfRobot`).
#[wasm_bindgen(js_name = UrdfDescription)]
pub struct JsUrdfDescription(urdf::UrdfRobot);

#[wasm_bindgen(js_class = UrdfDescription)]
impl JsUrdfDescription {
    /// `urdf::read_str`: reads URDF from a string. Mesh file names are kept as
    /// they are. `Error` when it is no valid URDF.
    pub fn read_str(
        #[wasm_bindgen(unchecked_param_type = "string")] xml: JsValue,
    ) -> Result<JsUrdfDescription> {
        let xml = string(&xml, &|| "xml".to_owned())?;
        urdf::read_str(&xml).map(Self).map_err(error)
    }

    /// Name of the robot.
    #[wasm_bindgen(getter)]
    pub fn name(&self) -> String {
        self.0.name.clone()
    }

    #[wasm_bindgen(getter, unchecked_return_type = "UrdfLink[]")]
    pub fn links(&self) -> Array {
        self.0
            .links
            .iter()
            .map(|link| {
                let visuals: Array = link.visuals.iter().map(visual).collect();
                object(&[
                    ("name", JsValue::from_str(&link.name)),
                    ("visuals", visuals.into()),
                ])
            })
            .collect()
    }

    #[wasm_bindgen(getter, unchecked_return_type = "UrdfJoint[]")]
    pub fn joints(&self) -> Array {
        self.0.joints.iter().map(joint).collect()
    }

    /// Top level materials.
    #[wasm_bindgen(getter, unchecked_return_type = "UrdfMaterial[]")]
    pub fn materials(&self) -> Array {
        self.0.materials.iter().map(material).collect()
    }
}

/// Joint positions by joint name, from a plain object or a `Map`.
fn joint_positions(input: &JsValue) -> Result<HashMap<String, f32>> {
    let entries = if let Some(map) = input.dyn_ref::<Map>() {
        Array::from(&map.entries())
    } else if input.is_object() {
        Object::entries(input.unchecked_ref())
    } else {
        return Err(type_error(format!(
            "input must be an object or a Map of joint positions by joint name (got {})",
            describe(input)
        )));
    };
    entries
        .iter()
        .map(|entry| {
            let entry: Array = entry.unchecked_into();
            let joint = string(&entry.get(0), &|| "a joint name in input".to_owned())?;
            let position = number(&entry.get(1), &|| format!("input[{joint:?}]"))? as f32;
            Ok((joint, position))
        })
        .collect()
}

/// Robot computing link transforms from joint positions
/// (`robot::UrdfRobot`). Create it with `from_description`.
#[wasm_bindgen(js_name = UrdfRobot)]
pub struct JsUrdfRobot(UrdfRobot);

#[wasm_bindgen(js_class = UrdfRobot)]
impl JsUrdfRobot {
    /// `Robot::from_description` with a copy of `description`.
    pub fn from_description(description: &JsUrdfDescription) -> JsUrdfRobot {
        Self(UrdfRobot::from_description(description.0.clone()))
    }

    /// `UrdfRobot::description`, as a copy.
    pub fn description(&self) -> JsUrdfDescription {
        JsUrdfDescription(self.0.description().clone())
    }

    /// `Robot::feed_sensor_input`: transforms for joint positions (radians or
    /// meters) by joint name, in a plain object or a `Map`. Joints left out
    /// are at 0.
    pub fn feed_sensor_input(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Record<string, number> | Map<string, number>")]
        input: JsValue,
    ) -> Result<JsRobotTransforms> {
        let input = joint_positions(&input)?;
        Ok(JsRobotTransforms(self.0.feed_sensor_input(&input)))
    }

    /// `Robot::last_transform`: a copy of what the last `feed_sensor_input`
    /// returned, `undefined` before the first one.
    pub fn last_transform(&self) -> Option<JsRobotTransforms> {
        self.0.last_transform().cloned().map(JsRobotTransforms)
    }
}

/// Transforms of every link and visual for one set of joint positions
/// (`robot::RobotTransforms`).
#[wasm_bindgen(js_name = RobotTransforms)]
pub struct JsRobotTransforms(RobotTransforms);

#[wasm_bindgen(js_class = RobotTransforms)]
impl JsRobotTransforms {
    /// World transform of every link, by link name.
    #[wasm_bindgen(getter, unchecked_return_type = "Record<string, TwistLike>")]
    pub fn links(&self) -> Result<Object> {
        dictionary(
            self.0
                .links
                .iter()
                .map(|(link, &twist)| (link.clone(), twist_object(twist))),
        )
    }

    /// World transform of every visual, by link name and then visual index.
    #[wasm_bindgen(getter, unchecked_return_type = "Record<string, TwistLike[]>")]
    pub fn visuals(&self) -> Result<Object> {
        let mut by_link: BTreeMap<&str, BTreeMap<usize, JsValue>> = BTreeMap::new();
        for ((link, visual_index), &twist) in &self.0.visuals {
            by_link
                .entry(link)
                .or_default()
                .insert(*visual_index, twist_object(twist));
        }
        // Every visual of a link has a transform, so the indices are 0..n
        dictionary(by_link.into_iter().map(|(link, visuals)| {
            let visuals: Array = visuals.into_values().collect();
            (link.to_owned(), visuals.into())
        }))
    }

    /// `RobotTransforms::transform_visual_point`: `point` given in the frame
    /// of visual number `visual_index` of `link`, in world space. `undefined`
    /// when there is no such visual.
    #[wasm_bindgen(unchecked_return_type = "Vec3 | undefined")]
    pub fn transform_visual_point(
        &self,
        #[wasm_bindgen(unchecked_param_type = "string")] link: JsValue,
        #[wasm_bindgen(unchecked_param_type = "number")] visual_index: JsValue,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] point: JsValue,
    ) -> Result<JsValue> {
        let link = string(&link, &|| "link".to_owned())?;
        let visual_index = index(&visual_index, &|| "visual_index".to_owned())?;
        let point = vector(&point, &|| "point".to_owned())?;
        let point = usize::try_from(visual_index)
            .ok()
            .and_then(|visual_index| self.0.transform_visual_point(&link, visual_index, point));
        Ok(point.map_or(JsValue::UNDEFINED, |point| numbers(&point).into()))
    }

    /// `RobotTransforms::get_link_origin`: world position of `link`,
    /// `undefined` when there is no such link.
    #[wasm_bindgen(unchecked_return_type = "Vec3 | undefined")]
    pub fn get_link_origin(
        &self,
        #[wasm_bindgen(unchecked_param_type = "string")] link: JsValue,
    ) -> Result<JsValue> {
        let link = string(&link, &|| "link".to_owned())?;
        let origin = self.0.get_link_origin(&link);
        Ok(origin.map_or(JsValue::UNDEFINED, |origin| {
            numbers(&[origin.x, origin.y, origin.z]).into()
        }))
    }
}
