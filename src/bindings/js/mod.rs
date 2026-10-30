//! JS module `r3v_renderer` (built for the web with wasm-pack). Every public
//! component maps 1:1 onto a class named like its C counterpart without the
//! `R3v` prefix, as in Python, and methods keep their snake_case names, as in
//! r3v-voxelizer's JS module.
//!
//! What goes in is read by property name, so the classes of this module and
//! plain objects with the same properties both work, e.g. r3v-voxelizer's
//! `PointI`s or `{x, y, z, i}`. Vectors are arrays of numbers. Data that only
//! comes out (triangle chunks, URDF descriptions, robot transforms) is plain
//! objects and arrays, which `structuredClone`, `postMessage` and
//! `JSON.stringify` keep; classes are for what has methods or is passed back
//! in.
//!
//! Errors are exceptions, or rejections for the async methods: `TypeError`
//! for an argument of the wrong type, `RangeError` for an invalid value
//! (nothing is changed then) or an unknown instance, `Error` for a component
//! used before it is configured or while it renders and for failures such as
//! parsing a file or getting a GPU. A Rust panic would trap and leave the
//! module unusable, so arguments are checked before they reach a component.

use js_sys::{Array, Object, Reflect};
use wasm_bindgen::prelude::*;

use super::checked::{self, ErrorKind};
use crate::{
    camera::CameraState,
    geometry::{self, PointI, RealTriangle, RealTwist, TriangleChunk},
};

mod camera;
mod renderer;
mod robot;
mod stl;
mod tesselator;

#[wasm_bindgen(typescript_custom_section)]
const TYPES: &str = r#"
/** [x, y, z] */
export type Vec3 = [number, number, number];
/** [x, y, z, w] */
export type Vec4 = [number, number, number, number];
/** Column-major 4x4 matrix: `matrix[column][row]`. */
export type Mat4 = [Vec4, Vec4, Vec4, Vec4];

/** A `PointI`, r3v-voxelizer's `PointI` or a plain object. */
export interface PointILike { x: number; y: number; z: number; i: number }
/** A `Twist`, or a plain object such as the twists of `RobotTransforms`. */
export interface TwistLike { linear: Vec3; rotational: Vec4 }
/** A `Triangle`, or a plain object such as the triangles of a
 * `TriangleChunk`. */
export interface TriangleLike { v0: Vec3; v1: Vec3; v2: Vec3 }
/** A `CameraState` or a plain object. */
export interface CameraStateLike { view_proj: Mat4; position: Vec3; forward: Vec3 }

/** What colors the terrain (`renderer::TerrainColoring`), a plain object.
 * What is left out takes its default. With `height_coloring_enabled` it is
 * the height on the world's Y axis, as on a topographic map, rather than the
 * distance along the camera's forward axis. Heights are measured from the
 * world's Y = 0, or with `relative_to_robot` from `robot_height`; to follow
 * the robot `configure` the renderer again, which at an unchanged frame size
 * only changes the settings.
 *
 * The position in the colormap of a height at the distance `d` from the
 * neutral height, `height_offset` above what heights are measured from, is
 * `0.5 + 0.5 * sign(d) * tanh(height_gain * ln(1 + |d| / height_knee))`.
 * `height_bands` splits the colormap into flat bands, 0 for a smooth
 * gradient. */
export interface TerrainColoring {
    height_coloring_enabled?: boolean;
    relative_to_robot?: boolean;
    robot_height?: number;
    height_offset?: number;
    height_knee?: number;
    height_gain?: number;
    height_bands?: number;
}

/** All triangles of one chunk of 16x16x16 voxels, whose origin in voxels is
 * `index`. The triangles are in world units, counter-clockwise when seen from
 * their front side. They replace what the chunk had before, no triangles
 * means it is empty now. */
export interface TriangleChunk { index: Vec3; triangles: TriangleLike[] }

/** Position and orientation: `rpy` is roll, pitch and yaw in radians. */
export interface UrdfOrigin { xyz: Vec3; rpy: Vec3 }
export type UrdfGeometry =
    | { type: "mesh"; filename: string; scale: Vec3 | null }
    | { type: "box"; size: Vec3 }
    | { type: "sphere"; radius: number }
    | { type: "cylinder"; radius: number; length: number }
    | { type: "capsule"; radius: number; length: number };
/** `rgba` from 0 to 1. */
export interface UrdfColor { rgba: Vec4 }
export interface UrdfMaterial { name: string | null; color: UrdfColor | null }
export interface UrdfVisual {
    name: string | null;
    origin: UrdfOrigin | null;
    geometry: UrdfGeometry | null;
    material: UrdfMaterial | null;
}
export interface UrdfLink { name: string; visuals: UrdfVisual[] }
export interface UrdfJointLimit {
    lower: number;
    upper: number;
    effort: number;
    velocity: number;
    acceleration: number;
    deceleration: number;
    jerk: number;
}
/** `joint_type` is "revolute", "continuous", "prismatic", "fixed", "floating",
 * "planar" or "spherical"; `parent` and `child` are link names. */
export interface UrdfJoint {
    name: string;
    joint_type: string;
    parent: string;
    child: string;
    origin: UrdfOrigin | null;
    axis: Vec3 | null;
    limit: UrdfJointLimit | null;
}
"#;

type Result<T> = std::result::Result<T, JsValue>;

/// Names a value in error messages, only built when there is an error.
type What<'a> = &'a dyn Fn() -> String;

const I32_MIN: f64 = i32::MIN as f64;
const I32_MAX: f64 = i32::MAX as f64;
const U32_MAX: f64 = u32::MAX as f64;
/// `Number.MAX_SAFE_INTEGER`: integers up to it are exact as numbers.
const MAX_SAFE_INTEGER: f64 = 9_007_199_254_740_991.0;

fn type_error(message: impl AsRef<str>) -> JsValue {
    js_sys::TypeError::new(message.as_ref()).into()
}

fn range_error(message: impl AsRef<str>) -> JsValue {
    js_sys::RangeError::new(message.as_ref()).into()
}

fn error(message: impl AsRef<str>) -> JsValue {
    js_sys::Error::new(message.as_ref()).into()
}

impl From<checked::Error> for JsValue {
    fn from(error: checked::Error) -> Self {
        match error.kind {
            ErrorKind::InvalidArgument | ErrorKind::OutOfRange => range_error(error.message),
            ErrorKind::NotConfigured | ErrorKind::Failed => self::error(error.message),
        }
    }
}

/// What `value` is, for error messages.
fn describe(value: &JsValue) -> String {
    if value.is_null() {
        "null".to_owned()
    } else if Array::is_array(value) {
        "an array".to_owned()
    } else {
        value.js_typeof().as_string().unwrap_or_default()
    }
}

// ---------------------------------------------------------------- reading

/// `object[key]`.
fn property(object: &JsValue, key: &str, what: What) -> Result<JsValue> {
    if !object.is_object() {
        return Err(type_error(format!(
            "{} is not an object (got {})",
            what(),
            describe(object)
        )));
    }
    Reflect::get(object, &JsValue::from_str(key))
}

fn number(value: &JsValue, what: What) -> Result<f64> {
    value.as_f64().ok_or_else(|| {
        type_error(format!(
            "{} is not a number (got {})",
            what(),
            describe(value)
        ))
    })
}

fn f32_property(object: &JsValue, key: &str, what: What) -> Result<f32> {
    let value = property(object, key, what)?;
    Ok(number(&value, &|| format!("{}.{key}", what()))? as f32)
}

/// `object[key]` as a boolean, false when it is `undefined`.
fn flag_property(object: &JsValue, key: &str, what: What) -> Result<bool> {
    let value = property(object, key, what)?;
    if value.is_undefined() {
        return Ok(false);
    }
    value.as_bool().ok_or_else(|| {
        type_error(format!(
            "{}.{key} is not a boolean (got {})",
            what(),
            describe(&value)
        ))
    })
}

/// An integer number from `min` to `max`.
fn integer(value: &JsValue, min: f64, max: f64, what: What) -> Result<f64> {
    let number = number(value, what)?;
    if number.fract() != 0.0 || !(min..=max).contains(&number) {
        return Err(range_error(format!(
            "{} must be an integer from {min} to {max}, got {number}",
            what()
        )));
    }
    Ok(number)
}

fn integer_property(object: &JsValue, key: &str, min: f64, max: f64, what: What) -> Result<f64> {
    let value = property(object, key, what)?;
    integer(&value, min, max, &|| format!("{}.{key}", what()))
}

/// An index, a count or an instance id: integers exact as numbers.
fn index(value: &JsValue, what: What) -> Result<u64> {
    Ok(integer(value, 0.0, MAX_SAFE_INTEGER, what)? as u64)
}

fn string(value: &JsValue, what: What) -> Result<String> {
    value.as_string().ok_or_else(|| {
        type_error(format!(
            "{} is not a string (got {})",
            what(),
            describe(value)
        ))
    })
}

/// An array (or typed array) of `N` numbers.
fn vector<const N: usize>(value: &JsValue, what: What) -> Result<[f32; N]> {
    let wrong = || type_error(format!("{} must be an array of {N} numbers", what()));
    if !value.is_object() || Reflect::get(value, &"length".into())?.as_f64() != Some(N as f64) {
        return Err(wrong());
    }
    let mut out = [0.0; N];
    for (k, out) in (0..).zip(&mut out) {
        *out = number(&Reflect::get_u32(value, k)?, &|| format!("{}[{k}]", what()))? as f32;
    }
    Ok(out)
}

fn vector_property<const N: usize>(object: &JsValue, key: &str, what: What) -> Result<[f32; N]> {
    let value = property(object, key, what)?;
    vector(&value, &|| format!("{}.{key}", what()))
}

fn array<'a>(value: &'a JsValue, what: What) -> Result<&'a Array> {
    value.dyn_ref::<Array>().ok_or_else(|| {
        type_error(format!(
            "{} is not an array (got {})",
            what(),
            describe(value)
        ))
    })
}

/// A block of voxels as `PointI` reads it.
fn point_i(value: &JsValue, what: What) -> Result<PointI> {
    let coordinate = |key| integer_property(value, key, I32_MIN, I32_MAX, what);
    Ok(PointI {
        x: coordinate("x")? as i32,
        y: coordinate("y")? as i32,
        z: coordinate("z")? as i32,
        i: integer_property(value, "i", 0.0, U32_MAX, what)? as u32,
    })
}

/// A twist as `Twist` reads it.
fn twist(value: &JsValue, what: What) -> Result<RealTwist> {
    let [x, y, z] = vector_property(value, "linear", what)?;
    let rotational = vector_property(value, "rotational", what)?;
    Ok(JsTwist {
        linear: [x, y, z],
        rotational,
    }
    .into())
}

/// A triangle as `Triangle` reads it.
fn triangle(value: &JsValue, what: What) -> Result<RealTriangle> {
    Ok(RealTriangle {
        v0: vector_property(value, "v0", what)?,
        v1: vector_property(value, "v1", what)?,
        v2: vector_property(value, "v2", what)?,
    })
}

fn view_proj(value: &JsValue, what: What) -> Result<[[f32; 4]; 4]> {
    let wrong = || type_error(format!("{} must be an array of 4 columns", what()));
    if !value.is_object() || Reflect::get(value, &"length".into())?.as_f64() != Some(4.0) {
        return Err(wrong());
    }
    let mut out = [[0.0; 4]; 4];
    for (k, column) in (0..).zip(&mut out) {
        *column = vector(&Reflect::get_u32(value, k)?, &|| format!("{}[{k}]", what()))?;
    }
    Ok(out)
}

/// A camera state as `CameraState` reads it.
fn camera_state(value: &JsValue, what: What) -> Result<CameraState> {
    let matrix = property(value, "view_proj", what)?;
    Ok(JsCameraState {
        view_proj: view_proj(&matrix, &|| format!("{}.view_proj", what()))?,
        position: vector_property(value, "position", what)?,
        forward: vector_property(value, "forward", what)?,
    }
    .into())
}

// ---------------------------------------------------------------- writing

/// A plain array of numbers.
fn numbers<T: Copy + Into<f64>>(values: &[T]) -> Array {
    values
        .iter()
        .map(|&value| JsValue::from_f64(value.into()))
        .collect()
}

/// A plain object with `properties`, whose keys are known not to be special
/// (such as `__proto__`).
fn object(properties: &[(&str, JsValue)]) -> Object {
    let object = Object::new();
    for (key, value) in properties {
        Reflect::set(&object, &JsValue::from_str(key), value)
            .expect("a new plain object takes any property");
    }
    object
}

/// A plain object with any keys (e.g. names read from a file), sorted.
fn dictionary(entries: impl IntoIterator<Item = (String, JsValue)>) -> Result<Object> {
    let mut entries: Vec<_> = entries.into_iter().collect();
    entries.sort_by(|(a, _), (b, _)| a.cmp(b));
    let entries: Array = entries
        .into_iter()
        .map(|(key, value)| Array::of2(&key.into(), &value))
        .collect();
    // Unlike setting properties one by one, makes `__proto__` a property too
    Object::from_entries(&entries)
}

/// `null` for `None`.
fn nullable<T>(value: Option<T>, f: impl FnOnce(T) -> JsValue) -> JsValue {
    value.map_or(JsValue::NULL, f)
}

fn twist_object(twist: RealTwist) -> JsValue {
    let twist = JsTwist::from(twist);
    object(&[
        ("linear", numbers(&twist.linear).into()),
        ("rotational", numbers(&twist.rotational).into()),
    ])
    .into()
}

fn triangle_chunk_object(chunk: &TriangleChunk) -> JsValue {
    let index = chunk.index;
    let triangles: Array = chunk
        .triangles
        .iter()
        .map(|triangle| {
            object(&[
                ("v0", numbers(&triangle.v0).into()),
                ("v1", numbers(&triangle.v1).into()),
                ("v2", numbers(&triangle.v2).into()),
            ])
        })
        .collect();
    object(&[
        ("index", numbers(&[index.x, index.y, index.z]).into()),
        ("triangles", triangles.into()),
    ])
    .into()
}

// ---------------------------------------------------------- value classes

/// One 4x4x2 block of voxels (`geometry::PointI`): x, y, z are the voxel
/// origin of the block, i its occupancy mask with bit x + 4 * y + 16 * z for
/// the voxel at (x, y, z) inside the block. Same interface as r3v-voxelizer's
/// `PointI`, whose compress mode output can be passed on as it is.
#[wasm_bindgen(js_name = PointI)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JsPointI {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub i: u32,
}

#[wasm_bindgen(js_class = PointI)]
impl JsPointI {
    #[wasm_bindgen(constructor)]
    pub fn new(x: i32, y: i32, z: i32, i: u32) -> Self {
        Self { x, y, z, i }
    }
}

/// Rigid transform (`geometry::RealTwist`): rotation by the quaternion
/// `rotational` (x, y, z, w), then translation by `linear`.
#[wasm_bindgen(js_name = Twist)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JsTwist {
    linear: [f32; 3],
    rotational: [f32; 4],
}

#[wasm_bindgen(js_class = Twist)]
impl JsTwist {
    /// `linear` is a `Vec3`, `rotational` a `Vec4`. Without them the
    /// identity: no translation, no rotation.
    #[wasm_bindgen(constructor)]
    pub fn new(
        #[wasm_bindgen(unchecked_optional_param_type = "Vec3")] linear: Option<JsValue>,
        #[wasm_bindgen(unchecked_optional_param_type = "Vec4")] rotational: Option<JsValue>,
    ) -> Result<JsTwist> {
        Ok(Self {
            linear: match linear {
                Some(linear) => vector(&linear, &|| "linear".to_owned())?,
                None => [0.0; 3],
            },
            rotational: match rotational {
                Some(rotational) => vector(&rotational, &|| "rotational".to_owned())?,
                None => [0.0, 0.0, 0.0, 1.0],
            },
        })
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec3")]
    pub fn linear(&self) -> Array {
        numbers(&self.linear)
    }

    #[wasm_bindgen(setter)]
    pub fn set_linear(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] linear: JsValue,
    ) -> Result<()> {
        self.linear = vector(&linear, &|| "linear".to_owned())?;
        Ok(())
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec4")]
    pub fn rotational(&self) -> Array {
        numbers(&self.rotational)
    }

    #[wasm_bindgen(setter)]
    pub fn set_rotational(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec4")] rotational: JsValue,
    ) -> Result<()> {
        self.rotational = vector(&rotational, &|| "rotational".to_owned())?;
        Ok(())
    }
}

impl From<JsTwist> for RealTwist {
    fn from(twist: JsTwist) -> Self {
        let [x, y, z] = twist.linear;
        let [qx, qy, qz, qw] = twist.rotational;
        Self {
            linear: geometry::Vector3D { x, y, z },
            rotational: geometry::Quaternion {
                x: qx,
                y: qy,
                z: qz,
                w: qw,
            },
        }
    }
}

impl From<RealTwist> for JsTwist {
    fn from(twist: RealTwist) -> Self {
        let geometry::Vector3D { x, y, z } = twist.linear;
        let geometry::Quaternion {
            x: qx,
            y: qy,
            z: qz,
            w: qw,
        } = twist.rotational;
        Self {
            linear: [x, y, z],
            rotational: [qx, qy, qz, qw],
        }
    }
}

/// Triangle (`geometry::RealTriangle`), counter-clockwise when seen from its
/// front side.
#[wasm_bindgen(js_name = Triangle)]
#[derive(Debug, Clone, Copy)]
pub struct JsTriangle(RealTriangle);

#[wasm_bindgen(js_class = Triangle)]
impl JsTriangle {
    #[wasm_bindgen(constructor)]
    pub fn new(
        #[wasm_bindgen(unchecked_param_type = "Vec3")] v0: JsValue,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] v1: JsValue,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] v2: JsValue,
    ) -> Result<JsTriangle> {
        Ok(Self(RealTriangle {
            v0: vector(&v0, &|| "v0".to_owned())?,
            v1: vector(&v1, &|| "v1".to_owned())?,
            v2: vector(&v2, &|| "v2".to_owned())?,
        }))
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec3")]
    pub fn v0(&self) -> Array {
        numbers(&self.0.v0)
    }

    #[wasm_bindgen(setter)]
    pub fn set_v0(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] v0: JsValue,
    ) -> Result<()> {
        self.0.v0 = vector(&v0, &|| "v0".to_owned())?;
        Ok(())
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec3")]
    pub fn v1(&self) -> Array {
        numbers(&self.0.v1)
    }

    #[wasm_bindgen(setter)]
    pub fn set_v1(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] v1: JsValue,
    ) -> Result<()> {
        self.0.v1 = vector(&v1, &|| "v1".to_owned())?;
        Ok(())
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec3")]
    pub fn v2(&self) -> Array {
        numbers(&self.0.v2)
    }

    #[wasm_bindgen(setter)]
    pub fn set_v2(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] v2: JsValue,
    ) -> Result<()> {
        self.0.v2 = vector(&v2, &|| "v2".to_owned())?;
        Ok(())
    }
}

/// What the renderer needs to know about a camera (`camera::CameraState`).
/// `view_proj` is projection times view, column-major: `view_proj[column][row]`.
#[wasm_bindgen(js_name = CameraState)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JsCameraState {
    view_proj: [[f32; 4]; 4],
    position: [f32; 3],
    forward: [f32; 3],
}

#[wasm_bindgen(js_class = CameraState)]
impl JsCameraState {
    #[wasm_bindgen(constructor)]
    pub fn new(
        #[wasm_bindgen(unchecked_param_type = "Mat4")] view_proj: JsValue,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] position: JsValue,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] forward: JsValue,
    ) -> Result<JsCameraState> {
        Ok(Self {
            view_proj: self::view_proj(&view_proj, &|| "view_proj".to_owned())?,
            position: vector(&position, &|| "position".to_owned())?,
            forward: vector(&forward, &|| "forward".to_owned())?,
        })
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Mat4")]
    pub fn view_proj(&self) -> Array {
        self.view_proj
            .iter()
            .map(|column| numbers(column))
            .collect()
    }

    #[wasm_bindgen(setter)]
    pub fn set_view_proj(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Mat4")] view_proj: JsValue,
    ) -> Result<()> {
        self.view_proj = self::view_proj(&view_proj, &|| "view_proj".to_owned())?;
        Ok(())
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec3")]
    pub fn position(&self) -> Array {
        numbers(&self.position)
    }

    #[wasm_bindgen(setter)]
    pub fn set_position(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] position: JsValue,
    ) -> Result<()> {
        self.position = vector(&position, &|| "position".to_owned())?;
        Ok(())
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec3")]
    pub fn forward(&self) -> Array {
        numbers(&self.forward)
    }

    #[wasm_bindgen(setter)]
    pub fn set_forward(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] forward: JsValue,
    ) -> Result<()> {
        self.forward = vector(&forward, &|| "forward".to_owned())?;
        Ok(())
    }
}

impl From<CameraState> for JsCameraState {
    fn from(state: CameraState) -> Self {
        let p = state.position;
        let f = state.forward;
        Self {
            view_proj: state.view_proj.into(),
            position: [p.x, p.y, p.z],
            forward: [f.x, f.y, f.z],
        }
    }
}

impl From<JsCameraState> for CameraState {
    fn from(state: JsCameraState) -> Self {
        let [px, py, pz] = state.position;
        let [fx, fy, fz] = state.forward;
        Self {
            view_proj: state.view_proj.into(),
            position: cgmath::Point3::new(px, py, pz),
            forward: cgmath::Vector3::new(fx, fy, fz),
        }
    }
}
