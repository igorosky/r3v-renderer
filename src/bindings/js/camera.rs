//! `camera::OrbitCamera` (trait `Camera`).

use js_sys::Array;
use wasm_bindgen::prelude::*;

use crate::{Camera, bindings::checked, camera::OrbitCamera};

use super::{JsCameraState, Result, numbers, vector};

/// Camera circling `target` (`camera::OrbitCamera`). Angles are in radians,
/// `near` and `far` are the clip planes. `new_uniform_available` is set by
/// every change and never cleared by the library.
#[wasm_bindgen(js_name = OrbitCamera)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JsOrbitCamera {
    target: [f32; 3],
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub near: f32,
    pub far: f32,
    pub new_uniform_available: bool,
}

impl From<OrbitCamera> for JsOrbitCamera {
    fn from(camera: OrbitCamera) -> Self {
        let t = camera.target;
        Self {
            target: [t.x, t.y, t.z],
            distance: camera.distance,
            yaw: camera.yaw,
            pitch: camera.pitch,
            near: camera.near,
            far: camera.far,
            new_uniform_available: camera.new_uniform_available,
        }
    }
}

impl From<JsOrbitCamera> for OrbitCamera {
    fn from(camera: JsOrbitCamera) -> Self {
        let [x, y, z] = camera.target;
        Self {
            target: cgmath::Point3::new(x, y, z),
            distance: camera.distance,
            yaw: camera.yaw,
            pitch: camera.pitch,
            near: camera.near,
            far: camera.far,
            new_uniform_available: camera.new_uniform_available,
        }
    }
}

impl JsOrbitCamera {
    /// Runs a `&mut self` method of `OrbitCamera` on this camera.
    fn update(&mut self, f: impl FnOnce(&mut OrbitCamera)) {
        let mut camera = OrbitCamera::from(*self);
        f(&mut camera);
        *self = camera.into();
    }
}

#[wasm_bindgen(js_class = OrbitCamera)]
impl JsOrbitCamera {
    /// `OrbitCamera::new`: `target` is a `Vec3`, angles are in degrees,
    /// clip planes at 0.1 and 100.
    /// Without arguments `OrbitCamera::default`, looking at the origin from 5
    /// units along +Z.
    #[wasm_bindgen(constructor)]
    pub fn new(
        #[wasm_bindgen(unchecked_optional_param_type = "Vec3")] target: Option<JsValue>,
        // After `target`, wasm-bindgen wants every parameter marked like it
        #[wasm_bindgen(unchecked_optional_param_type = "number")] distance: Option<f32>,
        #[wasm_bindgen(unchecked_optional_param_type = "number")] yaw_deg: Option<f32>,
        #[wasm_bindgen(unchecked_optional_param_type = "number")] pitch_deg: Option<f32>,
    ) -> Result<JsOrbitCamera> {
        let [x, y, z] = match target {
            Some(target) => vector(&target, &|| "target".to_owned())?,
            None => [0.0; 3],
        };
        Ok(OrbitCamera::new(
            cgmath::Point3::new(x, y, z),
            distance.unwrap_or(5.0),
            yaw_deg.unwrap_or(0.0),
            pitch_deg.unwrap_or(0.0),
        )
        .into())
    }

    #[wasm_bindgen(getter, unchecked_return_type = "Vec3")]
    pub fn target(&self) -> Array {
        numbers(&self.target)
    }

    #[wasm_bindgen(setter)]
    pub fn set_target(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] target: JsValue,
    ) -> Result<()> {
        self.target = vector(&target, &|| "target".to_owned())?;
        Ok(())
    }

    /// `OrbitCamera::with_clip_planes`: a copy with these clip planes.
    pub fn with_clip_planes(&self, near: f32, far: f32) -> JsOrbitCamera {
        OrbitCamera::from(*self).with_clip_planes(near, far).into()
    }

    /// `OrbitCamera::forward`: unit vector from the eye towards `target`.
    #[wasm_bindgen(unchecked_return_type = "Vec3")]
    pub fn forward(&self) -> Array {
        let f = OrbitCamera::from(*self).forward();
        numbers(&[f.x, f.y, f.z])
    }

    /// `Camera::look_at`: angles in degrees. The distance is kept at 0.1 or
    /// more and the pitch short of the poles.
    pub fn look_at(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Vec3")] target: JsValue,
        distance: f32,
        yaw_deg: f32,
        pitch_deg: f32,
    ) -> Result<()> {
        let [x, y, z] = vector(&target, &|| "target".to_owned())?;
        let target = crate::geometry::Point { x, y, z };
        self.update(|camera| camera.look_at(target, distance, yaw_deg, pitch_deg));
        Ok(())
    }

    /// `Camera::rotate`: deltas in radians, unlike the degrees of `look_at`.
    /// The pitch is kept short of the poles.
    pub fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.update(|camera| camera.rotate(delta_yaw, delta_pitch));
    }

    /// `Camera::zoom`: changes the distance, which is kept at 0.1 or more.
    pub fn zoom(&mut self, delta_distance: f32) {
        self.update(|camera| camera.zoom(delta_distance));
    }

    /// `Camera::pan`: moves `target` along the screen's right and up axes.
    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        self.update(|camera| camera.pan(delta_x, delta_y));
    }

    /// `Camera::get_state` for a view of `aspect_ratio` (width / height).
    /// `RangeError` unless `aspect_ratio` > 0, `near` > 0 and `far` > `near`
    /// (all finite).
    pub fn get_state(&self, aspect_ratio: f32) -> Result<JsCameraState> {
        let camera = OrbitCamera::from(*self);
        Ok(checked::camera_state(&camera, aspect_ratio)?.into())
    }
}
