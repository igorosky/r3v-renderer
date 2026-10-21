//! `camera::OrbitCamera` (trait `Camera`).

use pyo3::prelude::*;

use crate::{Camera, bindings::checked, camera::OrbitCamera};

use super::{PyCameraState, Vec3};

/// Camera circling `target` (`camera::OrbitCamera`). Angles are in radians,
/// `near` and `far` are the clip planes. `new_uniform_available` is set by
/// every change and never cleared by the library.
#[pyclass(
    module = "r3v_renderer",
    name = "OrbitCamera",
    from_py_object,
    eq,
    get_all,
    set_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyOrbitCamera {
    target: Vec3,
    distance: f32,
    yaw: f32,
    pitch: f32,
    near: f32,
    far: f32,
    new_uniform_available: bool,
}

impl From<OrbitCamera> for PyOrbitCamera {
    fn from(camera: OrbitCamera) -> Self {
        let t = camera.target;
        Self {
            target: (t.x, t.y, t.z),
            distance: camera.distance,
            yaw: camera.yaw,
            pitch: camera.pitch,
            near: camera.near,
            far: camera.far,
            new_uniform_available: camera.new_uniform_available,
        }
    }
}

impl From<PyOrbitCamera> for OrbitCamera {
    fn from(camera: PyOrbitCamera) -> Self {
        let (x, y, z) = camera.target;
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

impl PyOrbitCamera {
    /// Runs a `&mut self` method of `OrbitCamera` on this camera.
    fn update(&mut self, f: impl FnOnce(&mut OrbitCamera)) {
        let mut camera = OrbitCamera::from(*self);
        f(&mut camera);
        *self = camera.into();
    }
}

#[pymethods]
impl PyOrbitCamera {
    /// `OrbitCamera::new`: angles in degrees, clip planes at 0.1 and 100.
    /// Without arguments `OrbitCamera::default`, looking at the origin from 5
    /// units along +Z.
    #[new]
    #[pyo3(signature = (target = (0.0, 0.0, 0.0), distance = 5.0, yaw_deg = 0.0, pitch_deg = 0.0))]
    fn new(target: Vec3, distance: f32, yaw_deg: f32, pitch_deg: f32) -> Self {
        let (x, y, z) = target;
        OrbitCamera::new(cgmath::Point3::new(x, y, z), distance, yaw_deg, pitch_deg).into()
    }

    /// `OrbitCamera::with_clip_planes`: a copy with these clip planes.
    fn with_clip_planes(&self, near: f32, far: f32) -> Self {
        OrbitCamera::from(*self).with_clip_planes(near, far).into()
    }

    /// `OrbitCamera::forward`: unit vector from the eye towards `target`.
    fn forward(&self) -> Vec3 {
        let f = OrbitCamera::from(*self).forward();
        (f.x, f.y, f.z)
    }

    /// `Camera::look_at`: angles in degrees. The distance is kept at 0.1 or
    /// more and the pitch short of the poles.
    fn look_at(&mut self, target: Vec3, distance: f32, yaw_deg: f32, pitch_deg: f32) {
        let (x, y, z) = target;
        let target = crate::geometry::Point { x, y, z };
        self.update(|camera| camera.look_at(target, distance, yaw_deg, pitch_deg));
    }

    /// `Camera::rotate`: deltas in radians, unlike the degrees of `look_at`.
    /// The pitch is kept short of the poles.
    fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.update(|camera| camera.rotate(delta_yaw, delta_pitch));
    }

    /// `Camera::zoom`: changes the distance, which is kept at 0.1 or more.
    fn zoom(&mut self, delta_distance: f32) {
        self.update(|camera| camera.zoom(delta_distance));
    }

    /// `Camera::pan`: moves `target` along the screen's right and up axes.
    fn pan(&mut self, delta_x: f32, delta_y: f32) {
        self.update(|camera| camera.pan(delta_x, delta_y));
    }

    /// `Camera::get_state` for a view of `aspect_ratio` (width / height).
    /// `ValueError` unless `aspect_ratio` > 0, `near` > 0 and `far` > `near`
    /// (all finite).
    fn get_state(&self, aspect_ratio: f32) -> PyResult<PyCameraState> {
        let camera = OrbitCamera::from(*self);
        Ok(checked::camera_state(&camera, aspect_ratio)?.into())
    }

    fn __repr__(&self) -> String {
        format!(
            "OrbitCamera(target={:?}, distance={:?}, yaw={:?}, pitch={:?}, near={:?}, far={:?})",
            self.target, self.distance, self.yaw, self.pitch, self.near, self.far
        )
    }
}
