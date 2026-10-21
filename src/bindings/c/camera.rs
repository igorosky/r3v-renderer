//! `camera::OrbitCamera` (trait `Camera`). It is plain data, so C gets the
//! struct itself: methods taking `&self` take it by value, methods taking
//! `&mut self` a pointer.

use crate::{Camera, bindings::checked, camera::OrbitCamera};

use super::{R3vCameraState, R3vStatus, R3vVec3, arg_mut, status};

/// Camera circling `target` (`camera::OrbitCamera`). Change it through the
/// `r3v_orbit_camera_*` functions or by writing the fields directly.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vOrbitCamera {
    pub target: R3vVec3,
    pub distance: f32,
    /// Radians.
    pub yaw: f32,
    /// Radians.
    pub pitch: f32,
    /// Near clip plane distance.
    pub near: f32,
    /// Far clip plane distance.
    pub far: f32,
    /// Set by every change of the camera, never cleared by the library.
    pub new_uniform_available: bool,
}

impl From<OrbitCamera> for R3vOrbitCamera {
    fn from(camera: OrbitCamera) -> Self {
        let target = camera.target;
        Self {
            target: R3vVec3 {
                x: target.x,
                y: target.y,
                z: target.z,
            },
            distance: camera.distance,
            yaw: camera.yaw,
            pitch: camera.pitch,
            near: camera.near,
            far: camera.far,
            new_uniform_available: camera.new_uniform_available,
        }
    }
}

impl From<R3vOrbitCamera> for OrbitCamera {
    fn from(camera: R3vOrbitCamera) -> Self {
        let R3vVec3 { x, y, z } = camera.target;
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

/// Runs a `&mut self` method on the camera behind `camera`.
fn update(camera: *mut R3vOrbitCamera, f: impl FnOnce(&mut OrbitCamera)) -> R3vStatus {
    status(|| {
        let camera = unsafe { arg_mut(camera, "camera") }?;
        let mut orbit = OrbitCamera::from(*camera);
        f(&mut orbit);
        *camera = orbit.into();
        Ok(())
    })
}

/// `OrbitCamera::new`: angles in degrees, clip planes at 0.1 and 100.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_new(
    target: R3vVec3,
    distance: f32,
    yaw_deg: f32,
    pitch_deg: f32,
) -> R3vOrbitCamera {
    let R3vVec3 { x, y, z } = target;
    OrbitCamera::new(cgmath::Point3::new(x, y, z), distance, yaw_deg, pitch_deg).into()
}

/// `OrbitCamera::default`: looks at the origin from 5 units along +Z.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_default() -> R3vOrbitCamera {
    OrbitCamera::default().into()
}

/// `OrbitCamera::with_clip_planes`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_with_clip_planes(
    camera: R3vOrbitCamera,
    near: f32,
    far: f32,
) -> R3vOrbitCamera {
    OrbitCamera::from(camera).with_clip_planes(near, far).into()
}

/// `OrbitCamera::forward`: unit vector from the eye towards `target`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_forward(camera: R3vOrbitCamera) -> R3vVec3 {
    let forward = OrbitCamera::from(camera).forward();
    R3vVec3 {
        x: forward.x,
        y: forward.y,
        z: forward.z,
    }
}

/// `Camera::look_at`: angles in degrees. The distance is kept at 0.1 or more
/// and the pitch short of the poles.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_look_at(
    camera: *mut R3vOrbitCamera,
    target: R3vVec3,
    distance: f32,
    yaw_deg: f32,
    pitch_deg: f32,
) -> R3vStatus {
    update(camera, |camera| {
        camera.look_at(target.into(), distance, yaw_deg, pitch_deg)
    })
}

/// `Camera::rotate`: deltas in radians, unlike the degrees of `look_at`. The
/// pitch is kept short of the poles.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_rotate(
    camera: *mut R3vOrbitCamera,
    delta_yaw: f32,
    delta_pitch: f32,
) -> R3vStatus {
    update(camera, |camera| camera.rotate(delta_yaw, delta_pitch))
}

/// `Camera::zoom`: changes the distance, which is kept at 0.1 or more.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_zoom(
    camera: *mut R3vOrbitCamera,
    delta_distance: f32,
) -> R3vStatus {
    update(camera, |camera| camera.zoom(delta_distance))
}

/// `Camera::pan`: moves `target` along the screen's right and up axes.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_pan(
    camera: *mut R3vOrbitCamera,
    delta_x: f32,
    delta_y: f32,
) -> R3vStatus {
    update(camera, |camera| camera.pan(delta_x, delta_y))
}

/// `Camera::get_state` for a view of `aspect_ratio` (width / height). Fails
/// with `R3V_INVALID_ARGUMENT` unless `aspect_ratio` > 0, `near` > 0 and
/// `far` > `near` (all finite).
#[unsafe(no_mangle)]
pub extern "C" fn r3v_orbit_camera_get_state(
    camera: R3vOrbitCamera,
    aspect_ratio: f32,
    out: *mut R3vCameraState,
) -> R3vStatus {
    status(|| {
        let out = unsafe { arg_mut(out, "out") }?;
        *out = checked::camera_state(&camera.into(), aspect_ratio)?.into();
        Ok(())
    })
}
