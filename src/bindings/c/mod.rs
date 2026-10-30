//! C ABI of the renderer. Every public component maps 1:1 onto a handle (or a
//! plain struct for plain Rust data) and `r3v_<component>_<method>` functions.
//!
//! No function unwinds into C: panics are caught and reported like any other
//! failure, as an [`R3vStatus`] (or NULL from functions returning a handle)
//! with the reason in [`r3v_renderer_last_error`].

// Raw pointers are the C calling convention here. What every function
// expects of them is documented in the generated header.
#![allow(clippy::not_unsafe_ptr_arg_deref)]

use std::{
    cell::RefCell,
    ffi::{CStr, CString, c_char},
    panic::{self, AssertUnwindSafe},
    ptr,
};

use super::checked::{self, ErrorKind};
use crate::geometry;

mod camera;
mod renderer;
mod robot;
mod stl;
mod tesselator;

/// Result of a function that can fail. Anything but `R3V_OK` comes with a
/// message from `r3v_renderer_last_error`.
#[repr(C)]
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum R3vStatus {
    R3V_OK = 0,
    /// A NULL or otherwise invalid argument. Nothing was changed.
    R3V_INVALID_ARGUMENT,
    /// The object has to be configured first.
    R3V_NOT_CONFIGURED,
    /// An index or instance id past the end.
    R3V_OUT_OF_RANGE,
    /// Nothing with the given name.
    R3V_NOT_FOUND,
    /// The operation failed for another reason, e.g. reading a file.
    R3V_FAILED,
    /// A bug or an unrecoverable GPU error inside the library. The object the
    /// function was called on should be dropped.
    R3V_PANIC,
}

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::default());
}

/// Message of the last failure on the calling thread, NUL terminated and
/// valid until the next failure on this thread. Empty before the first
/// failure, never NULL. Successful calls leave it as it is.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_renderer_last_error() -> *const c_char {
    LAST_ERROR.with(|last| last.borrow().as_ptr())
}

struct Error {
    status: R3vStatus,
    message: String,
}

impl Error {
    fn new(status: R3vStatus, message: impl Into<String>) -> Self {
        Self {
            status,
            message: message.into(),
        }
    }

    fn invalid_argument(message: impl Into<String>) -> Self {
        Self::new(R3vStatus::R3V_INVALID_ARGUMENT, message)
    }

    fn out_of_range(message: impl Into<String>) -> Self {
        Self::new(R3vStatus::R3V_OUT_OF_RANGE, message)
    }

    fn not_found(message: impl Into<String>) -> Self {
        Self::new(R3vStatus::R3V_NOT_FOUND, message)
    }

    fn failed(message: impl Into<String>) -> Self {
        Self::new(R3vStatus::R3V_FAILED, message)
    }

    /// Makes this the last error of the thread.
    fn report(self) -> R3vStatus {
        let message = CString::new(self.message.replace('\0', "\\0")).unwrap_or_default();
        LAST_ERROR.with(|last| *last.borrow_mut() = message);
        self.status
    }
}

impl From<checked::Error> for Error {
    fn from(error: checked::Error) -> Self {
        let status = match error.kind {
            ErrorKind::InvalidArgument => R3vStatus::R3V_INVALID_ARGUMENT,
            ErrorKind::NotConfigured => R3vStatus::R3V_NOT_CONFIGURED,
            ErrorKind::OutOfRange => R3vStatus::R3V_OUT_OF_RANGE,
            ErrorKind::Failed => R3vStatus::R3V_FAILED,
        };
        Self::new(status, error.message)
    }
}

type Result<T> = std::result::Result<T, Error>;

fn catch<T>(f: impl FnOnce() -> Result<T>) -> Result<T> {
    panic::catch_unwind(AssertUnwindSafe(f)).unwrap_or_else(|payload| {
        let message = payload
            .downcast_ref::<&str>()
            .map(|message| message.to_string())
            .or_else(|| payload.downcast_ref::<String>().cloned())
            .unwrap_or_else(|| "unknown panic payload".to_owned());
        Err(Error::new(
            R3vStatus::R3V_PANIC,
            format!("panic: {message}"),
        ))
    })
}

/// Body of a function returning a status.
fn status(f: impl FnOnce() -> Result<()>) -> R3vStatus {
    match catch(f) {
        Ok(()) => R3vStatus::R3V_OK,
        Err(error) => error.report(),
    }
}

/// Body of a function returning a new handle, NULL on failure.
fn new_handle<T>(f: impl FnOnce() -> Result<T>) -> *mut T {
    match catch(f) {
        Ok(value) => Box::into_raw(Box::new(value)),
        Err(error) => {
            error.report();
            ptr::null_mut()
        }
    }
}

/// Frees a handle made by [`new_handle`], NULL is ignored.
///
/// # Safety
/// A non NULL `handle` comes from [`new_handle`] and was not freed yet.
unsafe fn drop_handle<T>(handle: *mut T) {
    if !handle.is_null() {
        let dropped = catch(|| {
            drop(unsafe { Box::from_raw(handle) });
            Ok(())
        });
        if let Err(error) = dropped {
            error.report();
        }
    }
}

/// Takes back ownership of a handle made by [`new_handle`].
///
/// # Safety
/// As for [`drop_handle`].
unsafe fn take_handle<T>(handle: *mut T, name: &str) -> Result<Box<T>> {
    if handle.is_null() {
        return Err(Error::invalid_argument(format!("`{name}` is NULL")));
    }
    Ok(unsafe { Box::from_raw(handle) })
}

/// # Safety
/// A non NULL `pointer` is valid for reads during `'a`.
unsafe fn arg<'a, T>(pointer: *const T, name: &str) -> Result<&'a T> {
    unsafe { pointer.as_ref() }.ok_or_else(|| Error::invalid_argument(format!("`{name}` is NULL")))
}

/// # Safety
/// A non NULL `pointer` is valid for reads and writes during `'a`.
unsafe fn arg_mut<'a, T>(pointer: *mut T, name: &str) -> Result<&'a mut T> {
    unsafe { pointer.as_mut() }.ok_or_else(|| Error::invalid_argument(format!("`{name}` is NULL")))
}

/// `len` elements at `pointer`, which may be NULL when `len` is 0.
///
/// # Safety
/// A non NULL `pointer` is valid for reads of `len` elements during `'a`.
unsafe fn slice_arg<'a, T>(pointer: *const T, len: usize, name: &str) -> Result<&'a [T]> {
    if len == 0 {
        return Ok(&[]);
    }
    if pointer.is_null() {
        return Err(Error::invalid_argument(format!(
            "`{name}` is NULL, but its length is {len}"
        )));
    }
    Ok(unsafe { std::slice::from_raw_parts(pointer, len) })
}

/// # Safety
/// A non NULL `pointer` is valid for reads and writes of `len` elements
/// during `'a`.
unsafe fn slice_arg_mut<'a, T>(pointer: *mut T, len: usize, name: &str) -> Result<&'a mut [T]> {
    if len == 0 {
        return Ok(&mut []);
    }
    if pointer.is_null() {
        return Err(Error::invalid_argument(format!(
            "`{name}` is NULL, but its length is {len}"
        )));
    }
    Ok(unsafe { std::slice::from_raw_parts_mut(pointer, len) })
}

/// # Safety
/// A non NULL `pointer` is a NUL terminated string valid during `'a`.
unsafe fn str_arg<'a>(pointer: *const c_char, name: &str) -> Result<&'a str> {
    unsafe { arg(pointer, name) }?;
    unsafe { CStr::from_ptr(pointer) }
        .to_str()
        .map_err(|_| Error::invalid_argument(format!("`{name}` is not valid UTF-8")))
}

/// 3D vector, also used for points.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct R3vVec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl R3vVec3 {
    const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
}

/// Rotation quaternion.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vQuat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

/// Rigid transform (`geometry::RealTwist`): rotation by `rotational`, then
/// translation by `linear`.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vTwist {
    pub linear: R3vVec3,
    pub rotational: R3vQuat,
}

/// Integer point, e.g. the origin of a chunk in voxels.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct R3vPoint3I {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// One 4x4x2 block of voxels (`geometry::PointI`): `x`, `y`, `z` are the
/// voxel origin of the block, `i` its occupancy mask with bit
/// `x + 4 * y + 16 * z` for the voxel at (`x`, `y`, `z`) inside the block.
/// Same layout as `PointI` of r3v-voxelizer, whose compress mode output can
/// be passed on as it is.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct R3vPointI {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub i: u32,
}

/// Words in an `R3vVoxelGroup`, one per 4x4x4 page.
// A literal, cbindgen does not follow paths to other modules
pub const R3V_VOXEL_GROUP_PAGES: usize = 4096;
const _: () = assert!(R3V_VOXEL_GROUP_PAGES == geometry::VOXEL_GROUP_PAGES);

/// Constructed voxels of a cube of 64x64x64 voxels (`geometry::VoxelGroup`).
/// Same layout as `VoxelGroup` of r3v-voxelizer, whose
/// `get_all_points_compressed` output can be passed on as it is.
///
/// `point` is the key of the group: voxel coordinates shifted right by 6,
/// `point.i` is ignored. `values[k]` is the occupancy of the 4x4x4 page whose
/// offset in the group, bits of x, y and z interleaved from the lowest one, is
/// `k`. Bit `x + 4 * y + 16 * z` of it is set when voxel (x, y, z) of the page
/// is constructed.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct R3vVoxelGroup {
    pub point: R3vPointI,
    pub values: [u64; R3V_VOXEL_GROUP_PAGES],
}

/// Triangle (`geometry::RealTriangle`), counter-clockwise when seen from
/// its front side.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vTriangle {
    pub v0: [f32; 3],
    pub v1: [f32; 3],
    pub v2: [f32; 3],
}

/// What the renderer needs to know about a camera (`camera::CameraState`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vCameraState {
    /// Projection times view, column-major: `view_proj[column][row]`.
    pub view_proj: [[f32; 4]; 4],
    /// Eye position.
    pub position: R3vVec3,
    /// Unit view direction.
    pub forward: R3vVec3,
}

// Same size and alignment as the core types (all `repr(C)` with the same
// fields), so slices of them are handed out without copying.
const _: () = {
    use std::mem::{align_of, size_of};
    assert!(size_of::<R3vTriangle>() == size_of::<geometry::RealTriangle>());
    assert!(align_of::<R3vTriangle>() == align_of::<geometry::RealTriangle>());
};

impl From<geometry::Vector3D<f32>> for R3vVec3 {
    fn from(v: geometry::Vector3D<f32>) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<R3vVec3> for geometry::Vector3D<f32> {
    fn from(v: R3vVec3) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<geometry::Point<f32>> for R3vVec3 {
    fn from(p: geometry::Point<f32>) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
        }
    }
}

impl From<R3vVec3> for geometry::Point<f32> {
    fn from(v: R3vVec3) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<R3vTwist> for geometry::RealTwist {
    fn from(twist: R3vTwist) -> Self {
        let R3vQuat { x, y, z, w } = twist.rotational;
        Self {
            linear: twist.linear.into(),
            rotational: geometry::Quaternion { x, y, z, w },
        }
    }
}

impl From<geometry::RealTwist> for R3vTwist {
    fn from(twist: geometry::RealTwist) -> Self {
        let geometry::Quaternion { x, y, z, w } = twist.rotational;
        Self {
            linear: twist.linear.into(),
            rotational: R3vQuat { x, y, z, w },
        }
    }
}

impl From<R3vTriangle> for geometry::RealTriangle {
    fn from(triangle: R3vTriangle) -> Self {
        let R3vTriangle { v0, v1, v2 } = triangle;
        Self { v0, v1, v2 }
    }
}

impl From<R3vPoint3I> for geometry::DiscretePoint {
    fn from(p: R3vPoint3I) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
        }
    }
}

impl From<geometry::DiscretePoint> for R3vPoint3I {
    fn from(p: geometry::DiscretePoint) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
        }
    }
}

impl From<R3vPointI> for geometry::PointI {
    fn from(p: R3vPointI) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
            i: p.i,
        }
    }
}

impl From<&R3vCameraState> for crate::camera::CameraState {
    fn from(state: &R3vCameraState) -> Self {
        let R3vVec3 { x, y, z } = state.position;
        let forward = state.forward;
        Self {
            view_proj: state.view_proj.into(),
            position: cgmath::Point3::new(x, y, z),
            forward: cgmath::Vector3::new(forward.x, forward.y, forward.z),
        }
    }
}

impl From<crate::camera::CameraState> for R3vCameraState {
    fn from(state: crate::camera::CameraState) -> Self {
        let position = state.position;
        let forward = state.forward;
        Self {
            view_proj: state.view_proj.into(),
            position: R3vVec3 {
                x: position.x,
                y: position.y,
                z: position.z,
            },
            forward: R3vVec3 {
                x: forward.x,
                y: forward.y,
                z: forward.z,
            },
        }
    }
}

// Handles may be moved to another thread between calls.
const _: () = {
    const fn assert_send<T: Send>() {}
    assert_send::<camera::R3vOrbitCamera>();
    assert_send::<renderer::R3vWgpuRenderer>();
    assert_send::<robot::R3vUrdfDescription>();
    assert_send::<robot::R3vUrdfRobot>();
    assert_send::<crate::robot::RobotTransforms>();
    assert_send::<stl::R3vStlMesh>();
    assert_send::<tesselator::R3vCpuTesselator>();
    assert_send::<tesselator::R3vGeometryDelta>();
};

#[cfg(test)]
mod tests {
    use super::*;

    fn last_error() -> String {
        unsafe { CStr::from_ptr(r3v_renderer_last_error()) }
            .to_string_lossy()
            .into_owned()
    }

    #[test]
    fn panics_become_a_status() {
        assert_eq!(status(|| panic!("boom")), R3vStatus::R3V_PANIC);
        assert_eq!(last_error(), "panic: boom");
        let handle = new_handle::<u32>(|| panic!("{} {}", "formatted", "boom"));
        assert!(handle.is_null());
        assert_eq!(last_error(), "panic: formatted boom");
    }

    #[test]
    fn errors_keep_their_status_and_message() {
        assert_eq!(
            status(|| Err(Error::not_found("no such thing"))),
            R3vStatus::R3V_NOT_FOUND
        );
        assert_eq!(last_error(), "no such thing");
        assert_eq!(status(|| Ok(())), R3vStatus::R3V_OK);
        assert_eq!(last_error(), "no such thing");
    }

    #[test]
    fn nul_bytes_in_messages_are_escaped() {
        status(|| Err(Error::failed("a\0b")));
        assert_eq!(last_error(), "a\\0b");
    }
}
