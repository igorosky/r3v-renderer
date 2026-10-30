//! `tesselator::CpuTesselator` (trait `Tesselator`) and its output,
//! `Vec<TriangleChunk>`.

use std::ptr;

use crate::{bindings::checked::CheckedTesselator, camera::CameraState, geometry::TriangleChunk};

use super::{
    Error, R3vCameraState, R3vPoint3I, R3vPointI, R3vStatus, R3vTriangle, R3vVoxelGroup, arg,
    arg_mut, drop_handle, new_handle, slice_arg, status,
};

/// Turns blocks of voxels into triangles (`tesselator::CpuTesselator`).
pub struct R3vCpuTesselator(CheckedTesselator);

/// `tesselator::CpuTesselatorConfiguration`.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vCpuTesselatorConfiguration {
    /// Edge length of a voxel in world units.
    pub voxel_size: f32,
    /// Smooths the surface with marching cubes instead of meshing the voxel
    /// faces.
    pub marching_cubes_enabled: bool,
}

/// Triangles of every chunk that changed (`Vec<TriangleChunk>`). Owned by the
/// caller until it is passed to `r3v_wgpu_renderer_update_geometry` or
/// dropped.
pub struct R3vGeometryDelta(pub(super) Vec<TriangleChunk>);

/// All triangles of one chunk of 16x16x16 voxels (`geometry::TriangleChunk`).
/// They replace what the chunk had before; no triangles means it is empty
/// now.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct R3vTriangleChunk {
    /// Origin of the chunk in voxels.
    pub index: R3vPoint3I,
    /// In world units. NULL when `triangle_count` is 0, otherwise valid until
    /// the delta is dropped or passed on.
    pub triangles: *const R3vTriangle,
    pub triangle_count: usize,
}

/// `CpuTesselator::new`, with marching cubes disabled. Configure it before
/// use.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_cpu_tesselator_create() -> *mut R3vCpuTesselator {
    new_handle(|| Ok(R3vCpuTesselator(CheckedTesselator::new())))
}

/// `Tesselator::configure`. `voxel_size` has to be positive.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_cpu_tesselator_configure(
    tesselator: *mut R3vCpuTesselator,
    configuration: R3vCpuTesselatorConfiguration,
) -> R3vStatus {
    status(|| {
        let tesselator = unsafe { arg_mut(tesselator, "tesselator") }?;
        Ok(tesselator.0.configure(
            configuration.voxel_size,
            configuration.marching_cubes_enabled,
        )?)
    })
}

/// `CpuTesselator::marching_cubes_enabled`, false for NULL.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_cpu_tesselator_marching_cubes_enabled(
    tesselator: *const R3vCpuTesselator,
) -> bool {
    unsafe { tesselator.as_ref() }.is_some_and(|tesselator| tesselator.0.marching_cubes_enabled())
}

/// `CpuTesselator::set_marching_cubes_enabled`: a change re-tesselates every
/// chunk with voxels, and the chunks next to them, on the next
/// `r3v_cpu_tesselator_get_output_geometry_delta`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_cpu_tesselator_set_marching_cubes_enabled(
    tesselator: *mut R3vCpuTesselator,
    enabled: bool,
) -> R3vStatus {
    status(|| {
        let tesselator = unsafe { arg_mut(tesselator, "tesselator") }?;
        tesselator.0.set_marching_cubes_enabled(enabled);
        Ok(())
    })
}

/// `Tesselator::update_input_geometry`: every block replaces the mask the
/// tesselator had for it. `blocks` may be NULL when `count` is 0.
///
/// `R3V_INVALID_ARGUMENT` when a block is not aligned to the 4x4x2 grid (`x`
/// and `y` multiples of 4, `z` of 2); none of the blocks is applied then.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_cpu_tesselator_update_input_geometry(
    tesselator: *mut R3vCpuTesselator,
    blocks: *const R3vPointI,
    count: usize,
) -> R3vStatus {
    status(|| {
        let tesselator = unsafe { arg_mut(tesselator, "tesselator") }?;
        let blocks = unsafe { slice_arg(blocks, count, "blocks") }?;
        let blocks = blocks.iter().map(|&block| block.into()).collect();
        Ok(tesselator.0.update_input_geometry(blocks)?)
    })
}

/// `CpuTesselator::replace_input_geometry`: drops every voxel the tesselator
/// holds and keeps only the ones of `groups`, e.g. what r3v-voxelizer's
/// `get_all_points_compressed` returns. Groups sharing a key add up. Only the
/// chunks whose voxels changed come out of the next
/// `r3v_cpu_tesselator_get_output_geometry_delta`, the ones left without
/// voxels with no triangles. `groups` may be NULL when `count` is 0, which
/// clears everything.
///
/// `R3V_INVALID_ARGUMENT` when a group key coordinate is outside
/// [-2^25, 2^25) (its voxels would not fit `int32_t`); none of the groups is
/// applied then.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_cpu_tesselator_replace_input_geometry(
    tesselator: *mut R3vCpuTesselator,
    groups: *const R3vVoxelGroup,
    count: usize,
) -> R3vStatus {
    status(|| {
        let tesselator = unsafe { arg_mut(tesselator, "tesselator") }?;
        let groups = unsafe { slice_arg(groups, count, "groups") }?;
        Ok(tesselator.0.replace_input_geometry(
            groups
                .iter()
                .map(|group| (group.point.into(), &group.values)),
        )?)
    })
}

/// `Tesselator::get_output_geometry_delta`: the chunks changed since the last
/// call, or NULL in `*out` when nothing changed. With a `camera_state`, only
/// the chunks in its view; the others stay changed for a later call. NULL
/// `camera_state` returns every changed chunk.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_cpu_tesselator_get_output_geometry_delta(
    tesselator: *mut R3vCpuTesselator,
    camera_state: *const R3vCameraState,
    out: *mut *mut R3vGeometryDelta,
) -> R3vStatus {
    status(|| {
        let out = unsafe { arg_mut(out, "out") }?;
        *out = ptr::null_mut();
        let tesselator = unsafe { arg_mut(tesselator, "tesselator") }?;
        let camera_state = unsafe { camera_state.as_ref() }.map(CameraState::from);
        if let Some(delta) = tesselator
            .0
            .get_output_geometry_delta(camera_state.as_ref())?
        {
            *out = Box::into_raw(Box::new(R3vGeometryDelta(delta)));
        }
        Ok(())
    })
}

/// Frees a tesselator. NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_cpu_tesselator_drop(tesselator: *mut R3vCpuTesselator) {
    unsafe { drop_handle(tesselator) }
}

/// Number of chunks in the delta, 0 for NULL.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_geometry_delta_chunk_count(delta: *const R3vGeometryDelta) -> usize {
    unsafe { delta.as_ref() }.map_or(0, |delta| delta.0.len())
}

/// Chunk number `chunk` of the delta.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_geometry_delta_chunk(
    delta: *const R3vGeometryDelta,
    chunk: usize,
    out: *mut R3vTriangleChunk,
) -> R3vStatus {
    status(|| {
        let delta = unsafe { arg(delta, "delta") }?;
        let out = unsafe { arg_mut(out, "out") }?;
        let TriangleChunk { index, triangles } = delta.0.get(chunk).ok_or_else(|| {
            Error::out_of_range(format!(
                "chunk {chunk} does not exist, the delta has {}",
                delta.0.len()
            ))
        })?;
        *out = R3vTriangleChunk {
            index: (*index).into(),
            triangles: if triangles.is_empty() {
                ptr::null()
            } else {
                triangles.as_ptr().cast()
            },
            triangle_count: triangles.len(),
        };
        Ok(())
    })
}

/// Frees a delta that was not passed to `r3v_wgpu_renderer_update_geometry`.
/// NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_geometry_delta_drop(delta: *mut R3vGeometryDelta) {
    unsafe { drop_handle(delta) }
}
