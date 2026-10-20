//! `tesselator::GreedyMeshTesselator` (trait `Tesselator`) and its output,
//! `Vec<QuadChunk>`.

use std::ptr;

use crate::{bindings::checked::CheckedTesselator, geometry::QuadChunk};

use super::{
    Error, R3vPoint3I, R3vPointI, R3vQuad, R3vStatus, arg, arg_mut, drop_handle, new_handle,
    slice_arg, status,
};

/// Turns blocks of voxels into quads (`tesselator::GreedyMeshTesselator`).
pub struct R3vGreedyMeshTesselator(CheckedTesselator);

/// `tesselator::GreedyMeshTesselatorConfiguration`.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vGreedyMeshTesselatorConfiguration {
    /// Edge length of a voxel in world units.
    pub voxel_size: f32,
}

/// Quads of every chunk that changed (`Vec<QuadChunk>`). Owned by the caller
/// until it is passed to `r3v_wgpu_renderer_update_geometry` or dropped.
pub struct R3vGeometryDelta(pub(super) Vec<QuadChunk>);

/// All quads of one chunk of 16x16x16 voxels (`geometry::QuadChunk`). They
/// replace what the chunk had before; no quads means it is empty now.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct R3vQuadChunk {
    /// Origin of the chunk in voxels.
    pub index: R3vPoint3I,
    /// In world units. NULL when `quad_count` is 0, otherwise valid until the
    /// delta is dropped or passed on.
    pub quads: *const R3vQuad,
    pub quad_count: usize,
}

/// `GreedyMeshTesselator::new`. Configure it before use.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_greedy_mesh_tesselator_create() -> *mut R3vGreedyMeshTesselator {
    new_handle(|| Ok(R3vGreedyMeshTesselator(CheckedTesselator::new())))
}

/// `Tesselator::configure`. `voxel_size` has to be positive.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_greedy_mesh_tesselator_configure(
    tesselator: *mut R3vGreedyMeshTesselator,
    configuration: R3vGreedyMeshTesselatorConfiguration,
) -> R3vStatus {
    status(|| {
        let tesselator = unsafe { arg_mut(tesselator, "tesselator") }?;
        Ok(tesselator.0.configure(configuration.voxel_size)?)
    })
}

/// `Tesselator::update_input_geometry`: every block replaces the mask the
/// tesselator had for it. `blocks` may be NULL when `count` is 0.
///
/// `R3V_INVALID_ARGUMENT` when a block is not aligned to the 4x4x2 grid (`x`
/// and `y` multiples of 4, `z` of 2); none of the blocks is applied then.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_greedy_mesh_tesselator_update_input_geometry(
    tesselator: *mut R3vGreedyMeshTesselator,
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

/// `Tesselator::get_output_geometry_delta`: the chunks changed since the last
/// call, or NULL in `*out` when nothing changed.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_greedy_mesh_tesselator_get_output_geometry_delta(
    tesselator: *mut R3vGreedyMeshTesselator,
    out: *mut *mut R3vGeometryDelta,
) -> R3vStatus {
    status(|| {
        let out = unsafe { arg_mut(out, "out") }?;
        *out = ptr::null_mut();
        let tesselator = unsafe { arg_mut(tesselator, "tesselator") }?;
        if let Some(delta) = tesselator.0.get_output_geometry_delta()? {
            *out = Box::into_raw(Box::new(R3vGeometryDelta(delta)));
        }
        Ok(())
    })
}

/// Frees a tesselator. NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_greedy_mesh_tesselator_drop(tesselator: *mut R3vGreedyMeshTesselator) {
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
    out: *mut R3vQuadChunk,
) -> R3vStatus {
    status(|| {
        let delta = unsafe { arg(delta, "delta") }?;
        let out = unsafe { arg_mut(out, "out") }?;
        let QuadChunk { index, quads } = delta.0.get(chunk).ok_or_else(|| {
            Error::out_of_range(format!(
                "chunk {chunk} does not exist, the delta has {}",
                delta.0.len()
            ))
        })?;
        *out = R3vQuadChunk {
            index: (*index).into(),
            quads: if quads.is_empty() {
                ptr::null()
            } else {
                quads.as_ptr().cast()
            },
            quad_count: quads.len(),
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
