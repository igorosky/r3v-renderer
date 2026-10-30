//! `renderer::WgpuAccelerated` (trait `Renderer`).

use crate::{bindings::checked::CheckedRenderer, renderer::TerrainColoring};

use super::{
    R3vCameraState, R3vStatus, R3vTwist, R3vVec3, arg, arg_mut, drop_handle, new_handle,
    slice_arg_mut, status, stl::R3vStlMesh, take_handle, tesselator::R3vGeometryDelta,
};

/// Renders voxel geometry and mesh instances on the GPU
/// (`renderer::WgpuAccelerated`).
pub struct R3vWgpuRenderer(CheckedRenderer);

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vTerrainColoring {
    pub height_coloring_enabled: bool,
    pub relative_to_robot: bool,
    pub robot_height: f32,
    pub height_offset: f32,
    pub height_knee: f32,
    pub height_gain: f32,
    pub height_bands: u32,
}

impl From<R3vTerrainColoring> for TerrainColoring {
    fn from(coloring: R3vTerrainColoring) -> Self {
        Self {
            height_coloring_enabled: coloring.height_coloring_enabled,
            relative_to_robot: coloring.relative_to_robot,
            robot_height: coloring.robot_height,
            height_offset: coloring.height_offset,
            height_knee: coloring.height_knee,
            height_gain: coloring.height_gain,
            height_bands: coloring.height_bands,
        }
    }
}

impl From<TerrainColoring> for R3vTerrainColoring {
    fn from(coloring: TerrainColoring) -> Self {
        Self {
            height_coloring_enabled: coloring.height_coloring_enabled,
            relative_to_robot: coloring.relative_to_robot,
            robot_height: coloring.robot_height,
            height_offset: coloring.height_offset,
            height_knee: coloring.height_knee,
            height_gain: coloring.height_gain,
            height_bands: coloring.height_bands,
        }
    }
}

/// `renderer::WgpuAcceleratedConfiguration`.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R3vWgpuRendererConfiguration {
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
    /// Opaque background color, sRGB: `x`, `y`, `z` are red, green and blue
    /// between 0 and 1.
    pub background_color: R3vVec3,
    /// Distance along the camera's forward axis over which the voxel colormap
    /// runs once. Has to be positive.
    pub color_depth_range: f32,
    pub terrain_coloring: R3vTerrainColoring,
}

#[unsafe(no_mangle)]
pub extern "C" fn r3v_terrain_coloring_default() -> R3vTerrainColoring {
    TerrainColoring::default().into()
}

/// `WgpuAccelerated::new`: blocks until a device of the default GPU adapter
/// is ready. NULL when there is no adapter or no device. Configure it before
/// rendering.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_create() -> *mut R3vWgpuRenderer {
    new_handle(|| Ok(R3vWgpuRenderer(CheckedRenderer::new()?)))
}

/// `Renderer::configure`: sets the frame size, the background color, the
/// colormap depth range and the terrain's coloring. Both sides have to be
/// between 1 and 8192. At an unchanged frame size only the settings change,
/// from the next frame on.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_configure(
    renderer: *mut R3vWgpuRenderer,
    configuration: R3vWgpuRendererConfiguration,
) -> R3vStatus {
    status(|| {
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        let R3vVec3 { x, y, z } = configuration.background_color;
        Ok(renderer.0.configure(
            configuration.width,
            configuration.height,
            [x, y, z],
            configuration.color_depth_range,
            configuration.terrain_coloring.into(),
        )?)
    })
}

/// `Renderer::update_geometry`: replaces the triangles of every chunk in
/// `delta`. Takes ownership of `delta`, which is freed on failure too.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_update_geometry(
    renderer: *mut R3vWgpuRenderer,
    delta: *mut R3vGeometryDelta,
) -> R3vStatus {
    status(|| {
        let delta = unsafe { take_handle(delta, "delta") }?;
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        renderer.0.update_geometry(delta.0);
        Ok(())
    })
}

/// `Renderer::upload_asset`: uploads a copy of `mesh` with `instances`
/// instances and writes their ids to `out_ids`, which has room for
/// `instances` ids (NULL when `instances` is 0). Instances start visible,
/// with the identity transform. Ids count up from 0 over all uploads.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_upload_asset(
    renderer: *mut R3vWgpuRenderer,
    mesh: *const R3vStlMesh,
    instances: usize,
    out_ids: *mut u64,
) -> R3vStatus {
    status(|| {
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        let mesh = unsafe { arg(mesh, "mesh") }?;
        // Before `out_ids` is looked at: its length is only that big
        renderer.0.check_upload(&mesh.0, instances as u64)?;
        let out_ids = unsafe { slice_arg_mut(out_ids, instances, "out_ids") }?;
        let ids = renderer.0.upload_asset(mesh.0.clone(), instances as u64)?;
        out_ids.copy_from_slice(&ids);
        Ok(())
    })
}

/// `Renderer::set_asset_transform`. `R3V_OUT_OF_RANGE` for an id that was
/// never returned by `r3v_wgpu_renderer_upload_asset`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_set_asset_transform(
    renderer: *mut R3vWgpuRenderer,
    instance_id: u64,
    transform: R3vTwist,
) -> R3vStatus {
    status(|| {
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        Ok(renderer
            .0
            .set_asset_transform(instance_id, transform.into())?)
    })
}

/// `Renderer::set_asset_visibility`. `R3V_OUT_OF_RANGE` for an id that was
/// never returned by `r3v_wgpu_renderer_upload_asset`.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_set_asset_visibility(
    renderer: *mut R3vWgpuRenderer,
    instance_id: u64,
    visible: bool,
) -> R3vStatus {
    status(|| {
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        Ok(renderer.0.set_asset_visibility(instance_id, visible)?)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_set_asset_color(
    renderer: *mut R3vWgpuRenderer,
    instance_id: u64,
    color: R3vVec3,
) -> R3vStatus {
    status(|| {
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        let R3vVec3 { x, y, z } = color;
        Ok(renderer.0.set_asset_color(instance_id, [x, y, z])?)
    })
}

/// `Renderer::render`: renders a frame seen by `camera_state` and writes the
/// previous one to `out` (frames are read back one call late). After
/// `r3v_wgpu_renderer_configure` changed the frame size, and on the first
/// call, `out` is filled with zeros instead.
///
/// Pixels are BGRA, 8 bit sRGB, rows top to bottom without padding: `out`
/// needs `width * height * 4` bytes, `out_len` is its size. Blocks until the
/// GPU is done.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_render(
    renderer: *mut R3vWgpuRenderer,
    camera_state: *const R3vCameraState,
    out: *mut u8,
    out_len: usize,
) -> R3vStatus {
    status(|| {
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        let camera_state = unsafe { arg(camera_state, "camera_state") }?;
        // Configured first, so a NULL `out` is not reported instead
        renderer.0.frame_len()?;
        let out = unsafe { slice_arg_mut(out, out_len, "out") }?;
        Ok(renderer.0.render(&camera_state.into(), out)?)
    })
}

/// Frees a renderer. NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_drop(renderer: *mut R3vWgpuRenderer) {
    unsafe { drop_handle(renderer) }
}
