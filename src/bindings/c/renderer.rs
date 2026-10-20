//! `renderer::WgpuAccelerated` (trait `Renderer`).

use crate::{
    Renderer,
    renderer::{WgpuAccelerated, WgpuAcceleratedConfiguration},
};

use super::{
    Error, R3vCameraState, R3vStatus, R3vTwist, arg, arg_mut, drop_handle, new_handle,
    slice_arg_mut, status, stl::R3vStlMesh, take_handle, tesselator::R3vGeometryDelta,
};

/// Bytes per pixel of a rendered frame.
const PIXEL_SIZE: usize = 4;

/// Renders voxel geometry and mesh instances on the GPU
/// (`renderer::WgpuAccelerated`).
pub struct R3vWgpuRenderer {
    renderer: WgpuAccelerated,
    // Tracked here to catch what the renderer would panic on
    configuration: Option<WgpuAcceleratedConfiguration>,
    instance_count: u64,
}

impl R3vWgpuRenderer {
    fn check_instance(&self, instance_id: u64) -> super::Result<()> {
        if instance_id < self.instance_count {
            Ok(())
        } else {
            Err(Error::out_of_range(format!(
                "instance {instance_id} does not exist, {} were uploaded",
                self.instance_count
            )))
        }
    }
}

/// `renderer::WgpuAcceleratedConfiguration`.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct R3vWgpuRendererConfiguration {
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
}

/// `WgpuAccelerated::new`: blocks until a device of the default GPU adapter
/// is ready. NULL when there is no adapter or no device. Configure it before
/// rendering.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_create() -> *mut R3vWgpuRenderer {
    new_handle(|| {
        let renderer = WgpuAccelerated::new()
            .ok_or_else(|| Error::failed("no GPU adapter or device is available"))?;
        Ok(R3vWgpuRenderer {
            renderer,
            configuration: None,
            instance_count: 0,
        })
    })
}

/// `Renderer::configure`: sets the frame size. Both sides have to be
/// between 1 and 8192.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_configure(
    renderer: *mut R3vWgpuRenderer,
    configuration: R3vWgpuRendererConfiguration,
) -> R3vStatus {
    status(|| {
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        let R3vWgpuRendererConfiguration { width, height } = configuration;
        // The renderer asks for a device with the default limits
        let max = wgpu::Limits::default().max_texture_dimension_2d;
        if !(1..=max).contains(&width) || !(1..=max).contains(&height) {
            return Err(Error::invalid_argument(format!(
                "frame size must be between 1x1 and {max}x{max}, got {width}x{height}"
            )));
        }
        let configuration = WgpuAcceleratedConfiguration { width, height };
        renderer.renderer.configure(configuration);
        renderer.configuration = Some(configuration);
        Ok(())
    })
}

/// `Renderer::update_geometry`: replaces the quads of every chunk in `delta`.
/// Takes ownership of `delta`, which is freed on failure too.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_update_geometry(
    renderer: *mut R3vWgpuRenderer,
    delta: *mut R3vGeometryDelta,
) -> R3vStatus {
    status(|| {
        let delta = unsafe { take_handle(delta, "delta") }?;
        let renderer = unsafe { arg_mut(renderer, "renderer") }?;
        renderer.renderer.update_geometry(delta.0);
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
        // The renderer keeps vertex and instance offsets in u32. Checked
        // before `out_ids` is looked at, its length is only that big.
        let instance_count = renderer
            .instance_count
            .checked_add(instances as u64)
            .filter(|&count| count <= u64::from(u32::MAX))
            .ok_or_else(|| {
                Error::invalid_argument(format!(
                    "{instances} more instances do not fit next to the {} uploaded",
                    renderer.instance_count
                ))
            })?;
        if mesh.0.triangles.len() > (u32::MAX / 3) as usize {
            return Err(Error::invalid_argument(format!(
                "mesh has {} triangles, too many for one asset",
                mesh.0.triangles.len()
            )));
        }
        let out_ids = unsafe { slice_arg_mut(out_ids, instances, "out_ids") }?;
        let ids = renderer.renderer.upload_asset(mesh.0.clone(), instances);
        out_ids.copy_from_slice(&ids);
        renderer.instance_count = instance_count;
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
        renderer.check_instance(instance_id)?;
        renderer
            .renderer
            .set_asset_transform(instance_id, transform.into());
        Ok(())
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
        renderer.check_instance(instance_id)?;
        renderer.renderer.set_asset_visibility(instance_id, visible);
        Ok(())
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
        let WgpuAcceleratedConfiguration { width, height } = renderer
            .configuration
            .ok_or_else(|| Error::not_configured("configure the renderer before rendering"))?;
        let frame_len = width as usize * height as usize * PIXEL_SIZE;
        if out_len < frame_len {
            return Err(Error::invalid_argument(format!(
                "a {width}x{height} frame needs {frame_len} bytes, `out_len` is {out_len}"
            )));
        }
        let out = unsafe { slice_arg_mut(out, out_len, "out") }?;
        renderer.renderer.render(&camera_state.into(), out);
        Ok(())
    })
}

/// Frees a renderer. NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_wgpu_renderer_drop(renderer: *mut R3vWgpuRenderer) {
    unsafe { drop_handle(renderer) }
}
