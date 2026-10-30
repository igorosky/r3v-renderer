//! `renderer::WgpuAccelerated` (trait `Renderer`).

use pyo3::{buffer::PyBuffer, exceptions::PyValueError, prelude::*};

use crate::{
    bindings::checked::CheckedRenderer,
    renderer::{DEFAULT_BACKGROUND_COLOR, DEFAULT_COLOR_DEPTH_RANGE},
};

use super::{PyCameraState, PyTwist, Vec3, stl::PyStlMesh, tesselator::PyGeometryDelta};

/// `renderer::WgpuAcceleratedConfiguration`: the frame size in pixels, the
/// opaque sRGB background color (components between 0 and 1) and the
/// distance along the camera's forward axis over which the voxel colormap
/// runs once.
#[pyclass(
    module = "r3v_renderer",
    name = "WgpuRendererConfiguration",
    from_py_object,
    eq,
    get_all,
    set_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyWgpuRendererConfiguration {
    width: u32,
    height: u32,
    background_color: Vec3,
    color_depth_range: f32,
}

#[pymethods]
impl PyWgpuRendererConfiguration {
    #[new]
    #[pyo3(signature = (
        width,
        height,
        background_color = (DEFAULT_BACKGROUND_COLOR[0], DEFAULT_BACKGROUND_COLOR[1], DEFAULT_BACKGROUND_COLOR[2]),
        color_depth_range = DEFAULT_COLOR_DEPTH_RANGE,
    ))]
    fn new(width: u32, height: u32, background_color: Vec3, color_depth_range: f32) -> Self {
        Self {
            width,
            height,
            background_color,
            color_depth_range,
        }
    }
    fn __repr__(&self) -> String {
        format!(
            "WgpuRendererConfiguration(width={}, height={}, background_color={:?}, color_depth_range={:?})",
            self.width, self.height, self.background_color, self.color_depth_range
        )
    }
}

/// Renders voxel geometry and mesh instances on the GPU
/// (`renderer::WgpuAccelerated`). Configure it before rendering.
#[pyclass(module = "r3v_renderer", name = "WgpuRenderer")]
pub(super) struct PyWgpuRenderer {
    renderer: CheckedRenderer,
    // Rendered into without the GIL, then copied to the caller's buffer
    frame: Vec<u8>,
}

#[pymethods]
impl PyWgpuRenderer {
    /// `WgpuAccelerated::new`: blocks until a device of the default GPU
    /// adapter is ready. `RuntimeError` when there is no adapter or no device.
    #[new]
    fn new(py: Python<'_>) -> PyResult<Self> {
        Ok(Self {
            renderer: py.detach(CheckedRenderer::new)?,
            frame: Vec::new(),
        })
    }

    /// `Renderer::configure`: sets the frame size, the background color and
    /// the colormap depth range. `ValueError` unless both sides are between 1
    /// and 8192, the color components between 0 and 1 and the depth range
    /// positive.
    fn configure(&mut self, configuration: PyWgpuRendererConfiguration) -> PyResult<()> {
        let (r, g, b) = configuration.background_color;
        Ok(self.renderer.configure(
            configuration.width,
            configuration.height,
            [r, g, b],
            configuration.color_depth_range,
        )?)
    }

    /// `Renderer::update_geometry`: replaces the triangles of every chunk in
    /// `delta`, which can not be used afterwards.
    fn update_geometry(
        &mut self,
        py: Python<'_>,
        mut delta: PyRefMut<'_, PyGeometryDelta>,
    ) -> PyResult<()> {
        let chunks = delta.take()?;
        py.detach(|| self.renderer.update_geometry(chunks));
        Ok(())
    }

    /// `Renderer::upload_asset`: uploads a copy of `mesh` with `instances`
    /// instances and returns their ids. Instances start visible, with the
    /// identity transform. Ids count up from 0 over all uploads.
    fn upload_asset(
        &mut self,
        py: Python<'_>,
        mesh: PyRef<'_, PyStlMesh>,
        instances: usize,
    ) -> PyResult<Vec<u64>> {
        let mesh = mesh.0.clone();
        Ok(py.detach(|| self.renderer.upload_asset(mesh, instances as u64))?)
    }

    /// `Renderer::set_asset_transform`. `IndexError` for an id that was never
    /// returned by `upload_asset`.
    fn set_asset_transform(&mut self, instance_id: u64, transform: PyTwist) -> PyResult<()> {
        Ok(self
            .renderer
            .set_asset_transform(instance_id, transform.into())?)
    }

    /// `Renderer::set_asset_visibility`. `IndexError` for an id that was never
    /// returned by `upload_asset`.
    fn set_asset_visibility(&mut self, instance_id: u64, visible: bool) -> PyResult<()> {
        Ok(self.renderer.set_asset_visibility(instance_id, visible)?)
    }

    fn set_asset_color(&mut self, instance_id: u64, color: Vec3) -> PyResult<()> {
        let (r, g, b) = color;
        Ok(self.renderer.set_asset_color(instance_id, [r, g, b])?)
    }

    /// `Renderer::render`: renders a frame seen by `camera_state` and writes
    /// the previous one to `out` (frames are read back one call late). After
    /// `configure` changed the frame size, and on the first call, `out` is
    /// filled with zeros instead.
    ///
    /// Pixels are BGRA, 8 bit sRGB, rows top to bottom without padding.
    /// `out` is a writable, contiguous buffer of bytes with room for
    /// `width * height * 4` of them, e.g. a `bytearray`. Blocks until the GPU
    /// is done.
    fn render(
        &mut self,
        py: Python<'_>,
        camera_state: PyCameraState,
        out: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let frame_len = self.renderer.frame_len()?;
        let out = PyBuffer::<u8>::get(out)?;
        if out.readonly() || !out.is_c_contiguous() {
            return Err(PyValueError::new_err(
                "`out` must be a writable, contiguous buffer",
            ));
        }
        if out.item_count() < frame_len {
            return Err(PyValueError::new_err(format!(
                "a frame needs {frame_len} bytes, `out` has {}",
                out.item_count()
            )));
        }
        self.frame.resize(frame_len, 0);
        let camera_state = camera_state.into();
        py.detach(|| self.renderer.render(&camera_state, &mut self.frame))?;
        let cells = out
            .as_mut_slice(py)
            .expect("checked to be writable and contiguous");
        for (cell, &byte) in cells.iter().zip(&self.frame) {
            cell.set(byte);
        }
        Ok(())
    }
}
