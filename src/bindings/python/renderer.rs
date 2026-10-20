//! `renderer::WgpuAccelerated` (trait `Renderer`).

use pyo3::{buffer::PyBuffer, exceptions::PyValueError, prelude::*};

use crate::bindings::checked::CheckedRenderer;

use super::{PyCameraState, PyTwist, stl::PyStlMesh, tesselator::PyGeometryDelta};

/// `renderer::WgpuAcceleratedConfiguration`: the frame size in pixels.
#[pyclass(
    module = "r3v_renderer",
    name = "WgpuRendererConfiguration",
    from_py_object,
    eq,
    get_all,
    set_all
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct PyWgpuRendererConfiguration {
    width: u32,
    height: u32,
}

#[pymethods]
impl PyWgpuRendererConfiguration {
    #[new]
    fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }
    fn __repr__(&self) -> String {
        format!(
            "WgpuRendererConfiguration(width={}, height={})",
            self.width, self.height
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

    /// `Renderer::configure`: sets the frame size. `ValueError` unless both
    /// sides are between 1 and 8192.
    fn configure(&mut self, configuration: PyWgpuRendererConfiguration) -> PyResult<()> {
        Ok(self
            .renderer
            .configure(configuration.width, configuration.height)?)
    }

    /// `Renderer::update_geometry`: replaces the quads of every chunk in
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
