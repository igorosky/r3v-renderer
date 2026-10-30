//! `renderer::WgpuAccelerated` (trait `Renderer`).

use pyo3::{buffer::PyBuffer, exceptions::PyValueError, prelude::*};

use crate::{
    bindings::checked::CheckedRenderer,
    renderer::{
        DEFAULT_BACKGROUND_COLOR, DEFAULT_COLOR_DEPTH_RANGE, DEFAULT_HEIGHT_GAIN,
        DEFAULT_HEIGHT_KNEE, TerrainColoring,
    },
};

use super::{PyCameraState, PyTwist, Vec3, stl::PyStlMesh, tesselator::PyGeometryDelta};

/// `renderer::TerrainColoring`: what colors the terrain. With
/// `height_coloring_enabled` that is its height on the world's Y axis, as on a
/// topographic map, rather than the distance along the camera's forward axis.
/// Heights are measured from the world's Y = 0, or with `relative_to_robot`
/// from `robot_height`; to follow the robot `configure` the renderer again,
/// which at an unchanged frame size only changes the settings.
///
/// The position in the colormap of a height at the distance `d` from the
/// neutral height, `height_offset` above what heights are measured from, is
/// `0.5 + 0.5 * sign(d) * tanh(height_gain * ln(1 + |d| / height_knee))`.
/// `height_bands` splits the colormap into flat bands, 0 for a smooth
/// gradient.
#[pyclass(
    module = "r3v_renderer",
    name = "TerrainColoring",
    from_py_object,
    eq,
    get_all,
    set_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyTerrainColoring {
    height_coloring_enabled: bool,
    relative_to_robot: bool,
    robot_height: f32,
    height_offset: f32,
    height_knee: f32,
    height_gain: f32,
    height_bands: u32,
}

impl From<TerrainColoring> for PyTerrainColoring {
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

impl From<PyTerrainColoring> for TerrainColoring {
    fn from(coloring: PyTerrainColoring) -> Self {
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

fn python_bool(value: bool) -> &'static str {
    if value { "True" } else { "False" }
}

#[pymethods]
impl PyTerrainColoring {
    #[new]
    #[pyo3(signature = (
        height_coloring_enabled = false,
        relative_to_robot = false,
        robot_height = 0.0,
        height_offset = 0.0,
        height_knee = DEFAULT_HEIGHT_KNEE,
        height_gain = DEFAULT_HEIGHT_GAIN,
        height_bands = 0,
    ))]
    fn new(
        height_coloring_enabled: bool,
        relative_to_robot: bool,
        robot_height: f32,
        height_offset: f32,
        height_knee: f32,
        height_gain: f32,
        height_bands: u32,
    ) -> Self {
        Self {
            height_coloring_enabled,
            relative_to_robot,
            robot_height,
            height_offset,
            height_knee,
            height_gain,
            height_bands,
        }
    }
    fn __repr__(&self) -> String {
        format!(
            "TerrainColoring(height_coloring_enabled={}, relative_to_robot={}, robot_height={:?}, height_offset={:?}, height_knee={:?}, height_gain={:?}, height_bands={})",
            python_bool(self.height_coloring_enabled),
            python_bool(self.relative_to_robot),
            self.robot_height,
            self.height_offset,
            self.height_knee,
            self.height_gain,
            self.height_bands,
        )
    }
}

/// `renderer::WgpuAcceleratedConfiguration`: the frame size in pixels, the
/// opaque sRGB background color (components between 0 and 1), the distance
/// along the camera's forward axis over which the voxel colormap runs once
/// and the terrain's coloring.
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
    terrain_coloring: PyTerrainColoring,
}

#[pymethods]
impl PyWgpuRendererConfiguration {
    #[new]
    #[pyo3(signature = (
        width,
        height,
        background_color = (DEFAULT_BACKGROUND_COLOR[0], DEFAULT_BACKGROUND_COLOR[1], DEFAULT_BACKGROUND_COLOR[2]),
        color_depth_range = DEFAULT_COLOR_DEPTH_RANGE,
        terrain_coloring = PyTerrainColoring::from(TerrainColoring::default()),
    ))]
    fn new(
        width: u32,
        height: u32,
        background_color: Vec3,
        color_depth_range: f32,
        terrain_coloring: PyTerrainColoring,
    ) -> Self {
        Self {
            width,
            height,
            background_color,
            color_depth_range,
            terrain_coloring,
        }
    }
    fn __repr__(&self) -> String {
        format!(
            "WgpuRendererConfiguration(width={}, height={}, background_color={:?}, color_depth_range={:?}, terrain_coloring={})",
            self.width,
            self.height,
            self.background_color,
            self.color_depth_range,
            self.terrain_coloring.__repr__(),
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

    /// `Renderer::configure`: sets the frame size, the background color, the
    /// colormap depth range and the terrain's coloring. At an unchanged frame
    /// size only the settings change, from the next frame on. `ValueError`
    /// unless both sides are between 1 and 8192, the color components between
    /// 0 and 1, the depth range, `height_knee` and `height_gain` positive and
    /// `height_offset` and `robot_height` finite.
    fn configure(&mut self, configuration: PyWgpuRendererConfiguration) -> PyResult<()> {
        let (r, g, b) = configuration.background_color;
        Ok(self.renderer.configure(
            configuration.width,
            configuration.height,
            [r, g, b],
            configuration.color_depth_range,
            configuration.terrain_coloring.into(),
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
