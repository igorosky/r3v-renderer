//! `tesselator::CpuTesselator` (trait `Tesselator`) and its output,
//! `Vec<TriangleChunk>`.

use pyo3::{
    exceptions::{PyAttributeError, PyIndexError, PyTypeError, PyValueError},
    prelude::*,
};

use crate::{
    bindings::checked::CheckedTesselator,
    geometry::{PointI, TriangleChunk},
};

use super::{PyPointI, PyTriangle};

/// `tesselator::CpuTesselatorConfiguration`: `voxel_size` is the edge length
/// of a voxel in world units, `marching_cubes_enabled` smooths the surface
/// with marching cubes instead of meshing the voxel faces.
#[pyclass(
    module = "r3v_renderer",
    name = "CpuTesselatorConfiguration",
    from_py_object,
    eq,
    get_all,
    set_all
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PyCpuTesselatorConfiguration {
    voxel_size: f32,
    marching_cubes_enabled: bool,
}

#[pymethods]
impl PyCpuTesselatorConfiguration {
    #[new]
    #[pyo3(signature = (voxel_size, marching_cubes_enabled = false))]
    fn new(voxel_size: f32, marching_cubes_enabled: bool) -> Self {
        Self {
            voxel_size,
            marching_cubes_enabled,
        }
    }
    fn __repr__(&self) -> String {
        format!(
            "CpuTesselatorConfiguration(voxel_size={:?}, marching_cubes_enabled={})",
            self.voxel_size,
            if self.marching_cubes_enabled {
                "True"
            } else {
                "False"
            }
        )
    }
}

/// Turns blocks of voxels into triangles (`tesselator::CpuTesselator`).
/// Configure it before use.
#[pyclass(module = "r3v_renderer", name = "CpuTesselator")]
pub(super) struct PyCpuTesselator(CheckedTesselator);

/// `blocks` as `PointI`s: `r3v_renderer.PointI`s, or anything else with
/// `x()`, `y()`, `z()` and `i()` methods, e.g. `r3v_voxelizer.PointI`.
fn extract_blocks(blocks: &Bound<'_, PyAny>) -> PyResult<Vec<PointI>> {
    let mut out = Vec::with_capacity(blocks.len().unwrap_or(0));
    for (k, block) in blocks.try_iter()?.enumerate() {
        let block = block?;
        if let Ok(point) = block.cast::<PyPointI>() {
            out.push((*point.borrow()).into());
            continue;
        }
        let call = |method: &str| {
            block.call_method0(method).map_err(|error| {
                if error.is_instance_of::<PyAttributeError>(block.py()) {
                    PyTypeError::new_err(format!(
                        "blocks[{k}] is a {}, not a PointI (anything with x(), y(), z() and i() \
                         methods)",
                        block
                            .get_type()
                            .name()
                            .map_or_else(|_| "?".to_owned(), |name| name.to_string())
                    ))
                } else {
                    error
                }
            })
        };
        out.push(PointI {
            x: call("x")?.extract()?,
            y: call("y")?.extract()?,
            z: call("z")?.extract()?,
            i: call("i")?.extract()?,
        });
    }
    Ok(out)
}

#[pymethods]
impl PyCpuTesselator {
    /// `CpuTesselator::new`, with marching cubes disabled.
    #[new]
    fn new() -> Self {
        Self(CheckedTesselator::new())
    }

    /// `Tesselator::configure`. `ValueError` unless `voxel_size` is positive.
    fn configure(&mut self, configuration: PyCpuTesselatorConfiguration) -> PyResult<()> {
        Ok(self.0.configure(
            configuration.voxel_size,
            configuration.marching_cubes_enabled,
        )?)
    }

    /// `CpuTesselator::marching_cubes_enabled`.
    fn marching_cubes_enabled(&self) -> bool {
        self.0.marching_cubes_enabled()
    }

    /// `CpuTesselator::set_marching_cubes_enabled`: a change re-tesselates
    /// every chunk with voxels, and the chunks next to them, on the next
    /// `get_output_geometry_delta`.
    fn set_marching_cubes_enabled(&mut self, enabled: bool) {
        self.0.set_marching_cubes_enabled(enabled);
    }

    /// `Tesselator::update_input_geometry`: every block replaces the mask the
    /// tesselator had for it. `blocks` is an iterable of `PointI`s, or of
    /// anything else with `x()`, `y()`, `z()` and `i()` methods, e.g. what
    /// `r3v_voxelizer` returns in compress mode.
    ///
    /// `ValueError` when a block is not aligned to the 4x4x2 grid (x and y
    /// multiples of 4, z of 2); none of the blocks is applied then.
    fn update_input_geometry(&mut self, py: Python<'_>, blocks: &Bound<'_, PyAny>) -> PyResult<()> {
        let blocks = extract_blocks(blocks)?;
        Ok(py.detach(|| self.0.update_input_geometry(blocks))?)
    }

    /// `Tesselator::get_output_geometry_delta`: the chunks changed since the
    /// last call, `None` when nothing changed.
    fn get_output_geometry_delta(&mut self, py: Python<'_>) -> PyResult<Option<PyGeometryDelta>> {
        let delta = py.detach(|| self.0.get_output_geometry_delta())?;
        Ok(delta.map(|delta| PyGeometryDelta(Some(delta))))
    }
}

/// Triangles of every chunk that changed (`Vec<TriangleChunk>`): a sequence
/// of `TriangleChunk`s. `WgpuRenderer.update_geometry` takes the chunks out of
/// it, using it afterwards raises `ValueError`.
#[pyclass(module = "r3v_renderer", name = "GeometryDelta")]
pub(super) struct PyGeometryDelta(Option<Vec<TriangleChunk>>);

impl PyGeometryDelta {
    fn chunks(&self) -> PyResult<&Vec<TriangleChunk>> {
        self.0.as_ref().ok_or_else(|| {
            PyValueError::new_err("the delta was passed to WgpuRenderer.update_geometry")
        })
    }

    pub(super) fn take(&mut self) -> PyResult<Vec<TriangleChunk>> {
        self.chunks()?;
        Ok(self.0.take().unwrap_or_default())
    }
}

#[pymethods]
impl PyGeometryDelta {
    fn __len__(&self) -> PyResult<usize> {
        Ok(self.chunks()?.len())
    }

    fn __getitem__(&self, index: isize) -> PyResult<PyTriangleChunk> {
        let chunks = self.chunks()?;
        let chunk = usize::try_from(index)
            .ok()
            .or_else(|| chunks.len().checked_sub(index.unsigned_abs()))
            .and_then(|index| chunks.get(index))
            .ok_or_else(|| {
                PyIndexError::new_err(format!(
                    "chunk {index} does not exist, the delta has {}",
                    chunks.len()
                ))
            })?;
        let index = chunk.index;
        Ok(PyTriangleChunk {
            index: (index.x, index.y, index.z),
            triangles: chunk.triangles.iter().map(Into::into).collect(),
        })
    }

    fn __repr__(&self) -> String {
        match &self.0 {
            Some(chunks) => format!("GeometryDelta(<{} chunks>)", chunks.len()),
            None => "GeometryDelta(<passed on>)".to_owned(),
        }
    }
}

/// All triangles of one chunk of 16x16x16 voxels (`geometry::TriangleChunk`):
/// `index` is the origin of the chunk in voxels, `triangles` are in world
/// units. They replace what the chunk had before; no triangles means it is
/// empty now.
#[pyclass(module = "r3v_renderer", name = "TriangleChunk", frozen, get_all)]
pub(super) struct PyTriangleChunk {
    index: (i32, i32, i32),
    triangles: Vec<PyTriangle>,
}

#[pymethods]
impl PyTriangleChunk {
    fn __repr__(&self) -> String {
        format!(
            "TriangleChunk(index={:?}, triangles=<{} triangles>)",
            self.index,
            self.triangles.len()
        )
    }
}
