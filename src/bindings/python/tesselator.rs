//! `tesselator::CpuTesselator` (trait `Tesselator`) and its output,
//! `Vec<TriangleChunk>`.

use pyo3::{
    exceptions::{PyAttributeError, PyIndexError, PyTypeError, PyValueError},
    prelude::*,
};

use crate::{
    bindings::checked::CheckedTesselator,
    camera::CameraState,
    geometry::{PointI, TriangleChunk, VOXEL_GROUP_PAGES},
};

use super::{PyCameraState, PyPointI, PyTriangle};

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

/// `object.method()`, a `TypeError` saying `what` is no `expected` when
/// `object` has no such method.
fn call<'py>(
    object: &Bound<'py, PyAny>,
    method: &str,
    what: &dyn Fn() -> String,
    expected: &str,
) -> PyResult<Bound<'py, PyAny>> {
    object.call_method0(method).map_err(|error| {
        if error.is_instance_of::<PyAttributeError>(object.py()) {
            PyTypeError::new_err(format!(
                "{} is a {}, not a {expected}",
                what(),
                object
                    .get_type()
                    .name()
                    .map_or_else(|_| "?".to_owned(), |name| name.to_string())
            ))
        } else {
            error
        }
    })
}

/// `object` as a `PointI`: a `r3v_renderer.PointI`, or anything else with
/// `x()`, `y()`, `z()` and `i()` methods, e.g. `r3v_voxelizer.PointI`.
fn point_i(object: &Bound<'_, PyAny>, what: &dyn Fn() -> String) -> PyResult<PointI> {
    if let Ok(point) = object.cast::<PyPointI>() {
        return Ok((*point.borrow()).into());
    }
    const EXPECTED: &str = "PointI (anything with x(), y(), z() and i() methods)";
    Ok(PointI {
        x: call(object, "x", what, EXPECTED)?.extract()?,
        y: call(object, "y", what, EXPECTED)?.extract()?,
        z: call(object, "z", what, EXPECTED)?.extract()?,
        i: call(object, "i", what, EXPECTED)?.extract()?,
    })
}

/// `blocks` as `PointI`s: `r3v_renderer.PointI`s, or anything else with
/// `x()`, `y()`, `z()` and `i()` methods, e.g. `r3v_voxelizer.PointI`.
fn extract_blocks(blocks: &Bound<'_, PyAny>) -> PyResult<Vec<PointI>> {
    let mut out = Vec::with_capacity(blocks.len().unwrap_or(0));
    for (k, block) in blocks.try_iter()?.enumerate() {
        out.push(point_i(&block?, &|| format!("blocks[{k}]"))?);
    }
    Ok(out)
}

/// `values` as the words of a group, `ValueError` unless it holds one for
/// every page.
fn words(values: Vec<u64>, what: &dyn Fn() -> String) -> PyResult<Box<[u64; VOXEL_GROUP_PAGES]>> {
    values
        .into_boxed_slice()
        .try_into()
        .map_err(|values: Box<[u64]>| {
            PyValueError::new_err(format!(
                "{} holds {} words, a group holds {VOXEL_GROUP_PAGES}",
                what(),
                values.len()
            ))
        })
}

type Group = (PointI, Box<[u64; VOXEL_GROUP_PAGES]>);

/// `groups` as keys and words: `r3v_renderer.VoxelGroup`s, or anything else
/// with `point()` and `values()` methods, e.g.
/// `r3v_voxelizer.CompressedVoxelGroup`.
fn extract_groups(groups: &Bound<'_, PyAny>) -> PyResult<Vec<Group>> {
    const EXPECTED: &str = "VoxelGroup (anything with point() and values() methods)";
    let mut out = Vec::with_capacity(groups.len().unwrap_or(0));
    for (k, group) in groups.try_iter()?.enumerate() {
        let group = group?;
        if let Ok(group) = group.cast::<PyVoxelGroup>() {
            let group = group.borrow();
            out.push((group.point.into(), group.values.clone()));
            continue;
        }
        let what = || format!("groups[{k}]");
        let point = call(&group, "point", &what, EXPECTED)?;
        let values = call(&group, "values", &what, EXPECTED)?.extract()?;
        out.push((
            point_i(&point, &|| format!("groups[{k}].point()"))?,
            words(values, &|| format!("groups[{k}].values()"))?,
        ));
    }
    Ok(out)
}

/// Constructed voxels of a cube of 64x64x64 voxels (`geometry::VoxelGroup`):
/// `point()` is the key of the group, voxel coordinates shifted right by 6
/// (its `i` is ignored), `values()` the occupancy of every 4x4x4 page of the
/// group, in z-order (bits of x, y and z interleaved from the lowest one). Bit
/// `x + 4 * y + 16 * z` of a word is set when voxel (x, y, z) of its page is
/// constructed. Same interface as `r3v_voxelizer.CompressedVoxelGroup`, which
/// can be passed on as it is.
#[pyclass(module = "r3v_renderer", name = "VoxelGroup", from_py_object, eq)]
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct PyVoxelGroup {
    point: PyPointI,
    values: Box<[u64; VOXEL_GROUP_PAGES]>,
}

#[pymethods]
impl PyVoxelGroup {
    /// `point` is a `PointI` or anything else with `x()`, `y()`, `z()` and
    /// `i()` methods. `ValueError` unless `values` holds a word for every page
    /// of the group (4096).
    #[new]
    fn new(point: &Bound<'_, PyAny>, values: Vec<u64>) -> PyResult<Self> {
        let PointI { x, y, z, i } = point_i(point, &|| "point".to_owned())?;
        Ok(Self {
            point: PyPointI { x, y, z, i },
            values: words(values, &|| "values".to_owned())?,
        })
    }
    fn point(&self) -> PyPointI {
        self.point
    }
    fn values(&self) -> Vec<u64> {
        self.values.to_vec()
    }
    fn __repr__(&self) -> String {
        format!(
            "VoxelGroup(point={}, values=<{} pages with voxels>)",
            self.point.__repr__(),
            self.values.iter().filter(|&&word| word != 0).count()
        )
    }
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

    /// `CpuTesselator::replace_input_geometry`: drops every voxel the
    /// tesselator holds and keeps only the ones of `groups`. `groups` is an
    /// iterable of `VoxelGroup`s, or of anything else with `point()` and
    /// `values()` methods, e.g. what `r3v_voxelizer`'s
    /// `get_all_points_compressed` returns. Groups sharing a key add up. Only
    /// the chunks whose voxels changed come out of the next
    /// `get_output_geometry_delta`, the ones left without voxels with no
    /// triangles.
    ///
    /// `ValueError` when a group does not hold a word for every page (4096) or
    /// a key coordinate is outside [-2^25, 2^25); none of the groups is
    /// applied then.
    fn replace_input_geometry(
        &mut self,
        py: Python<'_>,
        groups: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let groups = extract_groups(groups)?;
        Ok(py.detach(|| {
            self.0
                .replace_input_geometry(groups.iter().map(|(key, values)| (key.clone(), &**values)))
        })?)
    }

    /// `Tesselator::get_output_geometry_delta`: the chunks changed since the
    /// last call, `None` when nothing changed. With a `camera_state`, only the
    /// chunks in its view; the others stay changed for a later call.
    #[pyo3(signature = (camera_state=None))]
    fn get_output_geometry_delta(
        &mut self,
        py: Python<'_>,
        camera_state: Option<PyCameraState>,
    ) -> PyResult<Option<PyGeometryDelta>> {
        let camera_state = camera_state.map(CameraState::from);
        let delta = py.detach(|| self.0.get_output_geometry_delta(camera_state.as_ref()))?;
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
