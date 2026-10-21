//! `stl::StlMesh` and `stl::read_file`.

use std::path::PathBuf;

use pyo3::prelude::*;

use crate::{
    bindings::checked,
    stl::{self, StlMesh},
};

use super::PyTriangle;

/// Triangle mesh for `WgpuRenderer.upload_asset` (`stl::StlMesh`).
#[pyclass(module = "r3v_renderer", name = "StlMesh")]
pub(super) struct PyStlMesh(pub(super) StlMesh);

#[pymethods]
impl PyStlMesh {
    /// A mesh of `triangles`.
    #[new]
    fn new(triangles: Vec<PyTriangle>) -> Self {
        Self(StlMesh {
            triangles: triangles.into_iter().map(Into::into).collect(),
        })
    }

    /// `stl::read_file`: reads an ASCII or binary STL file and winds every
    /// triangle to match the normal declared for it. `RuntimeError` on
    /// failure.
    #[staticmethod]
    fn read_file(py: Python<'_>, path: PathBuf) -> PyResult<Self> {
        let mesh = py.detach(|| stl::read_file(&path).map_err(checked::Error::failed))?;
        Ok(Self(mesh))
    }

    /// The triangles, as copies: assign a list to change them.
    #[getter]
    fn triangles(&self) -> Vec<PyTriangle> {
        self.0.triangles.iter().map(Into::into).collect()
    }

    #[setter]
    fn set_triangles(&mut self, triangles: Vec<PyTriangle>) {
        self.0.triangles = triangles.into_iter().map(Into::into).collect();
    }

    fn __repr__(&self) -> String {
        format!("StlMesh(<{} triangles>)", self.0.triangles.len())
    }
}
