//! `mesh::Mesh` and `mesh::read_file`.

use std::path::PathBuf;

use pyo3::prelude::*;

use crate::{
    bindings::checked,
    mesh::{self, Mesh},
};

use super::PyTriangle;

/// Triangle mesh for `WgpuRenderer.upload_asset` (`mesh::Mesh`).
#[pyclass(module = "r3v_renderer", name = "Mesh")]
pub(super) struct PyMesh(pub(super) Mesh);

#[pymethods]
impl PyMesh {
    /// A mesh of `triangles`.
    #[new]
    fn new(triangles: Vec<PyTriangle>) -> Self {
        Self(Mesh {
            triangles: triangles.into_iter().map(Into::into).collect(),
            uvs: None,
        })
    }

    /// `mesh::read_file`: reads an STL or OBJ file, picked by its extension.
    /// `RuntimeError` on failure.
    #[staticmethod]
    fn read_file(py: Python<'_>, path: PathBuf) -> PyResult<Self> {
        let mesh = py.detach(|| mesh::read_file(&path).map_err(checked::Error::failed))?;
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
        self.0.uvs = None;
    }

    fn __repr__(&self) -> String {
        format!("Mesh(<{} triangles>)", self.0.triangles.len())
    }
}
