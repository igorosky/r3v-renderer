//! `stl::StlMesh` and `stl::read_file`.

use std::{ffi::c_char, path::Path, ptr};

use crate::stl::{self, StlMesh};

use super::{Error, R3vTriangle, drop_handle, new_handle, slice_arg, str_arg};

/// Triangle mesh for `r3v_wgpu_renderer_upload_asset` (`stl::StlMesh`).
pub struct R3vStlMesh(pub(super) StlMesh);

/// `stl::read_file`: reads an ASCII or binary STL file and winds every
/// triangle to match the normal declared for it. NULL on failure.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_stl_read_file(path: *const c_char) -> *mut R3vStlMesh {
    new_handle(|| {
        let path = unsafe { str_arg(path, "path") }?;
        stl::read_file(Path::new(path))
            .map(R3vStlMesh)
            .map_err(Error::failed)
    })
}

/// A mesh of the `count` triangles at `triangles` (copied), which may be NULL
/// when `count` is 0. NULL on failure.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_stl_mesh_from_triangles(
    triangles: *const R3vTriangle,
    count: usize,
) -> *mut R3vStlMesh {
    new_handle(|| {
        let triangles = unsafe { slice_arg(triangles, count, "triangles") }?;
        Ok(R3vStlMesh(StlMesh {
            triangles: triangles.iter().map(|&triangle| triangle.into()).collect(),
        }))
    })
}

/// Number of triangles of the mesh, 0 for NULL.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_stl_mesh_triangle_count(mesh: *const R3vStlMesh) -> usize {
    unsafe { mesh.as_ref() }.map_or(0, |mesh| mesh.0.triangles.len())
}

/// The triangles of the mesh, writable in place (e.g. to scale it) and valid
/// until the mesh is dropped. NULL for NULL or an empty mesh.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_stl_mesh_triangles(mesh: *mut R3vStlMesh) -> *mut R3vTriangle {
    match unsafe { mesh.as_mut() } {
        Some(mesh) if !mesh.0.triangles.is_empty() => mesh.0.triangles.as_mut_ptr().cast(),
        _ => ptr::null_mut(),
    }
}

/// Frees a mesh. NULL is ignored.
#[unsafe(no_mangle)]
pub extern "C" fn r3v_stl_mesh_drop(mesh: *mut R3vStlMesh) {
    unsafe { drop_handle(mesh) }
}
