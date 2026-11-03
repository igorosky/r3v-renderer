use std::path::Path;

use crate::{geometry::RealTriangle, obj, stl};

#[derive(Debug, Clone)]
pub struct Mesh {
    pub triangles: Vec<RealTriangle>,
    pub uvs: Option<Vec<[[f32; 2]; 3]>>,
}

pub fn read_file(path: &Path) -> Result<Mesh, String> {
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .unwrap_or_default();
    if extension.eq_ignore_ascii_case("stl") {
        stl::read_file(path)
    } else if extension.eq_ignore_ascii_case("obj") {
        obj::read_file(path)
    } else {
        Err(format!(
            "unsupported mesh format '{}', only STL and OBJ are supported",
            path.display()
        ))
    }
}
