use std::path::Path;

use crate::geometry::{self, RealTriangle};

#[derive(Debug, Clone)]
pub struct StlMesh {
    pub triangles: Vec<RealTriangle>,
}

pub fn read_file(path: &Path) -> Result<StlMesh, String> {
    let file = std::fs::File::open(path)
        .map_err(|err| format!("Failed to read STL '{}': {err}", path.display()))?;
    let mut reader = std::io::BufReader::new(file);
    let parsed = stl_io::read_stl(&mut reader)
        .map_err(|err| format!("stl_io failed to parse STL '{}': {err}", path.display()))?;
    let triangles = parsed
        .faces
        .into_iter()
        .map(|face| {
            let [v0, v1, v2] = face.vertices.map(|index| parsed.vertices[index].0);
            geometry::RealTriangle { v0, v1, v2 }
                .with_winding(geometry::Winding::CounterClockwise, face.normal.into())
        })
        .collect();
    Ok(StlMesh { triangles })
}
