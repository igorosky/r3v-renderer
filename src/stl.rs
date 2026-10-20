use std::{
    io::{Read, Seek},
    path::Path,
};

use crate::geometry::{self, RealTriangle};

#[derive(Debug, Clone)]
pub struct StlMesh {
    pub triangles: Vec<RealTriangle>,
}

pub fn read_file(path: &Path) -> Result<StlMesh, String> {
    let file = std::fs::File::open(path)
        .map_err(|err| format!("Failed to read STL '{}': {err}", path.display()))?;
    read(&mut std::io::BufReader::new(file))
        .map_err(|err| format!("stl_io failed to parse STL '{}': {err}", path.display()))
}

/// [`read_file`] for the contents of a file, e.g. where there is no file
/// system.
pub fn read_bytes(bytes: &[u8]) -> Result<StlMesh, String> {
    read(&mut std::io::Cursor::new(bytes))
        .map_err(|err| format!("stl_io failed to parse STL: {err}"))
}

fn read(reader: &mut (impl Read + Seek)) -> std::io::Result<StlMesh> {
    let parsed = stl_io::read_stl(reader)?;
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
