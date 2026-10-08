use std::path::Path;

use super::{StlMesh, StlReader, Triangle};

/// STL reader backed by the `stl_io` crate.
#[derive(Clone, Copy, Debug, Default)]
pub struct StlIoReader;

impl StlReader for StlIoReader {
    fn read_file(&self, path: &Path) -> Result<StlMesh, String> {
        let file = std::fs::File::open(path)
            .map_err(|err| format!("Failed to read STL '{}': {err}", path.display()))?;
        let mut reader = std::io::BufReader::new(file);
        let parsed = stl_io::read_stl(&mut reader)
            .map_err(|err| format!("stl_io failed to parse STL '{}': {err}", path.display()))?;
        let triangles = parsed
            .faces
            .into_iter()
            .map(|face| Triangle {
                normal: face.normal.0,
                vertices: face.vertices.map(|index| parsed.vertices[index].0),
            })
            .collect();
        Ok(StlMesh { triangles })
    }
}
