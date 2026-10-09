use std::path::Path;

#[derive(Debug, Clone, PartialEq)]
pub struct Triangle {
    pub normal: [f32; 3],
    pub vertices: [[f32; 3]; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct StlMesh {
    pub triangles: Vec<Triangle>,
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
        .map(|face| Triangle {
            normal: face.normal.0,
            vertices: face.vertices.map(|index| parsed.vertices[index].0),
        })
        .collect();
    Ok(StlMesh { triangles })
}
