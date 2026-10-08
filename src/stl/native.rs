use std::fs;
use std::path::{Path, PathBuf};

use super::StlReader;

#[derive(Debug, Clone, PartialEq)]
pub struct Triangle {
    pub normal: [f32; 3],
    pub vertices: [[f32; 3]; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct StlMesh {
    pub triangles: Vec<Triangle>,
}

impl StlMesh {
    /// Reads an STL file with the native reader for backward compatibility.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        NativeStlReader.read_file(path.as_ref())
    }

    fn parse_native_bytes(path: &Path, bytes: &[u8]) -> Result<Self, String> {
        if bytes.starts_with(b"solid ") || bytes.starts_with(b"Solid ") {
            Self::parse_ascii(&String::from_utf8_lossy(bytes))
        } else {
            Self::parse_binary(bytes)
                .map_err(|err| format!("Failed to parse STL '{}': {err}", path.display()))
        }
    }

    fn parse_ascii(contents: &str) -> Result<Self, String> {
        let mut triangles = Vec::new();
        let mut current_normal: Option<[f32; 3]> = None;
        let mut current_vertices: Vec<[f32; 3]> = Vec::new();
        let mut inside_loop = false;

        for line in contents.lines() {
            let trimmed = line.trim();

            if let Some(value) = trimmed.strip_prefix("facet normal") {
                current_normal = Some(parse_triplet(value)?);
                continue;
            }

            if trimmed == "outer loop" {
                inside_loop = true;
                current_vertices.clear();
                continue;
            }

            if trimmed == "endloop" {
                inside_loop = false;
                continue;
            }

            if trimmed == "endfacet" {
                if let Some(normal) = current_normal {
                    if current_vertices.len() == 3 {
                        triangles.push(Triangle {
                            normal,
                            vertices: [
                                current_vertices[0],
                                current_vertices[1],
                                current_vertices[2],
                            ],
                        });
                    }
                }
                current_normal = None;
                current_vertices.clear();
                continue;
            }

            if inside_loop && trimmed.starts_with("vertex") {
                current_vertices.push(parse_triplet(
                    trimmed.strip_prefix("vertex").unwrap_or(trimmed),
                )?);
            }
        }

        if triangles.is_empty() {
            return Err("No valid facets found in ASCII STL data.".to_string());
        }

        Ok(Self { triangles })
    }

    fn parse_binary(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() < 84 {
            return Err(
                "Binary STL is too small to contain a valid header and triangle count.".to_string(),
            );
        }

        let triangle_count = u32::from_le_bytes(bytes[80..84].try_into().unwrap());
        let mut triangles = Vec::new();
        let mut offset = 84usize;

        for _ in 0..triangle_count {
            if offset + 50 > bytes.len() {
                return Err(
                    "Binary STL is truncated before the end of its triangle data.".to_string(),
                );
            }

            let normal = read_vec3(&bytes[offset..offset + 12]);
            offset += 12;

            let vertices = [
                read_vec3(&bytes[offset..offset + 12]),
                read_vec3(&bytes[offset + 12..offset + 24]),
                read_vec3(&bytes[offset + 24..offset + 36]),
            ];
            offset += 36;

            let _attribute_byte_count =
                u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap());
            offset += 2;

            triangles.push(Triangle { normal, vertices });
        }

        Ok(Self { triangles })
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn vertex_count(&self) -> usize {
        self.triangles.len() * 3
    }
}

#[derive(Clone, Copy, Debug, Default)]
/// STL reader implemented by this crate's native parser.
pub struct NativeStlReader;

impl StlReader for NativeStlReader {
    fn read_file(&self, path: &Path) -> Result<StlMesh, String> {
        let bytes = fs::read(path)
            .map_err(|err| format!("Failed to read STL '{}': {err}", path.display()))?;
        StlMesh::parse_native_bytes(path, &bytes)
    }
}

fn parse_triplet(value: &str) -> Result<[f32; 3], String> {
    let parts: Vec<&str> = value.split_whitespace().collect();
    if parts.len() != 3 {
        return Err(format!("Expected 3 float values, got '{value}'"));
    }
    Ok([
        parse_float(parts[0])?,
        parse_float(parts[1])?,
        parse_float(parts[2])?,
    ])
}

fn read_vec3(bytes: &[u8]) -> [f32; 3] {
    [
        read_f32_le(&bytes[0..4]),
        read_f32_le(&bytes[4..8]),
        read_f32_le(&bytes[8..12]),
    ]
}

fn read_f32_le(bytes: &[u8]) -> f32 {
    let array: [u8; 4] = bytes.try_into().unwrap_or([0; 4]);
    f32::from_le_bytes(array)
}

fn parse_float(raw: &str) -> Result<f32, String> {
    raw.trim()
        .parse::<f32>()
        .map_err(|_| format!("Failed to parse float '{raw}'"))
}

pub fn resolve_stl_reference(urdf_path: &Path, reference: &str) -> PathBuf {
    let stripped = reference.strip_prefix("package://").unwrap_or(reference);
    let mut parts = stripped.split('/').filter(|part| !part.is_empty());

    let package_name = parts.next().unwrap_or("");
    let relative_path = parts.collect::<Vec<_>>().join("/");

    let candidates = vec![
        urdf_path
            .parent()
            .unwrap_or(Path::new("."))
            .join(package_name)
            .join(&relative_path),
        urdf_path
            .parent()
            .unwrap_or(Path::new("."))
            .join(&relative_path),
        Path::new(".").join(package_name).join(&relative_path),
        Path::new(".").join(&relative_path),
    ];

    candidates
        .into_iter()
        .find(|candidate| candidate.exists())
        .unwrap_or_else(|| {
            urdf_path
                .parent()
                .unwrap_or(Path::new("."))
                .join(package_name)
                .join(&relative_path)
        })
}
