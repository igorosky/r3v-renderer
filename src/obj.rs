use std::path::Path;

use crate::{geometry::RealTriangle, mesh::Mesh};

pub fn read_file(path: &Path) -> Result<Mesh, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|err| format!("Failed to read OBJ '{}': {err}", path.display()))?;
    read_str(&text).map_err(|err| format!("Failed to parse OBJ '{}': {err}", path.display()))
}

pub fn read_bytes(bytes: &[u8]) -> Result<Mesh, String> {
    let text = std::str::from_utf8(bytes).map_err(|err| format!("OBJ is not UTF-8: {err}"))?;
    read_str(text).map_err(|err| format!("Failed to parse OBJ: {err}"))
}

struct Corner {
    position: [f32; 3],
    uv: Option<[f32; 2]>,
}

fn read_str(text: &str) -> Result<Mesh, String> {
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut uvs: Vec<[f32; 2]> = Vec::new();
    let mut triangles = Vec::new();
    let mut triangle_uvs = Vec::new();
    let mut any_uv = false;

    let mut logical_line = String::new();
    for (index, raw_line) in text.lines().enumerate() {
        let line_number = index + 1;
        let raw_line = raw_line.split('#').next().unwrap_or_default();
        if let Some(continued) = raw_line.trim_end().strip_suffix('\\') {
            logical_line.push_str(continued);
            logical_line.push(' ');
            continue;
        }
        logical_line.push_str(raw_line);
        let line = std::mem::take(&mut logical_line);
        let mut fields = line.split_whitespace();
        let Some(keyword) = fields.next() else {
            continue;
        };
        let at_line = |err: String| format!("line {line_number}: {err}");
        match keyword {
            "v" => positions.push(floats::<3>(&mut fields, 3).map_err(at_line)?),
            "vt" => {
                let [u, v] = floats::<2>(&mut fields, 1).map_err(at_line)?;
                uvs.push([u, 1.0 - v]);
            }
            "f" => {
                let corners = fields
                    .map(|field| corner(field, &positions, &uvs))
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(at_line)?;
                if corners.len() < 3 {
                    return Err(at_line(format!(
                        "face has {} vertices, needs at least 3",
                        corners.len()
                    )));
                }
                for pair in corners[1..].windows(2) {
                    let face = [&corners[0], &pair[0], &pair[1]];
                    triangles.push(RealTriangle {
                        v0: face[0].position,
                        v1: face[1].position,
                        v2: face[2].position,
                    });
                    let face_uvs = face.map(|corner| corner.uv);
                    any_uv |= face_uvs.iter().any(Option::is_some);
                    triangle_uvs.push(face_uvs.map(|uv| uv.unwrap_or_default()));
                }
            }
            _ => {}
        }
    }

    Ok(Mesh {
        triangles,
        uvs: any_uv.then_some(triangle_uvs),
    })
}

fn floats<const N: usize>(
    fields: &mut std::str::SplitWhitespace<'_>,
    required: usize,
) -> Result<[f32; N], String> {
    let mut values = [0.0; N];
    for (axis, value) in values.iter_mut().enumerate() {
        match fields.next() {
            Some(field) => {
                *value = field
                    .parse()
                    .map_err(|err| format!("invalid number '{field}': {err}"))?;
            }
            None if axis < required => {
                return Err(format!("expected {required} numbers, found {axis}"));
            }
            None => break,
        }
    }
    Ok(values)
}

fn corner(field: &str, positions: &[[f32; 3]], uvs: &[[f32; 2]]) -> Result<Corner, String> {
    let mut indices = field.split('/');
    let position = indices
        .next()
        .ok_or_else(|| format!("invalid face vertex '{field}'"))?;
    let position = positions[resolve_index(position, positions.len(), "vertex")?];
    let uv = match indices.next() {
        Some(uv) if !uv.is_empty() => {
            Some(uvs[resolve_index(uv, uvs.len(), "texture coordinate")?])
        }
        _ => None,
    };
    Ok(Corner { position, uv })
}

fn resolve_index(field: &str, count: usize, what: &str) -> Result<usize, String> {
    let index: i64 = field
        .parse()
        .map_err(|err| format!("invalid {what} index '{field}': {err}"))?;
    let resolved = match index {
        1.. => index - 1,
        ..0 => count as i64 + index,
        0 => return Err(format!("{what} index 0 is not valid, indices start at 1")),
    };
    if (0..count as i64).contains(&resolved) {
        Ok(resolved as usize)
    } else {
        Err(format!(
            "{what} index {index} is out of range, {count} defined"
        ))
    }
}
