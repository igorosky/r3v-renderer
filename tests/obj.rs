mod common;

use common::TempDir;
use r3v_renderer::geometry::RealTriangle;
use r3v_renderer::obj::{read_bytes, read_file};

fn triangle(v0: [f32; 3], v1: [f32; 3], v2: [f32; 3]) -> [[f32; 3]; 3] {
    [v0, v1, v2]
}

fn positions(triangles: &[RealTriangle]) -> Vec<[[f32; 3]; 3]> {
    triangles
        .iter()
        .map(|triangle| [triangle.v0, triangle.v1, triangle.v2])
        .collect()
}

#[test]
fn reads_triangles_with_flipped_v_coordinates() {
    let mesh = read_bytes(
        b"# a triangle\n\
          v 0 0 0\n\
          v 1 0 0\n\
          v 0 1 0\n\
          vt 0 0\n\
          vt 1 0\n\
          vt 0 1\n\
          f 1/1 2/2 3/3\n",
    )
    .unwrap();
    assert_eq!(
        positions(&mesh.triangles),
        [triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])]
    );
    assert_eq!(mesh.uvs, Some(vec![[[0.0, 1.0], [1.0, 1.0], [0.0, 0.0]]]));
}

#[test]
fn fans_polygons_into_triangles() {
    let mesh = read_bytes(
        b"v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n\
          vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n\
          f 1/1/1 2/2/1 3/3/1 4/4/1\n",
    )
    .unwrap();
    let (a, b, c, d) = (
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    );
    assert_eq!(
        positions(&mesh.triangles),
        [triangle(a, b, c), triangle(a, c, d)]
    );
    assert_eq!(
        mesh.uvs,
        Some(vec![
            [[0.0, 1.0], [1.0, 1.0], [1.0, 0.0]],
            [[0.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
        ])
    );
}

#[test]
fn resolves_negative_indices() {
    let mesh =
        read_bytes(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0.5 0.25\nf -3/-1 -2/-1 -1/-1\n").unwrap();
    assert_eq!(
        positions(&mesh.triangles),
        [triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])]
    );
    assert_eq!(mesh.uvs, Some(vec![[[0.5, 0.75]; 3]]));
}

#[test]
fn has_no_uvs_without_texture_coordinates() {
    let mesh = read_bytes(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1//1 2//1 3//1\nf 1 2 3\n").unwrap();
    assert_eq!(mesh.triangles.len(), 2);
    assert_eq!(mesh.uvs, None);
}

#[test]
fn faces_without_texture_coordinates_get_zero_uvs_next_to_faces_with_them() {
    let mesh = read_bytes(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 1 1\nf 1 2 3\nf 1/1 2/1 3/1\n").unwrap();
    assert_eq!(mesh.uvs, Some(vec![[[0.0, 0.0]; 3], [[1.0, 0.0]; 3]]));
}

#[test]
fn ignores_unknown_statements_and_joins_continued_lines() {
    let mesh = read_bytes(
        b"mtllib robot.mtl\no part\ng group\nusemtl wood\ns off\nvn 0 0 1\n\
          v 0 0 0 1\nv 1 0 0\nv 0 1 0\nf 1 2 \\\n3\n",
    )
    .unwrap();
    assert_eq!(
        positions(&mesh.triangles),
        [triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])]
    );
}

#[test]
fn rejects_bad_faces() {
    for (obj, expected) in [
        ("v 0 0 0\nv 1 0 0\nf 1 2\n", "line 3: face has 2 vertices"),
        (
            "v 0 0 0\nf 1 2 3\n",
            "line 2: vertex index 2 is out of range",
        ),
        ("v 0 0 0\nf 0 1 1\n", "line 2: vertex index 0 is not valid"),
        (
            "v 0 0 0\nf 1/2 1 1\n",
            "line 2: texture coordinate index 2 is out of range",
        ),
        ("v 0 x 0\n", "line 1: invalid number 'x'"),
        ("v 0 0\n", "line 1: expected 3 numbers, found 2"),
    ] {
        let err = read_bytes(obj.as_bytes()).unwrap_err();
        assert!(
            err.contains(expected),
            "{err:?} does not contain {expected:?}"
        );
    }
}

#[test]
fn read_file_names_the_file_in_errors() {
    let dir = TempDir::new("obj");
    let good = dir.write("good.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    assert_eq!(read_file(&good).unwrap().triangles.len(), 1);

    let bad = dir.write("bad.obj", "f 1 2 3\n");
    let err = read_file(&bad).unwrap_err();
    assert!(err.contains("bad.obj"), "{err}");
    assert!(read_file(&dir.path().join("missing.obj")).is_err());
}
