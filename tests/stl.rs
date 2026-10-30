mod common;

use common::{TempDir, close3};
use r3v_renderer::geometry::RealTriangle;
use r3v_renderer::stl::{read_bytes, read_file};

type Facet = ([f32; 3], [[f32; 3]; 3]);

fn ascii_stl(facets: &[Facet]) -> String {
    let mut out = String::from("solid test\n");
    for (normal, vertices) in facets {
        out += &format!(
            "  facet normal {} {} {}\n    outer loop\n",
            normal[0], normal[1], normal[2]
        );
        for v in vertices {
            out += &format!("      vertex {} {} {}\n", v[0], v[1], v[2]);
        }
        out += "    endloop\n  endfacet\n";
    }
    out += "endsolid test\n";
    out
}

fn binary_stl(facets: &[Facet]) -> Vec<u8> {
    let mut out = vec![0u8; 80];
    out[..6].copy_from_slice(b"binary");
    out.extend((facets.len() as u32).to_le_bytes());
    for (normal, vertices) in facets {
        for value in normal.iter().chain(vertices.iter().flatten()) {
            out.extend(value.to_le_bytes());
        }
        out.extend(0u16.to_le_bytes());
    }
    out
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Positive when the triangle is counter-clockwise seen from the side `normal` points to.
fn orientation(t: &RealTriangle, normal: [f32; 3]) -> f32 {
    let a = sub(t.v1, t.v0);
    let b = sub(t.v2, t.v0);
    let n = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    n[0] * normal[0] + n[1] * normal[1] + n[2] * normal[2]
}

fn sorted_vertices(t: &RealTriangle) -> Vec<[f32; 3]> {
    let mut v = vec![t.v0, t.v1, t.v2];
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v
}

/// Same vertices in the same cyclic order (i.e. same winding), starting anywhere.
fn same_cycle(t: &RealTriangle, expected: [[f32; 3]; 3]) -> bool {
    let actual = [t.v0, t.v1, t.v2];
    (0..3).any(|shift| (0..3).all(|i| actual[(i + shift) % 3] == expected[i]))
}

const UP: [f32; 3] = [0.0, 0.0, 1.0];
const CCW_UP: [[f32; 3]; 3] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
const CW_UP: [[f32; 3]; 3] = [[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]];

/// A closed unit tetrahedron with outward normals and counter-clockwise facets.
fn tetrahedron() -> Vec<Facet> {
    let o = [0.0, 0.0, 0.0];
    let x = [1.0, 0.0, 0.0];
    let y = [0.0, 1.0, 0.0];
    let z = [0.0, 0.0, 1.0];
    let k = 1.0 / 3.0_f32.sqrt();
    vec![
        ([0.0, 0.0, -1.0], [o, y, x]),
        ([0.0, -1.0, 0.0], [o, x, z]),
        ([-1.0, 0.0, 0.0], [o, z, y]),
        ([k, k, k], [x, y, z]),
    ]
}

fn write_both(dir: &TempDir, facets: &[Facet]) -> [std::path::PathBuf; 2] {
    [
        dir.write("ascii.stl", ascii_stl(facets)),
        dir.write("binary.stl", binary_stl(facets)),
    ]
}

#[test]
fn reads_single_facet() {
    let dir = TempDir::new("stl");
    for path in write_both(&dir, &[(UP, CCW_UP)]) {
        let mesh = read_file(&path).unwrap();
        assert_eq!(mesh.triangles.len(), 1, "{}", path.display());
        assert!(same_cycle(&mesh.triangles[0], CCW_UP));
    }
}

#[test]
fn reads_closed_mesh_with_shared_vertices() {
    let dir = TempDir::new("stl");
    let facets = tetrahedron();
    for path in write_both(&dir, &facets) {
        let mesh = read_file(&path).unwrap();
        assert_eq!(mesh.triangles.len(), facets.len());
        for (triangle, (normal, vertices)) in mesh.triangles.iter().zip(&facets) {
            assert!(same_cycle(triangle, *vertices), "{triangle:?}");
            assert!(orientation(triangle, *normal) > 0.0);
            close3(
                [
                    triangle.normal().x,
                    triangle.normal().y,
                    triangle.normal().z,
                ],
                *normal,
            );
        }
    }
}

#[test]
fn ascii_and_binary_give_same_triangles() {
    let dir = TempDir::new("stl");
    let [ascii, binary] = write_both(&dir, &tetrahedron());
    let ascii = read_file(&ascii).unwrap();
    let binary = read_file(&binary).unwrap();
    assert_eq!(ascii.triangles.len(), binary.triangles.len());
    for (a, b) in ascii.triangles.iter().zip(&binary.triangles) {
        assert_eq!([a.v0, a.v1, a.v2], [b.v0, b.v1, b.v2]);
    }
}

#[test]
fn clockwise_facets_are_rewound_to_match_declared_normal() {
    let dir = TempDir::new("stl");
    for path in write_both(&dir, &[(UP, CW_UP), ([0.0, 0.0, -1.0], CCW_UP)]) {
        let mesh = read_file(&path).unwrap();
        assert_eq!(mesh.triangles.len(), 2);
        let first = &mesh.triangles[0];
        assert!(orientation(first, UP) > 0.0);
        assert_eq!(sorted_vertices(first), {
            let mut v = CW_UP.to_vec();
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v
        });
        assert!(orientation(&mesh.triangles[1], [0.0, 0.0, -1.0]) > 0.0);
    }
}

#[test]
fn approximate_declared_normal_is_enough_to_decide_winding() {
    let dir = TempDir::new("stl");
    let sloppy = [0.3, -0.2, 0.9];
    for path in write_both(&dir, &[(sloppy, CW_UP)]) {
        let mesh = read_file(&path).unwrap();
        assert!(orientation(&mesh.triangles[0], UP) > 0.0);
    }
}

#[test]
fn zero_normal_keeps_vertex_order() {
    // A zero facet normal means "derive it from the vertex order" (right-hand rule).
    let dir = TempDir::new("stl");
    for path in write_both(&dir, &[([0.0; 3], CW_UP)]) {
        let mesh = read_file(&path).unwrap();
        assert!(same_cycle(&mesh.triangles[0], CW_UP));
    }
}

#[test]
fn keeps_negative_and_fractional_coordinates() {
    let dir = TempDir::new("stl");
    let vertices = [
        [-1.5, -2.25, 3.125],
        [10.5, -2.25, 3.125],
        [-1.5, 7.75, 3.125],
    ];
    for path in write_both(&dir, &[(UP, vertices)]) {
        let mesh = read_file(&path).unwrap();
        assert!(same_cycle(&mesh.triangles[0], vertices));
    }
}

#[test]
fn empty_solid_has_no_triangles() {
    let dir = TempDir::new("stl");
    for path in write_both(&dir, &[]) {
        let mesh = read_file(&path).unwrap();
        assert!(mesh.triangles.is_empty(), "{}", path.display());
    }
}

#[test]
fn missing_file_is_an_error_naming_the_path() {
    let dir = TempDir::new("stl");
    let path = dir.path().join("does-not-exist.stl");
    let error = read_file(&path).unwrap_err();
    assert!(error.contains("does-not-exist.stl"), "{error}");
}

#[test]
fn truncated_binary_is_an_error() {
    let dir = TempDir::new("stl");
    let mut bytes = binary_stl(&tetrahedron());
    bytes.truncate(bytes.len() - 30);
    let path = dir.write("truncated.stl", bytes);
    let error = read_file(&path).unwrap_err();
    assert!(error.contains("truncated.stl"), "{error}");
}

#[test]
fn garbage_is_an_error() {
    let dir = TempDir::new("stl");
    for (name, contents) in [
        ("short.stl", b"not an stl".to_vec()),
        ("empty.stl", Vec::new()),
        (
            "bad-ascii.stl",
            b"solid broken\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0\n".to_vec(),
        ),
    ] {
        let path = dir.write(name, contents);
        assert!(read_file(&path).is_err(), "{name} should fail to parse");
    }
}

#[test]
fn binary_with_solid_header_is_read_as_binary() {
    let facets = tetrahedron();
    let mut bytes = binary_stl(&facets);
    bytes[..6].copy_from_slice(b"solid ");
    let mesh = read_bytes(&bytes).unwrap();
    assert_eq!(mesh.triangles.len(), facets.len());
}

#[test]
fn read_bytes_matches_read_file() {
    let dir = TempDir::new("stl");
    let facets = tetrahedron();
    for path in write_both(&dir, &facets) {
        let bytes = std::fs::read(&path).unwrap();
        let from_bytes = read_bytes(&bytes).unwrap();
        let from_file = read_file(&path).unwrap();
        assert_eq!(from_bytes.triangles.len(), from_file.triangles.len());
        for (a, b) in from_bytes.triangles.iter().zip(&from_file.triangles) {
            assert_eq!([a.v0, a.v1, a.v2], [b.v0, b.v1, b.v2]);
        }
    }
}

#[test]
fn read_bytes_rejects_garbage() {
    assert!(read_bytes(b"definitely not an stl").is_err());
}
