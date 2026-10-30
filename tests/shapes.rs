use std::collections::HashMap;

use r3v_renderer::{
    geometry::RealTriangle,
    shapes::{ROUND_SEGMENTS, capsule, cuboid, cylinder, tessellate},
    urdf::{UrdfGeometry, UrdfMesh, Vector3},
};

fn bounds(triangles: &[RealTriangle]) -> ([f32; 3], [f32; 3]) {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for vertex in triangles.iter().flat_map(|t| [t.v0, t.v1, t.v2]) {
        for axis in 0..3 {
            min[axis] = min[axis].min(vertex[axis]);
            max[axis] = max[axis].max(vertex[axis]);
        }
    }
    (min, max)
}

#[track_caller]
fn assert_bounds(triangles: &[RealTriangle], half: [f32; 3]) {
    let (min, max) = bounds(triangles);
    for axis in 0..3 {
        assert!(
            (max[axis] - half[axis]).abs() < 1e-5 && (min[axis] + half[axis]).abs() < 1e-5,
            "axis {axis}: expected +-{}, got {} to {}",
            half[axis],
            min[axis],
            max[axis]
        );
    }
}

#[track_caller]
fn assert_faces_outward(triangles: &[RealTriangle]) {
    for triangle in triangles {
        let normal = triangle.normal();
        let centroid =
            [0, 1, 2].map(|axis| (triangle.v0[axis] + triangle.v1[axis] + triangle.v2[axis]) / 3.0);
        let outward = normal.x * centroid[0] + normal.y * centroid[1] + normal.z * centroid[2];
        assert!(outward > 0.0, "{triangle:?} faces inward");
    }
}

#[track_caller]
fn assert_closed(triangles: &[RealTriangle]) {
    let key = |v: [f32; 3]| v.map(f32::to_bits);
    let mut edges = HashMap::new();
    for triangle in triangles {
        let [a, b, c] = [triangle.v0, triangle.v1, triangle.v2].map(key);
        for edge in [(a, b), (b, c), (c, a)] {
            *edges.entry(edge).or_insert(0) += 1;
        }
    }
    for (&(a, b), &count) in &edges {
        assert_eq!(count, 1, "edge {a:?} -> {b:?} is used {count} times");
        assert!(edges.contains_key(&(b, a)), "edge {a:?} -> {b:?} is open");
    }
}

#[test]
fn cuboid_is_a_closed_box_of_the_size() {
    let triangles = cuboid([1.0, 2.0, 3.0]);
    assert_eq!(triangles.len(), 12);
    assert_bounds(&triangles, [0.5, 1.0, 1.5]);
    assert_faces_outward(&triangles);
    assert_closed(&triangles);
}

#[test]
fn cylinder_runs_along_z() {
    let triangles = cylinder(0.5, 2.0);
    assert_eq!(triangles.len(), 4 * ROUND_SEGMENTS);
    assert_bounds(&triangles, [0.5, 0.5, 1.0]);
    assert_faces_outward(&triangles);
    assert_closed(&triangles);
}

#[test]
fn sphere_is_a_capsule_without_length() {
    let triangles = capsule(2.0, 0.0);
    assert_bounds(&triangles, [2.0, 2.0, 2.0]);
    assert_faces_outward(&triangles);
    assert_closed(&triangles);
}

#[test]
fn capsule_length_is_between_the_centers_of_its_ends() {
    let sphere = capsule(0.5, 0.0);
    let triangles = capsule(0.5, 3.0);
    assert_eq!(triangles.len(), sphere.len() + 2 * ROUND_SEGMENTS);
    assert_bounds(&triangles, [0.5, 0.5, 2.0]);
    assert_faces_outward(&triangles);
    assert_closed(&triangles);
}

#[test]
fn tessellates_every_primitive_but_not_meshes() {
    let size = Vector3 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    };
    for geometry in [
        UrdfGeometry::Box { size },
        UrdfGeometry::Sphere { radius: 1.0 },
        UrdfGeometry::Cylinder {
            radius: 1.0,
            length: 1.0,
        },
        UrdfGeometry::Capsule {
            radius: 1.0,
            length: 1.0,
        },
    ] {
        let mesh = tessellate(&geometry).expect("primitives have a mesh");
        assert!(!mesh.triangles.is_empty(), "{geometry:?}");
    }
    let mesh = UrdfGeometry::Mesh(UrdfMesh {
        filename: "part.stl".to_owned(),
        scale: None,
    });
    assert!(tessellate(&mesh).is_none());
}
