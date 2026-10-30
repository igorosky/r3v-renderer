mod common;

use std::collections::{BTreeSet, HashSet};

use common::{Lcg, block_voxels, close, close3, page_index, voxel_groups};
use r3v_renderer::geometry::{
    Axis, DiscretePoint, DiscreteQuad, Point, PointI, RealQuad, RealTriangle, RealTwist, Triangle,
    VOXEL_GROUP_PAGES, Vector2D, Vector3D, Vertex, VoxelGroup, VoxelNormal, Winding,
};

fn vec3(v: Vector3D<f32>) -> [f32; 3] {
    [v.x, v.y, v.z]
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Orientation of a triangle relative to `normal`: positive when counter-clockwise seen from
/// the side `normal` points to.
fn orientation(triangle: &RealTriangle, normal: [f32; 3]) -> f32 {
    dot(
        cross(sub(triangle.v1, triangle.v0), sub(triangle.v2, triangle.v0)),
        normal,
    )
}

fn axis_vector(axis: Axis) -> [i32; 3] {
    let mut v = [0; 3];
    v[axis as usize] = 1;
    v
}

// PointI voxel bit layout: bit = x | y << 2 | z << 4 within a 4x4x2 brick.

#[test]
fn voxel_bit_follows_documented_layout() {
    let local = |x, y, z| Point { x, y, z };
    assert_eq!(PointI::voxel_bit(local(0, 0, 0)), 0);
    assert_eq!(PointI::voxel_bit(local(1, 0, 0)), 1);
    assert_eq!(PointI::voxel_bit(local(3, 0, 0)), 3);
    assert_eq!(PointI::voxel_bit(local(0, 1, 0)), 4);
    assert_eq!(PointI::voxel_bit(local(0, 3, 0)), 12);
    assert_eq!(PointI::voxel_bit(local(0, 0, 1)), 16);
    assert_eq!(PointI::voxel_bit(local(3, 3, 1)), 31);
}

#[test]
fn voxel_bit_and_local_round_trip_over_whole_brick() {
    let mut seen = [false; 32];
    for z in 0..2 {
        for y in 0..4 {
            for x in 0..4 {
                let local = Point { x, y, z };
                let bit = PointI::voxel_bit(local);
                assert!(bit < 32, "bit {bit} out of range for {local:?}");
                assert!(!seen[bit as usize], "bit {bit} assigned twice");
                seen[bit as usize] = true;
                assert_eq!(PointI::voxel_local(bit), local);
            }
        }
    }
    assert!(seen.iter().all(|&s| s));
}

#[test]
fn voxel_local_round_trips_every_bit() {
    for bit in 0..32 {
        assert_eq!(PointI::voxel_bit(PointI::voxel_local(bit)), bit);
    }
}

// Axis

#[test]
fn axis_plane_is_right_handed_and_distinct() {
    for axis in Axis::ALL {
        let (u, v) = axis.plane();
        assert_ne!(u, axis);
        assert_ne!(v, axis);
        assert_ne!(u, v);
        let u = axis_vector(u).map(|c| c as f32);
        let v = axis_vector(v).map(|c| c as f32);
        assert_eq!(cross(u, v), axis_vector(axis).map(|c| c as f32));
    }
}

#[test]
fn axis_all_lists_each_axis_once_in_index_order() {
    assert_eq!(Axis::ALL, [Axis::X, Axis::Y, Axis::Z]);
    for (index, axis) in Axis::ALL.into_iter().enumerate() {
        assert_eq!(axis as usize, index);
    }
}

// VoxelNormal

#[test]
fn voxel_normal_all_covers_six_distinct_directions() {
    let all = VoxelNormal::ALL;
    for (i, a) in all.iter().enumerate() {
        for b in &all[i + 1..] {
            assert_ne!(a, b);
        }
    }
    for axis in Axis::ALL {
        assert!(all.contains(&VoxelNormal::Positive(axis)));
        assert!(all.contains(&VoxelNormal::Negative(axis)));
    }
}

#[test]
fn voxel_normal_reports_axis_and_sign() {
    for axis in Axis::ALL {
        assert_eq!(VoxelNormal::Positive(axis).axis(), axis);
        assert_eq!(VoxelNormal::Negative(axis).axis(), axis);
        assert!(VoxelNormal::Positive(axis).is_positive());
        assert!(!VoxelNormal::Negative(axis).is_positive());
    }
}

// Point

#[test]
fn point_array_conversions_round_trip() {
    let point = Point::from_array([1, -2, 3]);
    assert_eq!(point, Point { x: 1, y: -2, z: 3 });
    assert_eq!(point.to_array(), [1, -2, 3]);
}

#[test]
fn point_from_fn_receives_each_axis() {
    let point = Point::from_fn(|axis| axis as i32 * 10);
    assert_eq!(point, Point { x: 0, y: 10, z: 20 });
}

#[test]
fn point_map_applies_to_every_component() {
    let point = Point { x: 1, y: 2, z: 3 }.map(|c| c as f32 * 0.5);
    assert_eq!(point.to_array(), [0.5, 1.0, 1.5]);
}

#[test]
fn point_axis_access_agrees_with_fields() {
    let mut point = Point { x: 4, y: 5, z: 6 };
    assert_eq!(point.get(Axis::X), 4);
    assert_eq!(point.get(Axis::Y), 5);
    assert_eq!(point.get(Axis::Z), 6);
    for axis in Axis::ALL {
        assert_eq!(point[axis], point.get(axis));
    }
    point[Axis::Y] = -7;
    assert_eq!(point, Point { x: 4, y: -7, z: 6 });
}

#[test]
fn point_from_vector() {
    let point: Point<i32> = Vector3D { x: 1, y: 2, z: 3 }.into();
    assert_eq!(point, Point { x: 1, y: 2, z: 3 });
}

// Vector3D

#[test]
fn cross_product_of_basis_vectors_is_right_handed() {
    let x = Vector3D::<f32>::unit(Axis::X);
    let y = Vector3D::<f32>::unit(Axis::Y);
    let z = Vector3D::<f32>::unit(Axis::Z);
    assert_eq!(x.cross(y), z);
    assert_eq!(y.cross(z), x);
    assert_eq!(z.cross(x), y);
    assert_eq!(y.cross(x), z * -1.0);
}

#[test]
fn cross_product_is_anti_commutative_and_zero_for_parallel_vectors() {
    let a = Vector3D {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    let b = Vector3D {
        x: -4.0,
        y: 0.5,
        z: 2.0,
    };
    close3(vec3(a.cross(b)), vec3(b.cross(a) * -1.0));
    assert_eq!(
        a.cross(a),
        Vector3D {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    );
    assert_eq!(
        a.cross(a * 3.0),
        Vector3D {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    );
    // The cross product is perpendicular to both operands.
    close(a.cross(b).dot(a), 0.0);
    close(a.cross(b).dot(b), 0.0);
}

#[test]
fn dot_product() {
    let a = Vector3D { x: 1, y: 2, z: 3 };
    let b = Vector3D { x: 4, y: -5, z: 6 };
    assert_eq!(a.dot(b), 4 - 10 + 18);
    assert_eq!(a.dot(Vector3D { x: 0, y: 0, z: 0 }), 0);
}

#[test]
fn unit_vectors() {
    assert_eq!(
        Vector3D::<i32>::unit(Axis::X),
        Vector3D { x: 1, y: 0, z: 0 }
    );
    assert_eq!(
        Vector3D::<i32>::unit(Axis::Y),
        Vector3D { x: 0, y: 1, z: 0 }
    );
    assert_eq!(
        Vector3D::<i32>::unit(Axis::Z),
        Vector3D { x: 0, y: 0, z: 1 }
    );
}

#[test]
fn vector_arithmetic_and_conversions() {
    let a = Vector3D { x: 5, y: 7, z: 9 };
    let b = Vector3D { x: 1, y: 2, z: 3 };
    assert_eq!(a - b, Vector3D { x: 4, y: 5, z: 6 });
    assert_eq!(b * 3, Vector3D { x: 3, y: 6, z: 9 });
    assert_eq!(
        b * -1,
        Vector3D {
            x: -1,
            y: -2,
            z: -3
        }
    );
    let from_array: Vector3D<i32> = [1, 2, 3].into();
    assert_eq!(from_array, b);
    assert_eq!(b.map(|c| c * 2), Vector3D { x: 2, y: 4, z: 6 });
    let mut c = b;
    c[Axis::Z] = 10;
    assert_eq!(c.get(Axis::Z), 10);
    assert_eq!(c[Axis::X], 1);
}

// Triangle

fn ccw_xy_triangle() -> RealTriangle {
    // Counter-clockwise when seen from +Z.
    Triangle {
        v0: [0.0, 0.0, 0.0],
        v1: [1.0, 0.0, 0.0],
        v2: [0.0, 1.0, 0.0],
    }
}

#[test]
fn triangle_reverse_flips_orientation_and_keeps_vertices() {
    let triangle = ccw_xy_triangle();
    let reversed = triangle.reverse();
    assert!(orientation(&triangle, [0.0, 0.0, 1.0]) > 0.0);
    assert!(orientation(&reversed, [0.0, 0.0, 1.0]) < 0.0);
    let mut original = vec![triangle.v0, triangle.v1, triangle.v2];
    let mut flipped = vec![reversed.v0, reversed.v1, reversed.v2];
    original.sort_by(|a, b| a.partial_cmp(b).unwrap());
    flipped.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(original, flipped);
}

#[test]
fn triangle_map_converts_every_coordinate() {
    let triangle = Triangle {
        v0: [1, 2, 3],
        v1: [4, 5, 6],
        v2: [7, 8, 9],
    }
    .map(|c| c as f32 * 2.0);
    assert_eq!(triangle.v0, [2.0, 4.0, 6.0]);
    assert_eq!(triangle.v1, [8.0, 10.0, 12.0]);
    assert_eq!(triangle.v2, [14.0, 16.0, 18.0]);
}

#[test]
fn with_winding_counter_clockwise_matches_normal() {
    let up = Vector3D {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };
    let down = Vector3D {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    };
    let ccw = ccw_xy_triangle();
    let cw = ccw.reverse();

    for triangle in [ccw, cw] {
        let fixed = triangle.with_winding(Winding::CounterClockwise, up);
        assert!(orientation(&fixed, vec3(up)) > 0.0);
        let fixed = triangle.with_winding(Winding::CounterClockwise, down);
        assert!(orientation(&fixed, vec3(down)) > 0.0);
    }
    // Already correct triangles are returned unchanged.
    let same = ccw.with_winding(Winding::CounterClockwise, up);
    assert_eq!([same.v0, same.v1, same.v2], [ccw.v0, ccw.v1, ccw.v2]);
}

#[test]
fn with_winding_clockwise_opposes_normal() {
    let up = Vector3D {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };
    for triangle in [ccw_xy_triangle(), ccw_xy_triangle().reverse()] {
        let fixed = triangle.with_winding(Winding::Clockwise, up);
        assert!(orientation(&fixed, vec3(up)) < 0.0);
    }
}

#[test]
fn with_winding_uses_only_direction_of_normal() {
    let tilted = Vector3D {
        x: 3.0,
        y: -2.0,
        z: 0.1,
    };
    let fixed = ccw_xy_triangle()
        .reverse()
        .with_winding(Winding::CounterClockwise, tilted);
    assert!(orientation(&fixed, vec3(tilted)) > 0.0);
}

#[test]
fn with_winding_works_for_integer_triangles() {
    let triangle = Triangle {
        v0: [0, 0, 0],
        v1: [0, 1, 0],
        v2: [1, 0, 0],
    };
    let fixed = triangle.with_winding(Winding::CounterClockwise, Vector3D { x: 0, y: 0, z: 1 });
    let fixed = fixed.map(|c| c as f32);
    assert!(orientation(&fixed, [0.0, 0.0, 1.0]) > 0.0);
}

#[test]
fn triangle_normal_is_unit_length_and_right_handed() {
    close3(vec3(ccw_xy_triangle().normal()), [0.0, 0.0, 1.0]);
    close3(vec3(ccw_xy_triangle().reverse().normal()), [0.0, 0.0, -1.0]);

    let big = ccw_xy_triangle().map(|c| c * 1000.0);
    close3(vec3(big.normal()), [0.0, 0.0, 1.0]);

    let tiny = ccw_xy_triangle().map(|c| c * 1.0e-3);
    close3(vec3(tiny.normal()), [0.0, 0.0, 1.0]);

    let slanted = Triangle {
        v0: [1.0, 0.0, 0.0],
        v1: [0.0, 1.0, 0.0],
        v2: [0.0, 0.0, 1.0],
    };
    let k = 1.0 / 3.0_f32.sqrt();
    close3(vec3(slanted.normal()), [k, k, k]);
}

#[test]
fn degenerate_triangle_normal_is_zero_not_nan() {
    let collinear = Triangle {
        v0: [0.0, 0.0, 0.0],
        v1: [1.0, 1.0, 1.0],
        v2: [2.0, 2.0, 2.0],
    };
    let repeated = Triangle {
        v0: [1.0, 2.0, 3.0],
        v1: [1.0, 2.0, 3.0],
        v2: [1.0, 2.0, 3.0],
    };
    for triangle in [collinear, repeated] {
        let normal = vec3(triangle.normal());
        assert!(normal.iter().all(|c| c.is_finite()), "{normal:?}");
        close3(normal, [0.0, 0.0, 0.0]);
    }
}

#[test]
fn triangle_to_vertices_keeps_positions_and_shares_normal() {
    let triangle = Triangle {
        v0: [0.0, 0.0, 2.0],
        v1: [0.0, 3.0, 2.0],
        v2: [3.0, 0.0, 2.0],
    };
    let vertices = triangle.to_vertices();
    assert_eq!(vertices[0].position, triangle.v0);
    assert_eq!(vertices[1].position, triangle.v1);
    assert_eq!(vertices[2].position, triangle.v2);
    for vertex in vertices {
        close3(vertex.normal, [0.0, 0.0, -1.0]);
        assert_eq!(vertex.color, Vertex::DEFAULT_COLOR);
    }
}

// Quads

/// Corners of a voxel face, computed directly from the face definition.
fn expected_face_corners(normal: VoxelNormal, origin: [i32; 3], size: (i32, i32)) -> Vec<[i32; 3]> {
    let axis = normal.axis();
    let (u, v) = axis.plane();
    let mut base = origin;
    if normal.is_positive() {
        base[axis as usize] += 1;
    }
    let mut corners = Vec::new();
    for (du, dv) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
        let mut corner = base;
        corner[u as usize] += du * size.0;
        corner[v as usize] += dv * size.1;
        corners.push(corner);
    }
    corners.sort();
    corners
}

#[test]
fn voxel_face_covers_the_correct_face_and_faces_outwards() {
    for origin in [[0, 0, 0], [5, -3, 7], [-16, -1, -32]] {
        for size in [(1, 1), (3, 2), (1, 16), (16, 1)] {
            for normal in VoxelNormal::ALL {
                let quad = DiscreteQuad::voxel_face(
                    normal,
                    DiscretePoint::from_array(origin),
                    Vector2D {
                        x: size.0,
                        y: size.1,
                    },
                );
                let mut corners = vec![quad.v0, quad.v1, quad.v2, quad.v3];
                corners.sort();
                assert_eq!(
                    corners,
                    expected_face_corners(normal, origin, size),
                    "{normal:?} {origin:?} {size:?}"
                );

                let real: RealQuad = quad.map(|c| c as f32);
                let mut direction = axis_vector(normal.axis()).map(|c| c as f32);
                if !normal.is_positive() {
                    direction = direction.map(|c| -c);
                }
                close3(vec3(real.normal()), direction);
                // Both triangles of the quad (v0 v1 v2 and v0 v2 v3) face outwards too.
                let second = Triangle {
                    v0: real.v0,
                    v1: real.v2,
                    v2: real.v3,
                };
                close3(vec3(second.normal()), direction);
            }
        }
    }
}

#[test]
fn voxel_face_quad_is_a_rectangle_in_vertex_order() {
    // v0..v3 walk around the rectangle, so v0/v2 and v1/v3 are diagonals.
    for normal in VoxelNormal::ALL {
        let quad =
            DiscreteQuad::voxel_face(normal, Point { x: 1, y: 2, z: 3 }, Vector2D { x: 2, y: 3 })
                .map(|c| c as f32);
        let d1 = sub(quad.v2, quad.v0);
        let d2 = sub(quad.v3, quad.v1);
        close(dot(d1, d1), dot(d2, d2));
        let e1 = sub(quad.v1, quad.v0);
        let e2 = sub(quad.v2, quad.v1);
        close(dot(e1, e2), 0.0);
        // Opposite edges are equal and opposite.
        close3(sub(quad.v1, quad.v0), sub(quad.v2, quad.v3));
    }
}

#[test]
fn quad_map_and_triangles() {
    let quad = DiscreteQuad::voxel_face(
        VoxelNormal::Positive(Axis::Z),
        Point { x: 0, y: 0, z: 0 },
        Vector2D { x: 1, y: 1 },
    )
    .map(|c| c as f32 * 0.5);
    let [first, second] = quad.triangles();
    assert_eq!([first.v0, first.v1, first.v2], [quad.v0, quad.v1, quad.v2]);
    assert_eq!(
        [second.v0, second.v1, second.v2],
        [quad.v0, quad.v2, quad.v3]
    );
    for vertex in first.to_vertices().into_iter().chain(second.to_vertices()) {
        assert_eq!(vertex.position[2], 0.5);
        close3(vertex.normal, [0.0, 0.0, 1.0]);
        assert_eq!(vertex.color, Vertex::DEFAULT_COLOR);
    }
}

// Twist and Vertex

#[test]
fn identity_twist_has_no_translation_and_unit_quaternion() {
    let identity = RealTwist::identity();
    assert_eq!(vec3(identity.linear), [0.0, 0.0, 0.0]);
    assert_eq!(
        [
            identity.rotational.x,
            identity.rotational.y,
            identity.rotational.z,
            identity.rotational.w
        ],
        [0.0, 0.0, 0.0, 1.0]
    );
}

#[test]
fn vertex_conversions_use_default_colour_and_zero_normal() {
    let from_array = Vertex::from([1.0, 2.0, 3.0]);
    let from_point = Vertex::from(Point {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    });
    for vertex in [from_array, from_point] {
        assert_eq!(vertex.position, [1.0, 2.0, 3.0]);
        assert_eq!(vertex.color, [1.0, 1.0, 1.0]);
        assert_eq!(vertex.normal, [0.0, 0.0, 0.0]);
    }
    let lit = from_array.with_normal(Vector3D {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    });
    assert_eq!(lit.normal, [0.0, 1.0, 0.0]);
    assert_eq!(lit.position, from_array.position);
    assert_eq!(lit.color, from_array.color);
}

#[test]
fn gpu_types_have_packed_layouts() {
    assert_eq!(std::mem::size_of::<Vertex>(), 9 * 4);
    assert_eq!(std::mem::size_of::<RealTwist>(), 7 * 4);
    let vertex = Vertex {
        position: [1.0, 2.0, 3.0],
        color: [4.0, 5.0, 6.0],
        normal: [7.0, 8.0, 9.0],
    };
    let floats: &[f32] = bytemuck::cast_slice(std::slice::from_ref(&vertex));
    assert_eq!(floats, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    let twist = RealTwist::identity();
    let floats: &[f32] = bytemuck::cast_slice(std::slice::from_ref(&twist));
    assert_eq!(floats, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
}

fn voxel_group(key: [i32; 3], words: &[(usize, u64)]) -> VoxelGroup {
    let mut values = Box::new([0; VOXEL_GROUP_PAGES]);
    for &(index, word) in words {
        values[index] = word;
    }
    VoxelGroup {
        point: PointI {
            x: key[0],
            y: key[1],
            z: key[2],
            i: 0,
        },
        values,
    }
}

fn block(x: i32, y: i32, z: i32, i: u32) -> PointI {
    PointI { x, y, z, i }
}

#[test]
fn voxel_group_has_4096_pages() {
    assert_eq!(VOXEL_GROUP_PAGES, 4096);
}

#[test]
fn empty_voxel_group_has_no_blocks() {
    assert_eq!(voxel_group([0, 0, 0], &[]).blocks().count(), 0);
    assert_eq!(voxel_group([-3, 7, 1], &[]).blocks().count(), 0);
}

#[test]
fn first_voxel_of_a_group_is_at_its_origin() {
    let group = voxel_group([1, 2, 3], &[(0, 1)]);
    assert_eq!(group.blocks().collect::<Vec<_>>(), [block(64, 128, 192, 1)]);
}

#[test]
fn upper_half_of_a_page_is_the_block_above() {
    // Layers 2 and 3 of the page, bit 32 being voxel (0, 0, 2)
    let group = voxel_group([0, 0, 0], &[(0, 1 << 32 | 1 << 63)]);
    assert_eq!(
        group.blocks().collect::<Vec<_>>(),
        [block(0, 0, 2, 1 | 1 << 31)]
    );
    let group = voxel_group([0, 0, 0], &[(0, u64::MAX)]);
    assert_eq!(
        group.blocks().collect::<Vec<_>>(),
        [block(0, 0, 0, u32::MAX), block(0, 0, 2, u32::MAX)]
    );
    let group = voxel_group([0, 0, 0], &[(0, 0xFFFF_0000_0000_FFFF)]);
    assert_eq!(
        group.blocks().collect::<Vec<_>>(),
        [block(0, 0, 0, 0xFFFF), block(0, 0, 2, 0xFFFF_0000)]
    );
}

#[test]
fn pages_of_a_group_are_in_z_order() {
    // Word k holds k + 1, so the mask of a block tells the word it came from
    let words: Vec<(usize, u64)> = (0..VOXEL_GROUP_PAGES)
        .map(|index| (index, index as u64 + 1))
        .collect();
    let blocks: Vec<PointI> = voxel_group([-1, 2, 0], &words).blocks().collect();
    assert_eq!(blocks.len(), VOXEL_GROUP_PAGES);
    let mut pages = HashSet::new();
    for block in &blocks {
        let index = block.i as usize - 1;
        let page = [block.x + 64, block.y - 128, block.z].map(|c| {
            assert_eq!(c % 4, 0, "{block:?} is not aligned to a page");
            c / 4
        });
        assert!(
            page.iter().all(|c| (0..16).contains(c)),
            "{block:?} is outside its group"
        );
        assert_eq!(page_index(page), index, "word {index} is at page {page:?}");
        assert!(pages.insert(page), "page {page:?} appears twice");
    }
}

#[test]
fn voxels_of_a_group_match_the_voxelizer_format() {
    // Voxel (5, 9, 70): group (0, 0, 1), page (1, 2, 1) whose bits interleave
    // to 0b010_101 = 21, voxel (1, 1, 2) of the page, bit 1 + 4 + 32 = 37
    let group = voxel_group([0, 0, 1], &[(21, 1 << 37)]);
    let blocks: Vec<PointI> = group.blocks().collect();
    assert_eq!(blocks, [block(4, 8, 70, 1 << 5)]);
    assert_eq!(block_voxels(&blocks), BTreeSet::from([[5, 9, 70]]));

    // Voxel (-1, -1, -1): group (-1, -1, -1), page (15, 15, 15), last bit
    let group = voxel_group([-1, -1, -1], &[(4095, 1 << 63)]);
    let blocks: Vec<PointI> = group.blocks().collect();
    assert_eq!(blocks, [block(-4, -4, -2, 1 << 31)]);
    assert_eq!(block_voxels(&blocks), BTreeSet::from([[-1, -1, -1]]));
}

#[test]
fn i_of_the_group_key_is_ignored() {
    let mut group = voxel_group([2, -2, 0], &[(7, 0x10), (100, 1 << 40)]);
    let expected: Vec<PointI> = group.blocks().collect();
    for i in [1, 0xDEAD, u32::MAX] {
        group.point.i = i;
        assert_eq!(group.blocks().collect::<Vec<_>>(), expected);
    }
}

#[test]
fn extreme_group_keys_reach_the_ends_of_i32() {
    let max = i32::MAX >> 6;
    let group = voxel_group([max, max, max], &[(4095, 1 << 63)]);
    let blocks: Vec<PointI> = group.blocks().collect();
    assert_eq!(
        block_voxels(&blocks),
        BTreeSet::from([[i32::MAX, i32::MAX, i32::MAX]])
    );
    let min = i32::MIN >> 6;
    let group = voxel_group([min, min, min], &[(0, 1)]);
    let blocks: Vec<PointI> = group.blocks().collect();
    assert_eq!(blocks, [block(i32::MIN, i32::MIN, i32::MIN, 1)]);
}

#[test]
fn blocks_of_groups_round_trip_random_voxels() {
    for seed in 0..8 {
        let mut rng = Lcg::new(seed);
        let voxels: BTreeSet<[i32; 3]> = (0..rng.range(1, 3000))
            .map(|_| {
                [
                    rng.range(-150, 150),
                    rng.range(-150, 150),
                    rng.range(-70, 70),
                ]
            })
            .collect();
        let groups = voxel_groups(voxels.iter().copied());
        let blocks: Vec<PointI> = groups.iter().flat_map(VoxelGroup::blocks).collect();
        let mut origins = HashSet::new();
        for block in &blocks {
            assert_ne!(block.i, 0, "{block:?} is empty");
            assert!(
                block.x.rem_euclid(4) == 0
                    && block.y.rem_euclid(4) == 0
                    && block.z.rem_euclid(2) == 0,
                "{block:?} is not aligned to the 4x4x2 grid"
            );
            assert!(
                origins.insert([block.x, block.y, block.z]),
                "{block:?} appears twice"
            );
        }
        assert_eq!(block_voxels(&blocks), voxels, "seed {seed}");
    }
}
