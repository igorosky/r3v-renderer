mod common;

use std::collections::{HashMap, HashSet};

use common::Lcg;
use r3v_renderer::{
    Tesselator,
    geometry::{DiscretePoint, PointI, RealQuad, RealTriangle, TriangleChunk},
    tesselator::{GreedyMeshTesselator, GreedyMeshTesselatorConfiguration},
};

const BRICK: [i32; 3] = [4, 4, 2];
const FULL: u32 = u32::MAX;

type Cell = [i32; 3];
type Face = (Cell, usize);

const DIRECTIONS: [[i32; 3]; 6] = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0],
    [0, 0, -1],
];

fn brick(origin: Cell, mask: u32) -> PointI {
    PointI {
        x: origin[0],
        y: origin[1],
        z: origin[2],
        i: mask,
    }
}

fn bit(local: Cell) -> u32 {
    1 << (local[0] | local[1] << 2 | local[2] << 4)
}

#[derive(Default)]
struct World {
    bricks: HashMap<Cell, u32>,
}

impl World {
    fn apply(&mut self, delta: &[PointI]) {
        for point in delta {
            self.bricks.insert([point.x, point.y, point.z], point.i);
        }
    }

    fn voxels(&self) -> HashSet<Cell> {
        let mut voxels = HashSet::new();
        for (origin, mask) in &self.bricks {
            for z in 0..BRICK[2] {
                for y in 0..BRICK[1] {
                    for x in 0..BRICK[0] {
                        if mask & bit([x, y, z]) != 0 {
                            voxels.insert([origin[0] + x, origin[1] + y, origin[2] + z]);
                        }
                    }
                }
            }
        }
        voxels
    }

    fn exposed_faces(&self) -> HashSet<Face> {
        let voxels = self.voxels();
        let mut faces = HashSet::new();
        for voxel in &voxels {
            for (direction, d) in DIRECTIONS.iter().enumerate() {
                let neighbour = [voxel[0] + d[0], voxel[1] + d[1], voxel[2] + d[2]];
                if !voxels.contains(&neighbour) {
                    faces.insert((*voxel, direction));
                }
            }
        }
        faces
    }
}

struct Mesh {
    voxel_size: f32,
    chunks: HashMap<DiscretePoint, Vec<RealQuad>>,
}

impl Mesh {
    fn new(voxel_size: f32) -> Self {
        Self {
            voxel_size,
            chunks: HashMap::new(),
        }
    }

    fn apply(&mut self, delta: Vec<TriangleChunk>) {
        let mut seen = HashSet::new();
        for chunk in delta {
            assert!(
                seen.insert(chunk.index),
                "chunk {:?} appears twice in one delta",
                chunk.index
            );
            if chunk.triangles.is_empty() {
                self.chunks.remove(&chunk.index);
            } else {
                self.chunks
                    .insert(chunk.index, quads_from_triangles(&chunk.triangles));
            }
        }
    }

    fn quad_count(&self) -> usize {
        self.chunks.values().map(Vec::len).sum()
    }

    fn to_grid(&self, value: f32) -> i32 {
        let scaled = value / self.voxel_size;
        let rounded = scaled.round();
        assert!(
            (scaled - rounded).abs() < 1.0e-3,
            "coordinate {value} is not on the voxel grid of size {}",
            self.voxel_size
        );
        rounded as i32
    }

    fn faces(&self) -> HashSet<Face> {
        let mut faces = HashSet::new();
        for quad in self.chunks.values().flatten() {
            let corners = [quad.v0, quad.v1, quad.v2, quad.v3].map(|v| v.map(|c| self.to_grid(c)));
            let direction = quad_direction(quad);
            let axis = direction % 3;
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            let plane = corners[0][axis];
            assert!(
                corners.iter().all(|c| c[axis] == plane),
                "quad is not planar: {corners:?}"
            );
            let min = |a: usize| corners.iter().map(|c| c[a]).min().unwrap();
            let max = |a: usize| corners.iter().map(|c| c[a]).max().unwrap();
            let (u0, u1, v0, v1) = (min(u), max(u), min(v), max(v));
            assert!(u1 > u0 && v1 > v0, "degenerate quad: {corners:?}");
            let mut expected_corners = Vec::new();
            for (cu, cv) in [(u0, v0), (u1, v0), (u0, v1), (u1, v1)] {
                let mut c = [0; 3];
                c[axis] = plane;
                c[u] = cu;
                c[v] = cv;
                expected_corners.push(c);
            }
            let mut actual_corners = corners.to_vec();
            actual_corners.sort();
            expected_corners.sort();
            assert_eq!(actual_corners, expected_corners, "quad is not a rectangle");

            let layer = if direction < 3 { plane - 1 } else { plane };
            for cu in u0..u1 {
                for cv in v0..v1 {
                    let mut cell = [0; 3];
                    cell[axis] = layer;
                    cell[u] = cu;
                    cell[v] = cv;
                    assert!(
                        faces.insert((cell, direction)),
                        "face {cell:?} dir {direction} covered by more than one quad"
                    );
                }
            }
        }
        faces
    }
}

/// Greedy meshing emits each quad as the triangle pair from `RealQuad::triangles`.
fn quads_from_triangles(triangles: &[RealTriangle]) -> Vec<RealQuad> {
    let (pairs, rest) = triangles.as_chunks::<2>();
    assert!(rest.is_empty(), "odd triangle count {}", triangles.len());
    pairs
        .iter()
        .map(|[first, second]| {
            assert!(
                first.v0 == second.v0 && first.v2 == second.v1,
                "triangles do not form a quad: {:?} {:?} {:?} / {:?} {:?} {:?}",
                first.v0,
                first.v1,
                first.v2,
                second.v0,
                second.v1,
                second.v2
            );
            RealQuad {
                v0: first.v0,
                v1: first.v1,
                v2: first.v2,
                v3: second.v2,
            }
        })
        .collect()
}

fn quad_direction(quad: &RealQuad) -> usize {
    let normal = |t: RealTriangle| {
        let n = t.normal();
        [n.x, n.y, n.z]
    };
    let [first, second] = quad.triangles().map(normal);
    let direction = DIRECTIONS
        .iter()
        .position(|d| (0..3).all(|i| (first[i] - d[i] as f32).abs() < 1.0e-4))
        .unwrap_or_else(|| panic!("quad normal {first:?} is not axis aligned"));
    assert!(
        (0..3).all(|i| (second[i] - DIRECTIONS[direction][i] as f32).abs() < 1.0e-4),
        "quad triangles have different windings: {first:?} vs {second:?}"
    );
    direction
}

struct Harness {
    tesselator: GreedyMeshTesselator,
    world: World,
    mesh: Mesh,
}

impl Harness {
    fn new(voxel_size: f32) -> Self {
        let mut tesselator = GreedyMeshTesselator::new();
        tesselator.configure(GreedyMeshTesselatorConfiguration { voxel_size });
        Self {
            tesselator,
            world: World::default(),
            mesh: Mesh::new(voxel_size),
        }
    }

    #[track_caller]
    fn step(&mut self, delta: &[PointI]) {
        self.world.apply(delta);
        self.tesselator.update_input_geometry(&delta.to_vec());
        if let Some(output) = self.tesselator.get_output_geometry_delta() {
            self.mesh.apply(output);
        }
        assert!(self.tesselator.get_output_geometry_delta().is_none());
        self.check();
    }

    #[track_caller]
    fn check(&self) {
        let expected = self.world.exposed_faces();
        let actual = self.mesh.faces();
        let missing: Vec<_> = expected.difference(&actual).take(5).collect();
        let extra: Vec<_> = actual.difference(&expected).take(5).collect();
        assert!(
            missing.is_empty() && extra.is_empty(),
            "mesh differs from reference: missing {missing:?}, extra {extra:?}"
        );
    }

    #[track_caller]
    fn assert_box_is_merged(&self) {
        assert!(
            self.mesh.quad_count() <= 6 * self.mesh.chunks.len(),
            "{} quads for {} chunks",
            self.mesh.quad_count(),
            self.mesh.chunks.len()
        );
    }
}

fn box_bricks(from: Cell, to: Cell, mask: u32) -> Vec<PointI> {
    let mut bricks = Vec::new();
    let mut z = from[2];
    while z < to[2] {
        let mut y = from[1];
        while y < to[1] {
            let mut x = from[0];
            while x < to[0] {
                bricks.push(brick([x, y, z], mask));
                x += BRICK[0];
            }
            y += BRICK[1];
        }
        z += BRICK[2];
    }
    bricks
}

#[test]
fn single_voxel_has_six_unit_faces() {
    for origin in [[0, 0, 0], [8, -4, 6], [-4, -4, -2], [1000, 2000, -3000]] {
        for local in [[0, 0, 0], [3, 3, 1], [1, 2, 0]] {
            let mut harness = Harness::new(1.0);
            harness.step(&[brick(origin, bit(local))]);
            assert_eq!(harness.mesh.quad_count(), 6);
        }
    }
}

#[test]
fn full_brick_is_six_quads() {
    for origin in [[0, 0, 0], [-4, -4, -2], [12, 12, 14]] {
        let mut harness = Harness::new(1.0);
        harness.step(&[brick(origin, FULL)]);
        assert_eq!(harness.mesh.quad_count(), 6);
    }
}

#[test]
fn adjacent_voxels_share_no_faces() {
    let mut harness = Harness::new(1.0);
    harness.step(&[brick([0, 0, 0], bit([0, 0, 0]) | bit([1, 0, 0]))]);
    assert_eq!(harness.world.exposed_faces().len(), 10);

    let mut harness = Harness::new(1.0);
    harness.step(&[brick(
        [0, 0, 0],
        bit([0, 0, 0]) | bit([1, 0, 0]) | bit([2, 0, 0]) | bit([0, 1, 0]) | bit([0, 2, 0]),
    )]);
}

#[test]
fn diagonal_voxels_keep_all_faces() {
    let mut harness = Harness::new(1.0);
    harness.step(&[brick([0, 0, 0], bit([0, 0, 0]) | bit([1, 1, 1]))]);
    assert_eq!(harness.mesh.faces().len(), 12);
}

#[test]
fn checkerboard_brick() {
    let mut mask = 0;
    for z in 0..2 {
        for y in 0..4 {
            for x in 0..4 {
                if (x + y + z) % 2 == 0 {
                    mask |= bit([x, y, z]);
                }
            }
        }
    }
    let mut harness = Harness::new(1.0);
    harness.step(&[brick([0, 0, 0], mask)]);
    harness.step(&[brick([0, 0, 0], !mask)]);
}

#[test]
fn adjacent_bricks_cull_shared_faces_in_every_direction() {
    for offset in [
        [4, 0, 0],
        [-4, 0, 0],
        [0, 4, 0],
        [0, -4, 0],
        [0, 0, 2],
        [0, 0, -2],
    ] {
        let mut harness = Harness::new(1.0);
        harness.step(&[brick([0, 0, 0], FULL), brick(offset, FULL)]);
        harness.assert_box_is_merged();
    }
}

#[test]
fn bricks_far_apart_and_across_chunk_boundaries() {
    for boundary in [-64, -32, -16, -8, 0, 8, 16, 32, 64] {
        for axis in 0..3 {
            let mut low = [0, 0, 0];
            let mut high = [0, 0, 0];
            low[axis] = boundary - BRICK[axis];
            high[axis] = boundary;
            let mut harness = Harness::new(1.0);
            harness.step(&[brick(low, FULL), brick(high, FULL)]);
            harness.assert_box_is_merged();
            harness.step(&[brick(high, 0)]);
            assert_eq!(harness.mesh.quad_count(), 6);
            harness.step(&[brick(high, FULL)]);
            harness.step(&[brick(low, 0)]);
            assert_eq!(harness.mesh.quad_count(), 6);
        }
    }
}

#[test]
fn partial_bricks_across_boundaries() {
    for axis in 0..3 {
        let mut low = [0, 0, 0];
        low[axis] = -BRICK[axis];
        let mut low_mask = 0;
        let mut high_mask = 0;
        for z in 0..2 {
            for y in 0..4 {
                for x in 0..4 {
                    let local = [x, y, z];
                    if local[axis] == BRICK[axis] - 1 && (x + y) % 2 == 0 {
                        low_mask |= bit(local);
                    }
                    if local[axis] == 0 && (y + z) % 2 == 0 {
                        high_mask |= bit(local);
                    }
                }
            }
        }
        let mut harness = Harness::new(1.0);
        harness.step(&[brick(low, low_mask), brick([0, 0, 0], high_mask)]);
    }
}

#[test]
fn large_solid_box_is_merged_per_chunk() {
    for (from, to) in [
        ([0, 0, 0], [16, 16, 16]),
        ([-16, -16, -16], [16, 16, 16]),
        ([-8, 4, -6], [40, 20, 30]),
    ] {
        let mut harness = Harness::new(1.0);
        harness.step(&box_bricks(from, to, FULL));
        harness.assert_box_is_merged();
    }
}

#[test]
fn hollow_box_has_inner_and_outer_surfaces() {
    let mut harness = Harness::new(1.0);
    let mut bricks = box_bricks([0, 0, 0], [24, 24, 24], FULL);
    for brick in &mut bricks {
        let inside = [brick.x, brick.y, brick.z]
            .iter()
            .zip([24, 24, 24])
            .zip(BRICK)
            .all(|((&c, size), step)| c >= step && c < size - step);
        if inside {
            brick.i = 0;
        }
    }
    harness.step(&bricks);
}

#[test]
fn coordinates_are_scaled_by_voxel_size() {
    for voxel_size in [0.2, 1.0, 2.5, 0.05] {
        let mut harness = Harness::new(voxel_size);
        harness.step(&[brick([4, -8, 2], FULL)]);
        let mut min = [f32::MAX; 3];
        let mut max = [f32::MIN; 3];
        for quad in harness.mesh.chunks.values().flatten() {
            for v in [quad.v0, quad.v1, quad.v2, quad.v3] {
                for a in 0..3 {
                    min[a] = min[a].min(v[a]);
                    max[a] = max[a].max(v[a]);
                }
            }
        }
        let expected_min = [4.0, -8.0, 2.0].map(|c| c * voxel_size);
        let expected_max = [8.0, -4.0, 4.0].map(|c| c * voxel_size);
        for a in 0..3 {
            assert!((min[a] - expected_min[a]).abs() < 1.0e-4, "{min:?}");
            assert!((max[a] - expected_max[a]).abs() < 1.0e-4, "{max:?}");
        }
    }
}

#[test]
fn incremental_edits_keep_mesh_in_sync() {
    let mut harness = Harness::new(1.0);
    harness.step(&box_bricks([0, 0, 0], [32, 8, 8], FULL));
    // Carve a tunnel.
    harness.step(&box_bricks([8, 0, 2], [24, 4, 4], 0));
    // Partially refill it.
    harness.step(&[brick([12, 0, 2], bit([0, 0, 0]) | bit([3, 3, 1]))]);
    // Change a brick at a corner.
    harness.step(&[brick([28, 4, 6], 0x0F0F_0F0F)]);
    // Clear everything.
    harness.step(&box_bricks([0, 0, 0], [32, 8, 8], 0));
    assert_eq!(harness.mesh.quad_count(), 0);
    assert!(harness.mesh.chunks.is_empty());
}

#[test]
fn clearing_last_voxel_removes_its_chunk() {
    let mut harness = Harness::new(1.0);
    harness.step(&[brick([-4, -4, -2], bit([3, 3, 1]))]);
    assert_eq!(harness.mesh.chunks.len(), 1);
    harness.step(&[brick([-4, -4, -2], 0)]);
    assert!(harness.mesh.chunks.is_empty());
}

#[test]
fn randomized_edits_match_reference() {
    for seed in 0..6 {
        let mut rng = Lcg::new(seed);
        let mut harness = Harness::new(0.5);
        for _ in 0..25 {
            let count = rng.range(1, 40);
            let delta: Vec<PointI> = (0..count)
                .map(|_| {
                    let origin = [
                        rng.range(-10, 10) * BRICK[0],
                        rng.range(-6, 6) * BRICK[1],
                        rng.range(-12, 12) * BRICK[2],
                    ];
                    let mask = match rng.range(0, 4) {
                        0 => 0,
                        1 => FULL,
                        _ => rng.next_u32() & rng.next_u32(),
                    };
                    brick(origin, mask)
                })
                .collect();
            harness.step(&delta);
        }
    }
}

#[test]
fn no_delta_without_changes() {
    let mut tesselator = GreedyMeshTesselator::new();
    tesselator.configure(GreedyMeshTesselatorConfiguration { voxel_size: 1.0 });
    assert!(tesselator.get_output_geometry_delta().is_none());

    tesselator.update_input_geometry(&vec![]);
    assert!(tesselator.get_output_geometry_delta().is_none());

    // Clearing an empty brick changes nothing.
    tesselator.update_input_geometry(&vec![brick([0, 0, 0], 0)]);
    assert!(tesselator.get_output_geometry_delta().is_none());

    tesselator.update_input_geometry(&vec![brick([0, 0, 0], FULL)]);
    assert!(tesselator.get_output_geometry_delta().is_some());
    assert!(tesselator.get_output_geometry_delta().is_none());

    // Sending the same content again changes nothing.
    tesselator.update_input_geometry(&vec![brick([0, 0, 0], FULL)]);
    assert!(tesselator.get_output_geometry_delta().is_none());
}

#[test]
fn several_updates_before_reading_are_combined() {
    let mut harness = Harness::new(1.0);
    let steps = [
        vec![brick([0, 0, 0], FULL)],
        vec![brick([4, 0, 0], FULL), brick([0, 0, 0], bit([1, 1, 1]))],
        vec![brick([-4, 0, 0], 0xFFFF)],
    ];
    for delta in &steps {
        harness.world.apply(delta);
        harness.tesselator.update_input_geometry(delta);
    }
    harness
        .mesh
        .apply(harness.tesselator.get_output_geometry_delta().unwrap());
    harness.check();
}

#[test]
fn last_update_of_a_brick_in_one_batch_wins() {
    let mut harness = Harness::new(1.0);
    harness.step(&[
        brick([0, 0, 0], FULL),
        brick([0, 0, 0], 0),
        brick([0, 0, 0], bit([2, 2, 0])),
    ]);
    assert_eq!(harness.mesh.quad_count(), 6);

    let mut harness = Harness::new(1.0);
    harness.step(&[brick([0, 0, 0], FULL), brick([0, 0, 0], 0)]);
    assert_eq!(harness.mesh.quad_count(), 0);
}

#[test]
fn reconfiguring_voxel_size_applies_to_later_output() {
    let mut tesselator = GreedyMeshTesselator::default();
    tesselator.configure(GreedyMeshTesselatorConfiguration { voxel_size: 1.0 });
    tesselator.configure(GreedyMeshTesselatorConfiguration { voxel_size: 3.0 });
    tesselator.update_input_geometry(&vec![brick([0, 0, 0], bit([1, 0, 0]))]);
    let mut mesh = Mesh::new(3.0);
    mesh.apply(tesselator.get_output_geometry_delta().unwrap());
    let mut world = World::default();
    world.apply(&[brick([0, 0, 0], bit([1, 0, 0]))]);
    assert_eq!(mesh.faces(), world.exposed_faces());
}

#[test]
#[should_panic]
fn update_before_configure_panics() {
    let mut tesselator = GreedyMeshTesselator::new();
    tesselator.update_input_geometry(&vec![brick([0, 0, 0], FULL)]);
}

#[test]
#[should_panic]
fn output_before_configure_panics() {
    let mut tesselator = GreedyMeshTesselator::new();
    tesselator.get_output_geometry_delta();
}

fn marching_cubes_tesselator(mask: u32) -> GreedyMeshTesselator {
    let mut tesselator = GreedyMeshTesselator::new();
    tesselator.configure(GreedyMeshTesselatorConfiguration { voxel_size: 1.0 });
    tesselator.update_input_geometry(&vec![brick([0, 0, 0], mask)]);
    tesselator
}

fn triangle_count(geometry: &[TriangleChunk]) -> usize {
    geometry.iter().map(|chunk| chunk.triangles.len()).sum()
}

#[test]
fn marching_cubes_can_be_toggled_and_rebuilds_geometry() {
    let mut tesselator = marching_cubes_tesselator(bit([1, 1, 0]));
    let greedy = tesselator.get_output_geometry_delta().unwrap();
    let greedy_triangle_count = triangle_count(&greedy);

    tesselator.set_marching_cubes_enabled(true);
    assert!(tesselator.marching_cubes_enabled());
    let smoothed = tesselator.get_output_geometry_delta().unwrap();
    let smoothed_triangle_count = triangle_count(&smoothed);
    assert!(smoothed_triangle_count > 0);
    assert_ne!(smoothed_triangle_count, greedy_triangle_count);
    assert!(tesselator.get_output_geometry_delta().is_none());

    tesselator.set_marching_cubes_enabled(false);
    assert!(!tesselator.marching_cubes_enabled());
    assert!(tesselator.get_output_geometry_delta().is_some());
}

#[test]
fn marching_cubes_interpolates_surface_vertices() {
    let mut tesselator = marching_cubes_tesselator(bit([0, 0, 0]));
    tesselator.set_marching_cubes_enabled(true);
    let geometry = tesselator.get_output_geometry_delta().unwrap();

    let has_fractional_vertex = geometry
        .iter()
        .flat_map(|chunk| &chunk.triangles)
        .flat_map(|triangle| [&triangle.v0, &triangle.v1, &triangle.v2])
        .flatten()
        .any(|coordinate| (coordinate.fract()).abs() > f32::EPSILON);
    assert!(has_fractional_vertex);
}

#[test]
fn marching_cubes_is_scaled_by_fractional_voxel_size() {
    let voxel_size = 0.2;
    let mut tesselator = GreedyMeshTesselator::new();
    tesselator.configure(GreedyMeshTesselatorConfiguration { voxel_size });
    tesselator.update_input_geometry(&vec![brick([4, -8, 2], FULL)]);
    tesselator.set_marching_cubes_enabled(true);
    let geometry = tesselator.get_output_geometry_delta().unwrap();

    let mut min = [f32::MAX; 3];
    let mut max = [f32::MIN; 3];
    for triangle in geometry.iter().flat_map(|chunk| &chunk.triangles) {
        for v in [triangle.v0, triangle.v1, triangle.v2] {
            for a in 0..3 {
                min[a] = min[a].min(v[a]);
                max[a] = max[a].max(v[a]);
            }
        }
    }
    // The smoothed surface stays within one voxel of the brick's bounds.
    let lower = [3.0, -9.0, 1.0].map(|c| c * voxel_size);
    let upper = [9.0, -3.0, 5.0].map(|c| c * voxel_size);
    for a in 0..3 {
        assert!(
            max[a] > min[a],
            "geometry collapsed on axis {a}: {min:?} {max:?}"
        );
        assert!(min[a] >= lower[a] - 1.0e-4, "{min:?}");
        assert!(max[a] <= upper[a] + 1.0e-4, "{max:?}");
    }
}

#[test]
fn marching_cubes_handles_all_cube_corner_patterns() {
    for corner_pattern in 0u32..=u8::MAX as u32 {
        let mut mask = 0;
        for z in 0..2 {
            for y in 0..2 {
                for x in 0..2 {
                    let corner = x + y * 2 + z * 4;
                    if corner_pattern & (1 << corner) != 0 {
                        mask |= bit([x, y, z]);
                    }
                }
            }
        }
        let mut tesselator = marching_cubes_tesselator(mask);
        tesselator.set_marching_cubes_enabled(true);
        let geometry = tesselator.get_output_geometry_delta().unwrap_or_default();
        assert!(
            geometry
                .iter()
                .flat_map(|chunk| &chunk.triangles)
                .flat_map(|triangle| [&triangle.v0, &triangle.v1, &triangle.v2])
                .flatten()
                .all(|coordinate| coordinate.is_finite()),
            "non-finite vertex for input pattern {corner_pattern:#04x}"
        );
        if corner_pattern != 0 {
            assert!(
                triangle_count(&geometry) > 0,
                "terrain pattern {corner_pattern:#04x} produced no geometry"
            );
        }
    }
}

#[test]
fn marching_cubes_updates_diagonal_chunks_after_boundary_changes() {
    let mut tesselator = marching_cubes_tesselator(bit([1, 1, 0]));
    tesselator.set_marching_cubes_enabled(true);
    tesselator.get_output_geometry_delta().unwrap();

    tesselator.update_input_geometry(&vec![brick([0, 0, 0], bit([1, 1, 0]) | bit([0, 0, 0]))]);
    let delta = tesselator.get_output_geometry_delta().unwrap();
    assert!(delta.iter().any(|chunk| chunk.index
        == DiscretePoint {
            x: -16,
            y: -16,
            z: 0
        }));
}
