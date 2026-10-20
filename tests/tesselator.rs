use r3v_renderer::{
    Tesselator,
    geometry::{DiscretePoint, Point, PointI},
    tesselator::{GreedyMeshTesselator, GreedyMeshTesselatorConfiguration},
};

fn tesselator_with_voxels(mask: u32) -> GreedyMeshTesselator {
    let mut tesselator = GreedyMeshTesselator::new();
    tesselator.configure(GreedyMeshTesselatorConfiguration {
        voxel_size: 1,
        marching_cubes_enabled: false,
    });
    let input = vec![PointI {
        x: 0,
        y: 0,
        z: 0,
        i: mask,
    }];
    tesselator.update_input_geometry(&input);
    tesselator
}

fn voxel_bit(x: usize, y: usize, z: usize) -> u32 {
    1 << PointI::voxel_bit(Point { x, y, z })
}

fn triangle_count(geometry: &[r3v_renderer::geometry::TriangleChunk]) -> usize {
    geometry.iter().map(|chunk| chunk.triangles.len()).sum()
}

#[test]
fn marching_cubes_can_be_toggled_and_rebuilds_geometry() {
    let mut tesselator = tesselator_with_voxels(voxel_bit(1, 1, 0));
    let greedy = tesselator.get_output_geometry_delta().unwrap();
    let greedy_triangle_count = triangle_count(&greedy);

    tesselator.configure(GreedyMeshTesselatorConfiguration {
        voxel_size: 1,
        marching_cubes_enabled: true,
    });
    assert!(tesselator.marching_cubes_enabled());
    let smoothed = tesselator.get_output_geometry_delta().unwrap();
    let smoothed_triangle_count = triangle_count(&smoothed);
    assert!(smoothed_triangle_count > 0);
    assert_ne!(smoothed_triangle_count, greedy_triangle_count);
    assert!(tesselator.get_output_geometry_delta().is_none());

    tesselator.configure(GreedyMeshTesselatorConfiguration {
        voxel_size: 1,
        marching_cubes_enabled: false,
    });
    assert!(!tesselator.marching_cubes_enabled());
    assert!(tesselator.get_output_geometry_delta().is_some());
}

#[test]
fn marching_cubes_interpolates_surface_vertices() {
    let mut tesselator = tesselator_with_voxels(voxel_bit(0, 0, 0));
    tesselator.configure(GreedyMeshTesselatorConfiguration {
        voxel_size: 1,
        marching_cubes_enabled: true,
    });
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
fn marching_cubes_handles_all_cube_corner_patterns() {
    for corner_pattern in 0u32..=u8::MAX as u32 {
        let mut mask = 0;
        for z in 0..2 {
            for y in 0..2 {
                for x in 0..2 {
                    let corner = x + y * 2 + z * 4;
                    if corner_pattern & (1 << corner) != 0 {
                        mask |= voxel_bit(x, y, z);
                    }
                }
            }
        }
        let mut tesselator = tesselator_with_voxels(mask);
        tesselator.configure(GreedyMeshTesselatorConfiguration {
            voxel_size: 1,
            marching_cubes_enabled: true,
        });
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
    let mut tesselator = tesselator_with_voxels(voxel_bit(1, 1, 0));
    tesselator.configure(GreedyMeshTesselatorConfiguration {
        voxel_size: 1,
        marching_cubes_enabled: true,
    });
    tesselator.get_output_geometry_delta().unwrap();

    let updated_mask = voxel_bit(1, 1, 0) | voxel_bit(0, 0, 0);
    let input = vec![PointI {
        x: 0,
        y: 0,
        z: 0,
        i: updated_mask,
    }];
    tesselator.update_input_geometry(&input);
    let delta = tesselator.get_output_geometry_delta().unwrap();
    assert!(delta.iter().any(|chunk| chunk.index
        == DiscretePoint {
            x: -16,
            y: -16,
            z: 0
        }));
}
