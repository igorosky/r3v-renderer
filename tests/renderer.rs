use std::sync::{Mutex, MutexGuard};

use cgmath::Point3;
use r3v_renderer::{
    Camera, Renderer, Tesselator,
    camera::OrbitCamera,
    geometry::{PointI, RealTriangle},
    renderer::{
        DEFAULT_BACKGROUND_COLOR, DEFAULT_COLOR_DEPTH_RANGE, TerrainColoring, WgpuAccelerated,
        WgpuAcceleratedConfiguration,
    },
    stl::StlMesh,
    tesselator::{CpuTesselator, CpuTesselatorConfiguration, CpuTesselatorContext},
};

const SIZE: u32 = 32;
const BLOCK_TOP: f32 = 8.0;

fn gpu_lock() -> MutexGuard<'static, ()> {
    static LOCK: Mutex<()> = Mutex::new(());
    LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn configuration() -> WgpuAcceleratedConfiguration {
    WgpuAcceleratedConfiguration {
        width: SIZE,
        height: SIZE,
        background_color: DEFAULT_BACKGROUND_COLOR,
        color_depth_range: DEFAULT_COLOR_DEPTH_RANGE,
        terrain_coloring: TerrainColoring::default(),
    }
}

fn renderer() -> WgpuAccelerated {
    let mut renderer = WgpuAccelerated::new().expect("no GPU");
    renderer.configure(configuration());
    renderer
}

fn add_block(renderer: &mut WgpuAccelerated) {
    let mut tesselator = CpuTesselator::new();
    tesselator.configure(CpuTesselatorConfiguration {
        voxel_size: 1.0,
        marching_cubes_enabled: false,
    });
    tesselator.update_input_geometry(&vec![PointI {
        x: 0,
        y: BLOCK_TOP as i32 - 4,
        z: 0,
        i: u32::MAX,
    }]);
    let context = CpuTesselatorContext {
        camera_frustum: None,
    };
    let delta = tesselator.get_output_geometry_delta(&context);
    renderer.update_geometry(delta.expect("a delta"));
}

fn scene() -> WgpuAccelerated {
    let mut renderer = renderer();
    add_block(&mut renderer);
    renderer
}

fn render(renderer: &mut WgpuAccelerated) -> Vec<u8> {
    let camera = OrbitCamera::new(Point3::new(2.0, BLOCK_TOP, 1.0), 10.0, 0.0, 89.0);
    let state = camera.get_state(1.0);
    let mut frame = vec![0; (SIZE * SIZE * 4) as usize];
    renderer.render(&state, &mut frame);
    renderer.render(&state, &mut frame);
    frame
}

fn middle(frame: &[u8]) -> [u8; 3] {
    let offset = (SIZE as usize * (SIZE as usize / 2) + SIZE as usize / 2) * 4;
    [frame[offset], frame[offset + 1], frame[offset + 2]]
}

#[track_caller]
fn assert_close(a: [u8; 3], b: [u8; 3]) {
    assert!(
        a.iter().zip(&b).all(|(a, b)| a.abs_diff(*b) <= 1),
        "{a:?} and {b:?}"
    );
}

fn by_height(neutral_height: f32) -> TerrainColoring {
    TerrainColoring {
        height_coloring_enabled: true,
        height_offset: neutral_height,
        ..TerrainColoring::default()
    }
}

fn from_robot(robot_height: f32) -> TerrainColoring {
    TerrainColoring {
        relative_to_robot: true,
        robot_height,
        ..by_height(0.0)
    }
}

fn set_coloring(renderer: &mut WgpuAccelerated, terrain_coloring: TerrainColoring) {
    renderer.configure(WgpuAcceleratedConfiguration {
        terrain_coloring,
        ..configuration()
    });
}

#[test]
fn depth_stays_the_default() {
    let _gpu = gpu_lock();
    let mut renderer = scene();
    let depth = render(&mut renderer);
    assert_ne!(middle(&depth), [0; 3], "the block is not in view");

    set_coloring(&mut renderer, by_height(0.0));
    assert_ne!(render(&mut renderer), depth);

    set_coloring(&mut renderer, TerrainColoring::default());
    assert_eq!(render(&mut renderer), depth);
}

#[test]
fn neutral_height_separates_depths_from_heights() {
    let _gpu = gpu_lock();
    let mut renderer = scene();

    set_coloring(&mut renderer, by_height(BLOCK_TOP - 5.0));
    let [b, _, r] = middle(&render(&mut renderer));
    assert!(r > b, "above the neutral height: {r} red, {b} blue");

    set_coloring(&mut renderer, by_height(BLOCK_TOP + 5.0));
    let [b, _, r] = middle(&render(&mut renderer));
    assert!(b > r, "below the neutral height: {b} blue, {r} red");
}

#[test]
fn neutral_height_is_the_middle_of_the_colormap_at_any_scale() {
    let _gpu = gpu_lock();
    let mut renderer = scene();
    set_coloring(&mut renderer, by_height(BLOCK_TOP));
    let neutral = middle(&render(&mut renderer));
    let [b, g, r] = neutral;
    assert!(g > b && g > r, "{neutral:?}");

    set_coloring(
        &mut renderer,
        TerrainColoring {
            height_knee: 0.01,
            height_gain: 2.0,
            ..by_height(BLOCK_TOP)
        },
    );
    assert_close(middle(&render(&mut renderer)), neutral);
}

#[test]
fn robot_height_only_counts_relative_to_the_robot() {
    let _gpu = gpu_lock();
    let mut renderer = scene();
    set_coloring(&mut renderer, by_height(0.0));
    let frame = render(&mut renderer);

    set_coloring(
        &mut renderer,
        TerrainColoring {
            robot_height: BLOCK_TOP,
            ..by_height(0.0)
        },
    );
    assert_eq!(render(&mut renderer), frame);
}

#[test]
fn colors_follow_the_robot() {
    let _gpu = gpu_lock();
    let mut renderer = scene();
    set_coloring(&mut renderer, from_robot(BLOCK_TOP + 5.0));
    let [b, _, r] = middle(&render(&mut renderer));
    assert!(b > r, "below the robot: {b} blue, {r} red");

    set_coloring(&mut renderer, from_robot(BLOCK_TOP - 5.0));
    let [b, _, r] = middle(&render(&mut renderer));
    assert!(r > b, "above the robot: {r} red, {b} blue");
}

#[test]
fn offset_adds_to_the_robot() {
    let _gpu = gpu_lock();
    let mut renderer = scene();
    set_coloring(
        &mut renderer,
        TerrainColoring {
            height_offset: 2.0,
            ..from_robot(3.0)
        },
    );
    let frame = render(&mut renderer);

    set_coloring(&mut renderer, by_height(5.0));
    assert_eq!(render(&mut renderer), frame);
}

#[test]
fn function_parameters_change_the_color() {
    let _gpu = gpu_lock();
    let mut renderer = scene();
    let base = by_height(0.0);
    set_coloring(&mut renderer, base);
    let frame = render(&mut renderer);
    for coloring in [
        TerrainColoring {
            height_knee: 4.0,
            ..base
        },
        TerrainColoring {
            height_gain: 0.1,
            ..base
        },
        TerrainColoring {
            height_bands: 4,
            ..base
        },
    ] {
        set_coloring(&mut renderer, coloring);
        assert_ne!(
            middle(&render(&mut renderer)),
            middle(&frame),
            "{coloring:?}"
        );
    }
}

#[test]
fn bands_are_flat() {
    let _gpu = gpu_lock();
    let mut renderer = scene();
    let coloring = TerrainColoring {
        height_bands: 2,
        ..by_height(0.0)
    };
    set_coloring(&mut renderer, coloring);
    let at_zero = middle(&render(&mut renderer));
    set_coloring(
        &mut renderer,
        TerrainColoring {
            height_offset: 1.0,
            ..coloring
        },
    );
    assert_eq!(middle(&render(&mut renderer)), at_zero);
}

#[test]
fn depth_range_leaves_height_coloring_alone() {
    let _gpu = gpu_lock();
    let mut renderer = scene();
    set_coloring(&mut renderer, by_height(0.0));
    let frame = render(&mut renderer);

    renderer.configure(WgpuAcceleratedConfiguration {
        color_depth_range: 5.0,
        terrain_coloring: by_height(0.0),
        ..configuration()
    });
    assert_eq!(render(&mut renderer), frame);
}

#[test]
fn assets_keep_their_color() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    let triangle = RealTriangle {
        v0: [-20.0, BLOCK_TOP, 20.0],
        v1: [20.0, BLOCK_TOP, 20.0],
        v2: [0.0, BLOCK_TOP, -20.0],
    };
    renderer.upload_asset(
        StlMesh {
            triangles: vec![triangle],
        },
        1,
    );
    let frame = render(&mut renderer);
    assert_ne!(middle(&frame), [0; 3], "the asset is not in view");

    set_coloring(&mut renderer, by_height(0.0));
    assert_eq!(render(&mut renderer), frame);
}
