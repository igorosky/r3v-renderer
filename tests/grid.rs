use std::sync::{Mutex, MutexGuard};

use cgmath::Point3;
use r3v_renderer::{
    Camera, Renderer, Tesselator,
    camera::OrbitCamera,
    geometry::{PointI, Quaternion, RealTwist, Vector3D},
    grid::GridDescription,
    renderer::{
        DEFAULT_BACKGROUND_COLOR, DEFAULT_COLOR_DEPTH_RANGE, TerrainColoring, WgpuAccelerated,
        WgpuAcceleratedConfiguration,
    },
    tesselator::{CpuTesselator, CpuTesselatorConfiguration, CpuTesselatorContext},
};

const SIZE: u32 = 64;
/// sRGB orange, BGRA `[0, 128, 255]` in a frame
const ORANGE: [f32; 3] = [1.0, 0.5, 0.0];
const ORANGE_BGR: [u8; 3] = [0, 128, 255];
const GREEN: [f32; 3] = [0.0, 1.0, 0.0];
const GREEN_BGR: [u8; 3] = [0, 255, 0];

fn gpu_lock() -> MutexGuard<'static, ()> {
    static LOCK: Mutex<()> = Mutex::new(());
    LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn renderer() -> WgpuAccelerated {
    let mut renderer = WgpuAccelerated::new().expect("no GPU");
    renderer.configure(WgpuAcceleratedConfiguration {
        width: SIZE,
        height: SIZE,
        background_color: DEFAULT_BACKGROUND_COLOR,
        color_depth_range: DEFAULT_COLOR_DEPTH_RANGE,
        terrain_coloring: TerrainColoring::default(),
    });
    renderer
}

/// Seen from 10 above `target`, about 8 wide.
fn render_at(renderer: &mut WgpuAccelerated, target: Point3<f32>) -> Vec<u8> {
    let state = OrbitCamera::new(target, 10.0, 0.0, 89.0).get_state(1.0);
    let mut frame = vec![0; (SIZE * SIZE * 4) as usize];
    renderer.render(&state, &mut frame);
    renderer.render(&state, &mut frame);
    frame
}

fn render(renderer: &mut WgpuAccelerated) -> Vec<u8> {
    render_at(renderer, Point3::new(0.0, 0.0, 0.0))
}

fn pixels(frame: &[u8]) -> impl Iterator<Item = [u8; 3]> + '_ {
    frame
        .as_chunks::<4>()
        .0
        .iter()
        .map(|&[b, g, r, _]| [b, g, r])
}

fn count(frame: &[u8], bgr: [u8; 3]) -> usize {
    pixels(frame)
        .filter(|pixel| pixel.iter().zip(&bgr).all(|(a, b)| a.abs_diff(*b) <= 1))
        .count()
}

fn lit(frame: &[u8]) -> usize {
    pixels(frame).filter(|pixel| *pixel != [0; 3]).count()
}

fn at(x: f32, y: f32, z: f32) -> RealTwist {
    RealTwist {
        linear: Vector3D { x, y, z },
        ..RealTwist::identity()
    }
}

fn floor() -> GridDescription {
    GridDescription {
        size: [4.0, 0.0, 4.0],
        resolution: [1.0; 3],
        color: ORANGE,
        ..GridDescription::default()
    }
}

#[test]
fn grid_is_drawn_in_its_color() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    renderer.create_grid(floor());
    let frame = render(&mut renderer);
    let orange = count(&frame, ORANGE_BGR);
    // 5 lines across each way, each tens of pixels long
    assert!(orange > 100, "{orange} grid pixels");
    assert_eq!(lit(&frame), orange, "other colors than the grid's");
    assert_eq!(frame[..3], [0; 3], "the grid reaches the corner");
}

#[test]
fn removed_grid_is_not_drawn() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    let first = renderer.create_grid(floor());
    let second = renderer.create_grid(floor());
    assert_eq!(second, first + 1);
    renderer.remove_grid(first);
    assert!(!renderer.has_grid(first));
    assert!(renderer.has_grid(second));
    assert!(lit(&render(&mut renderer)) > 0);

    renderer.remove_grid(second);
    assert_eq!(lit(&render(&mut renderer)), 0);
    // Ids are not reused
    assert_eq!(renderer.create_grid(floor()), second + 1);
}

#[test]
#[should_panic(expected = "grid 0 does not exist")]
fn removing_twice_panics() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    let grid = renderer.create_grid(floor());
    renderer.remove_grid(grid);
    renderer.remove_grid(grid);
}

#[test]
fn update_changes_color_and_lines() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    let grid = renderer.create_grid(floor());
    let coarse = count(&render(&mut renderer), ORANGE_BGR);

    renderer.update_grid(
        grid,
        GridDescription {
            color: GREEN,
            ..floor()
        },
    );
    let frame = render(&mut renderer);
    assert_eq!(count(&frame, ORANGE_BGR), 0);
    assert_eq!(count(&frame, GREEN_BGR), coarse);

    renderer.update_grid(
        grid,
        GridDescription {
            resolution: [0.5; 3],
            ..floor()
        },
    );
    let fine = count(&render(&mut renderer), ORANGE_BGR);
    assert!(fine > coarse * 3 / 2, "{fine} and {coarse} grid pixels");
}

#[test]
fn grid_follows_its_frame() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    let grid = renderer.create_grid(GridDescription {
        frame: Some("base_link".to_owned()),
        ..floor()
    });
    assert_eq!(lit(&render(&mut renderer)), 0, "drawn without its frame");

    renderer.set_frame_transform("base_link", RealTwist::identity());
    let in_view = count(&render(&mut renderer), ORANGE_BGR);
    assert!(in_view > 100, "{in_view} grid pixels");

    renderer.set_frame_transform("base_link", at(100.0, 0.0, 0.0));
    assert_eq!(lit(&render(&mut renderer)), 0, "left behind by its frame");
    let moved = render_at(&mut renderer, Point3::new(100.0, 0.0, 0.0));
    assert_eq!(count(&moved, ORANGE_BGR), in_view);

    // The pose is relative to the frame
    renderer.update_grid(
        grid,
        GridDescription {
            frame: Some("base_link".to_owned()),
            pose: at(-100.0, 0.0, 0.0),
            ..floor()
        },
    );
    assert_eq!(count(&render(&mut renderer), ORANGE_BGR), in_view);

    // Other frames do not move it
    renderer.set_frame_transform("other_link", at(5.0, 0.0, 0.0));
    assert_eq!(count(&render(&mut renderer), ORANGE_BGR), in_view);

    renderer.remove_frame("base_link");
    assert_eq!(lit(&render(&mut renderer)), 0, "drawn after its frame left");
}

#[test]
fn frame_rotation_turns_the_grid() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    let floor_pixels = {
        let grid = renderer.create_grid(floor());
        let pixels = count(&render(&mut renderer), ORANGE_BGR);
        renderer.remove_grid(grid);
        pixels
    };

    // A wall in the XY plane, seen from above on its edge
    renderer.create_grid(GridDescription {
        size: [4.0, 4.0, 0.0],
        frame: Some("tilted".to_owned()),
        ..floor()
    });
    renderer.set_frame_transform("tilted", RealTwist::identity());
    let edge = count(&render(&mut renderer), ORANGE_BGR);
    assert!(edge < floor_pixels / 3, "{edge} and {floor_pixels} pixels");

    // A quarter turn about X lays it down
    let half = std::f32::consts::FRAC_PI_4;
    renderer.set_frame_transform(
        "tilted",
        RealTwist {
            rotational: Quaternion {
                x: half.sin(),
                y: 0.0,
                z: 0.0,
                w: half.cos(),
            },
            ..RealTwist::identity()
        },
    );
    let laid_down = count(&render(&mut renderer), ORANGE_BGR);
    assert!(
        laid_down.abs_diff(floor_pixels) <= floor_pixels / 10,
        "{laid_down} and {floor_pixels} pixels"
    );
}

#[test]
fn three_dimensional_grid_has_more_lines_than_its_floor() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    let grid = renderer.create_grid(floor());
    let plane = count(&render(&mut renderer), ORANGE_BGR);
    renderer.update_grid(
        grid,
        GridDescription {
            size: [4.0, 2.0, 4.0],
            // Tilted so the vertical lines are not seen end on
            pose: RealTwist {
                rotational: Quaternion {
                    x: 0.2,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                ..RealTwist::identity()
            },
            ..floor()
        },
    );
    let lattice = count(&render(&mut renderer), ORANGE_BGR);
    assert!(lattice > plane, "{lattice} and {plane} pixels");
}

#[test]
fn voxels_hide_the_grid_behind_them() {
    let _gpu = gpu_lock();
    let mut renderer = renderer();
    let target = Point3::new(2.0, 8.0, 1.0);
    renderer.create_grid(GridDescription {
        size: [8.0, 0.0, 8.0],
        pose: at(2.0, 0.0, 1.0),
        ..floor()
    });
    let open = count(&render_at(&mut renderer, target), ORANGE_BGR);

    // A 4x4x2 block of voxels from y = 4 to 8, between the camera and the grid
    let mut tesselator = CpuTesselator::new();
    tesselator.configure(CpuTesselatorConfiguration {
        voxel_size: 1.0,
        marching_cubes_enabled: false,
    });
    tesselator.update_input_geometry(&vec![PointI {
        x: 0,
        y: 4,
        z: 0,
        i: u32::MAX,
    }]);
    let delta = tesselator.get_output_geometry_delta(&CpuTesselatorContext {
        camera_frustum: None,
    });
    renderer.update_geometry(delta.expect("a delta"));
    let frame = render_at(&mut renderer, target);
    let hidden = count(&frame, ORANGE_BGR);
    let middle = ((SIZE / 2 * SIZE + SIZE / 2) * 4) as usize;
    assert_ne!(
        frame[middle..middle + 3],
        [0; 3],
        "the block is not in view"
    );
    assert!(hidden < open, "{hidden} and {open} grid pixels");
}
