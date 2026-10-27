mod common;

use cgmath::{EuclideanSpace, InnerSpace, Point3, Vector4};
use common::{Lcg, close, close3, close3_eps};
use r3v_renderer::{
    Camera,
    camera::{CameraState, CameraUniform, OrbitCamera},
    geometry::Point,
};

const ASPECT: f32 = 1.5;

/// Projects a world point to normalized device coordinates (x, y in [-1, 1], depth in [0, 1]).
fn project(state: &CameraState, point: [f32; 3]) -> [f32; 3] {
    let clip = state.view_proj * Vector4::new(point[0], point[1], point[2], 1.0);
    assert!(clip.w > 0.0, "point {point:?} is behind the camera");
    [clip.x / clip.w, clip.y / clip.w, clip.z / clip.w]
}

fn arr(p: Point3<f32>) -> [f32; 3] {
    [p.x, p.y, p.z]
}

fn add_scaled(p: [f32; 3], d: cgmath::Vector3<f32>, s: f32) -> [f32; 3] {
    [p[0] + d.x * s, p[1] + d.y * s, p[2] + d.z * s]
}

#[track_caller]
fn assert_finite(state: &CameraState) {
    let matrix: [[f32; 4]; 4] = state.view_proj.into();
    assert!(
        matrix.iter().flatten().all(|c| c.is_finite()),
        "non-finite view_proj: {matrix:?}"
    );
    assert!(arr(state.position).iter().all(|c| c.is_finite()));
    assert!(
        [state.forward.x, state.forward.y, state.forward.z]
            .iter()
            .all(|c| c.is_finite())
    );
}

/// Checks the invariants every orbit camera state must satisfy.
#[track_caller]
fn assert_valid(camera: &OrbitCamera) -> CameraState {
    let state = camera.get_state(ASPECT);
    assert_finite(&state);
    close(state.forward.magnitude(), 1.0);
    close(
        (state.position - camera.target).magnitude(),
        camera.distance,
    );
    close3_eps(
        add_scaled(arr(state.position), state.forward, camera.distance),
        arr(camera.target),
        1.0e-4,
    );
    state
}

/// Valid state, with the target in the centre of the screen and inside the depth range.
#[track_caller]
fn assert_consistent(camera: &OrbitCamera) {
    let state = assert_valid(camera);
    let ndc = project(&state, arr(camera.target));
    close(ndc[0], 0.0);
    close(ndc[1], 0.0);
    assert!(ndc[2] > 0.0 && ndc[2] < 1.0, "target depth {}", ndc[2]);
}

#[test]
fn default_camera_looks_down_negative_z_from_five_units() {
    let camera = OrbitCamera::default();
    let state = camera.get_state(ASPECT);
    close3(arr(state.position), [0.0, 0.0, 5.0]);
    close3(
        [state.forward.x, state.forward.y, state.forward.z],
        [0.0, 0.0, -1.0],
    );
    close3(camera.forward().into(), [0.0, 0.0, -1.0]);
    assert_consistent(&camera);
}

#[test]
fn default_camera_screen_axes_follow_world_axes() {
    let state = OrbitCamera::default().get_state(1.0);
    let right = project(&state, [1.0, 0.0, 0.0]);
    let up = project(&state, [0.0, 1.0, 0.0]);
    assert!(right[0] > 0.0);
    close(right[1], 0.0);
    assert!(up[1] > 0.0);
    close(up[0], 0.0);
    // Square aspect ratio: equal offsets give equal screen offsets.
    close(right[0], up[1]);
}

#[test]
fn closer_points_have_smaller_depth() {
    let state = OrbitCamera::default().get_state(ASPECT);
    let near = project(&state, [0.0, 0.0, 2.0]);
    let middle = project(&state, [0.0, 0.0, 0.0]);
    let far = project(&state, [0.0, 0.0, -20.0]);
    assert!(near[2] < middle[2]);
    assert!(middle[2] < far[2]);
}

#[test]
fn clip_planes_map_to_wgpu_depth_range() {
    for (near, far) in [(0.1, 100.0), (1.0, 10.0), (0.01, 1000.0)] {
        let camera = OrbitCamera::default().with_clip_planes(near, far);
        assert_eq!(camera.near, near);
        assert_eq!(camera.far, far);
        let state = camera.get_state(ASPECT);
        let eye = arr(state.position);
        close(
            project(&state, add_scaled(eye, state.forward, near))[2],
            0.0,
        );
        let depth = project(&state, add_scaled(eye, state.forward, far))[2];
        close(depth, 1.0);
    }
}

#[test]
fn aspect_ratio_only_scales_horizontal_axis() {
    let camera = OrbitCamera::default();
    let square = camera.get_state(1.0);
    let wide = camera.get_state(2.0);
    let point = [1.0, 1.0, 0.0];
    let a = project(&square, point);
    let b = project(&wide, point);
    close(b[0], a[0] / 2.0);
    close(b[1], a[1]);
    close(b[2], a[2]);
}

#[test]
fn new_converts_degrees_and_positions_eye() {
    let target = Point3::new(1.0, 2.0, 3.0);

    let camera = OrbitCamera::new(target, 4.0, 90.0, 0.0);
    close(camera.yaw, std::f32::consts::FRAC_PI_2);
    close3(arr(camera.get_state(ASPECT).position), [5.0, 2.0, 3.0]);
    assert_consistent(&camera);

    let camera = OrbitCamera::new(target, 4.0, 180.0, 0.0);
    close3_eps(
        arr(camera.get_state(ASPECT).position),
        [1.0, 2.0, -1.0],
        1.0e-5,
    );

    let camera = OrbitCamera::new(target, 4.0, 0.0, 30.0);
    let eye = arr(camera.get_state(ASPECT).position);
    close(eye[1], 2.0 + 4.0 * 0.5);
    close(eye[2], 3.0 + 4.0 * 3.0_f32.sqrt() / 2.0);
    assert_consistent(&camera);

    let camera = OrbitCamera::new(target, 4.0, 0.0, -45.0);
    assert!(camera.get_state(ASPECT).position.y < target.y);
    assert_consistent(&camera);
}

#[test]
fn look_at_replaces_whole_pose() {
    let mut camera = OrbitCamera::default();
    camera.rotate(1.0, 0.5);
    camera.pan(3.0, -2.0);
    camera.new_uniform_available = false;

    camera.look_at(
        Point {
            x: -1.0,
            y: 0.5,
            z: 2.0,
        },
        3.0,
        90.0,
        0.0,
    );

    assert!(camera.new_uniform_available);
    close3(arr(camera.target), [-1.0, 0.5, 2.0]);
    close(camera.distance, 3.0);
    close3(arr(camera.get_state(ASPECT).position), [2.0, 0.5, 2.0]);
    assert_consistent(&camera);
}

#[test]
fn look_at_straight_up_or_down_stays_finite() {
    for pitch in [90.0, -90.0, 180.0, 720.0] {
        let mut camera = OrbitCamera::default();
        camera.look_at(
            Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            5.0,
            0.0,
            pitch,
        );
        assert_consistent(&camera);
    }
}

#[test]
fn look_at_with_degenerate_distance_stays_finite() {
    for distance in [0.0, -3.0] {
        let mut camera = OrbitCamera::default();
        camera.look_at(
            Point {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            distance,
            0.0,
            0.0,
        );
        assert!(camera.distance > 0.0);
        assert_valid(&camera);
    }
}

#[test]
fn rotate_accumulates_yaw_and_pitch() {
    let mut camera = OrbitCamera::default();
    camera.rotate(0.25, 0.1);
    camera.rotate(0.25, 0.2);
    close(camera.yaw, 0.5);
    close(camera.pitch, 0.3);
    let eye = arr(camera.get_state(ASPECT).position);
    close(eye[0], 5.0 * 0.5_f32.sin() * 0.3_f32.cos());
    close(eye[1], 5.0 * 0.3_f32.sin());
    close(eye[2], 5.0 * 0.5_f32.cos() * 0.3_f32.cos());
    assert_consistent(&camera);
}

#[test]
fn rotate_full_turn_returns_to_same_view() {
    let mut camera = OrbitCamera::default();
    let before = camera.get_state(ASPECT);
    camera.rotate(std::f32::consts::TAU, 0.0);
    let after = camera.get_state(ASPECT);
    close3_eps(arr(after.position), arr(before.position), 1.0e-4);
}

#[test]
fn rotate_clamps_pitch_short_of_the_poles() {
    for delta in [
        10.0,
        -10.0,
        std::f32::consts::FRAC_PI_2,
        -std::f32::consts::FRAC_PI_2,
    ] {
        let mut camera = OrbitCamera::default();
        camera.rotate(0.0, delta);
        assert!(camera.pitch.abs() < std::f32::consts::FRAC_PI_2);
        assert!(camera.pitch.abs() > std::f32::consts::FRAC_PI_2 - 0.1);
        assert_eq!(camera.pitch.signum(), delta.signum());
        assert_consistent(&camera);
    }
}

#[test]
fn rotate_back_from_clamped_pitch() {
    let mut camera = OrbitCamera::default();
    camera.rotate(0.0, 100.0);
    camera.rotate(0.0, -0.5);
    assert!(camera.pitch < std::f32::consts::FRAC_PI_2 - 0.4);
}

#[test]
fn zoom_changes_distance_and_keeps_target() {
    let mut camera = OrbitCamera::default();
    camera.zoom(2.0);
    close(camera.distance, 7.0);
    camera.zoom(-4.0);
    close(camera.distance, 3.0);
    close3(arr(camera.target), [0.0, 0.0, 0.0]);
    assert_consistent(&camera);
}

#[test]
fn zoom_never_reaches_or_passes_target() {
    let mut camera = OrbitCamera::default();
    camera.zoom(-1000.0);
    assert!(camera.distance > 0.0);
    assert_valid(&camera);
    camera.zoom(-1.0);
    assert!(camera.distance > 0.0);
    assert_valid(&camera);
}

#[test]
fn pan_moves_target_in_screen_plane() {
    let mut camera = OrbitCamera::new(Point3::new(0.0, 0.0, 0.0), 5.0, 30.0, 20.0);
    let before = camera.get_state(1.0);
    camera.pan(1.0, 0.0);
    let after_x = camera.get_state(1.0);
    let moved = camera.target.to_vec();
    close(moved.magnitude(), 1.0);
    close(moved.dot(before.forward), 0.0);
    close(moved.y, 0.0); // screen-right stays horizontal for an orbit camera
    // Orientation and distance are unchanged.
    close3(
        [after_x.forward.x, after_x.forward.y, after_x.forward.z],
        [before.forward.x, before.forward.y, before.forward.z],
    );
    close(camera.distance, 5.0);
    // The old target now appears left of the screen centre.
    let ndc = project(&after_x, [0.0, 0.0, 0.0]);
    assert!(ndc[0] < 0.0);
    close(ndc[1], 0.0);

    let mut camera = OrbitCamera::new(Point3::new(0.0, 0.0, 0.0), 5.0, 30.0, 20.0);
    camera.pan(0.0, 1.0);
    let moved = camera.target.to_vec();
    close(moved.magnitude(), 1.0);
    close(moved.dot(before.forward), 0.0);
    assert!(moved.y > 0.0);
    // The old target now appears below the screen centre.
    let ndc = project(&camera.get_state(1.0), [0.0, 0.0, 0.0]);
    close(ndc[0], 0.0);
    assert!(ndc[1] < 0.0);
    assert_consistent(&camera);
}

#[test]
fn pan_by_zero_is_a_no_op() {
    let mut camera = OrbitCamera::new(Point3::new(1.0, 2.0, 3.0), 5.0, 10.0, 10.0);
    camera.pan(0.0, 0.0);
    close3(arr(camera.target), [1.0, 2.0, 3.0]);
}

#[test]
fn every_mutation_flags_a_new_uniform() {
    let mutations: [fn(&mut OrbitCamera); 4] = [
        |c| c.rotate(0.1, 0.1),
        |c| c.zoom(1.0),
        |c| c.pan(1.0, 1.0),
        |c| {
            c.look_at(
                Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                2.0,
                0.0,
                0.0,
            )
        },
    ];
    assert!(OrbitCamera::default().new_uniform_available);
    for mutate in mutations {
        let mut camera = OrbitCamera {
            new_uniform_available: false,
            ..OrbitCamera::default()
        };
        mutate(&mut camera);
        assert!(camera.new_uniform_available);
    }
}

#[test]
fn camera_uniform_is_std140_sized() {
    // mat4x4 + vec3 (padded) + vec3 (padded)
    assert_eq!(std::mem::size_of::<CameraUniform>(), 64 + 16 + 16);
}

#[test]
fn camera_uniform_carries_state() {
    let state = OrbitCamera::new(Point3::new(1.0, 2.0, 3.0), 4.0, 45.0, 10.0).get_state(ASPECT);
    let uniform = CameraUniform::from(&state);
    let floats: &[f32] = bytemuck::cast_slice(std::slice::from_ref(&uniform));
    let matrix: [[f32; 4]; 4] = state.view_proj.into();
    assert_eq!(&floats[..16], matrix.as_flattened());
    assert_eq!(&floats[16..19], &arr(state.position));
    assert_eq!(
        &floats[20..23],
        &[state.forward.x, state.forward.y, state.forward.z]
    );
}

#[test]
fn default_camera_uniform_is_identity_looking_down_negative_z() {
    let uniform = CameraUniform::default();
    let floats: &[f32] = bytemuck::cast_slice(std::slice::from_ref(&uniform));
    let identity: [[f32; 4]; 4] = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    assert_eq!(&floats[..16], identity.as_flattened());
    assert_eq!(&floats[16..19], &[0.0, 0.0, 0.0]);
    assert_eq!(&floats[20..23], &[0.0, 0.0, -1.0]);
}

#[test]
fn frustum_keeps_boxes_in_view() {
    // From (0, 0, 5) towards negative z
    let frustum = OrbitCamera::default().get_state(ASPECT).frustum();
    let boxes = [
        ([-1.0; 3], [1.0; 3]),                    // around the target
        ([-0.5, -0.5, 4.8], [0.5, 0.5, 5.2]),     // through the near plane
        ([-0.1, -0.1, -96.0], [0.1, 0.1, -94.0]), // through the far plane
        ([2.5, -0.5, -0.5], [10.0, 0.5, 0.5]),    // through the right plane
        ([-1000.0; 3], [1000.0; 3]),              // around the whole view
    ];
    for (min, max) in boxes {
        assert!(frustum.intersects_box(min, max), "{min:?}..{max:?}");
    }
}

#[test]
fn frustum_drops_boxes_beyond_each_plane() {
    let frustum = OrbitCamera::default().get_state(ASPECT).frustum();
    let boxes = [
        ([-1.0, -1.0, 6.0], [1.0, 1.0, 7.0]),       // behind the camera
        ([-1.0, -1.0, -200.0], [1.0, 1.0, -150.0]), // past the far plane
        ([-20.0, -1.0, -1.0], [-10.0, 1.0, 1.0]),   // left
        ([10.0, -1.0, -1.0], [20.0, 1.0, 1.0]),     // right
        ([-1.0, -20.0, -1.0], [1.0, -10.0, 1.0]),   // below
        ([-1.0, 10.0, -1.0], [1.0, 20.0, 1.0]),     // above
    ];
    for (min, max) in boxes {
        assert!(!frustum.intersects_box(min, max), "{min:?}..{max:?}");
    }
}

#[test]
fn frustum_agrees_with_projection() {
    let mut rng = Lcg::new(7);
    let mut camera = OrbitCamera::default();
    let (mut inside, mut outside) = (0, 0);
    for _ in 0..20 {
        camera.rotate(
            rng.range(-314, 314) as f32 / 100.0,
            rng.range(-150, 150) as f32 / 100.0,
        );
        let state = camera.get_state(ASPECT);
        let frustum = state.frustum();
        for _ in 0..500 {
            let point = [0; 3].map(|_| rng.range(-5000, 5000) as f32 / 100.0);
            let clip = state.view_proj * Vector4::new(point[0], point[1], point[2], 1.0);
            let ndc = [clip.x / clip.w, clip.y / clip.w, clip.z / clip.w];
            let tiny = |offset: f32| point.map(|c| c + offset);
            let hit = frustum.intersects_box(tiny(-1.0e-4), tiny(1.0e-4));
            let in_view = clip.w > 0.0
                && ndc[0].abs() <= 1.0
                && ndc[1].abs() <= 1.0
                && (0.0..=1.0).contains(&ndc[2]);
            // Clear of the frustum's sides, past the far plane, or behind the camera
            let clearly_out = clip.w > 0.0
                && (ndc[0].abs() > 1.01 || ndc[1].abs() > 1.01 || ndc[2] > 1.01)
                || clip.w < -0.01;
            if in_view {
                assert!(hit, "visible point {point:?} culled");
                inside += 1;
            } else if clearly_out {
                assert!(!hit, "point {point:?} outside the view kept");
                outside += 1;
            }
        }
    }
    assert!(
        inside > 100 && outside > 100,
        "{inside} inside, {outside} outside"
    );
}
