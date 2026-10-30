use cgmath::{InnerSpace, Rotation3, Vector3};
use image::{ImageBuffer, Rgba};
use r3v_renderer::{
    Camera, Renderer, Robot, Tesselator,
    camera::OrbitCamera,
    geometry::{self, PointI, RealTwist},
    renderer::{self, TerrainColoring, WgpuAccelerated},
    robot::UrdfRobot,
    shapes, stl,
    tesselator::{CpuTesselator, CpuTesselatorConfiguration, CpuTesselatorContext},
    urdf,
};
use rayon::prelude::*;
use std::{
    array,
    collections::{HashMap, HashSet},
    f32::consts::TAU,
    num::NonZeroU32,
    path::Path,
    rc::Rc,
    time::Instant,
};
use winit::{
    application::ApplicationHandler,
    dpi::{PhysicalPosition, PhysicalSize},
    event::{ElementState, KeyEvent, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowId},
};

const INITIAL_WIDTH: u32 = 1200;
const INITIAL_HEIGHT: u32 = 800;

// Brick layout used by GreedyMeshTesselator: 4x4x2 voxels packed into a u32,
// bit = lx | ly << 2 | lz << 4.
/// World metres per voxel edge; the world grid below is laid out in voxels.
const VOXEL_SIZE: f32 = 0.2;

const BRICK_X: i32 = 4;
const BRICK_Y: i32 = 4;
const BRICK_Z: i32 = 2;

const WORLD_BRICKS_X: i32 = 128;
const WORLD_BRICKS_Y: i32 = 16;
const WORLD_BRICKS_Z: i32 = 256;
const WORLD_X: i32 = WORLD_BRICKS_X * BRICK_X;
const WORLD_Y: i32 = WORLD_BRICKS_Y * BRICK_Y;
const WORLD_Z: i32 = WORLD_BRICKS_Z * BRICK_Z;

const BALL_COUNT: usize = 40;
const SNAKE_COUNT: usize = 12;
const SNAKE_SEGMENTS: usize = 16;
const SNAKE_WIGGLE: f32 = 3.0;
const WINDMILL_COUNT: usize = 8;
const WINDMILL_BLADE_LENGTH: f32 = 12.0;
const GYRO_COUNT: usize = 6;
const GYRO_RADIUS: f32 = 8.0;
const PISTON_FIELD_COUNT: usize = 3;
const PISTON_GRID: usize = 8;
const PISTON_PITCH: i32 = 4;
const PISTON_SIZE: i32 = 3;
const LAKE_COUNT: usize = 3;
const LAKE_RADIUS: f32 = 36.0;
const WAVE_AMPLITUDE: f32 = 2.0;
/// Structures and lakes are only placed on ground at most this high, so they fit under the world ceiling.
const MAX_STRUCTURE_GROUND: i32 = 28;
const TREE_CELL: i32 = 16;

/// Loaded when no URDF path is passed on the command line.
const DEFAULT_ROBOT_URDF: &str = "assets/scorpio8.urdf.xacro";
/// World metres per URDF metre.
const ROBOT_SCALE: f32 = 1.0;
/// Rover driving speed along its loop, in metres per second.
const ROVER_SPEED: f32 = 2.0;
/// Radius of the rover's loop, as a fraction of the world extent.
const ROVER_PATH_RADIUS: f32 = 0.35;
/// Half-width, in voxels, of the ground patch averaged to set the rover's height.
const ROVER_FOOTPRINT: i32 = 3;
/// Only voxels within this many metres of the lidar are sent to the tesselator.
const LIDAR_RANGE: f32 = 20.0;
/// Lidar mount height above the ground under the rover, in metres.
const LIDAR_HEIGHT: f32 = 1.0;
const ROBOT_JOINT_SPEED: f32 = 0.8;
/// Sweep amplitude, in radians, for continuous joints that carry other links (rockers, wrists).
const ROBOT_CONTINUOUS_SWEEP: f32 = 0.4;
/// In metres.
const ROBOT_FOCUS_DISTANCE: f32 = 3.5;
/// Joint mounting the manipulator on the rover: every link below it belongs to the arm.
const ROBOT_ARM_MOUNT_JOINT: &str = "mani_base_offset_joint";
const ROBOT_SHOW_SHAPES: bool = false;

// Camera framing is relative to the voxel world, so it follows VOXEL_SIZE.
const CAMERA_DISTANCE: f32 = 160.0 * VOXEL_SIZE;
const CAMERA_MIN_DISTANCE: f32 = 0.1;
const CAMERA_MAX_DISTANCE: f32 = 900.0 * VOXEL_SIZE;
const CAMERA_NEAR: f32 = 0.01;
const CAMERA_FAR: f32 = 2000.0 * VOXEL_SIZE;
const AUTO_ORBIT_SPEED: f32 = 0.15;
const MOUSE_ROTATE_SPEED: f32 = 0.005;
const MOUSE_PAN_SPEED: f32 = 0.0015;
const KEY_ROTATE_SPEED: f32 = 1.5;
const ZOOM_STEP: f32 = 0.88;
/// Keyboard movement speed, as a fraction of the orbit distance per second.
const MOVE_SPEED: f32 = 0.6;
const FAST_MULTIPLIER: f32 = 4.0;

/// Cycled with B: a name and an sRGB color.
const BACKGROUNDS: [(&str, [f32; 3]); 4] = [
    ("black", renderer::DEFAULT_BACKGROUND_COLOR),
    ("slate", [0.12, 0.13, 0.16]),
    ("sky", [0.53, 0.75, 0.95]),
    ("white", [1.0, 1.0, 1.0]),
];
/// Factor of one [ or ] press on the colormap depth range.
const COLOR_DEPTH_RANGE_STEP: f32 = 1.25;
const COLOR_DEPTH_RANGE_MIN: f32 = 5.0;
const COLOR_DEPTH_RANGE_MAX: f32 = 2000.0;
const HEIGHT_KNEE_STEP: f32 = 1.25;
const HEIGHT_KNEE_MIN: f32 = 0.05;
const HEIGHT_KNEE_MAX: f32 = 20.0;
const HEIGHT_GAIN_STEP: f32 = 1.1;
const HEIGHT_GAIN_MIN: f32 = 0.05;
const HEIGHT_GAIN_MAX: f32 = 2.0;
const HEIGHT_BANDS: [u32; 4] = [0, 8, 16, 32];

const STATS_INTERVAL_SECS: f32 = 0.25;
const OVERLAY_SCALE: usize = 4;

const CONTROLS_HELP: &str = "\
Controls:
  Left drag         orbit
  Right/Middle drag pan
  Scroll            zoom
  W/A/S/D           move horizontally
  Q/E               move down/up
  Arrow keys        orbit
  Shift             move faster
  O                 toggle auto-orbit
  C                 toggle marching-cubes terrain smoothing
  V                 toggle frustum culling of terrain chunks
  R                 reset camera
  F                 toggle following the robot
  M                 add/remove the robot arm
  B                 cycle background color
  [ / ]             shorten/lengthen the colormap depth range
  H                 toggle coloring the terrain by height
  G                 toggle measuring heights from the world's 0 or the robot
  J / K             shorten/lengthen the height knee
  U / I             lower/raise the height gain
  T                 cycle height color bands
  P                 save snapshot
  Esc               quit";

fn main() -> anyhow::Result<()> {
    println!("{}", CONTROLS_HELP);
    let event_loop = EventLoop::new()?;
    event_loop.set_control_flow(ControlFlow::Poll);

    let urdf_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ROBOT_URDF.to_owned());
    let mut demo = Demo::new(Path::new(&urdf_path));
    event_loop.run_app(&mut demo)?;
    Ok(())
}

#[derive(Default)]
struct InputState {
    keys: HashSet<KeyCode>,
    orbiting: bool,
    panning: bool,
    cursor: Option<PhysicalPosition<f64>>,
}

impl InputState {
    fn axis(&self, negative: KeyCode, positive: KeyCode) -> f32 {
        self.keys.contains(&positive) as i32 as f32 - self.keys.contains(&negative) as i32 as f32
    }
}

struct Demo {
    window: Option<Rc<Window>>,
    surface: Option<softbuffer::Surface<Rc<Window>, Rc<Window>>>,

    world: World,
    camera: OrbitCamera,
    tesselator: CpuTesselator,
    renderer: WgpuAccelerated,
    robot: Option<RobotScene>,

    input: InputState,
    auto_orbit: bool,
    follow_robot: bool,

    background_index: usize,
    color_depth_range: f32,
    terrain_coloring: TerrainColoring,

    width: u32,
    height: u32,
    pixel_buffer: Vec<u8>,

    start: Instant,
    last_frame: Instant,
    stats_frames: u32,
    stats_since: Instant,
    overlay_text: String,

    quad_count: usize,
    chunk_count: usize,
    snapshot_index: u32,
}

fn initial_camera() -> OrbitCamera {
    let center = cgmath::Point3::new(
        WORLD_X as f32 / 2.0 * VOXEL_SIZE,
        WORLD_Y as f32 / 4.0 * VOXEL_SIZE,
        WORLD_Z as f32 / 2.0 * VOXEL_SIZE,
    );
    OrbitCamera::new(center, CAMERA_DISTANCE, 30.0, 30.0).with_clip_planes(CAMERA_NEAR, CAMERA_FAR)
}

impl Demo {
    fn new(urdf_path: &Path) -> Self {
        let world_start = Instant::now();
        let world = World::new();
        println!(
            "Generated {}x{}x{} voxel world in {:.0} ms",
            WORLD_X,
            WORLD_Y,
            WORLD_Z,
            world_start.elapsed().as_secs_f32() * 1000.0,
        );
        println!(
            "Animating {} dynamic shapes: {} balls, {} snakes, {} windmills, {} gyroscopes, {} piston fields, {} lakes",
            world.dynamic_shapes(0.0).len(),
            world.balls.len(),
            world.snakes.len(),
            world.windmills.len(),
            world.gyroscopes.len(),
            world.piston_fields.len(),
            world.lakes.len(),
        );
        let mut tesselator = CpuTesselator::new();
        tesselator.configure(CpuTesselatorConfiguration {
            voxel_size: VOXEL_SIZE,
            marching_cubes_enabled: false,
        });
        let mut renderer = WgpuAccelerated::new().expect("Failed to initialize WgpuAccelerated");
        let robot = match RobotScene::load(urdf_path, rover_placement(&world, 0.0), &mut renderer) {
            Ok(robot) => Some(robot),
            Err(err) => {
                eprintln!("Running without a robot: {err}");
                None
            }
        };
        let now = Instant::now();

        Self {
            window: None,
            surface: None,
            world,
            camera: initial_camera(),
            tesselator,
            renderer,
            robot,
            input: InputState::default(),
            auto_orbit: true,
            follow_robot: false,
            background_index: 0,
            color_depth_range: renderer::DEFAULT_COLOR_DEPTH_RANGE,
            terrain_coloring: TerrainColoring::default(),
            width: 0,
            height: 0,
            pixel_buffer: Vec::new(),
            start: now,
            last_frame: now,
            stats_frames: 0,
            stats_since: now,
            overlay_text: String::new(),
            quad_count: 0,
            chunk_count: 0,
            snapshot_index: 0,
        }
    }

    fn resize(&mut self, width: u32, height: u32) {
        let (Some(surface), Some(w), Some(h)) = (
            self.surface.as_mut(),
            NonZeroU32::new(width),
            NonZeroU32::new(height),
        ) else {
            return;
        };
        surface.resize(w, h).expect("Failed to resize surface");
        self.pixel_buffer.resize((width * height * 4) as usize, 0);
        self.width = width;
        self.height = height;
        self.configure_renderer();
    }

    /// Passes the frame size and the render settings to the renderer. At an unchanged size only
    /// the settings change, from the next frame on.
    fn configure_renderer(&mut self) {
        self.renderer
            .configure(renderer::WgpuAcceleratedConfiguration {
                width: self.width,
                height: self.height,
                background_color: BACKGROUNDS[self.background_index].1,
                color_depth_range: self.color_depth_range,
                terrain_coloring: self.terrain_coloring,
            });
    }

    fn cycle_background(&mut self) {
        self.background_index = (self.background_index + 1) % BACKGROUNDS.len();
        self.configure_renderer();
        println!("Background: {}", BACKGROUNDS[self.background_index].0);
    }

    fn scale_color_depth_range(&mut self, factor: f32) {
        self.color_depth_range =
            (self.color_depth_range * factor).clamp(COLOR_DEPTH_RANGE_MIN, COLOR_DEPTH_RANGE_MAX);
        self.configure_renderer();
        println!("Colormap depth range: {:.1} m", self.color_depth_range);
    }

    fn scale_height_knee(&mut self, factor: f32) {
        self.update_terrain_coloring(|coloring| {
            coloring.height_knee =
                (coloring.height_knee * factor).clamp(HEIGHT_KNEE_MIN, HEIGHT_KNEE_MAX);
        });
    }

    fn scale_height_gain(&mut self, factor: f32) {
        self.update_terrain_coloring(|coloring| {
            coloring.height_gain =
                (coloring.height_gain * factor).clamp(HEIGHT_GAIN_MIN, HEIGHT_GAIN_MAX);
        });
    }

    fn update_terrain_coloring(&mut self, change: impl FnOnce(&mut TerrainColoring)) {
        change(&mut self.terrain_coloring);
        self.configure_renderer();
        println!("Terrain coloring: {}", self.terrain_coloring_summary());
    }

    fn terrain_coloring_summary(&self) -> String {
        let coloring = &self.terrain_coloring;
        if !coloring.height_coloring_enabled {
            return "depth".to_owned();
        }
        format!(
            "height from the {}, knee {:.2} m, gain {:.2}, {} bands",
            if coloring.relative_to_robot {
                "robot"
            } else {
                "world"
            },
            coloring.height_knee,
            coloring.height_gain,
            coloring.height_bands,
        )
    }

    fn redraw(&mut self) {
        if self.width == 0 || self.height == 0 {
            return;
        }
        let now = Instant::now();
        let t = (now - self.start).as_secs_f32();
        let dt = (now - self.last_frame).as_secs_f32();
        self.last_frame = now;

        // The rover drives first, so the camera can follow it and the lidar scans from it.
        let rover = rover_placement(&self.world, t);
        let lidar = (rover.translation / VOXEL_SIZE
            + Vector3::unit_y() * (LIDAR_HEIGHT / VOXEL_SIZE))
            .into();
        self.terrain_coloring.robot_height = rover.translation.y;
        self.configure_renderer();
        // Robot meshes were uploaded once; each frame only moves their instances.
        if let Some(robot) = &mut self.robot {
            robot.update(t, rover, &mut self.renderer);
        }

        // The camera moves before tesselating so only chunks in this frame's view are tesselated.
        self.update_camera(dt);
        let aspect_ratio = self.width as f32 / self.height as f32;
        let camera_state = self.camera.get_state(aspect_ratio);

        // Dynamic voxel update -> tesselation -> renderer geometry update.
        // Only bricks the lidar sees changing are sent.
        let brick_delta = self.world.update(t, lidar, LIDAR_RANGE / VOXEL_SIZE);
        self.tesselator.update_input_geometry(&brick_delta);
        let context = CpuTesselatorContext {
            camera_frustum: Some(camera_state.frustum()),
        };
        if let Some(geometry_delta) = self.tesselator.get_output_geometry_delta(&context) {
            for delta in &geometry_delta {
                self.quad_count += delta.triangles.len();
                self.chunk_count += !delta.triangles.is_empty() as usize;
            }
            self.renderer.update_geometry(geometry_delta);
        }

        self.renderer.render(&camera_state, &mut self.pixel_buffer);

        self.update_stats(now);
        draw_text(
            &mut self.pixel_buffer,
            self.width as usize,
            self.height as usize,
            12,
            12,
            &self.overlay_text,
        );
        self.present();
    }

    fn update_camera(&mut self, dt: f32) {
        let input = &self.input;
        let yaw = input.axis(KeyCode::ArrowRight, KeyCode::ArrowLeft);
        let pitch = input.axis(KeyCode::ArrowDown, KeyCode::ArrowUp);
        let forward = input.axis(KeyCode::KeyS, KeyCode::KeyW);
        let strafe = input.axis(KeyCode::KeyA, KeyCode::KeyD);
        let lift = input.axis(KeyCode::KeyQ, KeyCode::KeyE);
        let fast =
            input.keys.contains(&KeyCode::ShiftLeft) || input.keys.contains(&KeyCode::ShiftRight);

        if yaw != 0.0 || pitch != 0.0 {
            self.auto_orbit = false;
            self.camera
                .rotate(yaw * KEY_ROTATE_SPEED * dt, pitch * KEY_ROTATE_SPEED * dt);
        } else if self.auto_orbit {
            self.camera.rotate(AUTO_ORBIT_SPEED * dt, 0.0);
        }

        if forward != 0.0 || strafe != 0.0 || lift != 0.0 {
            self.auto_orbit = false;
            self.follow_robot = false;
            // Movement is relative to the horizontal view direction, so W never digs into the ground.
            let view_forward = self.camera.forward();
            let ahead = Vector3::new(view_forward.x, 0.0, view_forward.z).normalize();
            let right = ahead.cross(Vector3::unit_y());
            let speed = MOVE_SPEED
                * self.camera.distance.max(20.0 * VOXEL_SIZE)
                * if fast { FAST_MULTIPLIER } else { 1.0 };
            let target = &mut self.camera.target;
            *target += (ahead * forward + right * strafe + Vector3::unit_y() * lift) * speed * dt;
            let [world_x, world_y, world_z] =
                [WORLD_X, WORLD_Y, WORLD_Z].map(|extent| extent as f32 * VOXEL_SIZE);
            target.x = target.x.clamp(-world_x, 2.0 * world_x);
            target.y = target.y.clamp(-world_y, 4.0 * world_y);
            target.z = target.z.clamp(-world_z, 2.0 * world_z);
            self.camera.new_uniform_available = true;
        }

        if self.follow_robot
            && let Some(target) = self.robot.as_ref().and_then(RobotScene::focus_point)
        {
            self.camera.target = cgmath::Point3::new(target.x, target.y, target.z);
            self.camera.new_uniform_available = true;
        }
    }

    fn zoom(&mut self, steps: f32) {
        let distance = self.camera.distance;
        let target =
            (distance * ZOOM_STEP.powf(steps)).clamp(CAMERA_MIN_DISTANCE, CAMERA_MAX_DISTANCE);
        self.camera.zoom(target - distance);
    }

    fn cursor_moved(&mut self, position: PhysicalPosition<f64>) {
        if let Some(previous) = self.input.cursor {
            let dx = (position.x - previous.x) as f32;
            let dy = (position.y - previous.y) as f32;
            if self.input.orbiting {
                self.auto_orbit = false;
                self.camera
                    .rotate(-dx * MOUSE_ROTATE_SPEED, dy * MOUSE_ROTATE_SPEED);
            }
            if self.input.panning {
                self.auto_orbit = false;
                self.follow_robot = false;
                let scale = self.camera.distance * MOUSE_PAN_SPEED;
                self.camera.pan(-dx * scale, dy * scale);
            }
        }
        self.input.cursor = Some(position);
    }

    /// `repeat` for presses repeated while the key is held, which only the color scales follow.
    fn key_pressed(&mut self, event_loop: &ActiveEventLoop, code: KeyCode, repeat: bool) {
        match code {
            KeyCode::BracketLeft => self.scale_color_depth_range(COLOR_DEPTH_RANGE_STEP.recip()),
            KeyCode::BracketRight => self.scale_color_depth_range(COLOR_DEPTH_RANGE_STEP),
            KeyCode::KeyJ => self.scale_height_knee(HEIGHT_KNEE_STEP.recip()),
            KeyCode::KeyK => self.scale_height_knee(HEIGHT_KNEE_STEP),
            KeyCode::KeyU => self.scale_height_gain(HEIGHT_GAIN_STEP.recip()),
            KeyCode::KeyI => self.scale_height_gain(HEIGHT_GAIN_STEP),
            _ if repeat => {}
            KeyCode::KeyB => self.cycle_background(),
            KeyCode::KeyH => self.update_terrain_coloring(|coloring| {
                coloring.height_coloring_enabled = !coloring.height_coloring_enabled;
            }),
            KeyCode::KeyG => self.update_terrain_coloring(|coloring| {
                coloring.relative_to_robot = !coloring.relative_to_robot;
            }),
            KeyCode::KeyT => self.update_terrain_coloring(|coloring| {
                let index = HEIGHT_BANDS
                    .iter()
                    .position(|&bands| bands == coloring.height_bands);
                coloring.height_bands =
                    HEIGHT_BANDS[index.map_or(0, |index| (index + 1) % HEIGHT_BANDS.len())];
            }),
            KeyCode::Escape => event_loop.exit(),
            KeyCode::KeyP => self.save_snapshot(),
            KeyCode::KeyO => self.auto_orbit = !self.auto_orbit,
            KeyCode::KeyC => self
                .tesselator
                .set_marching_cubes_enabled(!self.tesselator.marching_cubes_enabled()),
            KeyCode::KeyV => self
                .renderer
                .set_frustum_culling(!self.renderer.frustum_culling()),
            KeyCode::KeyR => {
                self.camera = initial_camera();
                self.auto_orbit = true;
                self.follow_robot = false;
            }
            KeyCode::KeyM => {
                if let Some(robot) = &mut self.robot {
                    robot.toggle_arm(&mut self.renderer);
                }
            }
            KeyCode::KeyF => {
                self.follow_robot = !self.follow_robot;
                if self.follow_robot
                    && let Some(target) = self.robot.as_ref().and_then(RobotScene::focus_point)
                {
                    let (yaw, pitch) =
                        (self.camera.yaw.to_degrees(), self.camera.pitch.to_degrees());
                    self.camera
                        .look_at(target, ROBOT_FOCUS_DISTANCE, yaw, pitch);
                }
            }
            _ => {}
        }
    }

    fn update_stats(&mut self, now: Instant) {
        self.stats_frames += 1;
        let elapsed = (now - self.stats_since).as_secs_f32();
        if elapsed < STATS_INTERVAL_SECS {
            return;
        }
        let fps = self.stats_frames as f32 / elapsed;
        let frame_ms = 1000.0 / fps;
        let smoothing = if self.tesselator.marching_cubes_enabled() {
            "MC ON"
        } else {
            "MC OFF"
        };
        let (drawn, chunks) = self.renderer.drawn_chunks();
        let culling = if self.renderer.frustum_culling() {
            format!("CULL {drawn}/{chunks}")
        } else {
            "CULL OFF".to_owned()
        };
        self.overlay_text = format!("{:.0} FPS {:.1}MS {} {}", fps, frame_ms, smoothing, culling);
        if let Some(window) = &self.window {
            window.set_title(&format!(
                "r3v-renderer demo - {:.0} FPS ({:.1} ms) - {} quads re-meshed, {} chunks updated, {} of {} chunks drawn - {} background, colormap depth {:.1} m, {} coloring",
                fps,
                frame_ms,
                self.quad_count,
                self.chunk_count,
                drawn,
                chunks,
                BACKGROUNDS[self.background_index].0,
                self.color_depth_range,
                self.terrain_coloring_summary(),
            ));
        }
        self.quad_count = 0;
        self.chunk_count = 0;
        self.stats_frames = 0;
        self.stats_since = now;
    }
    fn present(&mut self) {
        let Some(surface) = self.surface.as_mut() else {
            return;
        };
        let mut buffer = surface.buffer_mut().expect("Failed to get surface buffer");
        for (dst, bgra) in buffer.iter_mut().zip(self.pixel_buffer.chunks_exact(4)) {
            *dst = u32::from_le_bytes([bgra[0], bgra[1], bgra[2], 0]);
        }
        buffer.present().expect("Failed to present surface buffer");
    }

    fn save_snapshot(&mut self) {
        let filename = format!("output_frame_{:02}.png", self.snapshot_index);
        save_frame(&filename, self.width, self.height, &self.pixel_buffer);
        println!("Saved {}", filename);
        self.snapshot_index += 1;
    }
}

impl ApplicationHandler for Demo {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attributes = Window::default_attributes()
            .with_title("r3v-renderer demo")
            .with_inner_size(PhysicalSize::new(INITIAL_WIDTH, INITIAL_HEIGHT));
        let window = Rc::new(
            event_loop
                .create_window(attributes)
                .expect("Failed to create window"),
        );
        let context =
            softbuffer::Context::new(window.clone()).expect("Failed to create softbuffer context");
        self.surface = Some(
            softbuffer::Surface::new(&context, window.clone()).expect("Failed to create surface"),
        );

        let size = window.inner_size();
        self.window = Some(window);
        self.resize(size.width, size.height);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => self.resize(size.width, size.height),
            WindowEvent::RedrawRequested => self.redraw(),
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: PhysicalKey::Code(code),
                        state,
                        repeat,
                        ..
                    },
                ..
            } => match state {
                ElementState::Pressed => {
                    self.input.keys.insert(code);
                    self.key_pressed(event_loop, code, repeat);
                }
                ElementState::Released => {
                    self.input.keys.remove(&code);
                }
            },
            WindowEvent::MouseInput { state, button, .. } => {
                let pressed = state == ElementState::Pressed;
                match button {
                    MouseButton::Left => self.input.orbiting = pressed,
                    MouseButton::Right | MouseButton::Middle => self.input.panning = pressed,
                    _ => {}
                }
            }
            WindowEvent::CursorMoved { position, .. } => self.cursor_moved(position),
            WindowEvent::CursorLeft { .. } => self.input.cursor = None,
            WindowEvent::MouseWheel { delta, .. } => self.zoom(match delta {
                MouseScrollDelta::LineDelta(_, y) => y,
                MouseScrollDelta::PixelDelta(position) => position.y as f32 / 40.0,
            }),
            WindowEvent::Focused(false) => {
                self.input = InputState::default();
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

/// Where the robot's root link sits in the voxel world: on the ground at its reserved site,
/// turned from URDF's Z-up into the world's Y-up.
/// Where the rover is at `t`: driving an ellipse across the world, facing along it, on the ground.
fn rover_placement(world: &World, t: f32) -> Placement {
    let radius = [WORLD_X, WORLD_Z].map(|extent| extent as f32 * ROVER_PATH_RADIUS);
    let angle = t * ROVER_SPEED / (radius[0].max(radius[1]) * VOXEL_SIZE);
    let (sin, cos) = angle.sin_cos();
    let x = WORLD_X as f32 / 2.0 + radius[0] * cos;
    let z = WORLD_Z as f32 / 2.0 + radius[1] * sin;
    // The URDF forward axis (+x) is turned towards the path tangent.
    let heading = (-radius[1] * cos).atan2(-radius[0] * sin);
    Placement {
        translation: Vector3::new(x, world.ground_height(x, z), z) * VOXEL_SIZE,
        rotation: cgmath::Quaternion::from_angle_y(cgmath::Rad(heading))
            * cgmath::Quaternion::from_angle_x(cgmath::Deg(-90.0)),
        scale: ROBOT_SCALE,
    }
}

/// Maps the robot frame into the world: scale, then rotate, then translate.
struct Placement {
    translation: Vector3<f32>,
    rotation: cgmath::Quaternion<f32>,
    scale: f32,
}

impl Placement {
    fn point(&self, point: Vector3<f32>) -> Vector3<f32> {
        self.translation + self.rotation * (point * self.scale)
    }

    /// The scale is baked into the uploaded vertices, so only the translation is scaled here.
    fn twist(&self, twist: RealTwist) -> RealTwist {
        let linear = self.point(Vector3::new(twist.linear.x, twist.linear.y, twist.linear.z));
        let q = twist.rotational;
        let rotation = (self.rotation * cgmath::Quaternion::new(q.w, q.x, q.y, q.z)).normalize();
        RealTwist {
            linear: geometry::Vector3D {
                x: linear.x,
                y: linear.y,
                z: linear.z,
            },
            rotational: geometry::Quaternion {
                x: rotation.v.x,
                y: rotation.v.y,
                z: rotation.v.z,
                w: rotation.s,
            },
        }
    }
}

/// How the demo drives a joint over time.
enum JointMotion {
    /// Spins forever, like a wheel.
    Spin { phase: f32 },
    /// Sweeps back and forth between its limits.
    Sweep {
        phase: f32,
        center: f32,
        amplitude: f32,
    },
}

/// A URDF robot whose meshes live on the GPU as asset instances, one per visual.
struct RobotScene {
    robot: UrdfRobot,
    placement: Placement,
    joints: Vec<(String, JointMotion)>,
    joint_positions: HashMap<String, f32>,
    /// Renderer instance for each visual, keyed like `RobotTransforms::visuals`.
    instances: Vec<((String, usize), u64)>,
    /// Renderer instances of the visuals on the arm's links.
    arm_instances: Vec<u64>,
    arm_attached: bool,
    focus_link: Option<String>,
}

impl RobotScene {
    fn load(
        urdf_path: &Path,
        placement: Placement,
        renderer: &mut WgpuAccelerated,
    ) -> Result<Self, String> {
        let load_start = Instant::now();
        let description = urdf::read_file(urdf_path)?;
        let urdf_dir = urdf_path.parent().unwrap_or(Path::new("."));

        type Visuals = Vec<((String, usize), Option<[f32; 3]>)>;
        let mut groups: Vec<(&urdf::UrdfGeometry, Visuals)> = Vec::new();
        let mut skipped = 0;
        for link in &description.links {
            for (visual_index, visual) in link.visuals.iter().enumerate() {
                let Some(geometry) = &visual.geometry else {
                    skipped += 1;
                    continue;
                };
                let color = visual
                    .material
                    .as_ref()
                    .and_then(|material| material.color.as_ref())
                    .map(|color| [color.rgba[0], color.rgba[1], color.rgba[2]]);
                let visual_key = ((link.name.clone(), visual_index), color);
                match groups.iter_mut().find(|(group, _)| *group == geometry) {
                    Some((_, visuals)) => visuals.push(visual_key),
                    None => groups.push((geometry, vec![visual_key])),
                }
            }
        }

        let mut instances = Vec::new();
        let mut triangle_count = 0;
        for (geometry, visuals) in groups {
            let mut mesh = match geometry {
                urdf::UrdfGeometry::Mesh(mesh) => {
                    let path = resolve_mesh_file(&mesh.filename, urdf_dir);
                    let mut stl_mesh = stl::read_file(&path)?;
                    let scale = mesh
                        .scale
                        .map_or([1.0; 3], |scale| [scale.x, scale.y, scale.z]);
                    scale_mesh(&mut stl_mesh, scale);
                    stl_mesh
                }
                primitive => shapes::tessellate(primitive).expect("only meshes are files"),
            };
            scale_mesh(&mut mesh, [placement.scale; 3]);
            triangle_count += mesh.triangles.len();
            let is_shape = !matches!(geometry, urdf::UrdfGeometry::Mesh(_));
            let ids = renderer.upload_asset(mesh, visuals.len());
            for ((visual_key, color), id) in visuals.into_iter().zip(ids) {
                if let Some(color) = color {
                    renderer.set_asset_color(id, color);
                }
                if is_shape && !ROBOT_SHOW_SHAPES {
                    renderer.set_asset_visibility(id, false);
                }
                instances.push((visual_key, id));
            }
        }

        let parent_links = description
            .joints
            .iter()
            .map(|joint| joint.parent.as_str())
            .collect::<HashSet<_>>();
        let joints = description
            .joints
            .iter()
            .enumerate()
            .filter_map(|(index, joint)| {
                let phase = index as f32 * 0.7;
                let motion = match (joint.joint_type.as_str(), joint.limit) {
                    // Only leaf links (wheels) spin freely; spinning a rocker would lift its wheels off the ground.
                    ("continuous", _) if !parent_links.contains(joint.child.as_str()) => {
                        JointMotion::Spin { phase }
                    }
                    ("continuous", _) => JointMotion::Sweep {
                        phase,
                        center: 0.0,
                        amplitude: ROBOT_CONTINUOUS_SWEEP,
                    },
                    ("revolute", Some(limit)) if limit.upper > limit.lower => JointMotion::Sweep {
                        phase,
                        center: (limit.upper + limit.lower) * 0.5,
                        amplitude: (limit.upper - limit.lower) * 0.5,
                    },
                    _ => return None,
                };
                Some((joint.name.clone(), motion))
            })
            .collect::<Vec<_>>();

        let arm_links = arm_links(&description);
        let arm_instances = instances
            .iter()
            .filter(|((link, _), _)| arm_links.contains(link.as_str()))
            .map(|(_, instance)| *instance)
            .collect();

        let focus_link = description
            .links
            .iter()
            .find(|link| !link.visuals.is_empty())
            .map(|link| link.name.clone());
        println!(
            "Loaded robot '{}' in {:.0} ms: {} visuals ({} without geometry skipped), {} triangles, {} animated joints",
            description.name,
            load_start.elapsed().as_secs_f32() * 1000.0,
            instances.len(),
            skipped,
            triangle_count,
            joints.len(),
        );

        Ok(Self {
            robot: UrdfRobot::from_description(description),
            placement,
            joints,
            joint_positions: HashMap::new(),
            instances,
            arm_instances,
            arm_attached: true,
            focus_link,
        })
    }

    fn toggle_arm(&mut self, renderer: &mut WgpuAccelerated) {
        if self.arm_instances.is_empty() {
            println!("Robot has no arm (no '{ROBOT_ARM_MOUNT_JOINT}' joint with visuals)");
            return;
        }
        self.arm_attached = !self.arm_attached;
        for &instance in &self.arm_instances {
            renderer.set_asset_visibility(instance, self.arm_attached);
        }
    }

    fn update(&mut self, t: f32, placement: Placement, renderer: &mut WgpuAccelerated) {
        self.placement = placement;
        for (name, motion) in &self.joints {
            let position = match *motion {
                JointMotion::Spin { phase } => t * ROBOT_JOINT_SPEED * 2.0 + phase,
                JointMotion::Sweep {
                    phase,
                    center,
                    amplitude,
                } => center + amplitude * (t * ROBOT_JOINT_SPEED + phase).sin(),
            };
            self.joint_positions.insert(name.clone(), position);
        }
        let transforms = self.robot.feed_sensor_input(&self.joint_positions);
        for (visual, instance) in &self.instances {
            if let Some(&twist) = transforms.visuals.get(visual) {
                renderer.set_asset_transform(*instance, self.placement.twist(twist));
            }
        }
    }

    fn focus_point(&self) -> Option<geometry::Point<f32>> {
        let origin = self
            .robot
            .last_transform()?
            .get_link_origin(self.focus_link.as_deref()?)?;
        let world = self
            .placement
            .point(Vector3::new(origin.x, origin.y, origin.z));
        Some(geometry::Point {
            x: world.x,
            y: world.y,
            z: world.z,
        })
    }
}

/// Links below the arm mount joint, i.e. the whole manipulator.
fn arm_links(description: &urdf::UrdfRobot) -> HashSet<&str> {
    let mut links = HashSet::new();
    let mut pending = description
        .joints
        .iter()
        .filter(|joint| joint.name == ROBOT_ARM_MOUNT_JOINT)
        .map(|joint| joint.child.as_str())
        .collect::<Vec<_>>();
    while let Some(link) = pending.pop() {
        if links.insert(link) {
            pending.extend(
                description
                    .joints
                    .iter()
                    .filter(|joint| joint.parent == link)
                    .map(|joint| joint.child.as_str()),
            );
        }
    }
    links
}

/// `urdf::read_file` resolves `package://<name>/...` only when an ancestor directory is called
/// `<name>`. Otherwise this falls back to the path relative to the URDF file, which is how the
/// bundled assets are laid out.
fn resolve_mesh_file(filename: &str, urdf_dir: &Path) -> std::path::PathBuf {
    match filename
        .strip_prefix("package://")
        .and_then(|uri| uri.split_once('/'))
    {
        Some((_, relative_path)) => urdf_dir.join(relative_path),
        None => Path::new(filename).to_path_buf(),
    }
}

fn scale_mesh(mesh: &mut stl::StlMesh, scale: [f32; 3]) {
    let mirrored = scale[0] * scale[1] * scale[2] < 0.0;
    for triangle in &mut mesh.triangles {
        for vertex in [&mut triangle.v0, &mut triangle.v1, &mut triangle.v2] {
            for axis in 0..3 {
                vertex[axis] *= scale[axis];
            }
        }
        // A mirroring scale flips the winding, which would get the triangle back-face culled.
        if mirrored {
            *triangle = triangle.reverse();
        }
    }
}

/// Rotation by a fixed angle around one coordinate axis.
#[derive(Clone, Copy)]
struct Rotation {
    axis: usize,
    sin: f32,
    cos: f32,
}

impl Rotation {
    const NONE: Self = Self {
        axis: 0,
        sin: 0.0,
        cos: 1.0,
    };

    fn new(axis: usize, angle: f32) -> Self {
        let (sin, cos) = angle.sin_cos();
        Self { axis, sin, cos }
    }

    fn apply(&self, v: [f32; 3]) -> [f32; 3] {
        self.rotate(v, self.sin)
    }

    fn invert(&self, v: [f32; 3]) -> [f32; 3] {
        self.rotate(v, -self.sin)
    }

    fn rotate(&self, v: [f32; 3], sin: f32) -> [f32; 3] {
        let (u, w) = ((self.axis + 1) % 3, (self.axis + 2) % 3);
        let mut rotated = v;
        rotated[u] = v[u] * self.cos - v[w] * sin;
        rotated[w] = v[u] * sin + v[w] * self.cos;
        rotated
    }
}

/// A solid volume in voxel space; a voxel is filled when its center lies inside.
#[derive(Clone, Copy)]
enum Shape {
    Sphere {
        center: [f32; 3],
        radius: f32,
    },
    /// Box with half-extents `half`, rotated around its center.
    Cuboid {
        center: [f32; 3],
        half: [f32; 3],
        rotation: Rotation,
    },
    /// Ring lying in its local XZ plane, rotated around its center.
    Torus {
        center: [f32; 3],
        major: f32,
        minor: f32,
        rotation: Rotation,
    },
    /// Circular patch of water from `floor` up to an animated wavy surface around `level`.
    Water {
        center: [f32; 2],
        radius: f32,
        level: f32,
        floor: f32,
        t: f32,
    },
}

impl Shape {
    /// Axis-aligned bounding box as (min, max).
    fn bounds(&self) -> ([f32; 3], [f32; 3]) {
        let around = |center: [f32; 3], extent: [f32; 3]| {
            (
                array::from_fn(|a| center[a] - extent[a]),
                array::from_fn(|a| center[a] + extent[a]),
            )
        };
        match *self {
            Shape::Sphere { center, radius } => around(center, [radius; 3]),
            Shape::Cuboid {
                center,
                half,
                rotation: Rotation { axis, sin, cos },
            } => {
                let (u, w) = ((axis + 1) % 3, (axis + 2) % 3);
                let mut extent = half;
                extent[u] = cos.abs() * half[u] + sin.abs() * half[w];
                extent[w] = sin.abs() * half[u] + cos.abs() * half[w];
                around(center, extent)
            }
            Shape::Torus {
                center,
                major,
                minor,
                ..
            } => around(center, [major + minor; 3]),
            Shape::Water {
                center: [x, z],
                radius,
                level,
                floor,
                ..
            } => (
                [x - radius, floor, z - radius],
                [x + radius, level + WAVE_AMPLITUDE, z + radius],
            ),
        }
    }

    fn contains(&self, p: [f32; 3]) -> bool {
        match *self {
            Shape::Sphere { center, radius } => distance_sq(p, center) < radius * radius,
            Shape::Cuboid {
                center,
                half,
                rotation,
            } => {
                let local = rotation.invert(sub(p, center));
                (0..3).all(|a| local[a].abs() <= half[a])
            }
            Shape::Torus {
                center,
                major,
                minor,
                rotation,
            } => {
                let [x, y, z] = rotation.invert(sub(p, center));
                let ring = (x * x + z * z).sqrt() - major;
                ring * ring + y * y < minor * minor
            }
            Shape::Water {
                center: [cx, cz],
                radius,
                level,
                floor,
                t,
            } => {
                let dist_sq = (p[0] - cx).powi(2) + (p[2] - cz).powi(2);
                p[1] >= floor
                    && dist_sq <= radius * radius
                    && p[1] < level + wave_height(p[0], p[2], dist_sq.sqrt(), t)
            }
        }
    }
}

/// Offset of the water surface from its rest level, within [-WAVE_AMPLITUDE, WAVE_AMPLITUDE].
fn wave_height(x: f32, z: f32, dist_from_center: f32, t: f32) -> f32 {
    1.2 * (0.21 * x + 1.1 * t).sin() * (0.17 * z + 0.8 * t).cos()
        + 0.8 * (0.45 * dist_from_center - 2.4 * t).sin()
}

/// A ball bouncing along a Lissajous path across the terrain.
struct Ball {
    radius: f32,
    amplitude: [f32; 2],
    frequency: [f32; 2],
    phase: [f32; 2],
    bounce_height: f32,
    bounce_frequency: f32,
}

impl Ball {
    fn random(seed: u32) -> Self {
        let r = |k: u32| hash2(seed as i32, k as i32, 0xba11);
        let radius = 2.5 + 4.0 * r(0);
        let margin = radius + 2.0;
        Self {
            radius,
            amplitude: [WORLD_X as f32 / 2.0 - margin, WORLD_Z as f32 / 2.0 - margin],
            frequency: [0.01 + 0.04 * r(1), 0.01 + 0.04 * r(2)],
            phase: [TAU * r(3), TAU * r(4)],
            bounce_height: 6.0 + 14.0 * r(5),
            bounce_frequency: 0.8 + 1.2 * r(6),
        }
    }

    fn shape(&self, t: f32, world: &World) -> Shape {
        let x = WORLD_X as f32 / 2.0
            + self.amplitude[0] * (self.frequency[0] * t + self.phase[0]).sin();
        let z = WORLD_Z as f32 / 2.0
            + self.amplitude[1] * (self.frequency[1] * t + self.phase[1]).sin();
        let ground = world.height_at(x as i32, z as i32) as f32;
        let y = ground
            + self.radius
            + self.bounce_height * (self.bounce_frequency * t + self.phase[0]).sin().abs();
        Shape::Sphere {
            center: [x, y.min(WORLD_Y as f32 - self.radius - 1.0), z],
            radius: self.radius,
        }
    }
}

/// A chain of shrinking spheres slithering along a figure-eight over the terrain.
struct Snake {
    home: [f32; 2],
    loop_radius: f32,
    angular_speed: f32,
    phase: f32,
    radius: f32,
}

impl Snake {
    fn random(seed: u32) -> Self {
        let r = |k: u32| hash2(seed as i32, k as i32, 0x5aa4e);
        let loop_radius = 30.0 + 40.0 * r(0);
        let margin = loop_radius + SNAKE_WIGGLE + 8.0;
        Self {
            home: [
                margin + (WORLD_X as f32 - 2.0 * margin).max(0.0) * r(1),
                margin + (WORLD_Z as f32 - 2.0 * margin).max(0.0) * r(2),
            ],
            loop_radius,
            angular_speed: (0.15 + 0.15 * r(3)) * if r(4) < 0.5 { -1.0 } else { 1.0 },
            phase: TAU * r(5),
            radius: 2.0 + 1.2 * r(6),
        }
    }

    fn push_shapes(&self, t: f32, world: &World, shapes: &mut Vec<Shape>) {
        // Delay each segment so neighbours stay roughly one radius apart.
        let lag = self.radius * 1.1 / (self.loop_radius * self.angular_speed.abs());
        for segment in 0..SNAKE_SEGMENTS {
            let a = self.angular_speed * (t - segment as f32 * lag) + self.phase;
            let x = self.home[0] + self.loop_radius * a.cos() + SNAKE_WIGGLE * (7.0 * a).sin();
            let z = self.home[1]
                + 0.6 * self.loop_radius * (2.0 * a).sin()
                + SNAKE_WIGGLE * (7.0 * a).cos();
            let radius = self.radius * (1.0 - 0.5 * segment as f32 / SNAKE_SEGMENTS as f32);
            let ground = world.height_at(x as i32, z as i32) as f32;
            shapes.push(Shape::Sphere {
                center: [x, ground + 0.8 * radius, z],
                radius,
            });
        }
    }
}

/// Four blades spinning around a horizontal hub on top of a static tower.
struct Windmill {
    hub: [f32; 3],
    speed: f32,
    phase: f32,
}

impl Windmill {
    fn push_shapes(&self, t: f32, shapes: &mut Vec<Shape>) {
        for blade in 0..4 {
            let rotation = Rotation::new(0, self.speed * t + self.phase + blade as f32 * TAU / 4.0);
            let offset = rotation.apply([0.0, WINDMILL_BLADE_LENGTH / 2.0 + 1.0, 0.0]);
            shapes.push(Shape::Cuboid {
                center: array::from_fn(|a| self.hub[a] + offset[a]),
                half: [0.6, WINDMILL_BLADE_LENGTH / 2.0, 1.6],
                rotation,
            });
        }
    }
}

/// Two nested rings hovering above the ground and tumbling around different axes.
struct Gyroscope {
    center: [f32; 3],
    speed: f32,
    phase: f32,
}

impl Gyroscope {
    fn push_shapes(&self, t: f32, shapes: &mut Vec<Shape>) {
        let a = self.speed * t + self.phase;
        let mut center = self.center;
        center[1] += 2.0 * (0.7 * t + self.phase).sin();
        shapes.push(Shape::Torus {
            center,
            major: GYRO_RADIUS,
            minor: 1.5,
            rotation: Rotation::new(0, a),
        });
        shapes.push(Shape::Torus {
            center,
            major: GYRO_RADIUS - 3.5,
            minor: 1.2,
            rotation: Rotation::new(2, -1.7 * a),
        });
    }
}

/// A grid of pillars pumping up and down in ripples spreading from the center.
struct PistonField {
    origin: [i32; 2],
    /// Ground height under each pillar, row-major.
    bases: Vec<i32>,
}

impl PistonField {
    fn push_shapes(&self, t: f32, shapes: &mut Vec<Shape>) {
        let middle = (PISTON_GRID - 1) as f32 / 2.0;
        for (i, &base) in self.bases.iter().enumerate() {
            let (col, row) = ((i % PISTON_GRID) as f32, (i / PISTON_GRID) as f32);
            let dist = ((col - middle).powi(2) + (row - middle).powi(2)).sqrt();
            let height = 1.0 + 6.0 * (1.0 + (2.2 * t - 0.9 * dist).sin());
            let x = self.origin[0] as f32 + col * PISTON_PITCH as f32 + PISTON_SIZE as f32 / 2.0;
            let z = self.origin[1] as f32 + row * PISTON_PITCH as f32 + PISTON_SIZE as f32 / 2.0;
            shapes.push(Shape::Cuboid {
                center: [x, base as f32 + height / 2.0, z],
                half: [
                    PISTON_SIZE as f32 / 2.0,
                    height / 2.0,
                    PISTON_SIZE as f32 / 2.0,
                ],
                rotation: Rotation::NONE,
            });
        }
    }
}

/// Water filling a terrain basin; only the band around the surface is animated.
struct Lake {
    center: [f32; 2],
    level: i32,
}

impl Lake {
    fn floor(&self) -> i32 {
        self.level - WAVE_AMPLITUDE.ceil() as i32
    }

    fn covers(&self, x: i32, z: i32) -> bool {
        let (dx, dz) = (
            x as f32 + 0.5 - self.center[0],
            z as f32 + 0.5 - self.center[1],
        );
        dx * dx + dz * dz <= LAKE_RADIUS * LAKE_RADIUS
    }

    fn shape(&self, t: f32) -> Shape {
        Shape::Water {
            center: self.center,
            radius: LAKE_RADIUS,
            level: self.level as f32,
            floor: self.floor() as f32,
            t,
        }
    }
}

/// Static procedural terrain with trees, plus lots of animated objects.
struct World {
    heights: Vec<i32>,
    static_bricks: Vec<u32>,
    /// Circles (center, radius) already taken by lakes and structures.
    reserved: Vec<([f32; 2], f32)>,
    balls: Vec<Ball>,
    snakes: Vec<Snake>,
    windmills: Vec<Windmill>,
    gyroscopes: Vec<Gyroscope>,
    piston_fields: Vec<PistonField>,
    lakes: Vec<Lake>,
    dynamic_bricks: Vec<u32>,
    dynamic_touched: Vec<usize>,
    /// Brick masks as last sent to the tesselator.
    sent_bricks: Vec<u32>,
}

impl World {
    fn new() -> Self {
        let mut world = Self {
            heights: Vec::with_capacity((WORLD_X * WORLD_Z) as usize),
            static_bricks: vec![0; (WORLD_BRICKS_X * WORLD_BRICKS_Y * WORLD_BRICKS_Z) as usize],
            reserved: Vec::new(),
            balls: (0..BALL_COUNT as u32).map(Ball::random).collect(),
            snakes: (0..SNAKE_COUNT as u32).map(Snake::random).collect(),
            windmills: Vec::new(),
            gyroscopes: Vec::new(),
            piston_fields: Vec::new(),
            lakes: Vec::new(),
            dynamic_bricks: vec![0; (WORLD_BRICKS_X * WORLD_BRICKS_Y * WORLD_BRICKS_Z) as usize],
            dynamic_touched: Vec::new(),
            sent_bricks: vec![0; (WORLD_BRICKS_X * WORLD_BRICKS_Y * WORLD_BRICKS_Z) as usize],
        };
        for z in 0..WORLD_Z {
            for x in 0..WORLD_X {
                world.heights.push(terrain_height(x, z));
            }
        }
        for z in 0..WORLD_Z {
            for x in 0..WORLD_X {
                for y in 0..world.height_at(x, z) {
                    world.set_static(x, y, z);
                }
            }
        }
        world.fill_lakes();
        world.build_windmills();
        world.build_piston_fields();
        world.place_gyroscopes();
        world.plant_trees();
        world
    }

    fn height_at(&self, x: i32, z: i32) -> i32 {
        let (x, z) = (x.clamp(0, WORLD_X - 1), z.clamp(0, WORLD_Z - 1));
        self.heights[(z * WORLD_X + x) as usize]
    }

    /// Terrain height, in voxels, averaged over the rover's footprint around `(x, z)`.
    fn ground_height(&self, x: f32, z: f32) -> f32 {
        let (x, z) = (x as i32, z as i32);
        let footprint = -ROVER_FOOTPRINT..=ROVER_FOOTPRINT;
        let total: i32 = footprint
            .clone()
            .flat_map(|dz| footprint.clone().map(move |dx| (dx, dz)))
            .map(|(dx, dz)| self.height_at(x + dx, z + dz))
            .sum();
        total as f32 / ((2 * ROVER_FOOTPRINT + 1) as f32).powi(2)
    }

    fn brick_index(bx: i32, by: i32, bz: i32) -> usize {
        ((bz * WORLD_BRICKS_Y + by) * WORLD_BRICKS_X + bx) as usize
    }

    fn set_static(&mut self, x: i32, y: i32, z: i32) {
        if !(0..WORLD_X).contains(&x) || !(0..WORLD_Y).contains(&y) || !(0..WORLD_Z).contains(&z) {
            return;
        }
        let index = Self::brick_index(x / BRICK_X, y / BRICK_Y, z / BRICK_Z);
        self.static_bricks[index] |= voxel_bit(x % BRICK_X, y % BRICK_Y, z % BRICK_Z);
    }

    fn is_free(&self, center: [f32; 2], radius: f32) -> bool {
        self.reserved.iter().all(|&(other, other_radius)| {
            (center[0] - other[0]).powi(2) + (center[1] - other[1]).powi(2)
                > (radius + other_radius).powi(2)
        })
    }

    /// Picks an unreserved spot on low ground for a structure of the given radius and reserves it.
    fn reserve_site(&mut self, seed: u32, radius: f32) -> Option<[i32; 2]> {
        let margin = radius.ceil() as i32 + 2;
        for attempt in 0..64 {
            let r = |k: i32| hash2(seed as i32, attempt * 2 + k, 0x517e);
            let x = margin + (r(0) * (WORLD_X - 2 * margin) as f32) as i32;
            let z = margin + (r(1) * (WORLD_Z - 2 * margin) as f32) as i32;
            let center = [x as f32 + 0.5, z as f32 + 0.5];
            if self.height_at(x, z) <= MAX_STRUCTURE_GROUND && self.is_free(center, radius) {
                self.reserved.push((center, radius));
                return Some([x, z]);
            }
        }
        None
    }

    /// Floods the deepest terrain basins up to their rim.
    fn fill_lakes(&mut self) {
        let radius = LAKE_RADIUS.ceil() as i32;
        let mut candidates = Vec::new();
        for cz in (radius + 2..WORLD_Z - radius - 2).step_by(16) {
            for cx in (radius + 2..WORLD_X - radius - 2).step_by(16) {
                let rim = (0..64)
                    .map(|k| {
                        let (sin, cos) = (k as f32 * TAU / 64.0).sin_cos();
                        self.height_at(
                            cx + (cos * LAKE_RADIUS) as i32,
                            cz + (sin * LAKE_RADIUS) as i32,
                        )
                    })
                    .min()
                    .unwrap();
                let bottom = (-radius..=radius)
                    .step_by(4)
                    .flat_map(|dz| (-radius..=radius).step_by(4).map(move |dx| (dx, dz)))
                    .filter(|&(dx, dz)| dx * dx + dz * dz <= radius * radius)
                    .map(|(dx, dz)| self.height_at(cx + dx, cz + dz))
                    .min()
                    .unwrap();
                if rim <= MAX_STRUCTURE_GROUND && rim - bottom >= 3 {
                    candidates.push((rim - bottom, [cx, cz], rim));
                }
            }
        }
        candidates.sort_by_key(|&(depth, ..)| std::cmp::Reverse(depth));

        for (_, [cx, cz], rim) in candidates {
            if self.lakes.len() == LAKE_COUNT {
                break;
            }
            let center = [cx as f32, cz as f32];
            if !self.is_free(center, LAKE_RADIUS + 8.0) {
                continue;
            }
            self.reserved.push((center, LAKE_RADIUS + 8.0));
            let lake = Lake {
                center,
                level: rim + 1,
            };
            for z in cz - radius..=cz + radius {
                for x in cx - radius..=cx + radius {
                    if !lake.covers(x, z)
                        || !(0..WORLD_X).contains(&x)
                        || !(0..WORLD_Z).contains(&z)
                    {
                        continue;
                    }
                    for y in self.height_at(x, z)..lake.floor() {
                        self.set_static(x, y, z);
                    }
                    // Let balls and snakes ride on the water.
                    let height = &mut self.heights[(z * WORLD_X + x) as usize];
                    *height = (*height).max(lake.level);
                }
            }
            self.lakes.push(lake);
        }
    }

    fn build_windmills(&mut self) {
        for i in 0..WINDMILL_COUNT as u32 {
            let Some([x, z]) = self.reserve_site(0x3111 + i, WINDMILL_BLADE_LENGTH + 2.0) else {
                continue;
            };
            let ground = self.height_at(x, z);
            let hub_y = ground + WINDMILL_BLADE_LENGTH as i32 + 4;
            // A 3x3 tower with a nacelle sticking out towards the blades.
            for y in ground..=hub_y {
                for dz in -1..=1 {
                    for dx in -1..=1 {
                        self.set_static(x + dx, y, z + dz);
                    }
                }
            }
            for dx in 2..=3 {
                self.set_static(x + dx, hub_y, z);
            }
            let r = |k: i32| hash2(x, z, 0x3112 + k as u32);
            self.windmills.push(Windmill {
                hub: [x as f32 + 4.5, hub_y as f32 + 0.5, z as f32 + 0.5],
                speed: 0.6 + 1.2 * r(0),
                phase: TAU * r(1),
            });
        }
    }

    fn build_piston_fields(&mut self) {
        let extent = (PISTON_GRID as i32 - 1) * PISTON_PITCH + PISTON_SIZE;
        for i in 0..PISTON_FIELD_COUNT as u32 {
            let Some([x, z]) = self.reserve_site(0x9157 + i, extent as f32 * 0.75) else {
                continue;
            };
            let origin = [x - extent / 2, z - extent / 2];
            let bases = (0..PISTON_GRID * PISTON_GRID)
                .map(|i| {
                    let col = origin[0] + (i % PISTON_GRID) as i32 * PISTON_PITCH + PISTON_SIZE / 2;
                    let row = origin[1] + (i / PISTON_GRID) as i32 * PISTON_PITCH + PISTON_SIZE / 2;
                    self.height_at(col, row)
                })
                .collect();
            self.piston_fields.push(PistonField { origin, bases });
        }
    }

    fn place_gyroscopes(&mut self) {
        for i in 0..GYRO_COUNT as u32 {
            let Some([x, z]) = self.reserve_site(0x6e40 + i, GYRO_RADIUS + 3.0) else {
                continue;
            };
            let ground = self.height_at(x, z) as f32;
            let r = |k: i32| hash2(x, z, 0x6e41 + k as u32);
            self.gyroscopes.push(Gyroscope {
                center: [x as f32 + 0.5, ground + GYRO_RADIUS + 6.0, z as f32 + 0.5],
                speed: (0.5 + 1.0 * r(0)) * if r(1) < 0.5 { -1.0 } else { 1.0 },
                phase: TAU * r(2),
            });
        }
    }

    fn plant_trees(&mut self) {
        for cell_z in 0..WORLD_Z / TREE_CELL {
            for cell_x in 0..WORLD_X / TREE_CELL {
                let r = |k: i32| hash2(cell_x, cell_z, 0x7ee5 + k as u32);
                if r(0) > 0.45 {
                    continue;
                }
                let x = cell_x * TREE_CELL + 3 + (r(1) * (TREE_CELL - 6) as f32) as i32;
                let z = cell_z * TREE_CELL + 3 + (r(2) * (TREE_CELL - 6) as f32) as i32;
                let ground = self.height_at(x, z);
                // No trees on the mountain tops, in lakes or inside structures.
                if ground > 30 || !self.is_free([x as f32 + 0.5, z as f32 + 0.5], 4.0) {
                    continue;
                }
                let trunk = 4 + (r(3) * 5.0) as i32;
                for y in ground..ground + trunk {
                    self.set_static(x, y, z);
                }
                let crown_radius = 2.5 + 2.0 * r(4);
                let reach = crown_radius.ceil() as i32;
                let crown_y = ground + trunk;
                for dz in -reach..=reach {
                    for dy in -reach..=reach {
                        for dx in -reach..=reach {
                            let dist_sq = (dx * dx + dy * dy + dz * dz) as f32;
                            if dist_sq <= crown_radius * crown_radius {
                                self.set_static(x + dx, crown_y + dy, z + dz);
                            }
                        }
                    }
                }
            }
        }
    }

    fn dynamic_shapes(&self, t: f32) -> Vec<Shape> {
        let mut shapes = Vec::new();
        shapes.extend(self.lakes.iter().map(|lake| lake.shape(t)));
        shapes.extend(self.balls.iter().map(|ball| ball.shape(t, self)));
        for snake in &self.snakes {
            snake.push_shapes(t, self, &mut shapes);
        }
        for windmill in &self.windmills {
            windmill.push_shapes(t, &mut shapes);
        }
        for gyroscope in &self.gyroscopes {
            gyroscope.push_shapes(t, &mut shapes);
        }
        for field in &self.piston_fields {
            field.push_shapes(t, &mut shapes);
        }
        shapes
    }

    /// Returns the bricks to send to the tesselator this frame, like a lidar scan: the bricks
    /// within `range` voxels of `sensor` that differ from what was last sent. Bricks out of
    /// range keep whatever was last seen.
    fn update(&mut self, t: f32, sensor: [f32; 3], range: f32) -> Vec<PointI> {
        let shapes = self.dynamic_shapes(t);
        let previous = std::mem::take(&mut self.dynamic_touched);
        for &index in &previous {
            self.dynamic_bricks[index] = 0;
        }
        // Shapes rasterize in parallel against the static world; the merge is cheap and serial.
        let masks: Vec<Vec<(usize, u32)>> = shapes
            .par_iter()
            .map(|shape| self.rasterize(shape))
            .collect();
        for (index, mask) in masks.into_iter().flatten() {
            if self.dynamic_bricks[index] == 0 {
                self.dynamic_touched.push(index);
            }
            self.dynamic_bricks[index] |= mask;
        }

        let brick_size = [BRICK_X, BRICK_Y, BRICK_Z];
        let bmin = brick_of(sensor.map(|c| (c - range).floor() as i32));
        let bmax = brick_of(sensor.map(|c| (c + range).ceil() as i32));
        let mut delta = Vec::new();
        for bz in bmin[2]..=bmax[2] {
            for by in bmin[1]..=bmax[1] {
                for bx in bmin[0]..=bmax[0] {
                    let center = array::from_fn(|axis| {
                        ([bx, by, bz][axis] as f32 + 0.5) * brick_size[axis] as f32
                    });
                    if distance_sq(center, sensor) > range * range {
                        continue;
                    }
                    let index = Self::brick_index(bx, by, bz);
                    let mask = self.static_bricks[index] | self.dynamic_bricks[index];
                    if mask != self.sent_bricks[index] {
                        self.sent_bricks[index] = mask;
                        delta.push(brick_point(index, mask));
                    }
                }
            }
        }
        delta
    }

    /// Returns the voxels inside `shape` that are not static, as masks per touched brick.
    fn rasterize(&self, shape: &Shape) -> Vec<(usize, u32)> {
        let (min, max) = shape.bounds();
        let (min, max) = (
            clamp_voxel(min.map(|c| c.floor() as i32)),
            clamp_voxel(max.map(|c| c.ceil() as i32)),
        );
        let (bmin, bmax) = (brick_of(min), brick_of(max));
        // Split by brick slab too, so one large shape (a lake) does not serialize the frame.
        (bmin[2]..=bmax[2])
            .into_par_iter()
            .flat_map_iter(|bz| {
                (bmin[1]..=bmax[1]).flat_map(move |by| {
                    (bmin[0]..=bmax[0]).filter_map(move |bx| {
                        let index = Self::brick_index(bx, by, bz);
                        let origin = [bx * BRICK_X, by * BRICK_Y, bz * BRICK_Z];
                        let filled = self.static_bricks[index];
                        let (lo, hi) = (
                            array::from_fn::<_, 3, _>(|axis| (min[axis] - origin[axis]).max(0)),
                            array::from_fn::<_, 3, _>(|axis| {
                                (max[axis] - origin[axis])
                                    .min([BRICK_X, BRICK_Y, BRICK_Z][axis] - 1)
                            }),
                        );
                        let mut mask = 0;
                        for lz in lo[2]..=hi[2] {
                            for ly in lo[1]..=hi[1] {
                                for lx in lo[0]..=hi[0] {
                                    let bit = voxel_bit(lx, ly, lz);
                                    if filled & bit != 0 {
                                        continue;
                                    }
                                    let p = [0, 1, 2].map(|axis| {
                                        (origin[axis] + [lx, ly, lz][axis]) as f32 + 0.5
                                    });
                                    if shape.contains(p) {
                                        mask |= bit;
                                    }
                                }
                            }
                        }
                        (mask != 0).then_some((index, mask))
                    })
                })
            })
            .collect()
    }
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    array::from_fn(|i| a[i] - b[i])
}

fn distance_sq(a: [f32; 3], b: [f32; 3]) -> f32 {
    sub(a, b).iter().map(|d| d * d).sum()
}

fn voxel_bit(lx: i32, ly: i32, lz: i32) -> u32 {
    1 << (lx | ly << 2 | lz << 4)
}

fn clamp_voxel(v: [i32; 3]) -> [i32; 3] {
    [
        v[0].clamp(0, WORLD_X - 1),
        v[1].clamp(0, WORLD_Y - 1),
        v[2].clamp(0, WORLD_Z - 1),
    ]
}

/// Brick coordinates containing voxel `v`, clamped to the world.
fn brick_of(v: [i32; 3]) -> [i32; 3] {
    let [x, y, z] = clamp_voxel(v);
    [x / BRICK_X, y / BRICK_Y, z / BRICK_Z]
}

/// Brick at `index` into `World::static_bricks`, as sent to the tesselator.
fn brick_point(index: usize, mask: u32) -> PointI {
    let index = index as i32;
    let (bx, by, bz) = (
        index % WORLD_BRICKS_X,
        index / WORLD_BRICKS_X % WORLD_BRICKS_Y,
        index / (WORLD_BRICKS_X * WORLD_BRICKS_Y),
    );
    PointI {
        x: bx * BRICK_X,
        y: by * BRICK_Y,
        z: bz * BRICK_Z,
        i: mask,
    }
}

/// Deterministic hash of a 2D lattice point to [0, 1).
fn hash2(x: i32, z: i32, seed: u32) -> f32 {
    let mut h = (x as u32).wrapping_mul(0x8da6_b343)
        ^ (z as u32).wrapping_mul(0xd816_3841)
        ^ seed.wrapping_mul(0xcb1a_b31f);
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^= h >> 15;
    (h & 0x00ff_ffff) as f32 / 16_777_216.0
}

fn value_noise(x: f32, z: f32, seed: u32) -> f32 {
    let (x0, z0) = (x.floor(), z.floor());
    let (fx, fz) = (x - x0, z - z0);
    let (sx, sz) = (fx * fx * (3.0 - 2.0 * fx), fz * fz * (3.0 - 2.0 * fz));
    let (ix, iz) = (x0 as i32, z0 as i32);
    let lerp = |a: f32, b: f32, t: f32| a + (b - a) * t;
    lerp(
        lerp(hash2(ix, iz, seed), hash2(ix + 1, iz, seed), sx),
        lerp(hash2(ix, iz + 1, seed), hash2(ix + 1, iz + 1, seed), sx),
        sz,
    )
}

/// Fractal value noise in [0, 1).
fn fbm(x: f32, z: f32, seed: u32) -> f32 {
    let (mut sum, mut amplitude, mut frequency, mut total) = (0.0, 0.5, 1.0, 0.0);
    for octave in 0..5 {
        sum += amplitude * value_noise(x * frequency, z * frequency, seed + octave);
        total += amplitude;
        amplitude *= 0.5;
        frequency *= 2.0;
    }
    sum / total
}

/// Rolling hills with occasional mountain ranges.
fn terrain_height(x: i32, z: i32) -> i32 {
    let (fx, fz) = (x as f32 / 96.0, z as f32 / 96.0);
    let hills = fbm(fx, fz, 1);
    let ranges = ((fbm(fx * 0.5 + 17.3, fz * 0.5 - 4.1, 101) - 0.42) * 3.5).clamp(0.0, 1.0);
    (2.0 + 18.0 * hills + 38.0 * ranges * ranges) as i32
}

/// 3x5 bitmap glyphs, one row per byte, high bit on the left.
fn glyph(c: char) -> [u8; 5] {
    match c {
        '0' => [0b111, 0b101, 0b101, 0b101, 0b111],
        '1' => [0b010, 0b110, 0b010, 0b010, 0b111],
        '2' => [0b111, 0b001, 0b111, 0b100, 0b111],
        '3' => [0b111, 0b001, 0b111, 0b001, 0b111],
        '4' => [0b101, 0b101, 0b111, 0b001, 0b001],
        '5' | 'S' => [0b111, 0b100, 0b111, 0b001, 0b111],
        '6' => [0b111, 0b100, 0b111, 0b101, 0b111],
        '7' => [0b111, 0b001, 0b001, 0b001, 0b001],
        '8' => [0b111, 0b101, 0b111, 0b101, 0b111],
        '9' => [0b111, 0b101, 0b111, 0b001, 0b111],
        '.' => [0b000, 0b000, 0b000, 0b000, 0b010],
        'F' => [0b111, 0b100, 0b110, 0b100, 0b100],
        'P' => [0b111, 0b101, 0b111, 0b100, 0b100],
        'M' => [0b101, 0b111, 0b111, 0b101, 0b101],
        'C' => [0b111, 0b100, 0b100, 0b100, 0b111],
        'O' => [0b111, 0b101, 0b101, 0b101, 0b111],
        'N' => [0b110, 0b101, 0b101, 0b101, 0b101],
        'U' => [0b101, 0b101, 0b101, 0b101, 0b111],
        'L' => [0b100, 0b100, 0b100, 0b100, 0b111],
        '/' => [0b001, 0b001, 0b010, 0b100, 0b100],
        _ => [0; 5],
    }
}

/// Draws `text` into a BGRA buffer on top of a dark backing box.
fn draw_text(bgra_pixels: &mut [u8], width: usize, height: usize, x: usize, y: usize, text: &str) {
    if text.is_empty() {
        return;
    }
    let advance = 4 * OVERLAY_SCALE;
    let pad = OVERLAY_SCALE * 2;
    let box_w = text.chars().count() * advance - OVERLAY_SCALE + 2 * pad;
    let box_h = 5 * OVERLAY_SCALE + 2 * pad;

    let mut put = |px: usize, py: usize, bgra: [u8; 4]| {
        if px < width && py < height {
            let offset = (py * width + px) * 4;
            bgra_pixels[offset..offset + 4].copy_from_slice(&bgra);
        }
    };

    for py in y..y + box_h {
        for px in x..x + box_w {
            put(px, py, [24, 24, 24, 255]);
        }
    }
    for (i, c) in text.chars().enumerate() {
        let glyph_x = x + pad + i * advance;
        for (row, bits) in glyph(c).iter().enumerate() {
            for col in 0..3 {
                if bits & (0b100 >> col) == 0 {
                    continue;
                }
                for sy in 0..OVERLAY_SCALE {
                    for sx in 0..OVERLAY_SCALE {
                        put(
                            glyph_x + col * OVERLAY_SCALE + sx,
                            y + pad + row * OVERLAY_SCALE + sy,
                            [80, 255, 80, 255],
                        );
                    }
                }
            }
        }
    }
}

fn save_frame(filename: &str, width: u32, height: u32, bgra_pixels: &[u8]) {
    let mut rgba_pixels = bgra_pixels.to_vec();
    for chunk in rgba_pixels.chunks_exact_mut(4) {
        chunk.swap(0, 2);
    }
    let img: ImageBuffer<Rgba<u8>, _> =
        ImageBuffer::from_raw(width, height, rgba_pixels).expect("Failed to create image buffer");
    img.save(filename).expect("Failed to save image");
}
