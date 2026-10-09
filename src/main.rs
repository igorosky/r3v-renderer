use r3v_renderer::{
    Camera, Tesselator, Renderer,
    geometry::PointI,
    renderer::{self, WgpuAccelerated},
    tesselator::{GreedyMeshTesselator},
    camera::{OrbitCamera},
};
use image::{ImageBuffer, Rgba};
use std::{num::NonZeroU32, rc::Rc, time::Instant};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalSize,
    event::{ElementState, KeyEvent, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    keyboard::{Key, NamedKey},
    window::{Window, WindowId},
};

const INITIAL_WIDTH: u32 = 1200;
const INITIAL_HEIGHT: u32 = 800;

// Chunk layout used by GreedyMeshTesselator: 4x4x2 voxels packed into a u32,
// bit = lx | ly << 2 | lz << 4.
const CHUNK_X: i32 = 4;
const CHUNK_Y: i32 = 4;
const CHUNK_Z: i32 = 2;

const WORLD_CHUNKS_X: i32 = 16;
const WORLD_CHUNKS_Y: i32 = 4;
const WORLD_CHUNKS_Z: i32 = 16;
const WORLD_X: i32 = WORLD_CHUNKS_X * CHUNK_X;
const WORLD_Y: i32 = WORLD_CHUNKS_Y * CHUNK_Y;
const WORLD_Z: i32 = WORLD_CHUNKS_Z * CHUNK_Z;

const CAMERA_DISTANCE: f32 = 70.0;
const CAMERA_ZOOM_AMPLITUDE: f32 = 10.0;

const STATS_INTERVAL_SECS: f32 = 0.25;
const OVERLAY_SCALE: usize = 4;

fn main() -> anyhow::Result<()> {
    let event_loop = EventLoop::new()?;
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut demo = Demo::new();
    event_loop.run_app(&mut demo)?;
    Ok(())
}

struct Demo {
    window: Option<Rc<Window>>,
    surface: Option<softbuffer::Surface<Rc<Window>, Rc<Window>>>,

    camera: OrbitCamera,
    tesselator: GreedyMeshTesselator,
    renderer: WgpuAccelerated,

    width: u32,
    height: u32,
    pixel_buffer: Vec<u8>,

    start: Instant,
    last_frame: Instant,
    stats_frames: u32,
    stats_since: Instant,
    overlay_text: String,

    triangle_count: usize,
    chunk_count: usize,
    snapshot_index: u32,
}

impl Demo {
    fn new() -> Self {
        let center = cgmath::Point3::new(WORLD_X as f32 / 2.0, WORLD_Y as f32 / 3.0, WORLD_Z as f32 / 2.0);
        let camera = OrbitCamera::new(center, CAMERA_DISTANCE, 30.0, 25.0);
        let tesselator = GreedyMeshTesselator::new();
        let renderer = WgpuAccelerated::new().expect("Failed to initialize WgpuAccelerated");
        let now = Instant::now();

        Self {
            window: None,
            surface: None,
            camera,
            tesselator,
            renderer,
            width: 0,
            height: 0,
            pixel_buffer: Vec::new(),
            start: now,
            last_frame: now,
            stats_frames: 0,
            stats_since: now,
            overlay_text: String::new(),
            triangle_count: 0,
            chunk_count: 0,
            snapshot_index: 0,
        }
    }

    fn resize(&mut self, width: u32, height: u32) {
        let (Some(surface), Some(w), Some(h)) = (self.surface.as_mut(), NonZeroU32::new(width), NonZeroU32::new(height)) else {
            return;
        };
        surface.resize(w, h).expect("Failed to resize surface");
        self.renderer.configure(renderer::WgpuAcceleratedConfiguration { width, height });
        self.pixel_buffer.resize((width * height * 4) as usize, 0);
        self.width = width;
        self.height = height;
    }

    fn redraw(&mut self) {
        if self.width == 0 || self.height == 0 {
            return;
        }
        let now = Instant::now();
        let t = (now - self.start).as_secs_f32();
        let dt = (now - self.last_frame).as_secs_f32();
        self.last_frame = now;

        // Dynamic voxel update -> tesselation -> renderer geometry update.
        let voxels = generate_world(t);
        self.tesselator.update_input_geometry(&voxels);
        if let Some(geometry_delta) = self.tesselator.get_output_geometry_delta() {
            let non_empty = geometry_delta.iter().filter(|delta| !delta.triangles.is_empty());
            self.chunk_count = non_empty.clone().count();
            self.triangle_count = non_empty.map(|delta| delta.triangles.len()).sum();
            self.renderer.update_geometry(geometry_delta);
        }

        // Slow orbit with a gentle pitch bob and dolly in/out.
        self.camera.rotate(0.4 * dt, 0.075 * (0.5 * t).cos() * dt);
        self.camera.zoom(CAMERA_ZOOM_AMPLITUDE * 0.3 * (0.3 * t).cos() * dt);

        let aspect_ratio = self.width as f32 / self.height as f32;
        let camera_uniform = self.camera.get_uniform(aspect_ratio);
        self.renderer.render(&camera_uniform, &mut self.pixel_buffer);

        self.update_stats(now);
        draw_text(&mut self.pixel_buffer, self.width as usize, self.height as usize, 12, 12, &self.overlay_text);
        self.present();
    }

    fn update_stats(&mut self, now: Instant) {
        self.stats_frames += 1;
        let elapsed = (now - self.stats_since).as_secs_f32();
        if elapsed < STATS_INTERVAL_SECS {
            return;
        }
        let fps = self.stats_frames as f32 / elapsed;
        let frame_ms = 1000.0 / fps;
        self.overlay_text = format!("{:.0} FPS {:.1}MS", fps, frame_ms);
        if let Some(window) = &self.window {
            window.set_title(&format!(
                "r3v-renderer demo - {:.0} FPS ({:.1} ms) - {} triangles, {} chunks",
                fps, frame_ms, self.triangle_count, self.chunk_count,
            ));
        }
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
        let window = Rc::new(event_loop.create_window(attributes).expect("Failed to create window"));
        let context = softbuffer::Context::new(window.clone()).expect("Failed to create softbuffer context");
        self.surface = Some(softbuffer::Surface::new(&context, window.clone()).expect("Failed to create surface"));

        let size = window.inner_size();
        self.window = Some(window);
        self.resize(size.width, size.height);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _window_id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => self.resize(size.width, size.height),
            WindowEvent::RedrawRequested => self.redraw(),
            WindowEvent::KeyboardInput {
                event: KeyEvent { logical_key, state: ElementState::Pressed, repeat: false, .. },
                ..
            } => match logical_key.as_ref() {
                Key::Named(NamedKey::Escape) => event_loop.exit(),
                Key::Character(c) if c.eq_ignore_ascii_case("s") => self.save_snapshot(),
                _ => {}
            },
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

/// Animated rolling terrain with a ball bouncing across it.
fn generate_world(t: f32) -> Vec<PointI> {
    let ball_x = WORLD_X as f32 / 2.0 + 20.0 * (0.4 * t).cos();
    let ball_z = WORLD_Z as f32 / 2.0 + 10.0 * (0.4 * t).sin();
    let ball_y = 5.0 + 7.0 * (1.5 * t).sin().abs();
    let ball_radius_sq = 3.0f32 * 3.0;

    let is_solid = |x: i32, y: i32, z: i32| {
        let (fx, fy, fz) = (x as f32 + 0.5, y as f32 + 0.5, z as f32 + 0.5);
        let height = 5.0 + 3.0 * (fx * 0.2 + t).sin() + 2.0 * (fz * 0.25 - 0.7 * t).cos();
        let (dx, dy, dz) = (fx - ball_x, fy - ball_y, fz - ball_z);
        fy < height || dx * dx + dy * dy + dz * dz < ball_radius_sq
    };

    let mut chunks = Vec::new();
    for cz in 0..WORLD_CHUNKS_Z {
        for cy in 0..WORLD_CHUNKS_Y {
            for cx in 0..WORLD_CHUNKS_X {
                let (ox, oy, oz) = (cx * CHUNK_X, cy * CHUNK_Y, cz * CHUNK_Z);
                let mut mask = 0u32;
                for lz in 0..CHUNK_Z {
                    for ly in 0..CHUNK_Y {
                        for lx in 0..CHUNK_X {
                            if is_solid(ox + lx, oy + ly, oz + lz) {
                                mask |= 1 << (lx | ly << 2 | lz << 4);
                            }
                        }
                    }
                }
                if mask != 0 {
                    chunks.push(PointI { x: ox, y: oy, z: oz, i: mask });
                }
            }
        }
    }
    chunks
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
                        put(glyph_x + col * OVERLAY_SCALE + sx, y + pad + row * OVERLAY_SCALE + sy, [80, 255, 80, 255]);
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
    let img: ImageBuffer<Rgba<u8>, _> = ImageBuffer::from_raw(width, height, rgba_pixels)
        .expect("Failed to create image buffer");
    img.save(filename).expect("Failed to save image");
}
