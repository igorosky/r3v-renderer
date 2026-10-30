use std::time::{Duration, Instant};

use r3v_renderer::{
    Camera, Renderer, Tesselator,
    camera::OrbitCamera,
    geometry::PointI,
    renderer::{
        DEFAULT_BACKGROUND_COLOR, DEFAULT_COLOR_DEPTH_RANGE, WgpuAccelerated,
        WgpuAcceleratedConfiguration,
    },
    tesselator::{CpuTesselator, CpuTesselatorConfiguration, CpuTesselatorContext},
};

const ALL_CHUNKS: CpuTesselatorContext = CpuTesselatorContext {
    camera_frustum: None,
};

const BRICK: [i32; 3] = [4, 4, 2];
const WORLD: [i32; 3] = [512, 64, 512];
const VOXEL_SIZE: f32 = 0.2;
const SIZE: [u32; 2] = [1200, 800];
const NEAR: f32 = 0.01;
const FAR: f32 = 2000.0 * VOXEL_SIZE;
const FRAMES: usize = 60;

fn ground(x: i32, z: i32) -> i32 {
    let (x, z) = (x as f32, z as f32);
    let rolling = 8.0 * (x / 23.0).sin() * (z / 31.0).cos() + 4.0 * ((x + z) / 11.0).sin();
    24 + rolling as i32
}

fn world() -> Vec<PointI> {
    let mut bricks = Vec::new();
    for bz in (0..WORLD[2]).step_by(BRICK[2] as usize) {
        for by in (0..WORLD[1]).step_by(BRICK[1] as usize) {
            for bx in (0..WORLD[0]).step_by(BRICK[0] as usize) {
                let mut i = 0;
                for lz in 0..BRICK[2] {
                    for ly in 0..BRICK[1] {
                        for lx in 0..BRICK[0] {
                            if by + ly <= ground(bx + lx, bz + lz) {
                                i |= 1 << (lx | ly << 2 | lz << 4);
                            }
                        }
                    }
                }
                bricks.push(PointI {
                    x: bx,
                    y: by,
                    z: bz,
                    i,
                });
            }
        }
    }
    bricks
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
}

fn frames(renderer: &mut WgpuAccelerated, distance: f32, pitch: f32) -> (f64, u32) {
    let centre = cgmath::Point3::new(
        WORLD[0] as f32 / 2.0 * VOXEL_SIZE,
        WORLD[1] as f32 / 4.0 * VOXEL_SIZE,
        WORLD[2] as f32 / 2.0 * VOXEL_SIZE,
    );
    let mut camera = OrbitCamera::new(centre, distance, 0.0, pitch).with_clip_planes(NEAR, FAR);
    let mut output = vec![0; (SIZE[0] * SIZE[1] * 4) as usize];
    let aspect = SIZE[0] as f32 / SIZE[1] as f32;
    renderer.render(&camera.get_state(aspect), &mut output);
    let mut times = Vec::with_capacity(FRAMES);
    let mut drawn = 0;
    for _ in 0..FRAMES {
        camera.rotate(std::f32::consts::TAU / FRAMES as f32, 0.0);
        let start = Instant::now();
        renderer.render(&camera.get_state(aspect), &mut output);
        times.push(start.elapsed());
        drawn += renderer.drawn_chunks().0;
    }
    times.sort();
    (millis(times[FRAMES / 2]), drawn / FRAMES as u32)
}

fn main() {
    let adapter = pollster::block_on(
        wgpu::Instance::default().request_adapter(&wgpu::RequestAdapterOptions::default()),
    )
    .expect("no GPU");
    let info = adapter.get_info();
    println!("{} ({:?}, {:?})", info.name, info.device_type, info.backend);
    println!(
        "{} voxels, {}x{}, median frame over a full turn\n",
        WORLD.map(|w| w.to_string()).join("x"),
        SIZE[0],
        SIZE[1],
    );
    println!(
        "{:<16} {:<22} {:>9} {:>9} {:>11} {:>9}",
        "terrain", "camera", "chunks", "drawn", "culled ms", "all ms"
    );
    let world = world();
    for (terrain, marching_cubes_enabled) in [("greedy", false), ("marching cubes", true)] {
        let mut tesselator = CpuTesselator::new();
        tesselator.configure(CpuTesselatorConfiguration {
            voxel_size: VOXEL_SIZE,
            marching_cubes_enabled,
        });
        tesselator.update_input_geometry(&world);
        let mut renderer = WgpuAccelerated::new().expect("no GPU");
        renderer.configure(WgpuAcceleratedConfiguration {
            width: SIZE[0],
            height: SIZE[1],
            background_color: DEFAULT_BACKGROUND_COLOR,
            color_depth_range: DEFAULT_COLOR_DEPTH_RANGE,
        });
        while let Some(delta) = tesselator.get_output_geometry_delta(&ALL_CHUNKS) {
            renderer.update_geometry(delta);
        }
        let cameras = [
            ("close, 6 m", 30.0 * VOXEL_SIZE, 15.0),
            ("demo start, 32 m", 160.0 * VOXEL_SIZE, 30.0),
            ("whole world, 180 m", 900.0 * VOXEL_SIZE, 60.0),
        ];
        for (camera, distance, pitch) in cameras {
            renderer.set_frustum_culling(true);
            let (culled, drawn) = frames(&mut renderer, distance, pitch);
            renderer.set_frustum_culling(false);
            let (all, _) = frames(&mut renderer, distance, pitch);
            println!(
                "{terrain:<16} {camera:<22} {:>9} {:>9} {:>11.2} {:>9.2}",
                renderer.drawn_chunks().1,
                drawn,
                culled,
                all,
            );
        }
    }
}
