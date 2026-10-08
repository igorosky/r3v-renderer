use r3v_renderer::{
    Camera,
    Renderer,
    geometry::PointI,
    wgpu::{self, WgpuAccelerated},
};
use image::{ImageBuffer, Rgba};
use std::time::{Duration, Instant};

fn main() {
    let program_start = Instant::now();

    println!("Initializing WGPU Renderer...");
    let init_start = Instant::now();
    let mut renderer = WgpuAccelerated::new().expect("Failed to initialize WgpuAccelerated");
    let init_time = init_start.elapsed();

    let setup_start = Instant::now();
    let width = 800;
    let height = 600;
    renderer.configure(wgpu::WgpuAcceleratedConfiguration { width, height });
    
    let mut voxels = Vec::new();
    for i in -2..=2 {
        voxels.push(PointI { x: i, y: 0, z: 0, i: 0 });
        voxels.push(PointI { x: 0, y: i, z: 0, i: 0 });
        voxels.push(PointI { x: 0, y: 0, z: i, i: 0 });
    }

    println!("Adding {} voxels to the scene...", voxels.len());
    renderer.add_voxels(&voxels);

    let mut pixel_buffer = vec![0u8; (width * height * 4) as usize];
    let setup_time = setup_start.elapsed();

    let total_frames = 12;
    let mut total_render_time = Duration::ZERO;
    let mut total_save_time = Duration::ZERO;

    let loop_start = Instant::now();
    for frame in 0..total_frames {
        println!("\n--- Frame {}/{} ---", frame + 1, total_frames);
    
        if frame > 0 {
            let camera = renderer.camera();
            camera.rotate(0.5, 0.0);
            if frame < 6 {
                camera.zoom(-0.5); 
            } else {
                camera.pan(0.1, 0.05);
            }
        }

        let render_start = Instant::now();
        renderer.render(&mut pixel_buffer);
        let render_time = render_start.elapsed();
        total_render_time += render_time;
        println!("  Render time: {:?}", render_time);

        let filename = format!("output_frame_{:02}.png", frame);
        let save_start = Instant::now();
        save_frame(&filename, width, height, &mut pixel_buffer);
        let save_time = save_start.elapsed();
        total_save_time += save_time;
        println!("  Save time:   {:?}", save_time);
    }
    let loop_time = loop_start.elapsed();
    let program_time = program_start.elapsed();

    println!("\n========================================");
    println!("          BENCHMARK SUMMARY             ");
    println!("========================================");
    println!("WGPU Initialization: {:?}", init_time);
    println!("Scene Setup:         {:?}", setup_time);
    println!("Total Loop Time:     {:?}", loop_time);
    println!("Total Program Time:  {:?}", program_time);
    println!("----------------------------------------");
    println!("Avg Render/Frame:    {:?}", total_render_time / total_frames);
    println!("Avg Save/Frame:      {:?}", total_save_time / total_frames);
    println!("========================================");
}

fn save_frame(filename: &str, width: u32, height: u32, bgra_pixels: &mut [u8]) {
    let rgba_pixels = bgra_pixels;
    for chunk in rgba_pixels.chunks_exact_mut(4) {
        let b = chunk[0];
        let r = chunk[2];
        chunk[0] = r;
        chunk[2] = b;
    }
    let img: ImageBuffer<Rgba<u8>, _> = ImageBuffer::from_raw(width, height, rgba_pixels.to_vec())
        .expect("Failed to create image buffer");
    img.save(filename).expect("Failed to save image");
}
