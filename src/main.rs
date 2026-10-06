use r3v_renderer::{
    Camera, Tesselator, Renderer,
    geometry::PointI,
    renderer::{self, WgpuAccelerated},
    tesselator::{self, GreedyMeshTesselator},
    camera::{self, OrbitCamera},
};
use image::{ImageBuffer, Rgba};
use std::time::{Duration, Instant};

fn main() {
    let program_start = Instant::now();
    let width = 800;
    let height = 600;
    let aspect_ratio = width as f32 / height as f32;

    println!("Initializing Pipeline Components...");
    let init_start = Instant::now();
    
    let mut camera = OrbitCamera::default();
    let mut tesselator = GreedyMeshTesselator::new();
    let mut renderer = WgpuAccelerated::new().expect("Failed to initialize WgpuAccelerated");

    renderer.configure(renderer::WgpuAcceleratedConfiguration { width, height });
    let init_time = init_start.elapsed();

    let setup_start = Instant::now();
    let voxels = vec![
        PointI { x: 0, y: 0, z: 0, i: 0 },
        PointI { x: 0, y: 1, z: 0, i: 0 },
        PointI { x: 0, y: 0, z: 1, i: 0 },
        PointI { x: 1, y: 0, z: 0, i: 0 },
        PointI { x: 1, y: 1, z: 0, i: 0 },
    ];

    println!("Feeding {} voxels to the Tesselator...", voxels.len());

    // tesselator.update_input_geometry(&voxels);
    
    if let Some(geometry_delta) = tesselator.get_output_geometry_delta() {
        renderer.update_geometry(geometry_delta);
    }

    let mut pixel_buffer = vec![0u8; (width * height * 4) as usize];
    let setup_time = setup_start.elapsed();

    let total_frames = 12;
    let mut total_render_time = Duration::ZERO;
    let mut total_save_time = Duration::ZERO;

    let loop_start = Instant::now();
    for frame in 0..total_frames {
        println!("\n--- Frame {}/{} ---", frame + 1, total_frames);
        
        if frame > 0 {
            camera.rotate(0.1, 0.05);
        }

        // if let Some(delta) = tesselator.get_output_geometry_delta() {
        //     renderer.update_geometry(delta);
        // }

        // 6. Render
        let render_start = Instant::now();
        let camera_uniform = camera.get_uniform(aspect_ratio); // Extract the matrix
        renderer.render(&camera_uniform, &mut pixel_buffer);   // Pass matrix + output buffer
        let render_time = render_start.elapsed();
        
        total_render_time += render_time;
        println!("  Render time: {:?}", render_time);

        // 7. Save Output
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
    println!("Initialization:      {:?}", init_time);
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
