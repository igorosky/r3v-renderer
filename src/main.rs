use r3v_renderer::{
    Camera,
    Renderer,
    geometry::PointI,
    wgpu::{self, WgpuAccelerated},
};
use image::{ImageBuffer, Rgba};

fn main() {
    println!("Initializing WGPU Renderer...");
    let mut renderer = WgpuAccelerated::new().expect("Failed to initialize WgpuAccelerated");
    let width = 800;
    let height = 600;
    renderer.configure(wgpu::WgpuAcceleratedConfiguration { width, height });
    let mut voxels = Vec::new();
    for i in -2..=2 {
        voxels.push(PointI { x: i, y: 0, z: 0, i: 0, });
        voxels.push(PointI { x: 0, y: i, z: 0, i: 0, });
        voxels.push(PointI { x: 0, y: 0, z: i, i: 0, });
    }
    println!("Adding {} voxels to the scene...", voxels.len());
    renderer.add_voxels(&voxels);
    let total_frames = 12;
    for frame in 0..total_frames {
        println!("Rendering frame {}/{}...", frame + 1, total_frames);
        if frame > 0 {
            let camera = renderer.camera();
            camera.rotate(0.5, 0.0);
            if frame < 6 {
                camera.zoom(-0.5); 
            } else {
                camera.pan(0.1, 0.05);
            }
        }
        let bgra_pixels = renderer.render();
        let filename = format!("output_frame_{:02}.png", frame);
        save_frame(&filename, width, height, bgra_pixels);
    }
    println!("Animation sequence complete!");
}

fn save_frame(filename: &str, width: u32, height: u32, bgra_pixels: Vec<u8>) {
    let mut rgba_pixels = bgra_pixels;
    for chunk in rgba_pixels.chunks_exact_mut(4) {
        let b = chunk[0];
        let r = chunk[2];
        chunk[0] = r;
        chunk[2] = b;
    }
    let img: ImageBuffer<Rgba<u8>, Vec<u8>> = 
        ImageBuffer::from_raw(width, height, rgba_pixels)
        .expect("Failed to create image buffer");
    img.save(filename).expect("Failed to save image");
    println!(" -> Saved {}", filename);
}
