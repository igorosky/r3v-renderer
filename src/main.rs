use r3v_renderer::{Renderer, wgpu::{self, WgpuAccelerated}};
use image::{ImageBuffer, Rgba};

fn main() {
    let mut renderer = WgpuAccelerated::new().unwrap();
    let width = 800;
    let height = 600;
    renderer.configure(wgpu::WgpuAcceleratedConfiguration { width, height });
    let bgra_pixels = renderer.render();
    let mut rgba_pixels = bgra_pixels.clone();
    for chunk in rgba_pixels.chunks_exact_mut(4) {
        let b = chunk[0];
        let r = chunk[2];
        chunk[0] = r;
        chunk[2] = b;
    }
    let img: ImageBuffer<Rgba<u8>, Vec<u8>> = 
        ImageBuffer::from_raw(width, height, rgba_pixels)
        .expect("Failed to create image buffer");
    img.save("output.png").expect("Failed to save image");
    println!("Saved render to output.png!");
}

