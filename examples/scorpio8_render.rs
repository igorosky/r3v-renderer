use image::{ImageBuffer, Rgba};
use r3v_renderer::{
    Camera, Renderer,
    wgpu::{WgpuAccelerated, WgpuAcceleratedConfiguration},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let urdf_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "scorpio8.urdf.xacro".to_string());
    let output_path = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "scorpio8_frame".to_string());
    let width = 1280;
    let height = 960;
    let frame_count = 12;

    let mut renderer =
        WgpuAccelerated::new().ok_or("Could not initialize a WGPU adapter/device")?;
    renderer.configure(WgpuAcceleratedConfiguration { width, height });
    let robot = renderer.load_urdf_file(&urdf_path)?;
    println!(
        "Loaded robot '{}' ({} links, {} joints)",
        robot.name,
        robot.links.len(),
        robot.joints.len()
    );

    let mut pixels = vec![0u8; (width * height * 4) as usize];
    for frame in 0..frame_count {
        if frame > 0 {
            renderer
                .camera()
                .rotate(std::f32::consts::TAU / frame_count as f32, 0.0);
        }

        renderer.render(&mut pixels);
        let mut rgba_pixels = pixels.clone();
        for pixel in rgba_pixels.chunks_exact_mut(4) {
            pixel.swap(0, 2);
        }
        let image: ImageBuffer<Rgba<u8>, _> = ImageBuffer::from_raw(width, height, rgba_pixels)
            .ok_or("Invalid rendered image buffer")?;
        let frame_path = format!("{output_path}_{frame:02}.png");
        image.save(&frame_path)?;
        println!("Saved render to {frame_path}");
    }
    Ok(())
}
