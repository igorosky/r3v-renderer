use image::{ImageBuffer, Rgba};
use r3v_renderer::{
    Renderer, StlIoReader, UrdfRsReader,
    wgpu::{WgpuAccelerated, WgpuAcceleratedConfiguration},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let urdf_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "scorpio8.urdf.xacro".to_string());
    let output_path = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "scorpio8_external.png".to_string());
    let (width, height) = (1280, 960);

    let mut renderer =
        WgpuAccelerated::new().ok_or("Could not initialize a WGPU adapter/device")?;
    renderer.configure(WgpuAcceleratedConfiguration { width, height });
    let robot = renderer.load_urdf_file_with_readers(
        &urdf_path,
        &UrdfRsReader,
        &StlIoReader,
    )?;
    println!(
        "Loaded '{}' with urdf-rs and stl_io ({} links, {} joints)",
        robot.name,
        robot.links.len(),
        robot.joints.len()
    );

    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    renderer.render(&mut pixels);
    for pixel in pixels.chunks_exact_mut(4) {
        pixel.swap(0, 2);
    }
    let image: ImageBuffer<Rgba<u8>, _> = ImageBuffer::from_raw(width, height, pixels)
        .ok_or("Invalid rendered image buffer")?;
    image.save(&output_path)?;
    println!("Saved {output_path}");

    Ok(())
}
