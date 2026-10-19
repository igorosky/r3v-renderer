use wgpu::util::{DeviceExt, DrawIndexedIndirectArgs};

use crate::geometry::Vertex;
use crate::turbo_colormap::TURBO_COLORMAP;

pub const DRAW_ARGS_STRIDE: wgpu::BufferAddress =
    std::mem::size_of::<DrawIndexedIndirectArgs>() as wgpu::BufferAddress;
pub const VERTEX_STRIDE: wgpu::BufferAddress = std::mem::size_of::<Vertex>() as wgpu::BufferAddress;

pub const BASIC_VERTEX_BUFFER_LAYOUT: wgpu::VertexBufferLayout = wgpu::VertexBufferLayout {
    array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
    step_mode: wgpu::VertexStepMode::Vertex,
    attributes: &wgpu::vertex_attr_array![
        0 => Float32x3, // position
        1 => Float32x3, // color
        2 => Float32x3, // normal
    ],
};

pub fn create_draw_buffer(device: &wgpu::Device, capacity: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Vertex Arena Draw Buffer"),
        size: capacity as wgpu::BufferAddress * DRAW_ARGS_STRIDE,
        usage: wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

pub const QUAD_VERTICES: u32 = 4;
pub const QUAD_INDICES: u32 = 6;

/// Index buffer splitting each run of four quad vertices into two triangles.
pub fn create_quad_index_buffer(device: &wgpu::Device, quad_capacity: usize) -> wgpu::Buffer {
    let indices = (0..quad_capacity as u32)
        .flat_map(|quad| [0, 1, 2, 0, 2, 3].map(|index| quad * QUAD_VERTICES + index))
        .collect::<Vec<u32>>();
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Quad Index Buffer"),
        contents: bytemuck::cast_slice(&indices),
        usage: wgpu::BufferUsages::INDEX,
    })
}

pub fn create_vertex_buffer(device: &wgpu::Device, capacity: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Vertex Arena Buffer"),
        size: capacity as wgpu::BufferAddress * VERTEX_STRIDE,
        usage: wgpu::BufferUsages::VERTEX
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    })
}

pub fn create_turbo_colormap_texture(device: &wgpu::Device, queue: &wgpu::Queue) -> wgpu::Texture {
    let texels = TURBO_COLORMAP
        .iter()
        .flat_map(|&[r, g, b]| {
            let to_unorm = |c: f32| (c * 255.0).round() as u8;
            [to_unorm(r), to_unorm(g), to_unorm(b), 255]
        })
        .collect::<Vec<u8>>();
    device.create_texture_with_data(
        queue,
        &wgpu::TextureDescriptor {
            label: Some("Turbo Colormap Texture"),
            size: wgpu::Extent3d {
                width: TURBO_COLORMAP.len() as u32,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D1,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        },
        wgpu::util::TextureDataOrder::LayerMajor,
        &texels,
    )
}
