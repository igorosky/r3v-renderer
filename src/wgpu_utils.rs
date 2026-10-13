use wgpu::util::DrawIndirectArgs;

use crate::geometry::Vertex;

pub const DRAW_ARGS_STRIDE: wgpu::BufferAddress =
    std::mem::size_of::<DrawIndirectArgs>() as wgpu::BufferAddress;
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
