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
        usage: wgpu::BufferUsages::INDIRECT
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
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
        usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_SRC,
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

#[cfg(test)]
pub(crate) mod test_support {
    use std::sync::{Mutex, MutexGuard};

    /// GPU device for unit tests. Tests holding one run one at a time: some drivers crash
    /// when several devices are used from different threads at once.
    pub(crate) struct TestGpu {
        pub device: wgpu::Device,
        pub queue: wgpu::Queue,
        _guard: MutexGuard<'static, ()>,
    }

    impl TestGpu {
        pub fn new() -> Self {
            static LOCK: Mutex<()> = Mutex::new(());
            let guard = LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
            let instance = wgpu::Instance::default();
            let adapter = pollster::block_on(
                instance.request_adapter(&wgpu::RequestAdapterOptions::default()),
            )
            .expect("no wgpu adapter available");
            let (device, queue) =
                pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
                    .expect("failed to create wgpu device");
            Self {
                device,
                queue,
                _guard: guard,
            }
        }

        /// Reads back the whole contents of a buffer created with `COPY_SRC`.
        pub fn read_buffer(&self, buffer: &wgpu::Buffer) -> Vec<u8> {
            let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("Test Readback Buffer"),
                size: buffer.size(),
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            let mut encoder = self
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
            encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, buffer.size());
            self.queue.submit(Some(encoder.finish()));
            let slice = staging.slice(..);
            let (sender, receiver) = std::sync::mpsc::channel();
            slice.map_async(wgpu::MapMode::Read, move |result| {
                sender.send(result).unwrap();
            });
            let _ = self.device.poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            });
            receiver.recv().unwrap().unwrap();
            let data = slice.get_mapped_range().unwrap().to_vec();
            staging.unmap();
            data
        }
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::TestGpu;
    use super::*;

    #[test]
    fn strides_match_gpu_layouts() {
        // position, color and normal: three vec3<f32>.
        assert_eq!(VERTEX_STRIDE, 36);
        assert_eq!(VERTEX_STRIDE, std::mem::size_of::<Vertex>() as u64);
        // DrawIndexedIndirect: index_count, instance_count, first_index, base_vertex,
        // first_instance.
        assert_eq!(DRAW_ARGS_STRIDE, 5 * 4);
    }

    #[test]
    fn vertex_layout_matches_vertex_struct() {
        let layout = &BASIC_VERTEX_BUFFER_LAYOUT;
        assert_eq!(layout.array_stride, VERTEX_STRIDE);
        assert_eq!(layout.step_mode, wgpu::VertexStepMode::Vertex);
        let expected = [
            (0, std::mem::offset_of!(Vertex, position)),
            (1, std::mem::offset_of!(Vertex, color)),
            (2, std::mem::offset_of!(Vertex, normal)),
        ];
        assert_eq!(layout.attributes.len(), expected.len());
        for (attribute, (location, offset)) in layout.attributes.iter().zip(expected) {
            assert_eq!(attribute.shader_location, location);
            assert_eq!(attribute.offset, offset as u64);
            assert_eq!(attribute.format, wgpu::VertexFormat::Float32x3);
        }
    }

    #[test]
    fn quad_constants_describe_two_triangles() {
        assert_eq!(QUAD_VERTICES, 4);
        assert_eq!(QUAD_INDICES, 2 * 3);
    }

    #[test]
    fn buffers_have_requested_capacity_and_usage() {
        let gpu = TestGpu::new();
        for capacity in [1, 7, 1024] {
            let vertices = create_vertex_buffer(&gpu.device, capacity);
            assert_eq!(vertices.size(), capacity as u64 * VERTEX_STRIDE);
            assert!(vertices.usage().contains(
                wgpu::BufferUsages::VERTEX
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC
            ));

            let draws = create_draw_buffer(&gpu.device, capacity);
            assert_eq!(draws.size(), capacity as u64 * DRAW_ARGS_STRIDE);
            assert!(
                draws
                    .usage()
                    .contains(wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_DST)
            );

            let indices = create_quad_index_buffer(&gpu.device, capacity);
            assert_eq!(indices.size(), capacity as u64 * QUAD_INDICES as u64 * 4);
            assert!(indices.usage().contains(wgpu::BufferUsages::INDEX));
        }
    }

    /// Each quad's vertices are listed in order around its outline (v0..v3). Its indices
    /// must form two triangles that keep that winding and together cover the quad.
    #[test]
    fn quad_indices_split_each_quad_into_two_triangles() {
        let gpu = TestGpu::new();
        let quads = 300;
        let buffer = create_quad_index_buffer(&gpu.device, quads);
        let bytes = gpu.read_buffer(&buffer);
        let indices: &[u32] = bytemuck::cast_slice(&bytes);
        assert_eq!(indices.len(), quads * QUAD_INDICES as usize);

        for (quad, indices) in indices
            .as_chunks::<{ QUAD_INDICES as usize }>()
            .0
            .iter()
            .enumerate()
        {
            let base = quad as u32 * QUAD_VERTICES;
            let local: Vec<u32> = indices
                .iter()
                .map(|&index| {
                    assert!(
                        (base..base + QUAD_VERTICES).contains(&index),
                        "quad {quad} references vertex {index}"
                    );
                    index - base
                })
                .collect();
            let (first, second) = local.split_at(3);
            for triangle in [first, second] {
                let mut sorted = triangle.to_vec();
                sorted.sort();
                sorted.dedup();
                assert_eq!(sorted.len(), 3, "degenerate triangle {triangle:?}");
                // Same winding as the outline: walking the triangle goes once forward around
                // the quad (a backwards triangle goes around twice).
                let span: u32 = (0..3)
                    .map(|i| (triangle[(i + 1) % 3] + QUAD_VERTICES - triangle[i]) % QUAD_VERTICES)
                    .sum();
                assert_eq!(
                    span, QUAD_VERTICES,
                    "triangle {triangle:?} is wound backwards"
                );
            }
            // The triangles share exactly one diagonal and cover all four corners.
            let shared = first.iter().filter(|i| second.contains(i)).count();
            assert_eq!(shared, 2);
            let mut all = local.clone();
            all.sort();
            all.dedup();
            assert_eq!(all, [0, 1, 2, 3]);
            let mut diagonal: Vec<u32> = first
                .iter()
                .copied()
                .filter(|i| second.contains(i))
                .collect();
            diagonal.sort();
            assert!(
                diagonal == [0, 2] || diagonal == [1, 3],
                "shared edge {diagonal:?} is not a diagonal"
            );
        }
    }

    #[test]
    fn turbo_texture_holds_whole_colormap() {
        let gpu = TestGpu::new();
        let texture = create_turbo_colormap_texture(&gpu.device, &gpu.queue);
        assert_eq!(texture.dimension(), wgpu::TextureDimension::D1);
        assert_eq!(texture.width(), 256);
        assert_eq!(texture.height(), 1);
        assert_eq!(texture.format(), wgpu::TextureFormat::Rgba8Unorm);
        assert!(
            texture
                .usage()
                .contains(wgpu::TextureUsages::TEXTURE_BINDING)
        );
    }
}
