use bytemuck::Zeroable;
use wgpu::util::{DeviceExt, DrawIndirectArgs};

use crate::geometry::Vertex;
use crate::hypsometric_colormap::{HYPSOMETRIC_COLORMAP_LEN, hypsometric_colormap};
use crate::turbo_colormap::TURBO_COLORMAP;

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
        usage: wgpu::BufferUsages::INDIRECT
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
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

pub fn srgb_to_linear(component: f32) -> f64 {
    let c = f64::from(component);
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// Offset of the colormap parameters behind the colors.
pub const COLORMAP_PARAMS_OFFSET: wgpu::BufferAddress =
    ((TURBO_COLORMAP.len() + HYPSOMETRIC_COLORMAP_LEN) * std::mem::size_of::<[f32; 4]>())
        as wgpu::BufferAddress;

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ColormapParams {
    pub depth_range: f32,
    pub color_by_height: u32,
    pub neutral_height: f32,
    pub height_knee: f32,
    pub height_gain: f32,
    pub height_bands: u32,
    pub _pad: [f32; 2],
}

/// Both colormaps in one `array<vec4<f32>, 512>` (vec4 for the uniform array stride of 16 bytes),
/// followed by zeroed parameters to write at `COLORMAP_PARAMS_OFFSET`.
pub fn create_colormap_buffer(device: &wgpu::Device) -> wgpu::Buffer {
    let hypsometric = hypsometric_colormap().map(|color| color.map(|c| srgb_to_linear(c) as f32));
    let colors = TURBO_COLORMAP
        .iter()
        .copied()
        .chain(hypsometric)
        .map(|[r, g, b]| [r, g, b, 1.0])
        .collect::<Vec<[f32; 4]>>();
    let mut contents = bytemuck::cast_slice::<_, u8>(&colors).to_vec();
    contents.extend_from_slice(bytemuck::bytes_of(&ColormapParams::zeroed()));
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Colormap Buffer"),
        contents: &contents,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    })
}

/// A command buffer making a copy into `buffer` visible to the host: a TRANSFER_WRITE ->
/// HOST_READ barrier, which wgpu does not record before mapping. Without it the Jetson's GPU
/// may not have written the copy back from its cache yet when the buffer gets mapped, and the
/// CPU reads what was there before. Submit it right after the copy; it is recorded on an
/// encoder of its own since wgpu does not let raw commands share one with its own. `None`
/// when the adapter is not a Vulkan one.
#[cfg(any(windows, target_os = "linux", target_os = "android"))]
pub fn host_read_barrier(
    device: &wgpu::Device,
    buffer: &wgpu::Buffer,
) -> Option<wgpu::CommandBuffer> {
    use wgpu::{
        hal::{BufferBarrier, CommandEncoder as _, StateTransition, api::Vulkan},
        wgt::BufferUses,
    };

    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("Host Read Barrier Encoder"),
    });
    {
        // SAFETY: the buffer is only named in a barrier, never destroyed
        let buffer = unsafe { buffer.as_hal::<Vulkan>() }?;
        // SAFETY: the barrier is all the encoder records, and nothing else touches it meanwhile
        let recorded = unsafe {
            encoder.as_hal_mut::<Vulkan, _, _>(|hal_encoder| {
                hal_encoder.map(|hal_encoder| {
                    hal_encoder.transition_buffers(std::iter::once(BufferBarrier {
                        buffer: &*buffer,
                        usage: StateTransition {
                            from: BufferUses::COPY_DST,
                            to: BufferUses::MAP_READ,
                        },
                    }));
                })
            })
        };
        recorded?;
    }
    Some(encoder.finish())
}

/// Only Vulkan needs the barrier (see above), and only there can it be recorded.
#[cfg(not(any(windows, target_os = "linux", target_os = "android")))]
pub fn host_read_barrier(
    _device: &wgpu::Device,
    _buffer: &wgpu::Buffer,
) -> Option<wgpu::CommandBuffer> {
    None
}

/// Cleans and invalidates the CPU data cache lines holding `data`, so that it is read from
/// memory next: on the Jetson the GPU's writes do not evict the CPU's stale copies of the lines
/// they hit. Returns whether it did: only on aarch64, where Linux lets user space maintain the
/// cache (`SCTLR_EL1.UCI`).
pub fn invalidate_cpu_cache(data: &[u8]) -> bool {
    #[cfg(target_arch = "aarch64")]
    {
        let ctr: u64;
        // SAFETY: reads a register
        unsafe { core::arch::asm!("mrs {}, ctr_el0", out(reg) ctr, options(nomem, nostack)) };
        // DminLine: log2 of the words in the smallest data cache line
        let line = 4usize << ((ctr >> 16) & 0xf);
        let end = data.as_ptr() as usize + data.len();
        let mut address = data.as_ptr() as usize & !(line - 1);
        while address < end {
            // SAFETY: the CPU only reads mapped readback memory, so no line is dirty and
            // writing lines back loses nothing
            unsafe { core::arch::asm!("dc civac, {}", in(reg) address, options(nostack)) };
            address += line;
        }
        // SAFETY: a barrier, ordering the maintenance before the reads
        unsafe { core::arch::asm!("dsb sy", options(nostack)) };
        true
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let _ = data;
        false
    }
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
            self.queue.submit(
                std::iter::once(encoder.finish())
                    .chain(super::host_read_barrier(&self.device, &staging)),
            );
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
            let mapped = slice.get_mapped_range().unwrap();
            super::invalidate_cpu_cache(&mapped);
            let data = mapped.to_vec();
            drop(mapped);
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
        // DrawIndirect: vertex_count, instance_count, first_vertex, first_instance.
        assert_eq!(DRAW_ARGS_STRIDE, 4 * 4);
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
        }
    }

    #[test]
    fn colormap_buffer_holds_both_colormaps() {
        let gpu = TestGpu::new();
        let buffer = create_colormap_buffer(&gpu.device);
        // 2 * 256 entries of vec4<f32>, then the parameters padded to whole vec4<f32>s.
        assert_eq!(COLORMAP_PARAMS_OFFSET, 512 * 16);
        assert_eq!(std::mem::size_of::<ColormapParams>(), 2 * 16);
        assert_eq!(buffer.size(), 512 * 16 + 2 * 16);
        assert!(
            buffer
                .usage()
                .contains(wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST)
        );
    }

    #[test]
    fn srgb_to_linear_keeps_the_ends() {
        assert_eq!(srgb_to_linear(0.0), 0.0);
        assert!((srgb_to_linear(1.0) - 1.0).abs() < 1e-12);
        assert!((srgb_to_linear(0.5) - 0.214).abs() < 1e-3);
    }

    #[test]
    fn host_read_barrier_recorded_on_vulkan() {
        // Frame readbacks on the Jetson rely on it, so it must not quietly record nothing.
        let gpu = TestGpu::new();
        let buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: 16,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let barrier = host_read_barrier(&gpu.device, &buffer);
        assert_eq!(
            barrier.is_some(),
            gpu.device.adapter_info().backend == wgpu::Backend::Vulkan
        );
        gpu.queue.submit(barrier);
    }

    #[test]
    fn invalidate_cpu_cache_keeps_data() {
        // Unaligned on both ends, so the first and last lines are partial.
        let data: Vec<u8> = (0..1000).map(|n| n as u8).collect();
        let slice = &data[3..997];
        assert_eq!(invalidate_cpu_cache(slice), cfg!(target_arch = "aarch64"));
        assert!(slice.iter().zip(3..).all(|(&b, n)| b == n as u8));
    }
}
