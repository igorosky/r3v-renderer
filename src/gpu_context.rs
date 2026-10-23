use std::sync::atomic::{AtomicU64, Ordering};

#[derive(Clone, Debug)]
pub struct GpuContext {
    id: u64,
    device: wgpu::Device,
    queue: wgpu::Queue,
    supports_compute: bool,
}

fn next_id() -> u64 {
    static NEXT_ID: AtomicU64 = AtomicU64::new(0);
    NEXT_ID.fetch_add(1, Ordering::Relaxed)
}

impl GpuContext {
    pub fn new() -> Option<Self> {
        pollster::block_on(Self::new_async())
    }

    pub async fn new_async() -> Option<Self> {
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        let required_limits = wgpu::Limits {
            max_buffer_size: adapter.limits().max_buffer_size,
            max_storage_buffer_binding_size: adapter.limits().max_storage_buffer_binding_size,
            ..Default::default()
        };
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                required_limits,
                ..Default::default()
            })
            .await
            .ok()?;
        Some(Self::from_wgpu(&adapter, device, queue))
    }

    pub fn from_wgpu(adapter: &wgpu::Adapter, device: wgpu::Device, queue: wgpu::Queue) -> Self {
        let supports_compute = adapter
            .get_downlevel_capabilities()
            .flags
            .contains(wgpu::DownlevelFlags::COMPUTE_SHADERS);
        Self {
            id: next_id(),
            device,
            queue,
            supports_compute,
        }
    }

    pub fn id(&self) -> u64 {
        self.id
    }

    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }

    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }

    pub fn supports_compute(&self) -> bool {
        self.supports_compute
    }
}

#[cfg(test)]
mod tests {
    use crate::wgpu_utils::test_support::TestGpu;

    #[test]
    fn clones_share_an_id_and_contexts_do_not() {
        let gpu = TestGpu::new();
        let clone = gpu.context.clone();
        assert_eq!(clone.id(), gpu.context.id());
        let other = super::GpuContext::from_wgpu(
            &gpu.adapter,
            gpu.context.device().clone(),
            gpu.context.queue().clone(),
        );
        assert_ne!(other.id(), gpu.context.id());
    }
}
