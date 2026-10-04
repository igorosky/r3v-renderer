use crate::{
    Renderer, camera::{self, CameraInternal, OrbitCamera}, geometry::PointI,
};
use wgpu::util::DeviceExt;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WgpuAcceleratedConfiguration {
    pub width: u32,
    pub height: u32,
}

struct WgpuAcceleratedState {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub pipeline: wgpu::RenderPipeline,
    pub pixel_size: u32,
    pub camera_buffer: wgpu::Buffer,
    pub camera_bind_group: wgpu::BindGroup,
}

struct RenderTargets {
    width: u32,
    height: u32,
    render_texture: wgpu::Texture,
    render_texture_view: wgpu::TextureView,
    depth_texture: wgpu::Texture,
    depth_texture_view: wgpu::TextureView,
    output_buffer: wgpu::Buffer,
    padded_bytes_per_row: u32,
    unpadded_bytes_per_row: u32,
}

pub struct WgpuAccelerated {
    state: WgpuAcceleratedState,
    camera: camera::OrbitCamera,
    active_configuration: Option<WgpuAcceleratedConfiguration>,
    render_targets: Option<RenderTargets>,
    active_voxels: Vec<PointI>,
}

impl WgpuAccelerated {
    pub fn new() -> Option<Self> {
        pollster::block_on(Self::new_async())
    }

    pub async fn new_async() -> Option<Self> { 
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor::default())
            .await
            .ok()?;

        // Shader
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Geometry Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/voxel.wgsl").into()),
        });

        // Camera
        let camera_uniform = camera::CameraUniform::default();
        let camera = OrbitCamera::default();
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let camera_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Camera Bind Group Layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Camera Bind Group"),
            layout: &camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        // Pipeline
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Pipeline Layout"),
            bind_group_layouts: &[
                Some(&camera_bind_group_layout)
            ],
            immediate_size: 0,
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3, // position
                        1 => Float32x3, // color
                    ],
                })],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: wgpu::TextureFormat::Bgra8UnormSrgb,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let state = WgpuAcceleratedState {
            device,
            queue,
            pipeline,
            pixel_size: std::mem::size_of::<[u8; 4]>() as u32,
            camera_buffer,
            camera_bind_group,
        };

        Some(Self {
            state,
            camera,
            active_configuration: None,
            render_targets: None,
            active_voxels: vec![],
        })
    }

    fn compute_visible_vertices(&self) -> Vec<Vertex> {
        vec![
            Vertex { position: [-0.5, -0.5, 0.0], color: [1.0, 0.0, 0.0] },
            Vertex { position: [0.5, -0.5, 0.0], color: [0.0, 1.0, 0.0] },
            Vertex { position: [0.0, 0.5, 0.0], color: [0.0, 0.0, 1.0] },
        ]
    }

    fn rebuild_targets(&mut self) {
        assert!(self.active_configuration.is_some(), "Renderer is not configured. Call configure() before render().");
        let config = self.active_configuration.as_ref().unwrap();
        let unpadded_bytes_per_row = config.width * self.state.pixel_size;
        let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let padded_bytes_per_row = (unpadded_bytes_per_row + align - 1) & !(align - 1);

        let texture_descriptor = wgpu::TextureDescriptor {
            label: Some("Render Texture"),
            size: wgpu::Extent3d {
                width: config.width,
                height: config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        };
        let texture = self.state.device.create_texture(&texture_descriptor);
        let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let depth_texture_descriptor = wgpu::TextureDescriptor {
            label: Some("Depth Texture"),
            size: wgpu::Extent3d {
                width: config.width,
                height: config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        };
        let depth_texture = self.state.device.create_texture(&depth_texture_descriptor);
        let depth_texture_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let output_buffer = self.state.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Output Buffer"),
            size: (padded_bytes_per_row * config.height) as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        self.render_targets = Some(RenderTargets {
            width: config.width,
            height: config.height,
            render_texture: texture,
            render_texture_view: texture_view,
            depth_texture: depth_texture,
            depth_texture_view: depth_texture_view,
            output_buffer,
            padded_bytes_per_row,
            unpadded_bytes_per_row,
        });
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 3],
    color: [f32; 3],
}

impl Renderer for WgpuAccelerated {
    type Configuration = WgpuAcceleratedConfiguration;
    type Camera = camera::OrbitCamera;

    fn add_voxels(&mut self, voxels: &[PointI]) {
        self.active_voxels.extend_from_slice(voxels);
    }
    
    fn render(&mut self, output: &mut [u8]) {
        // Ensure valid configuration
        let config = self.active_configuration.expect("Renderer is not configured. Call configure() before render().");
        assert!(output.len() >= (config.width * config.height * self.state.pixel_size) as usize, "Output buffer is too small for the configured dimensions.");
        
        // Ensure render targets are built and match the current configuration
        let needs_rebuild = match &self.render_targets {
            Some(targets) => targets.width != config.width || targets.height != config.height,
            None => true,
        };
        if needs_rebuild {
            self.rebuild_targets();
        }
        let targets = self.render_targets.as_ref().unwrap();

        // Load vertices
        let vertices = self.compute_visible_vertices();
        // TODO(@mariusz): Implement a persistent vertex buffer instead of reallocating
        let vertices_buffer = self.state.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Update camera uniform
        if self.camera.is_dirty() {
            let aspect_ratio = config.width as f32 / config.height as f32;
            let camera_uniform = self.camera.get_uniform(aspect_ratio);
            self.state.queue.write_buffer(&self.state.camera_buffer, 0, bytemuck::cast_slice(&[camera_uniform]));
        }

        // Command encoder and render pass
        let mut encoder = self.state.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &targets.render_texture_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &targets.depth_texture_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            render_pass.set_pipeline(&self.state.pipeline);
            render_pass.set_bind_group(0, &self.state.camera_bind_group, &[]);
            render_pass.set_vertex_buffer(0, vertices_buffer.slice(..));
            render_pass.draw(0..vertices.len() as u32, 0..1);
        }
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &targets.render_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &targets.output_buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(targets.padded_bytes_per_row),
                    rows_per_image: None,
                },
            },
            wgpu::Extent3d {
                width: targets.width,
                height: targets.height,
                depth_or_array_layers: 1,
            },
        );
        self.state.queue.submit(Some(encoder.finish()));
        let buffer_slice = targets.output_buffer.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            sender.send(result).unwrap();
        });
        let _ = self.state.device.poll(wgpu::PollType::Wait { submission_index: None, timeout: None });
        receiver.recv().unwrap().unwrap();
        {
            let padded_data = buffer_slice.get_mapped_range().unwrap();
            let mut offset = 0;
            for chunk in padded_data.chunks(targets.padded_bytes_per_row as usize) {
                let unpadded_chunk = &chunk[..targets.unpadded_bytes_per_row as usize];
                output[offset..offset + unpadded_chunk.len()].copy_from_slice(unpadded_chunk);
                offset += unpadded_chunk.len();
            }
        }
        targets.output_buffer.unmap();
    }
    
    fn configure(&mut self, configuration: Self::Configuration) {
        self.active_configuration = Some(configuration);
    }

    fn camera(&mut self) -> &mut Self::Camera {
        &mut self.camera
    }
}
