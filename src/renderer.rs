use crate::{
    Renderer,
    camera,
    geometry,
    vertex_arena::VertexArena,
};

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
    output_buffers: [wgpu::Buffer; 2],
    padded_bytes_per_row: u32,
    unpadded_bytes_per_row: u32,
}

pub struct WgpuAccelerated {
    state: WgpuAcceleratedState,
    active_configuration: Option<WgpuAcceleratedConfiguration>,
    render_targets: Option<RenderTargets>,
    vertex_arena: VertexArena,
    frame_index: usize, 
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
        let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Camera Buffer"),
            size: std::mem::size_of::<camera::CameraUniform>() as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
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
                    array_stride: std::mem::size_of::<geometry::Vertex>() as wgpu::BufferAddress,
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
            primitive: wgpu::PrimitiveState {
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
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

        let vertex_arena = VertexArena::new(&device);

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
            active_configuration: None,
            render_targets: None,
            vertex_arena,
            frame_index: 0,
        })
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
        let create_output_buffer = || self.state.device.create_buffer(&wgpu::BufferDescriptor {
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
            output_buffers: [create_output_buffer(), create_output_buffer()],
            padded_bytes_per_row,
            unpadded_bytes_per_row,
        });
        self.frame_index = 0; 
    }
}

impl Renderer for WgpuAccelerated {
    type Configuration = WgpuAcceleratedConfiguration;
    type GeometryDelta = Vec<geometry::TriangleChunk>;

    fn configure(&mut self, configuration: Self::Configuration) {
        self.active_configuration = Some(configuration);
    }

    fn render(
        &mut self,
        camera_uniform: &camera::CameraUniform,
        output: &mut [u8]
    ) {
        let config = self.active_configuration.expect("Renderer is not configured.");
        assert!(output.len() >= (config.width * config.height * self.state.pixel_size) as usize, "Output buffer is too small.");

        let needs_rebuild = match &self.render_targets {
            Some(targets) => targets.width != config.width || targets.height != config.height,
            None => true,
        };
        if needs_rebuild {
            self.rebuild_targets();
        }

        let targets = self.render_targets.as_ref().unwrap();
        if self.frame_index > 0 {
            let read_index = (self.frame_index + 1) % 2;
            let read_buffer = &targets.output_buffers[read_index];
            let buffer_slice = read_buffer.slice(..);

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
            read_buffer.unmap();
        } else {
            output.fill(0);
        }
        self.state.queue.write_buffer(&self.state.camera_buffer, 0, bytemuck::cast_slice(&[*camera_uniform]));
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
            let draw_count = self.vertex_arena.draw_count();
            if draw_count > 0 {
                render_pass.set_pipeline(&self.state.pipeline);
                render_pass.set_bind_group(0, &self.state.camera_bind_group, &[]);
                render_pass.set_vertex_buffer(0, self.vertex_arena.vertex_buffer().slice(..));
                render_pass.multi_draw_indirect(self.vertex_arena.draw_buffer(), 0, draw_count);
            }
        }
        let write_index = self.frame_index % 2;
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &targets.render_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &targets.output_buffers[write_index],
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
        self.frame_index += 1;
    }

    fn update_geometry(&mut self, geometry_delta: Self::GeometryDelta) {
        let device = &self.state.device;
        let queue = &self.state.queue;
        let mut vertices = Vec::new();
        for geometry::TriangleChunk { index, triangles } in geometry_delta {
            vertices.clear();
            vertices.extend(triangles.into_iter().flat_map(|triangle| {
                [
                    geometry::Vertex::with_default_color(triangle.v0[0], triangle.v0[1], triangle.v0[2]),
                    geometry::Vertex::with_default_color(triangle.v1[0], triangle.v1[1], triangle.v1[2]),
                    geometry::Vertex::with_default_color(triangle.v2[0], triangle.v2[1], triangle.v2[2]),
                ]
            }));
            self.vertex_arena.set_chunk(device, queue, index, &vertices);
        }
        self.vertex_arena.finish_update(device, queue);
    }
}
