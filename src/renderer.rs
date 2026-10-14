use crate::wgpu_utils::*;
use crate::{Renderer, camera, geometry, stl, vertex_arena::VertexArena};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WgpuAcceleratedConfiguration {
    pub width: u32,
    pub height: u32,
}

struct WgpuAcceleratedState {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub environment_pipeline: wgpu::RenderPipeline,
    pub asset_pipeline: wgpu::RenderPipeline,
    pub pixel_size: u32,
    pub camera_uniform_buffer: wgpu::Buffer,
    pub camera_bind_group: wgpu::BindGroup,
    pub colormap_bind_group: wgpu::BindGroup,
}

struct RenderTargets {
    width: u32,
    height: u32,
    render_texture: wgpu::Texture,
    render_texture_view: wgpu::TextureView,
    _depth_texture: wgpu::Texture,
    depth_texture_view: wgpu::TextureView,
    output_buffers: [wgpu::Buffer; 2],
    padded_bytes_per_row: u32,
    unpadded_bytes_per_row: u32,
}

const INITIAL_ASSET_VERTEX_CAPACITY: usize = 1 << 20;
const INITIAL_ASSET_TRANSFORM_CAPACITY: usize = 1024;
const ASSET_TRANSFORM_STRIDE: wgpu::BufferAddress =
    std::mem::size_of::<geometry::RealTwist>() as wgpu::BufferAddress;

struct Asset {
    vertex_offset: u32,
    vertex_count: u32,
    first_instance: u32,
    instance_count: u32,
}

struct Assets {
    vertex_buffer: wgpu::Buffer,
    vertex_capacity: usize,
    vertex_count: usize,
    transform_buffer: wgpu::Buffer,
    transform_capacity: usize,
    transforms: Vec<geometry::RealTwist>,
    transforms_dirty: bool,
    assets: Vec<Asset>,
}

fn create_asset_transform_buffer(device: &wgpu::Device, capacity: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Asset Transform Buffer"),
        size: capacity as wgpu::BufferAddress * ASSET_TRANSFORM_STRIDE,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

impl Assets {
    fn new(device: &wgpu::Device) -> Self {
        Self {
            vertex_buffer: create_vertex_buffer(device, INITIAL_ASSET_VERTEX_CAPACITY),
            vertex_capacity: INITIAL_ASSET_VERTEX_CAPACITY,
            vertex_count: 0,
            transform_buffer: create_asset_transform_buffer(
                device,
                INITIAL_ASSET_TRANSFORM_CAPACITY,
            ),
            transform_capacity: INITIAL_ASSET_TRANSFORM_CAPACITY,
            transforms: Vec::new(),
            transforms_dirty: false,
            assets: Vec::new(),
        }
    }

    fn is_empty(&self) -> bool {
        self.assets.is_empty()
    }

    fn upload(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        vertices: &[geometry::Vertex],
        instances: usize,
    ) -> usize {
        self.reserve_vertices(device, queue, vertices.len());
        queue.write_buffer(
            &self.vertex_buffer,
            self.vertex_count as wgpu::BufferAddress * VERTEX_STRIDE,
            bytemuck::cast_slice(vertices),
        );
        self.reserve_transforms(device, instances);
        let first_instance = self.transforms.len();
        self.transforms.extend(std::iter::repeat_n(
            geometry::RealTwist::identity(),
            instances,
        ));
        self.transforms_dirty = true;
        self.assets.push(Asset {
            vertex_offset: self.vertex_count as u32,
            vertex_count: vertices.len() as u32,
            first_instance: first_instance as u32,
            instance_count: instances as u32,
        });
        self.vertex_count += vertices.len();
        first_instance
    }

    fn set_transform(&mut self, instance: usize, transform: geometry::RealTwist) {
        let current = &mut self.transforms[instance];
        if *current == transform {
            return;
        }
        *current = transform;
        self.transforms_dirty = true;
    }

    fn flush_transforms(&mut self, queue: &wgpu::Queue) {
        if !self.transforms_dirty {
            return;
        }
        queue.write_buffer(
            &self.transform_buffer,
            0,
            bytemuck::cast_slice(&self.transforms),
        );
        self.transforms_dirty = false;
    }

    fn reserve_vertices(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, additional: usize) {
        let required = self.vertex_count + additional;
        if required <= self.vertex_capacity {
            return;
        }
        let mut new_capacity = self.vertex_capacity;
        while new_capacity < required {
            new_capacity *= 2;
        }
        let new_buffer = create_vertex_buffer(device, new_capacity);
        if self.vertex_count > 0 {
            let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Asset Vertex Buffer Grow Encoder"),
            });
            encoder.copy_buffer_to_buffer(
                &self.vertex_buffer,
                0,
                &new_buffer,
                0,
                self.vertex_count as wgpu::BufferAddress * VERTEX_STRIDE,
            );
            queue.submit(Some(encoder.finish()));
        }
        self.vertex_buffer = new_buffer;
        self.vertex_capacity = new_capacity;
    }

    fn reserve_transforms(&mut self, device: &wgpu::Device, additional: usize) {
        let required = self.transforms.len() + additional;
        if required <= self.transform_capacity {
            return;
        }
        let mut new_capacity = self.transform_capacity;
        while new_capacity < required {
            new_capacity *= 2;
        }
        self.transform_buffer = create_asset_transform_buffer(device, new_capacity);
        self.transform_capacity = new_capacity;
        self.transforms_dirty = true;
    }
}

pub struct WgpuAccelerated {
    state: WgpuAcceleratedState,
    active_configuration: Option<WgpuAcceleratedConfiguration>,
    render_targets: Option<RenderTargets>,
    vertex_arena: VertexArena,
    assets: Assets,
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

        // Camera
        let camera_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Camera Buffer"),
            size: std::mem::size_of::<camera::CameraUniform>() as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let camera_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
                resource: camera_uniform_buffer.as_entire_binding(),
            }],
        });

        // Colormap
        let colormap_texture = create_turbo_colormap_texture(&device, &queue);
        let colormap_texture_view =
            colormap_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let colormap_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Colormap Sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let colormap_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Colormap Bind Group Layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D1,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
            });
        let colormap_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Colormap Bind Group"),
            layout: &colormap_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&colormap_texture_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&colormap_sampler),
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Pipeline Layout"),
            bind_group_layouts: &[Some(&camera_bind_group_layout)],
            immediate_size: 0,
        });
        let environment_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Environment Pipeline Layout"),
                bind_group_layouts: &[
                    Some(&camera_bind_group_layout),
                    Some(&colormap_bind_group_layout),
                ],
                immediate_size: 0,
            });

        // Environment shader
        let environment_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Geometry Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/voxel.wgsl").into()),
        });

        // Environment pipeline
        let environment_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Environment Render Pipeline"),
            layout: Some(&environment_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &environment_shader,
                entry_point: Some("vs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[Some(BASIC_VERTEX_BUFFER_LAYOUT)],
            },
            fragment: Some(wgpu::FragmentState {
                module: &environment_shader,
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

        // Asset shader
        let asset_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Asset Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/model.wgsl").into()),
        });

        // Asset pipeline
        let asset_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Asset Render Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &asset_shader,
                entry_point: Some("vs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[
                    Some(BASIC_VERTEX_BUFFER_LAYOUT),
                    Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<geometry::RealTwist>()
                            as wgpu::BufferAddress,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![
                            3 => Float32x3, // linear.x, linear.y, linear.z
                            4 => Float32x4, // rotational.x, rotational.y, rotational.z, rotational.w
                        ],
                    }),
                ],
            },
            fragment: Some(wgpu::FragmentState {
                module: &asset_shader,
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

        let assets = Assets::new(&device);

        let state = WgpuAcceleratedState {
            device,
            queue,
            environment_pipeline,
            asset_pipeline,
            pixel_size: std::mem::size_of::<[u8; 4]>() as u32,
            camera_uniform_buffer,
            camera_bind_group,
            colormap_bind_group,
        };

        Some(Self {
            state,
            active_configuration: None,
            render_targets: None,
            vertex_arena,
            assets,
            frame_index: 0,
        })
    }

    fn rebuild_targets(&mut self) {
        assert!(
            self.active_configuration.is_some(),
            "Renderer is not configured. Call configure() before render()."
        );
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
        let create_output_buffer = || {
            self.state.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("Output Buffer"),
                size: (padded_bytes_per_row * config.height) as wgpu::BufferAddress,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            })
        };
        self.render_targets = Some(RenderTargets {
            width: config.width,
            height: config.height,
            render_texture: texture,
            render_texture_view: texture_view,
            _depth_texture: depth_texture,
            depth_texture_view,
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

    type AssetInstanceId = u64;
    type AssetDescription = stl::StlMesh;
    type AssetTransform = geometry::RealTwist;

    type CameraState = camera::CameraState;

    fn configure(&mut self, configuration: Self::Configuration) {
        self.active_configuration = Some(configuration);
    }

    fn render(&mut self, camera_state: &Self::CameraState, output: &mut [u8]) {
        let config = self
            .active_configuration
            .expect("Renderer is not configured.");
        assert!(
            output.len() >= (config.width * config.height * self.state.pixel_size) as usize,
            "Output buffer is too small."
        );

        let needs_rebuild = match &self.render_targets {
            Some(targets) => targets.width != config.width || targets.height != config.height,
            None => true,
        };
        if needs_rebuild {
            self.rebuild_targets();
        }
        self.assets.flush_transforms(&self.state.queue);

        let targets = self.render_targets.as_ref().unwrap();
        if self.frame_index > 0 {
            let read_index = (self.frame_index + 1) % 2;
            let read_buffer = &targets.output_buffers[read_index];
            let buffer_slice = read_buffer.slice(..);

            let (sender, receiver) = std::sync::mpsc::channel();
            buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
                sender.send(result).unwrap();
            });
            let _ = self.state.device.poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            });
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
        self.state.queue.write_buffer(
            &self.state.camera_uniform_buffer,
            0,
            bytemuck::bytes_of::<camera::CameraUniform>(&camera_state.into()),
        );
        let mut encoder =
            self.state
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
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
                render_pass.set_pipeline(&self.state.environment_pipeline);
                render_pass.set_bind_group(0, &self.state.camera_bind_group, &[]);
                render_pass.set_bind_group(1, &self.state.colormap_bind_group, &[]);
                render_pass.set_vertex_buffer(0, self.vertex_arena.vertex_buffer().slice(..));
                render_pass.multi_draw_indirect(self.vertex_arena.draw_buffer(), 0, draw_count);
            }
            if !self.assets.is_empty() {
                render_pass.set_pipeline(&self.state.asset_pipeline);
                render_pass.set_bind_group(0, &self.state.camera_bind_group, &[]);
                render_pass.set_vertex_buffer(0, self.assets.vertex_buffer.slice(..));
                render_pass.set_vertex_buffer(1, self.assets.transform_buffer.slice(..));
                for asset in &self.assets.assets {
                    render_pass.draw(
                        asset.vertex_offset..asset.vertex_offset + asset.vertex_count,
                        asset.first_instance..asset.first_instance + asset.instance_count,
                    );
                }
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
            vertices.extend(triangles.iter().flat_map(|triangle| triangle.to_vertices()));
            self.vertex_arena.set_chunk(device, queue, index, &vertices);
        }
        self.vertex_arena.finish_update(device, queue);
    }

    fn upload_asset(
        &mut self,
        description: Self::AssetDescription,
        instances: usize,
    ) -> Vec<Self::AssetInstanceId> {
        let vertices = description
            .triangles
            .iter()
            .flat_map(|triangle| triangle.to_vertices())
            .collect::<Vec<geometry::Vertex>>();
        let first_instance =
            self.assets
                .upload(&self.state.device, &self.state.queue, &vertices, instances)
                as u64;
        (first_instance..first_instance + instances as u64).collect()
    }

    fn set_asset_transform(
        &mut self,
        instance_id: Self::AssetInstanceId,
        transform: Self::AssetTransform,
    ) {
        self.assets.set_transform(instance_id as usize, transform);
    }
}
