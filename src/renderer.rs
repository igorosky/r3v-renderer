use std::ops::Range;

use crate::wgpu_utils::*;
use crate::{Renderer, camera, geometry, stl, vertex_arena::VertexArena};

pub const DEFAULT_BACKGROUND_COLOR: [f32; 3] = [0.0; 3];
pub const DEFAULT_COLOR_DEPTH_RANGE: f32 = 50.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WgpuAcceleratedConfiguration {
    pub width: u32,
    pub height: u32,
    /// sRGB, each component in 0..1. The background is opaque.
    pub background_color: [f32; 3],
    /// Distance along the camera's forward axis over which the colormap runs once.
    pub color_depth_range: f32,
}

fn srgb_to_linear(component: f32) -> f64 {
    let c = f64::from(component);
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

struct WgpuAcceleratedState {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub environment_pipeline: wgpu::RenderPipeline,
    pub asset_pipeline: wgpu::RenderPipeline,
    pub pixel_size: u32,
    pub camera_uniform_buffer: wgpu::Buffer,
    pub camera_bind_group: wgpu::BindGroup,
    pub colormap_buffer: wgpu::Buffer,
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
const INITIAL_ASSET_INSTANCE_CAPACITY: usize = 1024;

struct Asset {
    vertex_offset: u32,
    vertex_count: u32,
    first_instance: u32,
    instance_count: u32,
}

struct InstanceData<T> {
    label: &'static str,
    buffer: wgpu::Buffer,
    capacity: usize,
    values: Vec<T>,
    dirty: bool,
}

impl<T: bytemuck::Pod + PartialEq> InstanceData<T> {
    const STRIDE: wgpu::BufferAddress = std::mem::size_of::<T>() as wgpu::BufferAddress;

    fn new(device: &wgpu::Device, label: &'static str) -> Self {
        Self {
            label,
            buffer: create_instance_buffer::<T>(device, label, INITIAL_ASSET_INSTANCE_CAPACITY),
            capacity: INITIAL_ASSET_INSTANCE_CAPACITY,
            values: Vec::new(),
            dirty: false,
        }
    }

    fn len(&self) -> usize {
        self.values.len()
    }

    fn extend(&mut self, device: &wgpu::Device, value: T, count: usize) {
        let required = self.values.len() + count;
        if required > self.capacity {
            while self.capacity < required {
                self.capacity *= 2;
            }
            self.buffer = create_instance_buffer::<T>(device, self.label, self.capacity);
        }
        self.values.extend(std::iter::repeat_n(value, count));
        self.dirty = true;
    }

    fn set(&mut self, instance: usize, value: T) {
        let current = &mut self.values[instance];
        if *current == value {
            return;
        }
        *current = value;
        self.dirty = true;
    }

    fn flush(&mut self, queue: &wgpu::Queue) {
        if !self.dirty {
            return;
        }
        queue.write_buffer(&self.buffer, 0, bytemuck::cast_slice(&self.values));
        self.dirty = false;
    }
}

struct Assets {
    vertex_buffer: wgpu::Buffer,
    vertex_capacity: usize,
    vertex_count: usize,
    transforms: InstanceData<geometry::RealTwist>,
    colors: InstanceData<[f32; 3]>,
    visible: Vec<bool>,
    assets: Vec<Asset>,
}

fn visible_runs(visible: &[bool], first: u32) -> impl Iterator<Item = Range<u32>> + '_ {
    visible
        .chunk_by(|a, b| a == b)
        .scan(first, |start, run| {
            let range = *start..*start + run.len() as u32;
            *start = range.end;
            Some(run[0].then_some(range))
        })
        .flatten()
}

fn create_instance_buffer<T>(
    device: &wgpu::Device,
    label: &'static str,
    capacity: usize,
) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: (capacity * std::mem::size_of::<T>()) as wgpu::BufferAddress,
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
            transforms: InstanceData::new(device, "Asset Transform Buffer"),
            colors: InstanceData::new(device, "Asset Color Buffer"),
            visible: Vec::new(),
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
        let first_instance = self.transforms.len();
        self.transforms
            .extend(device, geometry::RealTwist::identity(), instances);
        self.colors
            .extend(device, geometry::Vertex::DEFAULT_COLOR, instances);
        self.visible.extend(std::iter::repeat_n(true, instances));
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
        self.transforms.set(instance, transform);
    }

    fn set_color(&mut self, instance: usize, color: [f32; 3]) {
        self.colors.set(instance, color);
    }

    fn set_visible(&mut self, instance: usize, visible: bool) {
        self.visible[instance] = visible;
    }

    fn visible_instances(&self, asset: &Asset) -> impl Iterator<Item = Range<u32>> + '_ {
        let first = asset.first_instance as usize;
        visible_runs(
            &self.visible[first..first + asset.instance_count as usize],
            asset.first_instance,
        )
    }

    fn flush_instances(&mut self, queue: &wgpu::Queue) {
        self.transforms.flush(queue);
        self.colors.flush(queue);
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
}

pub struct WgpuAccelerated {
    state: WgpuAcceleratedState,
    active_configuration: Option<WgpuAcceleratedConfiguration>,
    render_targets: Option<RenderTargets>,
    vertex_arena: VertexArena,
    assets: Assets,
    frame_index: usize,
    frustum_culling: bool,
    drawn_chunks: u32,
}

impl WgpuAccelerated {
    pub fn set_frustum_culling(&mut self, enabled: bool) {
        self.frustum_culling = enabled;
    }

    pub fn frustum_culling(&self) -> bool {
        self.frustum_culling
    }

    pub fn drawn_chunks(&self) -> (u32, u32) {
        (self.drawn_chunks, self.vertex_arena.draw_count())
    }

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
            ..Default::default()
        };
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                required_limits,
                ..Default::default()
            })
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
        let colormap_buffer = create_turbo_colormap_buffer(&device, DEFAULT_COLOR_DEPTH_RANGE);
        let colormap_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Colormap Bind Group Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });
        let colormap_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Colormap Bind Group"),
            layout: &colormap_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: colormap_buffer.as_entire_binding(),
            }],
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
                        array_stride: InstanceData::<geometry::RealTwist>::STRIDE,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![
                            3 => Float32x3, // linear.x, linear.y, linear.z
                            4 => Float32x4, // rotational.x, rotational.y, rotational.z, rotational.w
                        ],
                    }),
                    Some(wgpu::VertexBufferLayout {
                        array_stride: InstanceData::<[f32; 3]>::STRIDE,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![5 => Float32x3],
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
            colormap_buffer,
            colormap_bind_group,
        };

        Some(Self {
            state,
            active_configuration: None,
            render_targets: None,
            vertex_arena,
            assets,
            frame_index: 0,
            frustum_culling: true,
            drawn_chunks: 0,
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

    /// `Renderer::render` for callers that can not block: waiting for the
    /// previous frame to be read back is awaited instead. The web has to
    /// render this way, blocking there never lets the read back finish.
    pub async fn render_async(&mut self, camera_state: &camera::CameraState, output: &mut [u8]) {
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
        self.assets.flush_instances(&self.state.queue);
        self.drawn_chunks = if self.frustum_culling {
            self.vertex_arena.cull(
                &self.state.device,
                &self.state.queue,
                &camera_state.frustum(),
            )
        } else {
            self.vertex_arena.draw_count()
        };

        let [r, g, b] = config.background_color.map(srgb_to_linear);
        let background = wgpu::Color { r, g, b, a: 1.0 };
        let targets = self.render_targets.as_ref().unwrap();
        if self.frame_index > 0 {
            let read_index = (self.frame_index + 1) % 2;
            let read_buffer = &targets.output_buffers[read_index];
            let buffer_slice = read_buffer.slice(..);

            let (sender, receiver) = futures_channel::oneshot::channel();
            buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
                let _ = sender.send(result);
            });
            // Native backends map (and call back) while polling, the web maps
            // on its own and polling does nothing there
            let _ = self.state.device.poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            });
            receiver
                .await
                .expect("the GPU dropped the mapping callback")
                .expect("mapping the frame buffer failed");

            {
                let padded_data = buffer_slice.get_mapped_range().unwrap();
                // The CPU may still hold lines of this buffer from two frames ago
                invalidate_cpu_cache(&padded_data);
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
                        load: wgpu::LoadOp::Clear(background),
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
            let draw_count = self.drawn_chunks;
            let draw_buffer = if self.frustum_culling {
                self.vertex_arena.visible_buffer()
            } else {
                self.vertex_arena.draw_buffer()
            };
            if draw_count > 0 {
                render_pass.set_pipeline(&self.state.environment_pipeline);
                render_pass.set_bind_group(0, &self.state.camera_bind_group, &[]);
                render_pass.set_bind_group(1, &self.state.colormap_bind_group, &[]);
                render_pass.set_vertex_buffer(0, self.vertex_arena.vertex_buffer().slice(..));
                render_pass.multi_draw_indirect(draw_buffer, 0, draw_count);
            }
            if !self.assets.is_empty() {
                render_pass.set_pipeline(&self.state.asset_pipeline);
                render_pass.set_bind_group(0, &self.state.camera_bind_group, &[]);
                render_pass.set_vertex_buffer(0, self.assets.vertex_buffer.slice(..));
                render_pass.set_vertex_buffer(1, self.assets.transforms.buffer.slice(..));
                render_pass.set_vertex_buffer(2, self.assets.colors.buffer.slice(..));
                for asset in &self.assets.assets {
                    for instances in self.assets.visible_instances(asset) {
                        render_pass.draw(
                            asset.vertex_offset..asset.vertex_offset + asset.vertex_count,
                            instances,
                        );
                    }
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
        self.state
            .queue
            .submit(std::iter::once(encoder.finish()).chain(host_read_barrier(
                &self.state.device,
                &targets.output_buffers[write_index],
            )));
        self.frame_index += 1;
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
        self.state.queue.write_buffer(
            &self.state.colormap_buffer,
            COLORMAP_PARAMS_OFFSET,
            bytemuck::bytes_of(&colormap_params(configuration.color_depth_range)),
        );
        self.active_configuration = Some(configuration);
    }

    fn render(&mut self, camera_state: &Self::CameraState, output: &mut [u8]) {
        pollster::block_on(self.render_async(camera_state, output))
    }

    fn update_geometry(&mut self, geometry_delta: Self::GeometryDelta) {
        let device = &self.state.device;
        let queue = &self.state.queue;
        let mut vertices = Vec::new();
        for geometry::TriangleChunk {
            index,
            triangles: quads,
        } in geometry_delta
        {
            vertices.clear();
            vertices.extend(quads.iter().flat_map(|triangle| triangle.to_vertices()));
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

    fn set_asset_visibility(&mut self, instance_id: Self::AssetInstanceId, visible: bool) {
        self.assets.set_visible(instance_id as usize, visible);
    }

    fn set_asset_color(&mut self, instance_id: Self::AssetInstanceId, color: [f32; 3]) {
        self.assets.set_color(instance_id as usize, color);
    }
}

#[cfg(test)]
mod tests {
    use super::visible_runs;

    fn runs(visible: &[bool], first: u32) -> Vec<(u32, u32)> {
        visible_runs(visible, first)
            .map(|range| (range.start, range.end))
            .collect()
    }

    #[test]
    fn all_visible_is_one_run() {
        assert_eq!(runs(&[true; 5], 10), [(10, 15)]);
    }

    #[test]
    fn all_hidden_or_empty_draws_nothing() {
        assert!(runs(&[false; 5], 10).is_empty());
        assert!(runs(&[], 10).is_empty());
    }

    #[test]
    fn hidden_instances_split_runs() {
        assert_eq!(
            runs(&[true, false, false, true, true, false, true], 3),
            [(3, 4), (6, 8), (9, 10)]
        );
        assert_eq!(runs(&[false, true, true], 0), [(1, 3)]);
        assert_eq!(runs(&[true, true, false], 0), [(0, 2)]);
    }

    #[test]
    fn asset_color_tints_only_its_instance() {
        use super::{WgpuAccelerated, WgpuAcceleratedConfiguration};
        use crate::{
            Camera, Renderer,
            camera::OrbitCamera,
            geometry::{RealTriangle, RealTwist, Vector3D},
            stl::StlMesh,
            wgpu_utils::test_support::TestGpu,
        };

        let _lock = TestGpu::new();
        let mut renderer = WgpuAccelerated::new().expect("no wgpu adapter available");
        let size = 32;
        renderer.configure(WgpuAcceleratedConfiguration {
            width: size,
            height: size,
        });
        let [a, b, c, d] = [
            [-0.5, -0.5, 0.0],
            [0.5, -0.5, 0.0],
            [0.5, 0.5, 0.0],
            [-0.5, 0.5, 0.0],
        ];
        let triangles = [[a, b, c], [a, c, d], [a, c, b], [a, d, c]]
            .map(|[v0, v1, v2]| RealTriangle { v0, v1, v2 })
            .to_vec();
        let ids = renderer.upload_asset(StlMesh { triangles }, 2);
        for (id, x) in ids.iter().zip([-0.6, 0.6]) {
            let mut transform = RealTwist::identity();
            transform.linear = Vector3D { x, y: 0.0, z: 0.0 };
            renderer.set_asset_transform(*id, transform);
        }

        let camera = OrbitCamera::default().get_state(1.0);
        let frame = |renderer: &mut WgpuAccelerated| {
            let mut output = vec![0; (size * size * 4) as usize];
            renderer.render(&camera, &mut output);
            renderer.render(&camera, &mut output);
            let pixel = |x: u32| {
                let offset = ((size / 2 * size + x) * 4) as usize;
                let [b, g, r, _] = output[offset..offset + 4] else {
                    unreachable!()
                };
                [r, g, b]
            };
            (pixel(size * 3 / 8), pixel(size * 5 / 8))
        };
        let (left, right) = frame(&mut renderer);
        assert!(left[0] > 0 && left == right, "{left:?} and {right:?}");
        let white = left;

        renderer.set_asset_color(ids[0], [1.0, 0.0, 0.0]);
        let (left, right) = frame(&mut renderer);
        assert_eq!(left, [white[0], 0, 0]);
        assert_eq!(right, white);
    }

    #[test]
    fn culling_does_not_change_the_image() {
        use super::{WgpuAccelerated, WgpuAcceleratedConfiguration};
        use crate::{
            Camera, Renderer,
            camera::OrbitCamera,
            geometry::{DiscretePoint, RealTriangle, TriangleChunk},
            wgpu_utils::test_support::TestGpu,
        };

        let _lock = TestGpu::new();
        let mut renderer = WgpuAccelerated::new().expect("no wgpu adapter available");
        let size = 64;
        renderer.configure(WgpuAcceleratedConfiguration {
            width: size,
            height: size,
        });
        let square = |x: f32, z: f32| {
            let [a, b, c, d] = [
                [x - 1.0, -1.0, z],
                [x + 1.0, -1.0, z],
                [x + 1.0, 1.0, z],
                [x - 1.0, 1.0, z],
            ];
            [[a, b, c], [a, c, d], [a, c, b], [a, d, c]]
                .map(|[v0, v1, v2]| RealTriangle { v0, v1, v2 })
                .to_vec()
        };
        let chunks = (-8..=8)
            .flat_map(|i| (0..4).map(move |j| (i, j)))
            .map(|(i, j)| TriangleChunk {
                index: DiscretePoint {
                    x: i * 16,
                    y: 0,
                    z: j * 16,
                },
                triangles: square(i as f32 * 3.0, j as f32 * -3.0),
            })
            .collect();
        renderer.update_geometry(chunks);

        let camera = OrbitCamera::default().get_state(1.0);
        let mut frame = |culling| {
            renderer.set_frustum_culling(culling);
            let mut output = vec![0; (size * size * 4) as usize];
            renderer.render(&camera, &mut output);
            renderer.render(&camera, &mut output);
            (output, renderer.drawn_chunks())
        };
        let (culled, (drawn, total)) = frame(true);
        let (all, (drawn_unculled, _)) = frame(false);
        assert_eq!(total, 17 * 4);
        assert_eq!(drawn_unculled, total);
        assert!(0 < drawn && drawn < total / 2, "{drawn} of {total} drawn");
        assert!(culled.iter().any(|&byte| byte != 0), "nothing was drawn");
        assert!(culled == all, "culling changed the image");
    }
}
