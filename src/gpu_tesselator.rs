use std::ops::Range;

use futures_channel::oneshot;
use rustc_hash::FxHashMap;
use wgpu::util::DeviceExt;

use crate::{
    Tesselator,
    cpu_tesselator::{
        CHUNK_RATIO, CHUNK_WIDTH, DirtyQueue, NEIGHBOUR_OFFSETS, VoxelStore, is_face_offset,
        marching_cubes::{MC_CUBE_CORNERS, MC_CUBE_EDGES, MC_EDGE_MASKS, MC_TRIANGLE_EDGES},
        neighbour_chunk,
    },
    geometry::{DiscretePoint, DiscreteUnit, PointI},
    gpu_context::GpuContext,
    wgpu_utils::VERTEX_STRIDE,
};

const WORKGROUP_SIZE: u32 = 4;
const WORKGROUPS_PER_CHUNK_AXIS: u32 = CHUNK_WIDTH as u32 / WORKGROUP_SIZE;
const INITIAL_VERTEX_CAPACITY: u32 = 1 << 15;
const NEIGHBOURHOOD: usize = 27;
const EMPTY_SLOT: u32 = 0;

const FACES_SHADER: &str = concat!(
    include_str!("shaders/tesselate_common.wgsl"),
    include_str!("shaders/tesselate_faces.wgsl"),
);
const MARCHING_CUBES_SHADER: &str = concat!(
    include_str!("shaders/tesselate_common.wgsl"),
    include_str!("shaders/tesselate_marching_cubes.wgsl"),
);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpuTesselatorConfiguration {
    pub voxel_size: f32,
    pub marching_cubes_enabled: bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Params {
    voxel_size: f32,
    chunk_count: u32,
    vertex_capacity: u32,
    padding: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
struct DirtyChunk {
    origin: [DiscreteUnit; 4],
    neighbours: [u32; NEIGHBOURHOOD],
    padding: u32,
}

pub struct GpuGeometryDelta {
    context_id: u64,
    vertices: wgpu::Buffer,
    chunks: Vec<(DiscretePoint, Range<u32>)>,
}

impl GpuGeometryDelta {
    pub(crate) fn new(
        context: &GpuContext,
        vertices: wgpu::Buffer,
        chunks: Vec<(DiscretePoint, Range<u32>)>,
    ) -> Self {
        Self {
            context_id: context.id(),
            vertices,
            chunks,
        }
    }

    pub fn context_id(&self) -> u64 {
        self.context_id
    }

    pub fn vertices(&self) -> &wgpu::Buffer {
        &self.vertices
    }

    pub fn chunks(&self) -> &[(DiscretePoint, Range<u32>)] {
        &self.chunks
    }
}

struct Batch {
    chunks: Vec<DiscretePoint>,
    vertex_capacity: u32,
    vertices: wgpu::Buffer,
    counts: wgpu::Buffer,
    mapped: oneshot::Receiver<Result<(), wgpu::BufferAsyncError>>,
}

pub struct GpuTesselator {
    context: GpuContext,
    faces_pipeline: wgpu::ComputePipeline,
    marching_cubes_pipeline: wgpu::ComputePipeline,
    tables_bind_group: wgpu::BindGroup,
    bind_group_layout: wgpu::BindGroupLayout,
    config: Option<GpuTesselatorConfiguration>,
    voxels: VoxelStore,
    dirty: DirtyQueue,
    marching_cubes_enabled: bool,
    vertex_capacity: u32,
    in_flight: Option<Batch>,
}

fn storage_entry(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn max_vertex_capacity(limits: &wgpu::Limits) -> u32 {
    let bytes = limits
        .max_storage_buffer_binding_size
        .min(limits.max_buffer_size);
    (bytes / VERTEX_STRIDE).min(u64::from(u32::MAX)) as u32
}

fn batch_size(vertex_capacity: u32, limits: &wgpu::Limits) -> usize {
    let chunk_bytes = u64::from(vertex_capacity) * VERTEX_STRIDE;
    let by_memory = limits
        .max_storage_buffer_binding_size
        .min(limits.max_buffer_size)
        / chunk_bytes;
    let by_dispatch =
        u64::from(limits.max_compute_workgroups_per_dimension / WORKGROUPS_PER_CHUNK_AXIS);
    by_memory.min(by_dispatch).max(1) as usize
}

fn neighbour(chunk: DiscretePoint, n: usize) -> DiscretePoint {
    let d = |i: usize| i as i8 - 1;
    neighbour_chunk(chunk, [d(n % 3), d(n / 3 % 3), d(n / 9)])
}

fn neighbourhood_tables(
    voxels: &VoxelStore,
    chunks: &[DiscretePoint],
) -> (Vec<u32>, Vec<DirtyChunk>) {
    let mut masks = vec![0; CHUNK_RATIO];
    let mut slots = FxHashMap::default();
    let mut dirty = Vec::with_capacity(chunks.len());
    for &chunk in chunks {
        let mut neighbours = [EMPTY_SLOT; NEIGHBOURHOOD];
        for (n, slot) in neighbours.iter_mut().enumerate() {
            let neighbour = neighbour(chunk, n);
            let Some(stored) = voxels.subchunk_masks(neighbour) else {
                continue;
            };
            *slot = *slots.entry(neighbour).or_insert_with(|| {
                masks.extend_from_slice(stored);
                (masks.len() / CHUNK_RATIO - 1) as u32
            });
        }
        dirty.push(DirtyChunk {
            origin: [chunk.x, chunk.y, chunk.z, 0],
            neighbours,
            padding: 0,
        });
    }
    (masks, dirty)
}

/// `MarchingCubesTables` of the marching cubes shader, as words.
fn marching_cubes_tables() -> Vec<u32> {
    let mut words = Vec::new();
    for [x, y, z] in MC_CUBE_CORNERS {
        words.extend([x as u32, y as u32, z as u32, 0]);
    }
    for [a, b] in MC_CUBE_EDGES {
        words.extend([a as u32, b as u32]);
    }
    words.extend(MC_EDGE_MASKS.iter().map(|&mask| u32::from(mask)));
    words.extend(
        MC_TRIANGLE_EDGES
            .iter()
            .flatten()
            .map(|&edge| i32::from(edge) as u32),
    );
    words
}

fn create_pipeline(
    device: &wgpu::Device,
    label: &str,
    bind_group_layouts: &[Option<&wgpu::BindGroupLayout>],
    source: &str,
) -> wgpu::ComputePipeline {
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(label),
        bind_group_layouts,
        immediate_size: 0,
    });
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(label),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });
    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(label),
        layout: Some(&layout),
        module: &module,
        entry_point: Some("cs_main"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    })
}

impl GpuTesselator {
    pub fn new(context: &GpuContext) -> Option<Self> {
        if !context.supports_compute() {
            return None;
        }
        let device = context.device();
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Tesselator Bind Group Layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                storage_entry(1, true),  // chunk_masks
                storage_entry(2, true),  // dirty
                storage_entry(3, false), // vertices
                storage_entry(4, false), // counts
            ],
        });
        let tables_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Marching Cubes Tables Bind Group Layout"),
                entries: &[storage_entry(0, true)],
            });
        let tables = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Marching Cubes Tables Buffer"),
            contents: bytemuck::cast_slice(&marching_cubes_tables()),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let tables_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Marching Cubes Tables Bind Group"),
            layout: &tables_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: tables.as_entire_binding(),
            }],
        });
        let faces_pipeline = create_pipeline(
            device,
            "Face Tesselator",
            &[Some(&bind_group_layout)],
            FACES_SHADER,
        );
        let marching_cubes_pipeline = create_pipeline(
            device,
            "Marching Cubes Tesselator",
            &[Some(&bind_group_layout), Some(&tables_bind_group_layout)],
            MARCHING_CUBES_SHADER,
        );
        Some(Self {
            context: context.clone(),
            faces_pipeline,
            marching_cubes_pipeline,
            tables_bind_group,
            bind_group_layout,
            config: None,
            voxels: VoxelStore::new(),
            dirty: DirtyQueue::default(),
            marching_cubes_enabled: false,
            vertex_capacity: INITIAL_VERTEX_CAPACITY.min(max_vertex_capacity(&device.limits())),
            in_flight: None,
        })
    }

    pub fn context(&self) -> &GpuContext {
        &self.context
    }

    pub fn marching_cubes_enabled(&self) -> bool {
        self.marching_cubes_enabled
    }

    pub fn set_marching_cubes_enabled(&mut self, enabled: bool) {
        if self.marching_cubes_enabled == enabled {
            return;
        }
        self.marching_cubes_enabled = enabled;
        for chunk in self.voxels.chunks() {
            self.dirty.mark(chunk);
            for offset in NEIGHBOUR_OFFSETS {
                if self.voxels.has_voxel_on_boundary(chunk, offset) {
                    self.dirty.mark(neighbour_chunk(chunk, offset));
                }
            }
        }
    }

    pub fn wait(&mut self) -> Option<GpuGeometryDelta> {
        loop {
            if self.in_flight.is_none() {
                if self.dirty.is_empty() {
                    return None;
                }
                self.submit_batch();
            }
            let _ = self.context.device().poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            });
            if let Some(delta) = self.finish_batch() {
                return Some(delta);
            }
        }
    }

    fn submit_batch(&mut self) {
        let config = self
            .config
            .expect("Tesselator must be configured before generating output geometry");
        let device = self.context.device();
        let chunks = self
            .dirty
            .take(batch_size(self.vertex_capacity, &device.limits()));
        let (masks, dirty) = neighbourhood_tables(&self.voxels, &chunks);

        let params = Params {
            voxel_size: config.voxel_size,
            chunk_count: chunks.len() as u32,
            vertex_capacity: self.vertex_capacity,
            padding: 0,
        };
        let params = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Tesselator Params Buffer"),
            contents: bytemuck::bytes_of(&params),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let masks = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Tesselator Chunk Mask Buffer"),
            contents: bytemuck::cast_slice(&masks),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let dirty = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Tesselator Dirty Chunk Buffer"),
            contents: bytemuck::cast_slice(&dirty),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let vertices = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Tesselator Vertex Buffer"),
            size: chunks.len() as u64 * u64::from(self.vertex_capacity) * VERTEX_STRIDE,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let count_bytes = (chunks.len() * std::mem::size_of::<u32>()) as wgpu::BufferAddress;
        let counts = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Tesselator Vertex Count Buffer"),
            size: count_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let counts_readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Tesselator Vertex Count Readback Buffer"),
            size: count_bytes,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Tesselator Bind Group"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: masks.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: dirty.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: vertices.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: counts.as_entire_binding(),
                },
            ],
        });

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Tesselator Encoder"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Tesselator Pass"),
                timestamp_writes: None,
            });
            pass.set_bind_group(0, &bind_group, &[]);
            if self.marching_cubes_enabled {
                pass.set_pipeline(&self.marching_cubes_pipeline);
                pass.set_bind_group(1, &self.tables_bind_group, &[]);
            } else {
                pass.set_pipeline(&self.faces_pipeline);
            }
            pass.dispatch_workgroups(
                WORKGROUPS_PER_CHUNK_AXIS,
                WORKGROUPS_PER_CHUNK_AXIS,
                WORKGROUPS_PER_CHUNK_AXIS * chunks.len() as u32,
            );
        }
        encoder.copy_buffer_to_buffer(&counts, 0, &counts_readback, 0, count_bytes);
        self.context.queue().submit(Some(encoder.finish()));

        let (sender, mapped) = oneshot::channel();
        counts_readback
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = sender.send(result);
            });
        self.in_flight = Some(Batch {
            chunks,
            vertex_capacity: self.vertex_capacity,
            vertices,
            counts: counts_readback,
            mapped,
        });
    }

    fn finish_batch(&mut self) -> Option<GpuGeometryDelta> {
        let batch = self.in_flight.as_mut()?;
        batch
            .mapped
            .try_recv()
            .expect("the GPU dropped the mapping callback")?
            .expect("mapping the vertex counts failed");
        let batch = self.in_flight.take().unwrap();
        let counts: Vec<u32> = {
            let mapped = batch.counts.slice(..).get_mapped_range().unwrap();
            bytemuck::cast_slice(&mapped).to_vec()
        };
        batch.counts.unmap();

        let max_capacity = max_vertex_capacity(&self.context.device().limits());
        let mut chunks = Vec::with_capacity(batch.chunks.len());
        let mut most = 0;
        for (slot, (&chunk, &count)) in batch.chunks.iter().zip(&counts).enumerate() {
            if count > batch.vertex_capacity && batch.vertex_capacity < max_capacity {
                most = most.max(count);
                self.dirty.mark(chunk);
                continue;
            }
            let count = count.min(batch.vertex_capacity);
            let first = slot as u32 * batch.vertex_capacity;
            chunks.push((chunk, first..first + count));
        }
        if most > 0 {
            self.vertex_capacity = most
                .checked_next_power_of_two()
                .unwrap_or(u32::MAX)
                .max(self.vertex_capacity.saturating_mul(2))
                .min(max_capacity);
        }
        Some(GpuGeometryDelta::new(&self.context, batch.vertices, chunks))
    }
}

impl Tesselator for GpuTesselator {
    type Configuration = GpuTesselatorConfiguration;
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = GpuGeometryDelta;

    fn configure(&mut self, configuration: Self::Configuration) {
        self.config = Some(configuration);
        self.set_marching_cubes_enabled(configuration.marching_cubes_enabled);
    }

    fn update_input_geometry(&mut self, geometry_delta: &Self::InputGeometryDelta) {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before updating input geometry"
        );
        for change in self.voxels.update(geometry_delta) {
            self.dirty.mark(change.chunk);
            for offset in change.touched_neighbours() {
                if self.marching_cubes_enabled || is_face_offset(offset) {
                    self.dirty.mark(neighbour_chunk(change.chunk, offset));
                }
            }
        }
    }

    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before generating output geometry"
        );
        if self.in_flight.is_none() {
            if self.dirty.is_empty() {
                return None;
            }
            self.submit_batch();
        }
        let _ = self.context.device().poll(wgpu::PollType::Poll);
        self.finish_batch()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wgpu_utils::test_support::TestGpu;

    const VERTEX_FLOATS: u64 = 9;

    fn block(x: i32, y: i32, z: i32) -> PointI {
        PointI { x, y, z, i: 1 }
    }

    #[test]
    fn layouts_match_the_shader() {
        assert_eq!(std::mem::size_of::<Params>(), 16);
        // vec4<i32>, array<u32, 27> and the padding to 16 bytes
        assert_eq!(std::mem::size_of::<DirtyChunk>(), 128);
        assert_eq!(VERTEX_FLOATS * 4, VERTEX_STRIDE);
        assert_eq!(CHUNK_WIDTH as u32 % WORKGROUP_SIZE, 0);
    }

    #[test]
    fn neighbours_are_ordered_x_first() {
        let origin = DiscretePoint { x: 0, y: 0, z: 0 };
        let w = CHUNK_WIDTH as DiscreteUnit;
        assert_eq!(
            neighbour(origin, 0),
            DiscretePoint {
                x: -w,
                y: -w,
                z: -w
            }
        );
        assert_eq!(neighbour(origin, 13), origin);
        assert_eq!(neighbour(origin, 14), DiscretePoint { x: w, y: 0, z: 0 });
        assert_eq!(neighbour(origin, 16), DiscretePoint { x: 0, y: w, z: 0 });
        assert_eq!(neighbour(origin, 22), DiscretePoint { x: 0, y: 0, z: w });
    }

    #[test]
    fn neighbourhoods_share_mask_slots() {
        let mut voxels = VoxelStore::new();
        let w = CHUNK_WIDTH as i32;
        voxels.update(&[block(0, 0, 0), block(w, 0, 0)]);
        let chunks = [
            DiscretePoint { x: 0, y: 0, z: 0 },
            DiscretePoint { x: w, y: 0, z: 0 },
            DiscretePoint {
                x: 5 * w,
                y: 0,
                z: 0,
            },
        ];
        let (masks, dirty) = neighbourhood_tables(&voxels, &chunks);
        assert_eq!(masks.len(), 3 * CHUNK_RATIO);
        assert!(masks[..CHUNK_RATIO].iter().all(|&mask| mask == 0));
        assert_eq!(dirty[0].origin, [0, 0, 0, 0]);
        // Chunk 0 sees itself at 13 and chunk 1 at 14, chunk 1 sees chunk 0 at 12
        assert_eq!(dirty[0].neighbours[13], dirty[1].neighbours[12]);
        assert_eq!(dirty[0].neighbours[14], dirty[1].neighbours[13]);
        assert_ne!(dirty[0].neighbours[13], dirty[0].neighbours[14]);
        let filled = |dirty: &DirtyChunk| {
            dirty
                .neighbours
                .iter()
                .filter(|&&slot| slot != EMPTY_SLOT)
                .count()
        };
        assert_eq!(filled(&dirty[0]), 2);
        assert_eq!(filled(&dirty[2]), 0);
        let slot = dirty[0].neighbours[13] as usize;
        assert_eq!(masks[slot * CHUNK_RATIO], 1);
    }

    #[test]
    fn batches_fit_the_limits() {
        let limits = wgpu::Limits::default();
        let size = batch_size(INITIAL_VERTEX_CAPACITY, &limits);
        assert!(
            size as u64 * u64::from(INITIAL_VERTEX_CAPACITY) * VERTEX_STRIDE
                <= limits.max_storage_buffer_binding_size
        );
        assert!(
            size as u32 * WORKGROUPS_PER_CHUNK_AXIS <= limits.max_compute_workgroups_per_dimension
        );
        assert_eq!(batch_size(max_vertex_capacity(&limits), &limits), 1);
    }

    #[test]
    fn dirty_chunks_come_back_from_both_pipelines() {
        let gpu = TestGpu::new();
        let mut tesselator = GpuTesselator::new(&gpu.context).expect("no compute shaders");
        tesselator.configure(GpuTesselatorConfiguration {
            voxel_size: 1.0,
            marching_cubes_enabled: false,
        });
        tesselator.update_input_geometry(&vec![block(0, 0, 0)]);
        for marching_cubes_enabled in [false, true] {
            tesselator.set_marching_cubes_enabled(marching_cubes_enabled);
            let delta = tesselator.wait().expect("a dirty chunk");
            assert_eq!(delta.context_id(), gpu.context.id());
            assert!(
                delta
                    .chunks()
                    .iter()
                    .any(|(index, _)| *index == DiscretePoint { x: 0, y: 0, z: 0 })
            );
            for (_, range) in delta.chunks() {
                assert!(range.end <= range.start + tesselator.vertex_capacity);
                assert!(u64::from(range.end) * VERTEX_STRIDE <= delta.vertices().size());
            }
            assert!(tesselator.wait().is_none());
            assert!(tesselator.get_output_geometry_delta().is_none());
        }
    }

    #[test]
    fn marching_cubes_tables_match_the_shader_layout() {
        let words = marching_cubes_tables();
        // array<vec4<u32>, 8>, array<vec2<u32>, 12>, array<u32, 256>, array<i32, 4096>
        let (corners, edges, masks, triangles) = (0, 8 * 4, 8 * 4 + 12 * 2, 8 * 4 + 12 * 2 + 256);
        assert_eq!(words.len(), triangles + 4096);
        for (i, corner) in MC_CUBE_CORNERS.iter().enumerate() {
            let word = |k: usize| words[corners + i * 4 + k] as usize;
            assert_eq!([word(0), word(1), word(2)], *corner);
        }
        for (i, edge) in MC_CUBE_EDGES.iter().enumerate() {
            let word = |k: usize| words[edges + i * 2 + k] as usize;
            assert_eq!([word(0), word(1)], *edge);
        }
        for (i, &mask) in MC_EDGE_MASKS.iter().enumerate() {
            assert_eq!(words[masks + i], u32::from(mask));
        }
        for (i, &edge) in MC_TRIANGLE_EDGES.iter().flatten().enumerate() {
            assert_eq!(words[triangles + i] as i32, i32::from(edge));
        }
    }
}
