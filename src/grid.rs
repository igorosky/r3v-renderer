//! Line grids in the rendered world: a lattice of cells, or a plane when one
//! of the sizes is 0, placed in the world or relative to a named frame (e.g.
//! a robot link) whose pose the caller keeps up to date.

use std::collections::{BTreeMap, HashMap};

use wgpu::util::DeviceExt;

use crate::{geometry::RealTwist, robot::compose_twists, wgpu_utils::srgb_to_linear};

/// A flat grid on the ground: the renderer's world is Y-up.
pub const DEFAULT_GRID_SIZE: [f32; 3] = [10.0, 0.0, 10.0];
pub const DEFAULT_GRID_RESOLUTION: [f32; 3] = [1.0; 3];
pub const DEFAULT_GRID_COLOR: [f32; 3] = [0.5; 3];
/// Most lines one grid can have.
pub const MAX_GRID_LINES: u64 = 1 << 20;

/// A line closer to the boundary than this many cells is the boundary.
const BOUNDARY_TOLERANCE: f64 = 1e-4;

/// Lines `resolution` apart along the x, y and z axes of `pose`, spanning
/// `size` centered on its origin. Lines go through the origin and along the
/// boundary, so a size that is not a multiple of the resolution leaves
/// narrower cells at the edges.
#[derive(Clone, Debug, PartialEq)]
pub struct GridDescription {
    /// Extent along x, y and z, each at least 0. There are no lines along an
    /// axis of size 0: `[10, 0, 10]` is a plane.
    pub size: [f32; 3],
    /// Distance between neighbouring lines (the cell size) along x, y and z,
    /// each positive.
    pub resolution: [f32; 3],
    /// sRGB, each component in 0..1.
    pub color: [f32; 3],
    /// The frame `pose` is relative to, `None` for the world. The grid is
    /// drawn only while the renderer knows the frame's transform.
    pub frame: Option<String>,
    pub pose: RealTwist,
}

impl Default for GridDescription {
    fn default() -> Self {
        Self {
            size: DEFAULT_GRID_SIZE,
            resolution: DEFAULT_GRID_RESOLUTION,
            color: DEFAULT_GRID_COLOR,
            frame: None,
            pose: RealTwist::identity(),
        }
    }
}

/// First and last multiple of the resolution strictly inside an axis of
/// `size`, in cells. `last < first` when there is none.
fn interior_cells(size: f32, resolution: f32) -> (f64, f64) {
    let half = f64::from(size) / 2.0 / f64::from(resolution);
    let first = (-half + BOUNDARY_TOLERANCE).floor() + 1.0;
    let last = (half - BOUNDARY_TOLERANCE).ceil() - 1.0;
    (first, last)
}

/// Lines across an axis: both boundaries and the multiples of the
/// resolution between them, or the origin alone on an axis of size 0.
fn position_count(size: f32, resolution: f32) -> f64 {
    if size == 0.0 {
        return 1.0;
    }
    let (first, last) = interior_cells(size, resolution);
    2.0 + (last - first + 1.0).max(0.0)
}

fn positions(size: f32, resolution: f32) -> Vec<f32> {
    if size == 0.0 {
        return vec![0.0];
    }
    let half = size / 2.0;
    let (first, last) = interior_cells(size, resolution);
    let resolution = f64::from(resolution);
    std::iter::once(-half)
        .chain((first as i64..=last as i64).map(|cell| (cell as f64 * resolution) as f32))
        .chain(std::iter::once(half))
        .collect()
}

/// Lines of a grid, without building it. Saturates at `u64::MAX`.
pub fn line_count(size: [f32; 3], resolution: [f32; 3]) -> u64 {
    let counts: [f64; 3] = std::array::from_fn(|axis| position_count(size[axis], resolution[axis]));
    (0..3)
        .filter(|&axis| size[axis] > 0.0)
        .map(|axis| counts[(axis + 1) % 3] * counts[(axis + 2) % 3])
        .sum::<f64>() as u64
}

/// Endpoints of the lines of a grid in its own frame, two per line: first
/// the lines along x, then y, then z.
pub fn grid_lines(size: [f32; 3], resolution: [f32; 3]) -> Vec<[f32; 3]> {
    let positions: [Vec<f32>; 3] =
        std::array::from_fn(|axis| positions(size[axis], resolution[axis]));
    let mut vertices = Vec::with_capacity(2 * line_count(size, resolution) as usize);
    for axis in (0..3).filter(|&axis| size[axis] > 0.0) {
        let (b, c) = ((axis + 1) % 3, (axis + 2) % 3);
        let half = size[axis] / 2.0;
        for &across_b in &positions[b] {
            for &across_c in &positions[c] {
                let mut start = [0.0; 3];
                start[b] = across_b;
                start[c] = across_c;
                let mut end = start;
                start[axis] = -half;
                end[axis] = half;
                vertices.extend([start, end]);
            }
        }
    }
    vertices
}

fn assert_valid(description: &GridDescription) {
    let GridDescription {
        size, resolution, ..
    } = description;
    assert!(
        size.iter().all(|size| size.is_finite() && *size >= 0.0),
        "grid size must be finite and at least 0, got {size:?}"
    );
    assert!(
        resolution
            .iter()
            .all(|resolution| resolution.is_finite() && *resolution > 0.0),
        "grid resolution must be positive and finite, got {resolution:?}"
    );
    let lines = line_count(*size, *resolution);
    assert!(
        lines <= MAX_GRID_LINES,
        "grid has {lines} lines, at most {MAX_GRID_LINES} fit"
    );
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct GridInstance {
    transform: RealTwist,
    color: [f32; 3],
}

const GRID_VERTEX_STRIDE: wgpu::BufferAddress =
    std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress;
const GRID_INSTANCE_STRIDE: wgpu::BufferAddress =
    std::mem::size_of::<GridInstance>() as wgpu::BufferAddress;
const INITIAL_GRID_INSTANCE_CAPACITY: usize = 16;

struct Grid {
    description: GridDescription,
    /// `description.color` in linear
    color: [f32; 3],
    /// `None` without lines
    vertex_buffer: Option<wgpu::Buffer>,
    vertex_count: u32,
}

fn linear_color(color: [f32; 3]) -> [f32; 3] {
    color.map(|component| srgb_to_linear(component) as f32)
}

fn grid_vertex_buffer(device: &wgpu::Device, description: &GridDescription) -> Grid {
    let vertices = grid_lines(description.size, description.resolution);
    let vertex_buffer = (!vertices.is_empty()).then(|| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Grid Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        })
    });
    Grid {
        description: description.clone(),
        color: linear_color(description.color),
        vertex_buffer,
        vertex_count: vertices.len() as u32,
    }
}

fn create_grid_instance_buffer(device: &wgpu::Device, capacity: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Grid Instance Buffer"),
        size: capacity as wgpu::BufferAddress * GRID_INSTANCE_STRIDE,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

/// The grids of a renderer and the frames they can be placed in.
pub(crate) struct Grids {
    pipeline: wgpu::RenderPipeline,
    grids: BTreeMap<u64, Grid>,
    frames: HashMap<String, RealTwist>,
    next_id: u64,
    /// Grids `prepare` found drawable, in instance order
    drawn: Vec<u64>,
    instances: Vec<GridInstance>,
    instance_buffer: wgpu::Buffer,
    instance_capacity: usize,
}

impl Grids {
    pub(crate) fn new(device: &wgpu::Device, camera_layout: &wgpu::BindGroupLayout) -> Self {
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Grid Pipeline Layout"),
            bind_group_layouts: &[Some(camera_layout)],
            immediate_size: 0,
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Grid Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/grid.wgsl").into()),
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Grid Render Pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[
                    Some(wgpu::VertexBufferLayout {
                        array_stride: GRID_VERTEX_STRIDE,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0 => Float32x3],
                    }),
                    Some(wgpu::VertexBufferLayout {
                        array_stride: GRID_INSTANCE_STRIDE,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![
                            1 => Float32x3, // linear.x, linear.y, linear.z
                            2 => Float32x4, // rotational.x, rotational.y, rotational.z, rotational.w
                            3 => Float32x3, // color
                        ],
                    }),
                ],
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
                topology: wgpu::PrimitiveTopology::LineList,
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
        Self {
            pipeline,
            grids: BTreeMap::new(),
            frames: HashMap::new(),
            next_id: 0,
            drawn: Vec::new(),
            instances: Vec::new(),
            instance_buffer: create_grid_instance_buffer(device, INITIAL_GRID_INSTANCE_CAPACITY),
            instance_capacity: INITIAL_GRID_INSTANCE_CAPACITY,
        }
    }

    pub(crate) fn create(&mut self, device: &wgpu::Device, description: GridDescription) -> u64 {
        assert_valid(&description);
        let id = self.next_id;
        self.next_id += 1;
        self.grids
            .insert(id, grid_vertex_buffer(device, &description));
        id
    }

    fn grid_mut(&mut self, id: u64) -> &mut Grid {
        self.grids
            .get_mut(&id)
            .unwrap_or_else(|| panic!("grid {id} does not exist"))
    }

    pub(crate) fn update(&mut self, device: &wgpu::Device, id: u64, description: GridDescription) {
        assert_valid(&description);
        let grid = self.grid_mut(id);
        if grid.description.size == description.size
            && grid.description.resolution == description.resolution
        {
            grid.color = linear_color(description.color);
            grid.description = description;
        } else {
            *grid = grid_vertex_buffer(device, &description);
        }
    }

    pub(crate) fn remove(&mut self, id: u64) {
        self.grids
            .remove(&id)
            .unwrap_or_else(|| panic!("grid {id} does not exist"));
    }

    pub(crate) fn contains(&self, id: u64) -> bool {
        self.grids.contains_key(&id)
    }

    pub(crate) fn set_frame(&mut self, frame: &str, transform: RealTwist) {
        match self.frames.get_mut(frame) {
            Some(current) => *current = transform,
            None => {
                self.frames.insert(frame.to_owned(), transform);
            }
        }
    }

    pub(crate) fn remove_frame(&mut self, frame: &str) {
        self.frames.remove(frame);
    }

    /// Poses the grids for `draw`: the ones with lines whose frame is known.
    pub(crate) fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        self.drawn.clear();
        self.instances.clear();
        for (&id, grid) in &self.grids {
            if grid.vertex_buffer.is_none() {
                continue;
            }
            let frame = match &grid.description.frame {
                None => RealTwist::identity(),
                Some(frame) => match self.frames.get(frame) {
                    Some(&transform) => transform,
                    None => continue,
                },
            };
            self.drawn.push(id);
            self.instances.push(GridInstance {
                transform: compose_twists(frame, grid.description.pose),
                color: grid.color,
            });
        }
        if self.instances.len() > self.instance_capacity {
            while self.instance_capacity < self.instances.len() {
                self.instance_capacity *= 2;
            }
            self.instance_buffer = create_grid_instance_buffer(device, self.instance_capacity);
        }
        if !self.instances.is_empty() {
            queue.write_buffer(
                &self.instance_buffer,
                0,
                bytemuck::cast_slice(&self.instances),
            );
        }
    }

    pub(crate) fn draw(
        &self,
        pass: &mut wgpu::RenderPass<'_>,
        camera_bind_group: &wgpu::BindGroup,
    ) {
        if self.drawn.is_empty() {
            return;
        }
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, camera_bind_group, &[]);
        pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
        for (instance, id) in (0..).zip(&self.drawn) {
            let grid = &self.grids[id];
            let vertex_buffer = grid.vertex_buffer.as_ref().expect("prepared with lines");
            pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            pass.draw(0..grid.vertex_count, instance..instance + 1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{grid_lines, line_count, positions};

    fn lines(size: [f32; 3], resolution: [f32; 3]) -> Vec<[[f32; 3]; 2]> {
        let vertices = grid_lines(size, resolution);
        assert_eq!(vertices.len() as u64, 2 * line_count(size, resolution));
        let (lines, rest) = vertices.as_chunks::<2>();
        assert!(rest.is_empty());
        lines.to_vec()
    }

    #[test]
    fn plane_has_lines_along_its_two_axes() {
        let lines = lines([2.0, 0.0, 2.0], [1.0; 3]);
        assert_eq!(lines.len(), 6);
        assert!(lines.iter().flatten().all(|vertex| vertex[1] == 0.0));
        let along_x = lines
            .iter()
            .filter(|[start, end]| start[0] == -1.0 && end[0] == 1.0)
            .map(|[start, end]| {
                assert_eq!(start[2], end[2]);
                start[2]
            })
            .collect::<Vec<_>>();
        assert_eq!(along_x, [-1.0, 0.0, 1.0]);
    }

    #[test]
    fn cube_has_lines_along_every_axis() {
        let lines = lines([2.0; 3], [1.0; 3]);
        assert_eq!(lines.len(), 27);
        for axis in 0..3 {
            let along = lines
                .iter()
                .filter(|[start, end]| start[axis] == -1.0 && end[axis] == 1.0)
                .count();
            assert_eq!(along, 9, "axis {axis}");
        }
    }

    #[test]
    fn lines_go_through_the_origin_and_along_the_boundary() {
        assert_eq!(positions(2.5, 1.0), [-1.25, -1.0, 0.0, 1.0, 1.25]);
        assert_eq!(positions(0.6, 1.0), [-0.3, 0.0, 0.3]);
        assert_eq!(positions(1.0, 1.0), [-0.5, 0.0, 0.5]);
    }

    #[test]
    fn lines_near_the_boundary_are_the_boundary() {
        let positions = positions(3.0, 0.1);
        assert_eq!(positions.len(), 31);
        assert!(
            positions.windows(2).all(|pair| pair[1] - pair[0] > 0.099),
            "{positions:?}"
        );
        assert_eq!(positions.first(), Some(&-1.5));
        assert_eq!(positions.last(), Some(&1.5));
    }

    #[test]
    fn axes_have_their_own_resolution() {
        let lines = lines([4.0, 2.0, 0.0], [2.0, 0.5, 1.0]);
        // Along x one per y position (5), along y one per x position (3)
        assert_eq!(lines.len(), 5 + 3);
    }

    #[test]
    fn grid_without_size_has_no_lines() {
        assert!(lines([0.0; 3], [1.0; 3]).is_empty());
    }

    #[test]
    fn huge_grids_are_counted_without_building_them() {
        assert_eq!(line_count([1e6, 0.0, 1e6], [1.0; 3]), 2 * 1_000_001);
        assert_eq!(line_count([f32::MAX; 3], [f32::MIN_POSITIVE; 3]), u64::MAX);
    }
}
