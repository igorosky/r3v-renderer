use rustc_hash::FxHashMap;
use wgpu::util::DrawIndirectArgs;

use crate::camera::Frustum;
use crate::geometry::{DiscretePoint, Vertex};
use crate::wgpu_utils::*;

const MIN_CLASS_SHIFT: u32 = 6;
const INITIAL_VERTEX_CAPACITY: usize = 1 << 16;
const INITIAL_DRAW_CAPACITY: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
struct Bounds {
    min: [f32; 3],
    max: [f32; 3],
}

impl Bounds {
    fn of(vertices: &[Vertex]) -> Self {
        let mut bounds = Self {
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
        };
        for vertex in vertices {
            for axis in 0..3 {
                bounds.min[axis] = bounds.min[axis].min(vertex.position[axis]);
                bounds.max[axis] = bounds.max[axis].max(vertex.position[axis]);
            }
        }
        bounds
    }
}

struct ChunkAlloc {
    offset: u32,
    capacity: u32,
    count: u32,
    draw_slot: usize,
}

pub(crate) struct VertexArena {
    vertex_buffer: wgpu::Buffer,
    vertex_capacity: usize,
    high_water: u32,
    free_lists: Vec<Vec<u32>>,
    free_slots: u32,

    chunks: FxHashMap<DiscretePoint, ChunkAlloc>,

    draws: Vec<DrawIndirectArgs>,
    draw_owners: Vec<DiscretePoint>,
    draw_bounds: Vec<Bounds>,
    draw_buffer: wgpu::Buffer,
    draw_capacity: usize,
    draws_dirty: bool,

    visible_draws: Vec<DrawIndirectArgs>,
    visible_buffer: wgpu::Buffer,
    visible_capacity: usize,
}

fn size_class(count: u32) -> u32 {
    count.max(1 << MIN_CLASS_SHIFT).next_power_of_two()
}

fn class_index(capacity: u32) -> usize {
    (capacity.trailing_zeros() - MIN_CLASS_SHIFT) as usize
}

impl VertexArena {
    pub fn new(device: &wgpu::Device) -> Self {
        Self {
            vertex_buffer: create_vertex_buffer(device, INITIAL_VERTEX_CAPACITY),
            vertex_capacity: INITIAL_VERTEX_CAPACITY,
            high_water: 0,
            free_lists: Vec::new(),
            free_slots: 0,
            chunks: FxHashMap::default(),
            draws: Vec::new(),
            draw_owners: Vec::new(),
            draw_bounds: Vec::new(),
            draw_buffer: create_draw_buffer(device, INITIAL_DRAW_CAPACITY),
            draw_capacity: INITIAL_DRAW_CAPACITY,
            draws_dirty: false,
            visible_draws: Vec::new(),
            visible_buffer: create_draw_buffer(device, INITIAL_DRAW_CAPACITY),
            visible_capacity: INITIAL_DRAW_CAPACITY,
        }
    }

    pub fn vertex_buffer(&self) -> &wgpu::Buffer {
        &self.vertex_buffer
    }

    pub fn draw_buffer(&self) -> &wgpu::Buffer {
        &self.draw_buffer
    }

    pub fn draw_count(&self) -> u32 {
        self.draws.len() as u32
    }

    pub fn visible_buffer(&self) -> &wgpu::Buffer {
        &self.visible_buffer
    }

    pub fn cull(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, frustum: &Frustum) -> u32 {
        self.visible_draws.clear();
        self.visible_draws.extend(
            self.draws
                .iter()
                .zip(&self.draw_bounds)
                .filter(|(_, bounds)| frustum.intersects_box(bounds.min, bounds.max))
                .map(|(draw, _)| *draw),
        );
        if self.visible_draws.len() > self.visible_capacity {
            self.visible_capacity = self.visible_draws.len().next_power_of_two();
            self.visible_buffer = create_draw_buffer(device, self.visible_capacity);
        }
        queue.write_buffer(
            &self.visible_buffer,
            0,
            bytemuck::cast_slice(&self.visible_draws),
        );
        self.visible_draws.len() as u32
    }

    pub fn set_chunk(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        index: DiscretePoint,
        vertices: &[Vertex],
    ) {
        if vertices.is_empty() {
            self.remove_chunk(index);
            return;
        }
        let count = vertices.len() as u32;
        let bounds = Bounds::of(vertices);
        if let Some(alloc) = self.chunks.get_mut(&index)
            && count <= alloc.capacity
        {
            alloc.count = count;
            queue.write_buffer(
                &self.vertex_buffer,
                alloc.offset as wgpu::BufferAddress * VERTEX_STRIDE,
                bytemuck::cast_slice(vertices),
            );
            self.draws[alloc.draw_slot].vertex_count = count;
            self.draw_bounds[alloc.draw_slot] = bounds;
            self.draws_dirty = true;
            return;
        }
        if let Some(alloc) = self.chunks.get(&index) {
            self.release(alloc.offset, alloc.capacity);
        }
        let capacity = size_class(count);
        let offset = self.allocate(device, queue, capacity);
        queue.write_buffer(
            &self.vertex_buffer,
            offset as wgpu::BufferAddress * VERTEX_STRIDE,
            bytemuck::cast_slice(vertices),
        );

        let args = DrawIndirectArgs {
            vertex_count: count,
            instance_count: 1,
            first_vertex: offset,
            first_instance: 0,
        };
        match self.chunks.get_mut(&index) {
            Some(alloc) => {
                alloc.offset = offset;
                alloc.capacity = capacity;
                alloc.count = count;
                self.draws[alloc.draw_slot] = args;
                self.draw_bounds[alloc.draw_slot] = bounds;
            }
            None => {
                self.chunks.insert(
                    index,
                    ChunkAlloc {
                        offset,
                        capacity,
                        count,
                        draw_slot: self.draws.len(),
                    },
                );
                self.draws.push(args);
                self.draw_owners.push(index);
                self.draw_bounds.push(bounds);
            }
        }
        self.draws_dirty = true;
    }

    pub fn remove_chunk(&mut self, index: DiscretePoint) {
        let Some(alloc) = self.chunks.remove(&index) else {
            return;
        };
        self.release(alloc.offset, alloc.capacity);
        self.draws.swap_remove(alloc.draw_slot);
        self.draw_owners.swap_remove(alloc.draw_slot);
        self.draw_bounds.swap_remove(alloc.draw_slot);
        if let Some(moved) = self.draw_owners.get(alloc.draw_slot) {
            self.chunks
                .get_mut(moved)
                .expect("Draw owner without allocation")
                .draw_slot = alloc.draw_slot;
        }
        self.draws_dirty = true;
    }

    pub fn finish_update(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        if self.free_slots > self.high_water / 2 {
            self.compact(device, queue);
        }
        if !self.draws_dirty {
            return;
        }
        if self.draws.len() > self.draw_capacity {
            self.draw_capacity = self.draws.len().next_power_of_two();
            self.draw_buffer = create_draw_buffer(device, self.draw_capacity);
        }
        queue.write_buffer(&self.draw_buffer, 0, bytemuck::cast_slice(&self.draws));
        self.draws_dirty = false;
    }

    fn allocate(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, capacity: u32) -> u32 {
        if let Some(offset) = self
            .free_lists
            .get_mut(class_index(capacity))
            .and_then(Vec::pop)
        {
            self.free_slots -= capacity;
            return offset;
        }
        if (self.high_water + capacity) as usize > self.vertex_capacity {
            self.grow(device, queue, (self.high_water + capacity) as usize);
        }
        let offset = self.high_water;
        self.high_water += capacity;
        offset
    }

    fn release(&mut self, offset: u32, capacity: u32) {
        let class = class_index(capacity);
        if self.free_lists.len() <= class {
            self.free_lists.resize_with(class + 1, Vec::new);
        }
        self.free_lists[class].push(offset);
        self.free_slots += capacity;
    }

    fn grow(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, required: usize) {
        let mut new_capacity = self.vertex_capacity;
        while new_capacity < required {
            new_capacity *= 2;
        }
        let new_buffer = create_vertex_buffer(device, new_capacity);
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Vertex Arena Grow Encoder"),
        });
        if self.high_water > 0 {
            encoder.copy_buffer_to_buffer(
                &self.vertex_buffer,
                0,
                &new_buffer,
                0,
                self.high_water as wgpu::BufferAddress * VERTEX_STRIDE,
            );
        }
        queue.submit(Some(encoder.finish()));
        self.vertex_buffer = new_buffer;
        self.vertex_capacity = new_capacity;
    }

    fn compact(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        let new_buffer = create_vertex_buffer(device, self.vertex_capacity);
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Vertex Arena Compact Encoder"),
        });
        let mut high_water = 0;
        for alloc in self.chunks.values_mut() {
            encoder.copy_buffer_to_buffer(
                &self.vertex_buffer,
                alloc.offset as wgpu::BufferAddress * VERTEX_STRIDE,
                &new_buffer,
                high_water as wgpu::BufferAddress * VERTEX_STRIDE,
                alloc.count as wgpu::BufferAddress * VERTEX_STRIDE,
            );
            alloc.offset = high_water;
            alloc.capacity = size_class(alloc.count);
            self.draws[alloc.draw_slot].first_vertex = high_water;
            high_water += alloc.capacity;
        }
        queue.submit(Some(encoder.finish()));

        self.vertex_buffer = new_buffer;
        self.high_water = high_water;
        self.free_lists.clear();
        self.free_slots = 0;
        self.draws_dirty = true;
    }
}

#[cfg(test)]
mod tests {
    use std::collections::{HashMap, HashSet};

    use super::*;
    use crate::wgpu_utils::test_support::TestGpu;

    fn index(i: i32) -> DiscretePoint {
        DiscretePoint {
            x: i * 16,
            y: -i * 16,
            z: 0,
        }
    }

    fn vertices(chunk: i32, version: u32, triangles: usize) -> Vec<Vertex> {
        (0..triangles * 3)
            .map(|i| Vertex {
                position: [chunk as f32, version as f32, i as f32],
                color: [1.0, 0.5, 0.25],
                normal: [0.0, 1.0, 0.0],
            })
            .collect()
    }

    struct Harness {
        gpu: TestGpu,
        arena: VertexArena,
        expected: HashMap<DiscretePoint, Vec<Vertex>>,
    }

    impl Harness {
        fn new() -> Self {
            let gpu = TestGpu::new();
            let arena = VertexArena::new(&gpu.device);
            Self {
                gpu,
                arena,
                expected: HashMap::new(),
            }
        }

        fn set(&mut self, chunk: i32, version: u32, triangles: usize) {
            let data = vertices(chunk, version, triangles);
            self.arena
                .set_chunk(&self.gpu.device, &self.gpu.queue, index(chunk), &data);
            if data.is_empty() {
                self.expected.remove(&index(chunk));
            } else {
                self.expected.insert(index(chunk), data);
            }
        }

        fn remove(&mut self, chunk: i32) {
            self.arena.remove_chunk(index(chunk));
            self.expected.remove(&index(chunk));
        }

        /// Finishes the update and checks that the indirect draws read from the GPU draw
        /// exactly the expected chunks: one draw per chunk, covering exactly its vertices,
        /// with no two draws sharing vertices.
        #[track_caller]
        fn finish_and_check(&mut self) {
            self.arena.finish_update(&self.gpu.device, &self.gpu.queue);
            let draw_count = self.arena.draw_count() as usize;
            assert_eq!(draw_count, self.expected.len());

            let draw_bytes = self.gpu.read_buffer(self.arena.draw_buffer());
            let draws: &[DrawIndirectArgs] =
                bytemuck::cast_slice(&draw_bytes[..draw_count * DRAW_ARGS_STRIDE as usize]);
            let vertex_bytes = self.gpu.read_buffer(self.arena.vertex_buffer());
            let gpu_vertices: &[Vertex] = bytemuck::cast_slice(&vertex_bytes);

            let mut used_vertices = HashSet::new();
            let mut matched = HashSet::new();
            for draw in draws {
                assert_eq!(draw.instance_count, 1);
                let first = draw.first_vertex as usize;
                let count = draw.vertex_count as usize;
                assert!(
                    first + count <= gpu_vertices.len(),
                    "draw reads past vertex buffer"
                );
                for vertex in first..first + count {
                    assert!(used_vertices.insert(vertex), "vertex {vertex} drawn twice");
                }
                let drawn = &gpu_vertices[first..first + count];
                let owner = self
                    .expected
                    .iter()
                    .find(|(_, expected)| {
                        bytemuck::cast_slice::<Vertex, u8>(expected)
                            == bytemuck::cast_slice::<Vertex, u8>(drawn)
                    })
                    .map(|(index, _)| *index)
                    .unwrap_or_else(|| panic!("draw {draw:?} does not match any chunk"));
                assert!(matched.insert(owner), "chunk {owner:?} drawn twice");
            }
        }
    }

    #[test]
    fn new_arena_draws_nothing() {
        let mut harness = Harness::new();
        harness.finish_and_check();
        assert_eq!(harness.arena.draw_count(), 0);
    }

    #[test]
    fn chunks_are_drawn() {
        let mut harness = Harness::new();
        harness.set(0, 0, 1);
        harness.finish_and_check();
        harness.set(1, 0, 3);
        harness.set(2, 0, 64);
        harness.set(3, 0, 65);
        harness.finish_and_check();
    }

    #[test]
    fn updating_a_chunk_replaces_its_vertices() {
        let mut harness = Harness::new();
        harness.set(0, 0, 10);
        harness.set(1, 0, 10);
        harness.finish_and_check();
        // Shrink, same size, grow a little and grow a lot.
        for (version, triangles) in [(1, 4), (2, 4), (3, 12), (4, 500), (5, 1)] {
            harness.set(0, version, triangles);
            harness.finish_and_check();
        }
    }

    #[test]
    fn several_updates_before_finish() {
        let mut harness = Harness::new();
        harness.set(0, 0, 10);
        harness.set(0, 1, 100);
        harness.set(1, 0, 5);
        harness.remove(1);
        harness.set(1, 1, 7);
        harness.finish_and_check();
    }

    #[test]
    fn removing_chunks_in_any_position() {
        let mut harness = Harness::new();
        for chunk in 0..6 {
            harness.set(chunk, 0, chunk as usize + 1);
        }
        harness.finish_and_check();
        for chunk in [0, 5, 2, 3, 1, 4] {
            harness.remove(chunk);
            harness.finish_and_check();
        }
    }

    #[test]
    fn empty_vertices_remove_the_chunk() {
        let mut harness = Harness::new();
        harness.set(0, 0, 3);
        harness.set(1, 0, 3);
        harness.finish_and_check();
        harness.set(0, 1, 0);
        harness.finish_and_check();
        // Removing again, or removing something never added, is a no-op.
        harness.set(0, 2, 0);
        harness.remove(0);
        harness.remove(42);
        harness.finish_and_check();
        // A removed chunk can come back.
        harness.set(0, 3, 2);
        harness.finish_and_check();
    }

    #[test]
    fn space_is_reused_and_compacted_after_removals() {
        let mut harness = Harness::new();
        for chunk in 0..40 {
            harness.set(chunk, 0, 50);
        }
        harness.finish_and_check();
        for chunk in (0..40).filter(|c| c % 4 != 0) {
            harness.remove(chunk);
        }
        harness.finish_and_check();
        for chunk in 40..60 {
            harness.set(chunk, 0, 30);
        }
        harness.finish_and_check();
    }

    #[test]
    fn more_chunks_than_initial_draw_capacity() {
        let mut harness = Harness::new();
        for chunk in 0..(INITIAL_DRAW_CAPACITY as i32 * 3 + 5) {
            harness.set(chunk, 0, 1);
        }
        harness.finish_and_check();
    }

    #[test]
    fn more_vertices_than_initial_capacity() {
        let mut harness = Harness::new();
        let triangles = INITIAL_VERTEX_CAPACITY / 3 / 3;
        for chunk in 0..5 {
            harness.set(chunk, 0, triangles);
            harness.finish_and_check();
        }
    }

    #[test]
    fn randomized_operations() {
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut next = move |bound: u64| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state % bound
        };
        let mut harness = Harness::new();
        for round in 0..30 {
            for _ in 0..next(20) + 1 {
                let chunk = next(40) as i32;
                match next(5) {
                    0 => harness.remove(chunk),
                    1 => harness.set(chunk, round, 0),
                    _ => harness.set(chunk, round, next(300) as usize + 1),
                }
            }
            harness.finish_and_check();
        }
    }

    fn box_vertices(min: [f32; 3], max: [f32; 3], tag: f32) -> Vec<Vertex> {
        (0..24)
            .map(|i| {
                let corner = i / 3;
                let pick = |axis: usize| {
                    if corner >> axis & 1 == 0 {
                        min[axis]
                    } else {
                        max[axis]
                    }
                };
                Vertex {
                    position: [pick(0), pick(1), pick(2)],
                    color: [tag; 3],
                    normal: [0.0, 1.0, 0.0],
                }
            })
            .collect()
    }

    #[test]
    fn culling_draws_only_chunks_in_view() {
        use crate::{Camera, camera::OrbitCamera};

        let gpu = TestGpu::new();
        let mut arena = VertexArena::new(&gpu.device);
        let frustum = OrbitCamera::default().get_state(1.0).frustum();
        let cull = |arena: &mut VertexArena| {
            arena.finish_update(&gpu.device, &gpu.queue);
            let count = arena.cull(&gpu.device, &gpu.queue, &frustum) as usize;
            let draws = gpu.read_buffer(arena.visible_buffer());
            let vertices = gpu.read_buffer(arena.vertex_buffer());
            let vertices: &[Vertex] = bytemuck::cast_slice(&vertices);
            let mut tags: Vec<f32> = bytemuck::cast_slice::<u8, DrawIndirectArgs>(
                &draws[..count * DRAW_ARGS_STRIDE as usize],
            )
            .iter()
            .map(|draw| {
                assert_eq!(draw.vertex_count, 24);
                vertices[draw.first_vertex as usize].color[0]
            })
            .collect();
            tags.sort_by(f32::total_cmp);
            tags
        };

        let in_view = box_vertices([-1.0; 3], [1.0; 3], 1.0);
        let set = |arena: &mut VertexArena, chunk, vertices: &[Vertex]| {
            arena.set_chunk(&gpu.device, &gpu.queue, index(chunk), vertices);
        };
        set(&mut arena, 1, &in_view);
        set(&mut arena, 2, &box_vertices([50.0; 3], [60.0; 3], 2.0));
        set(
            &mut arena,
            3,
            &box_vertices([-1.0, -1.0, 8.0], [1.0, 1.0, 9.0], 3.0),
        );
        set(&mut arena, 4, &box_vertices([-0.5; 3], [0.5; 3], 4.0));
        assert_eq!(cull(&mut arena), [1.0, 4.0]);

        set(&mut arena, 2, &box_vertices([0.0; 3], [0.5; 3], 2.0));
        assert_eq!(cull(&mut arena), [1.0, 2.0, 4.0]);
        arena.remove_chunk(index(1));
        assert_eq!(cull(&mut arena), [2.0, 4.0]);
        set(
            &mut arena,
            3,
            &box_vertices([-0.2, -0.2, -3.0], [0.2, 0.2, -2.0], 3.0),
        );
        assert_eq!(cull(&mut arena), [2.0, 3.0, 4.0]);
        assert_eq!(arena.draw_count(), 3);
    }
}
