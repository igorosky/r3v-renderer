use rustc_hash::FxHashMap;
use wgpu::util::DrawIndexedIndirectArgs;

use crate::geometry::{DiscretePoint, Vertex};
use crate::wgpu_utils::*;

const MIN_CLASS_SHIFT: u32 = 6;
const INITIAL_VERTEX_CAPACITY: usize = 1 << 16;
const INITIAL_DRAW_CAPACITY: usize = 64;
const INITIAL_QUAD_CAPACITY: usize = 1 << 12;

struct ChunkAlloc {
    offset: u32,
    capacity: u32,
    count: u32,
    draw_slot: usize,
}

pub(crate) struct VertexArena {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    quad_capacity: usize,
    vertex_capacity: usize,
    high_water: u32,
    free_lists: Vec<Vec<u32>>,
    free_slots: u32,

    chunks: FxHashMap<DiscretePoint, ChunkAlloc>,

    draws: Vec<DrawIndexedIndirectArgs>,
    draw_owners: Vec<DiscretePoint>,
    draw_buffer: wgpu::Buffer,
    draw_capacity: usize,
    draws_dirty: bool,
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
            index_buffer: create_quad_index_buffer(device, INITIAL_QUAD_CAPACITY),
            quad_capacity: INITIAL_QUAD_CAPACITY,
            high_water: 0,
            free_lists: Vec::new(),
            free_slots: 0,
            chunks: FxHashMap::default(),
            draws: Vec::new(),
            draw_owners: Vec::new(),
            draw_buffer: create_draw_buffer(device, INITIAL_DRAW_CAPACITY),
            draw_capacity: INITIAL_DRAW_CAPACITY,
            draws_dirty: false,
        }
    }

    pub fn vertex_buffer(&self) -> &wgpu::Buffer {
        &self.vertex_buffer
    }

    pub fn index_buffer(&self) -> &wgpu::Buffer {
        &self.index_buffer
    }

    pub fn draw_buffer(&self) -> &wgpu::Buffer {
        &self.draw_buffer
    }

    pub fn draw_count(&self) -> u32 {
        self.draws.len() as u32
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
        debug_assert!(vertices.len().is_multiple_of(QUAD_VERTICES as usize));
        let count = vertices.len() as u32;
        let quads = (count / QUAD_VERTICES) as usize;
        if quads > self.quad_capacity {
            self.quad_capacity = quads.next_power_of_two();
            self.index_buffer = create_quad_index_buffer(device, self.quad_capacity);
        }
        if let Some(alloc) = self.chunks.get_mut(&index)
            && count <= alloc.capacity
        {
            alloc.count = count;
            queue.write_buffer(
                &self.vertex_buffer,
                alloc.offset as wgpu::BufferAddress * VERTEX_STRIDE,
                bytemuck::cast_slice(vertices),
            );
            self.draws[alloc.draw_slot].index_count = count / QUAD_VERTICES * QUAD_INDICES;
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

        let args = DrawIndexedIndirectArgs {
            index_count: count / QUAD_VERTICES * QUAD_INDICES,
            instance_count: 1,
            first_index: 0,
            base_vertex: offset as i32,
            first_instance: 0,
        };
        match self.chunks.get_mut(&index) {
            Some(alloc) => {
                alloc.offset = offset;
                alloc.capacity = capacity;
                alloc.count = count;
                self.draws[alloc.draw_slot] = args;
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
            self.draws[alloc.draw_slot].base_vertex = high_water as i32;
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

    /// `quads` quads of vertices tagged with the chunk and a version, so every upload is
    /// distinguishable in the vertex buffer.
    fn vertices(chunk: i32, version: u32, quads: usize) -> Vec<Vertex> {
        (0..quads * QUAD_VERTICES as usize)
            .map(|i| Vertex {
                position: [chunk as f32, version as f32, i as f32],
                color: [1.0, 0.5, 0.25],
                normal: [0.0, 1.0, 0.0],
            })
            .collect()
    }

    /// Arena plus the content it is expected to draw.
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

        fn set(&mut self, chunk: i32, version: u32, quads: usize) {
            let data = vertices(chunk, version, quads);
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
            let draws: &[DrawIndexedIndirectArgs] =
                bytemuck::cast_slice(&draw_bytes[..draw_count * DRAW_ARGS_STRIDE as usize]);
            let index_bytes = self.gpu.read_buffer(self.arena.index_buffer());
            let indices: &[u32] = bytemuck::cast_slice(&index_bytes);
            let vertex_bytes = self.gpu.read_buffer(self.arena.vertex_buffer());
            let gpu_vertices: &[Vertex] = bytemuck::cast_slice(&vertex_bytes);

            let mut used_vertices = HashSet::new();
            let mut matched = HashSet::new();
            for draw in draws {
                assert_eq!(draw.instance_count, 1);
                let first = draw.first_index as usize;
                let count = draw.index_count as usize;
                assert!(
                    first + count <= indices.len(),
                    "draw reads past index buffer"
                );
                let mut referenced: Vec<usize> = indices[first..first + count]
                    .iter()
                    .map(|&i| (draw.base_vertex as i64 + i as i64) as usize)
                    .collect();
                referenced.sort();
                referenced.dedup();
                assert!(*referenced.last().unwrap() < gpu_vertices.len());
                for &vertex in &referenced {
                    assert!(used_vertices.insert(vertex), "vertex {vertex} drawn twice");
                }
                let drawn: Vec<Vertex> = referenced.iter().map(|&i| gpu_vertices[i]).collect();
                let owner = self
                    .expected
                    .iter()
                    .find(|(_, expected)| {
                        bytemuck::cast_slice::<Vertex, u8>(expected)
                            == bytemuck::cast_slice::<Vertex, u8>(&drawn)
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
        for (version, quads) in [(1, 4), (2, 4), (3, 12), (4, 500), (5, 1)] {
            harness.set(0, version, quads);
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
        let quads = INITIAL_VERTEX_CAPACITY / QUAD_VERTICES as usize / 3;
        for chunk in 0..5 {
            harness.set(chunk, 0, quads);
            harness.finish_and_check();
        }
    }

    #[test]
    fn chunk_larger_than_initial_index_capacity() {
        let mut harness = Harness::new();
        harness.set(0, 0, 2);
        harness.set(1, 0, INITIAL_QUAD_CAPACITY + 1);
        harness.finish_and_check();
        harness.set(2, 0, INITIAL_QUAD_CAPACITY * 3);
        harness.finish_and_check();
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
}
