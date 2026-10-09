use rustc_hash::FxHashMap;
use wgpu::util::DrawIndirectArgs;

use crate::geometry::{DiscretePoint, Vertex};

const VERTEX_STRIDE: wgpu::BufferAddress = std::mem::size_of::<Vertex>() as wgpu::BufferAddress;
const DRAW_ARGS_STRIDE: wgpu::BufferAddress = std::mem::size_of::<DrawIndirectArgs>() as wgpu::BufferAddress;

const MIN_CLASS_SHIFT: u32 = 6;
const INITIAL_VERTEX_CAPACITY: u32 = 1 << 16;
const INITIAL_DRAW_CAPACITY: usize = 64;

struct ChunkAlloc {
    offset: u32,
    capacity: u32,
    count: u32,
    draw_slot: usize,
}

pub(crate) struct VertexArena {
    vertex_buffer: wgpu::Buffer,
    vertex_capacity: u32,
    high_water: u32,
    free_lists: Vec<Vec<u32>>,
    free_slots: u32,

    chunks: FxHashMap<DiscretePoint, ChunkAlloc>,

    draws: Vec<DrawIndirectArgs>,
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

fn create_vertex_buffer(device: &wgpu::Device, capacity: u32) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Vertex Arena Buffer"),
        size: capacity as wgpu::BufferAddress * VERTEX_STRIDE,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    })
}

fn create_draw_buffer(device: &wgpu::Device, capacity: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Vertex Arena Draw Buffer"),
        size: capacity as wgpu::BufferAddress * DRAW_ARGS_STRIDE,
        usage: wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
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
            draw_buffer: create_draw_buffer(device, INITIAL_DRAW_CAPACITY),
            draw_capacity: INITIAL_DRAW_CAPACITY,
            draws_dirty: false,
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

    pub fn set_chunk(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, index: DiscretePoint, vertices: &[Vertex]) {
        if vertices.is_empty() {
            self.remove_chunk(index);
            return;
        }
        let count = vertices.len() as u32;
        if let Some(alloc) = self.chunks.get_mut(&index) && count <= alloc.capacity {
            alloc.count = count;
            queue.write_buffer(&self.vertex_buffer, alloc.offset as wgpu::BufferAddress * VERTEX_STRIDE, bytemuck::cast_slice(vertices));
            self.draws[alloc.draw_slot].vertex_count = count;
            self.draws_dirty = true;
            return;
        }
        if let Some(alloc) = self.chunks.get(&index) {
            self.release(alloc.offset, alloc.capacity);
        }
        let capacity = size_class(count);
        let offset = self.allocate(device, queue, capacity);
        queue.write_buffer(&self.vertex_buffer, offset as wgpu::BufferAddress * VERTEX_STRIDE, bytemuck::cast_slice(vertices));

        let args = DrawIndirectArgs { vertex_count: count, instance_count: 1, first_vertex: offset, first_instance: 0 };
        match self.chunks.get_mut(&index) {
            Some(alloc) => {
                alloc.offset = offset;
                alloc.capacity = capacity;
                alloc.count = count;
                self.draws[alloc.draw_slot] = args;
            }
            None => {
                self.chunks.insert(index, ChunkAlloc { offset, capacity, count, draw_slot: self.draws.len() });
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
            self.chunks.get_mut(moved).expect("Draw owner without allocation").draw_slot = alloc.draw_slot;
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
        if let Some(offset) = self.free_lists.get_mut(class_index(capacity)).and_then(Vec::pop) {
            self.free_slots -= capacity;
            return offset;
        }
        if self.high_water + capacity > self.vertex_capacity {
            self.grow(device, queue, self.high_water + capacity);
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

    fn grow(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, required: u32) {
        let mut new_capacity = self.vertex_capacity;
        while new_capacity < required {
            new_capacity *= 2;
        }
        let new_buffer = create_vertex_buffer(device, new_capacity);
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Vertex Arena Grow Encoder"),
        });
        if self.high_water > 0 {
            encoder.copy_buffer_to_buffer(&self.vertex_buffer, 0, &new_buffer, 0, self.high_water as wgpu::BufferAddress * VERTEX_STRIDE);
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
