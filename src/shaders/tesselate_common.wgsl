const CHUNK_WIDTH: i32 = 16;
const CHUNK_RATIO: u32 = 128u;
const SUBCHUNK_SIZE: vec3<i32> = vec3<i32>(4, 4, 2);
const CHUNK_SUBCHUNKS: vec3<i32> = vec3<i32>(4, 4, 8);
const VERTEX_FLOATS: u32 = 9u;
const EMPTY_SLOT: u32 = 0u;

struct Params {
    voxel_size: f32,
    chunk_count: u32,
    vertex_capacity: u32,
    padding: u32,
}

struct DirtyChunk {
    origin: vec4<i32>,
    neighbours: array<u32, 27>,
    padding: u32,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> chunk_masks: array<u32>;
@group(0) @binding(2) var<storage, read> dirty: array<DirtyChunk>;
@group(0) @binding(3) var<storage, read_write> vertices: array<f32>;
@group(0) @binding(4) var<storage, read_write> counts: array<atomic<u32>>;

fn voxel(chunk: u32, local: vec3<i32>) -> bool {
    if any(local < vec3<i32>(-CHUNK_WIDTH)) || any(local >= vec3<i32>(2 * CHUNK_WIDTH)) {
        return false;
    }
    let offset = (local + vec3<i32>(CHUNK_WIDTH)) / CHUNK_WIDTH - vec3<i32>(1);
    let slot = dirty[chunk].neighbours[(offset.z + 1) * 9 + (offset.y + 1) * 3 + (offset.x + 1)];
    if slot == EMPTY_SLOT {
        return false;
    }
    let in_chunk = local - offset * CHUNK_WIDTH;
    let subchunk = in_chunk / SUBCHUNK_SIZE;
    let in_subchunk = in_chunk % SUBCHUNK_SIZE;
    let subchunk_index = u32((subchunk.z * CHUNK_SUBCHUNKS.y + subchunk.y) * CHUNK_SUBCHUNKS.x + subchunk.x);
    let bit = u32(in_subchunk.x | (in_subchunk.y << 2u) | (in_subchunk.z << 4u));
    return ((chunk_masks[slot * CHUNK_RATIO + subchunk_index] >> bit) & 1u) != 0u;
}

fn reserve(chunk: u32, n: u32) -> i32 {
    let first = atomicAdd(&counts[chunk], n);
    if first + n > params.vertex_capacity {
        return -1;
    }
    return i32(first);
}

fn write_vertex(chunk: u32, vertex: u32, position: vec3<f32>, color: vec3<f32>, normal: vec3<f32>) {
    let base = (chunk * params.vertex_capacity + vertex) * VERTEX_FLOATS;
    vertices[base + 0u] = position.x;
    vertices[base + 1u] = position.y;
    vertices[base + 2u] = position.z;
    vertices[base + 3u] = color.x;
    vertices[base + 4u] = color.y;
    vertices[base + 5u] = color.z;
    vertices[base + 6u] = normal.x;
    vertices[base + 7u] = normal.y;
    vertices[base + 8u] = normal.z;
}
