
struct MarchingCubesTables {
    cube_corners: array<vec4<u32>, 8>,
    cube_edges: array<vec2<u32>, 12>,
    edge_masks: array<u32, 256>,
    triangle_edges: array<i32, 4096>,
}

@group(1) @binding(0) var<storage, read> tables: MarchingCubesTables;

@compute @workgroup_size(4, 4, 4)
fn cs_main(@builtin(global_invocation_id) id: vec3<u32>) {
    let chunk = id.z / u32(CHUNK_WIDTH);
    if chunk >= params.chunk_count {
        return;
    }
    let local = vec3<i32>(vec3<u32>(id.x, id.y, id.z % u32(CHUNK_WIDTH)));
    let origin = dirty[chunk].origin.xyz;

    // TODO: polygonize the cell with corners `local` .. `local + 1`
    _ = local;
    _ = origin;
    _ = tables.edge_masks[0];
}
