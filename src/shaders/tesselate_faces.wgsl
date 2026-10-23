
@compute @workgroup_size(4, 4, 4)
fn cs_main(@builtin(global_invocation_id) id: vec3<u32>) {
    let chunk = id.z / u32(CHUNK_WIDTH);
    if chunk >= params.chunk_count {
        return;
    }
    let local = vec3<i32>(vec3<u32>(id.x, id.y, id.z % u32(CHUNK_WIDTH)));
    let origin = dirty[chunk].origin.xyz;

    // TODO: emit the exposed faces of voxel `local`
    _ = local;
    _ = origin;
}
