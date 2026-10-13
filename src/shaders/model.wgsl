struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
};

struct InstanceInput {
    @location(2) translation: vec3<f32>,
    @location(3) rotation: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) barycentric: vec3<f32>,
};

fn rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

@vertex
fn vs_main(
    model: VertexInput,
    instance: InstanceInput,
    @builtin(vertex_index) v_idx: u32,
) -> VertexOutput {
    var out: VertexOutput;
    out.color = model.color;
    let world_position = rotate(instance.rotation, model.position) + instance.translation;
    out.clip_position = camera.view_proj * vec4<f32>(world_position, 1.0);
    let corner = v_idx % 3u;
    if corner == 0u {
        out.barycentric = vec3<f32>(1.0, 0.0, 0.0);
    } else if corner == 1u {
        out.barycentric = vec3<f32>(0.0, 1.0, 0.0);
    } else {
        out.barycentric = vec3<f32>(0.0, 0.0, 1.0);
    }
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let deltas = fwidth(in.barycentric);
    let line_width = 1.0;
    let smoothing = 1.0;
    let bary_pixels = in.barycentric / max(deltas, vec3<f32>(0.0001));
    let min_dist = min(min(bary_pixels.x, bary_pixels.y), bary_pixels.z);
    let edge_factor = smoothstep(line_width - smoothing, line_width + smoothing, min_dist);
    let wire_color = vec3<f32>(0.0, 0.0, 0.0);
    let surface_color = in.color;
    let final_rgb = mix(wire_color, surface_color, edge_factor);
    return vec4<f32>(final_rgb, 1.0);
}
