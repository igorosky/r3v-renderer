const light_direction = vec3<f32>(-0.25, 1.0, -0.25);
const minimum_light_intensity = 0.25;

struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var asset_texture: texture_2d<f32>;
@group(1) @binding(1)
var asset_sampler: sampler;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct InstanceInput {
    @location(2) translation: vec3<f32>,
    @location(3) rotation: vec4<f32>,
    @location(4) color: vec3<f32>,
    @location(5) bounds_min: vec3<f32>,
    @location(6) bounds_max: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

fn rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

fn box_uv(position: vec3<f32>, normal: vec3<f32>, bounds_min: vec3<f32>, bounds_max: vec3<f32>) -> vec2<f32> {
    let p = (position - bounds_min) / max(bounds_max - bounds_min, vec3<f32>(1e-6));
    let n = abs(normal);
    var uv = p.xy;
    if n.x >= n.y && n.x >= n.z {
        uv = p.yz;
    } else if n.y >= n.z {
        uv = p.xz;
    }
    return vec2<f32>(uv.x, 1.0 - uv.y);
}

@vertex
fn vs_main(
    model: VertexInput,
    instance: InstanceInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.color = instance.color;
    let world_position = rotate(instance.rotation, model.position) + instance.translation;
    out.clip_position = camera.view_proj * vec4<f32>(world_position, 1.0);
    out.normal = rotate(instance.rotation, model.normal);
    out.uv = box_uv(model.position, model.normal, instance.bounds_min, instance.bounds_max);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let smooth_normal = normalize(in.normal);
    var light_intensity = max(dot(smooth_normal, normalize(light_direction)), minimum_light_intensity);
    light_intensity = smoothstep(0.2, 1.0, light_intensity);
    let texel = textureSample(asset_texture, asset_sampler, in.uv).rgb;
    return vec4<f32>(in.color * texel * light_intensity, 1.0);
}
