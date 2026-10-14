const light_direction = vec3<f32>(-0.25, 1.0, -0.25);
const minimum_light_intensity = 0.25;

struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) normal: vec3<f32>,
};

struct InstanceInput {
    @location(3) translation: vec3<f32>,
    @location(4) rotation: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

fn rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

@vertex
fn vs_main(
    model: VertexInput,
    instance: InstanceInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.color = model.color;
    let world_position = rotate(instance.rotation, model.position) + instance.translation;
    out.clip_position = camera.view_proj * vec4<f32>(world_position, 1.0);
    out.normal = rotate(instance.rotation, model.normal);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let smooth_normal = normalize(in.normal);
    var light_intensity = max(dot(smooth_normal, normalize(light_direction)), minimum_light_intensity);
    light_intensity = smoothstep(0.2, 1.0, light_intensity);
    return vec4<f32>(in.color * light_intensity, 1.0);    
}
