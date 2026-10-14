const light_direction = vec3<f32>(0.25, 1.0, 0.25);
const color_depth_range = 50.0;
const minimum_light_intensity = 0.125;

struct CameraUniform {
    view_proj: mat4x4<f32>,
    position: vec3<f32>,
    forward: vec3<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var colormap_texture: texture_1d<f32>;
@group(1) @binding(1)
var colormap_sampler: sampler;

fn turbo_colormap(x: f32) -> vec3<f32> {
    let size = f32(textureDimensions(colormap_texture));
    let u = (clamp(x, 0.0, 1.0) * (size - 1.0) + 0.5) / size;
    return textureSample(colormap_texture, colormap_sampler, u).rgb;
}

fn distance_from_camera(world_position: vec3<f32>) -> f32 {
    return dot(world_position - camera.position, camera.forward) / color_depth_range;
}

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) distance_from_camera: f32,
};

@vertex
fn vs_main(
    model: VertexInput,
    @builtin(vertex_index) v_idx: u32,
) -> VertexOutput {
    var out: VertexOutput;
    out.color = model.color;
    out.clip_position = camera.view_proj * vec4<f32>(model.position, 1.0);
    out.normal = model.normal;
    out.distance_from_camera = distance_from_camera(model.position);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let smooth_normal = normalize(in.normal);
    let light_intensity = max(dot(smooth_normal, normalize(light_direction)), minimum_light_intensity);
    return vec4<f32>(turbo_colormap(in.distance_from_camera) * light_intensity, 1.0);
}
