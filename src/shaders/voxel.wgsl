const light_direction = vec3<f32>(0.25, 1.0, 0.25);
const minimum_light_intensity = 0.125;

struct CameraUniform {
    view_proj: mat4x4<f32>,
    position: vec3<f32>,
    forward: vec3<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

const colormap_len = 256u;
const turbo_first = 0u;
const hypsometric_first = 256u;

// A uniform buffer rather than a 1D texture: sampling one gave black on the Jetson's NVIDIA driver.
struct Colormap {
    colors: array<vec4<f32>, 512>,
    depth_range: f32,
    color_by_height: u32,
    neutral_height: f32,
    height_knee: f32,
    height_gain: f32,
    height_bands: u32,
};

@group(1) @binding(0)
var<uniform> colormap: Colormap;

fn sample_colormap(first: u32, x: f32) -> vec3<f32> {
    let position = clamp(x, 0.0, 1.0) * f32(colormap_len - 1u);
    let index = min(u32(position), colormap_len - 2u);
    return mix(colormap.colors[first + index].rgb, colormap.colors[first + index + 1u].rgb, position - f32(index));
}

// Where a height falls in the hypsometric colormap: the neutral height at 0.5, both sides reaching
// for the ends as a sigmoid of the logarithm of their distance from it.
fn hypsometric_position(height: f32) -> f32 {
    let distance = height - colormap.neutral_height;
    let scaled = log(1.0 + abs(distance) / colormap.height_knee);
    let position = 0.5 + 0.5 * sign(distance) * tanh(colormap.height_gain * scaled);
    if colormap.height_bands == 0u {
        return position;
    }
    let bands = f32(colormap.height_bands);
    return (floor(position * bands) + 0.5) / bands;
}

fn terrain_color(in: VertexOutput) -> vec3<f32> {
    if colormap.color_by_height != 0u {
        return sample_colormap(hypsometric_first, hypsometric_position(in.height));
    }
    return sample_colormap(turbo_first, in.distance_from_camera / colormap.depth_range);
}

fn distance_from_camera(world_position: vec3<f32>) -> f32 {
    return dot(world_position - camera.position, camera.forward);
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
    @location(3) height: f32,
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
    out.height = model.position.y;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let smooth_normal = normalize(in.normal);
    let light_intensity = max(dot(smooth_normal, normalize(light_direction)), minimum_light_intensity);
    return vec4<f32>(terrain_color(in) * light_intensity, 1.0);
}
