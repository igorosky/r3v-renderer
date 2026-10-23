pub mod camera;
pub mod cpu_tesselator;
pub mod geometry;
pub mod gpu_context;
#[cfg(feature = "gpu-tesselator")]
pub mod gpu_tesselator;
pub mod renderer;
pub mod robot;
pub mod stl;
pub mod urdf;

mod bindings;
mod turbo_colormap;
mod vertex_arena;
mod wgpu_utils;

pub trait Camera {
    fn look_at(
        &mut self,
        target: geometry::Point<f32>,
        distance: f32,
        yaw_deg: f32,
        pitch_deg: f32,
    );
    fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32);
    fn zoom(&mut self, delta_distance: f32);
    fn pan(&mut self, delta_x: f32, delta_y: f32);

    fn get_state(&self, aspect_ratio: f32) -> camera::CameraState;
}

pub trait Robot {
    type RobotDescription;
    type SensorInput;
    type Transform;

    fn from_description(description: Self::RobotDescription) -> Self;
    fn feed_sensor_input(&mut self, input: &Self::SensorInput) -> Self::Transform;
    fn last_transform(&self) -> Option<&Self::Transform>;
}

pub trait Tesselator {
    type Configuration;
    type InputGeometryDelta;
    type OutputGeometryDelta;

    fn configure(&mut self, configuration: Self::Configuration);
    fn update_input_geometry(&mut self, geometry_delta: &Self::InputGeometryDelta);
    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta>;
}

pub trait Renderer {
    type Configuration;
    type GeometryDelta;

    type AssetInstanceId;
    type AssetDescription;
    type AssetTransform;

    type CameraState;

    fn configure(&mut self, configuration: Self::Configuration);
    fn update_geometry(&mut self, geometry_delta: Self::GeometryDelta);
    fn upload_asset(
        &mut self,
        description: Self::AssetDescription,
        instances: usize,
    ) -> Vec<Self::AssetInstanceId>;
    fn set_asset_transform(
        &mut self,
        instance_id: Self::AssetInstanceId,
        transform: Self::AssetTransform,
    );
    fn set_asset_visibility(&mut self, instance_id: Self::AssetInstanceId, visible: bool);
    fn render(&mut self, camera_state: &Self::CameraState, output: &mut [u8]);
}
