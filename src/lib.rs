pub mod camera;
pub mod geometry;
pub mod renderer;
pub mod tesselator;

mod vertex_arena;

pub trait Camera {
    fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32);
    fn zoom(&mut self, delta_distance: f32);
    fn pan(&mut self, delta_x: f32, delta_y: f32);

    fn get_uniform(&self, aspect_ratio: f32) -> camera::CameraUniform;
}

// pub trait Robot {
//     type RobotDescription;
//     type SensorInput;

//     fn from_description(description: Self::RobotDescription) -> Self;
//     fn feed_sensor_input(&mut self, input: &Self::SensorInput);
// }

pub trait Tesselator {
    type Configuration;
    type InputGeometryDelta;
    type OutputGeometryDelta;

    fn configure(&mut self, configuration: Self::Configuration);
    fn update_input_geometry(
        &mut self, 
        geometry_delta: &Self::InputGeometryDelta
    );
    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta>;
}

pub trait Renderer {
    type Configuration;
    type GeometryDelta;

    fn configure(&mut self, configuration: Self::Configuration);
    fn update_geometry(&mut self, geometry_delta: Self::GeometryDelta);
    fn render(
        &mut self,
        camera_uniform: &camera::CameraUniform,
        output:         &mut [u8]
    );
}
