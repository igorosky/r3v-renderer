use crate::geometry::PointI;

pub mod camera;
pub mod geometry;
pub mod wgpu;

pub trait Camera {
    fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32);
    fn zoom(&mut self, delta_distance: f32);
    fn pan(&mut self, delta_x: f32, delta_y: f32);
}

pub trait Renderer {
    type Configuration;
    type Camera: Camera;
    
    fn configure(&mut self, configuration: Self::Configuration);
    fn add_voxels(&mut self, voxels: &[PointI]);
    fn render(&mut self) -> Vec<u8>;
    fn camera(&mut self) -> &mut Self::Camera;
}
