use crate::geometry::PointI;

pub mod geometry;
pub mod wgpu;

pub trait Renderer {
    type Configuration;
    
    fn add_voxels(&mut self, voxels: &[PointI]);
    fn configure(&mut self, configuration: Self::Configuration);
    fn render(&mut self) -> Vec<u8>;
}
