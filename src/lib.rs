use crate::geometry::PointI;

pub mod camera;
pub mod geometry;
pub mod stl;
pub mod urdf;
pub mod wgpu;

#[cfg(test)]
mod reader_tests;

pub use stl::{NativeStlReader, StlIoReader, StlMesh, StlReader, Triangle};
pub use urdf::{
    NativeUrdfReader, UrdfJoint, UrdfLink, UrdfMaterial, UrdfOrigin, UrdfReader, UrdfRobot,
    UrdfRsReader, UrdfVisual,
};

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
    fn render(&mut self, output: &mut [u8]);
    fn camera(&mut self) -> &mut Self::Camera;
    fn load_urdf(&mut self, source: &str) -> Result<urdf::UrdfRobot, String> {
        let _ = source;
        Err("URDF loading is not supported by this renderer implementation.".to_string())
    }
}
