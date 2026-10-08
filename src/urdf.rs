use std::fs;
use std::path::Path;

pub mod native;
pub mod urdf_rs;

pub use native::{
    NativeUrdfReader, UrdfColor, UrdfGeometry, UrdfJoint, UrdfLink, UrdfMaterial, UrdfMesh,
    UrdfOrigin, UrdfRobot, UrdfVisual, Vector3,
};
pub use urdf_rs::UrdfRsReader;

/// Reads URDF sources into the renderer's parser-independent model.
pub trait UrdfReader {
    fn read_str(&self, xml: &str) -> Result<UrdfRobot, String>;

    fn read_file(&self, path: &Path) -> Result<UrdfRobot, String> {
        let contents = fs::read_to_string(path)
            .map_err(|err| format!("Failed to read URDF '{}': {err}", path.display()))?;
        self.read_str(&contents)
    }
}
