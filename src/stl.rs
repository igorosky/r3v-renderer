use std::path::Path;

pub mod native;
pub mod stl_io;

pub use native::{NativeStlReader, StlMesh, Triangle, resolve_stl_reference};
pub use stl_io::StlIoReader;

/// Reads STL files into the renderer's parser-independent mesh representation.
pub trait StlReader {
    fn read_file(&self, path: &Path) -> Result<StlMesh, String>;
}
