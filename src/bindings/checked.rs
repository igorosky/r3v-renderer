//! The components as the language bindings use them: what a component would
//! panic on (or silently get wrong) is checked first and returned as an
//! [`Error`], so every binding reports the same failures.

use crate::{
    Camera, Renderer, Tesselator,
    camera::{CameraState, OrbitCamera},
    geometry::{PointI, RealTwist, TriangleChunk},
    renderer::{WgpuAccelerated, WgpuAcceleratedConfiguration},
    stl::StlMesh,
    tesselator::{CpuTesselator, CpuTesselatorConfiguration, SUBCHUNK_SIZE},
};

/// Bytes per pixel of a rendered frame.
const PIXEL_SIZE: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ErrorKind {
    /// Nothing was changed.
    InvalidArgument,
    NotConfigured,
    OutOfRange,
    /// E.g. reading a file or getting a GPU.
    Failed,
}

#[derive(Debug)]
pub(crate) struct Error {
    pub(crate) kind: ErrorKind,
    pub(crate) message: String,
}

impl Error {
    pub(crate) fn new(kind: ErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
        }
    }

    pub(crate) fn invalid_argument(message: impl Into<String>) -> Self {
        Self::new(ErrorKind::InvalidArgument, message)
    }

    pub(crate) fn out_of_range(message: impl Into<String>) -> Self {
        Self::new(ErrorKind::OutOfRange, message)
    }

    pub(crate) fn failed(message: impl Into<String>) -> Self {
        Self::new(ErrorKind::Failed, message)
    }
}

pub(crate) type Result<T> = std::result::Result<T, Error>;

/// `Camera::get_state`, which asserts on these.
pub(crate) fn camera_state(camera: &OrbitCamera, aspect_ratio: f32) -> Result<CameraState> {
    let positive = |value: f32| value.is_finite() && value > 0.0;
    if !positive(aspect_ratio) {
        return Err(Error::invalid_argument(format!(
            "aspect ratio must be positive, got {aspect_ratio}"
        )));
    }
    if !positive(camera.near) || !camera.far.is_finite() || camera.far <= camera.near {
        return Err(Error::invalid_argument(format!(
            "clip planes need 0 < near < far, got near {} and far {}",
            camera.near, camera.far
        )));
    }
    Ok(camera.get_state(aspect_ratio))
}

/// `CpuTesselator`, which asserts it is configured.
pub(crate) struct CheckedTesselator {
    tesselator: CpuTesselator,
    configured: bool,
}

impl CheckedTesselator {
    pub(crate) fn new() -> Self {
        Self {
            tesselator: CpuTesselator::new(),
            configured: false,
        }
    }

    fn check_configured(&self) -> Result<()> {
        if self.configured {
            Ok(())
        } else {
            Err(Error::new(
                ErrorKind::NotConfigured,
                "configure the tesselator before passing geometry through it",
            ))
        }
    }

    pub(crate) fn configure(
        &mut self,
        voxel_size: f32,
        marching_cubes_enabled: bool,
    ) -> Result<()> {
        if !(voxel_size.is_finite() && voxel_size > 0.0) {
            return Err(Error::invalid_argument(format!(
                "voxel size must be positive, got {voxel_size}"
            )));
        }
        self.tesselator.configure(CpuTesselatorConfiguration {
            voxel_size,
            marching_cubes_enabled,
        });
        self.configured = true;
        Ok(())
    }

    pub(crate) fn marching_cubes_enabled(&self) -> bool {
        self.tesselator.marching_cubes_enabled()
    }

    pub(crate) fn set_marching_cubes_enabled(&mut self, enabled: bool) {
        self.tesselator.set_marching_cubes_enabled(enabled);
    }

    /// Applies none of the blocks when one is not aligned to the block grid,
    /// which the tesselator only checks in debug builds.
    pub(crate) fn update_input_geometry(&mut self, blocks: Vec<PointI>) -> Result<()> {
        self.check_configured()?;
        let size = SUBCHUNK_SIZE.map(|size| size as i32);
        if let Some((k, block)) = blocks.iter().enumerate().find(|(_, block)| {
            block.x.rem_euclid(size.x) != 0
                || block.y.rem_euclid(size.y) != 0
                || block.z.rem_euclid(size.z) != 0
        }) {
            return Err(Error::invalid_argument(format!(
                "blocks[{k}] at ({}, {}, {}) is not aligned to the {}x{}x{} block grid",
                block.x, block.y, block.z, size.x, size.y, size.z
            )));
        }
        self.tesselator.update_input_geometry(&blocks);
        Ok(())
    }

    pub(crate) fn get_output_geometry_delta(&mut self) -> Result<Option<Vec<TriangleChunk>>> {
        self.check_configured()?;
        Ok(self.tesselator.get_output_geometry_delta())
    }
}

/// `WgpuAccelerated`, which panics on a frame size the GPU cannot take, on
/// rendering before it is configured, on a too small frame buffer and on
/// unknown instances.
pub(crate) struct CheckedRenderer {
    renderer: WgpuAccelerated,
    configuration: Option<WgpuAcceleratedConfiguration>,
    instance_count: u64,
}

impl CheckedRenderer {
    #[cfg(any(feature = "c", feature = "cpp", feature = "python"))]
    pub(crate) fn new() -> Result<Self> {
        Self::from_renderer(WgpuAccelerated::new())
    }

    #[cfg(feature = "js")]
    pub(crate) async fn new_async() -> Result<Self> {
        Self::from_renderer(WgpuAccelerated::new_async().await)
    }

    fn from_renderer(renderer: Option<WgpuAccelerated>) -> Result<Self> {
        let renderer =
            renderer.ok_or_else(|| Error::failed("no GPU adapter or device is available"))?;
        Ok(Self {
            renderer,
            configuration: None,
            instance_count: 0,
        })
    }

    pub(crate) fn configure(&mut self, width: u32, height: u32) -> Result<()> {
        // The renderer asks for a device with the default limits
        let max = wgpu::Limits::default().max_texture_dimension_2d;
        if !(1..=max).contains(&width) || !(1..=max).contains(&height) {
            return Err(Error::invalid_argument(format!(
                "frame size must be between 1x1 and {max}x{max}, got {width}x{height}"
            )));
        }
        let configuration = WgpuAcceleratedConfiguration { width, height };
        self.renderer.configure(configuration);
        self.configuration = Some(configuration);
        Ok(())
    }

    pub(crate) fn update_geometry(&mut self, delta: Vec<TriangleChunk>) {
        self.renderer.update_geometry(delta);
    }

    /// Checks that `instances` more instances of `mesh` fit, before anything
    /// is uploaded.
    pub(crate) fn check_upload(&self, mesh: &StlMesh, instances: u64) -> Result<()> {
        // The renderer keeps vertex and instance offsets in u32
        let fits = self
            .instance_count
            .checked_add(instances)
            .is_some_and(|count| count <= u64::from(u32::MAX));
        if !fits {
            return Err(Error::invalid_argument(format!(
                "{instances} more instances do not fit next to the {} uploaded",
                self.instance_count
            )));
        }
        if mesh.triangles.len() > (u32::MAX / 3) as usize {
            return Err(Error::invalid_argument(format!(
                "mesh has {} triangles, too many for one asset",
                mesh.triangles.len()
            )));
        }
        Ok(())
    }

    pub(crate) fn upload_asset(&mut self, mesh: StlMesh, instances: u64) -> Result<Vec<u64>> {
        self.check_upload(&mesh, instances)?;
        // Checked to fit in u32
        let ids = self.renderer.upload_asset(mesh, instances as usize);
        self.instance_count += instances;
        Ok(ids)
    }

    fn check_instance(&self, instance_id: u64) -> Result<()> {
        if instance_id < self.instance_count {
            Ok(())
        } else {
            Err(Error::out_of_range(format!(
                "instance {instance_id} does not exist, {} were uploaded",
                self.instance_count
            )))
        }
    }

    pub(crate) fn set_asset_transform(
        &mut self,
        instance_id: u64,
        transform: RealTwist,
    ) -> Result<()> {
        self.check_instance(instance_id)?;
        self.renderer.set_asset_transform(instance_id, transform);
        Ok(())
    }

    pub(crate) fn set_asset_visibility(&mut self, instance_id: u64, visible: bool) -> Result<()> {
        self.check_instance(instance_id)?;
        self.renderer.set_asset_visibility(instance_id, visible);
        Ok(())
    }

    /// Bytes of a frame at the configured size.
    pub(crate) fn frame_len(&self) -> Result<usize> {
        let WgpuAcceleratedConfiguration { width, height } =
            self.configuration.ok_or_else(|| {
                Error::new(
                    ErrorKind::NotConfigured,
                    "configure the renderer before rendering",
                )
            })?;
        Ok(width as usize * height as usize * PIXEL_SIZE)
    }

    fn check_frame_buffer(&self, out: &[u8]) -> Result<()> {
        let frame_len = self.frame_len()?;
        if out.len() < frame_len {
            return Err(Error::invalid_argument(format!(
                "a frame needs {frame_len} bytes, the buffer has {}",
                out.len()
            )));
        }
        Ok(())
    }

    #[cfg(any(feature = "c", feature = "cpp", feature = "python"))]
    pub(crate) fn render(&mut self, camera_state: &CameraState, out: &mut [u8]) -> Result<()> {
        self.check_frame_buffer(out)?;
        self.renderer.render(camera_state, out);
        Ok(())
    }

    #[cfg(feature = "js")]
    pub(crate) async fn render_async(
        &mut self,
        camera_state: &CameraState,
        out: &mut [u8],
    ) -> Result<()> {
        self.check_frame_buffer(out)?;
        self.renderer.render_async(camera_state, out).await;
        Ok(())
    }
}
