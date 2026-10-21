//! `tesselator::CpuTesselator` (trait `Tesselator`) and its output,
//! `Vec<TriangleChunk>`.

use wasm_bindgen::prelude::*;

use crate::{
    bindings::checked::CheckedTesselator,
    geometry::{PointI, TriangleChunk},
};

use super::{
    Result, array, error, f32_property, flag_property, index, point_i, range_error,
    triangle_chunk_object,
};

/// `tesselator::CpuTesselatorConfiguration`: `voxel_size` is the edge length
/// of a voxel in world units, `marching_cubes_enabled` smooths the surface
/// with marching cubes instead of meshing the voxel faces.
#[wasm_bindgen(js_name = CpuTesselatorConfiguration)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JsCpuTesselatorConfiguration {
    pub voxel_size: f32,
    pub marching_cubes_enabled: bool,
}

#[wasm_bindgen(js_class = CpuTesselatorConfiguration)]
impl JsCpuTesselatorConfiguration {
    /// `marching_cubes_enabled` is false when left out.
    #[wasm_bindgen(constructor)]
    pub fn new(voxel_size: f32, marching_cubes_enabled: Option<bool>) -> Self {
        Self {
            voxel_size,
            marching_cubes_enabled: marching_cubes_enabled.unwrap_or(false),
        }
    }
}

/// Turns blocks of voxels into triangles (`tesselator::CpuTesselator`).
/// Configure it before use.
#[wasm_bindgen(js_name = CpuTesselator)]
pub struct JsCpuTesselator(CheckedTesselator);

/// `blocks` as `PointI`s: an array of `PointILike`s.
fn blocks(value: &JsValue) -> Result<Vec<PointI>> {
    array(value, &|| "blocks".to_owned())?
        .iter()
        .enumerate()
        .map(|(k, item)| point_i(&item, &|| format!("blocks[{k}]")))
        .collect()
}

#[wasm_bindgen(js_class = CpuTesselator)]
impl JsCpuTesselator {
    /// `CpuTesselator::new`, with marching cubes disabled.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self(CheckedTesselator::new())
    }

    /// `Tesselator::configure`. `RangeError` unless `voxel_size` is positive.
    /// `marching_cubes_enabled` is false when left out.
    pub fn configure(
        &mut self,
        #[wasm_bindgen(
            unchecked_param_type = "{ voxel_size: number; marching_cubes_enabled?: boolean }"
        )]
        configuration: JsValue,
    ) -> Result<()> {
        let what = || "configuration".to_owned();
        let voxel_size = f32_property(&configuration, "voxel_size", &what)?;
        let marching_cubes_enabled =
            flag_property(&configuration, "marching_cubes_enabled", &what)?;
        Ok(self.0.configure(voxel_size, marching_cubes_enabled)?)
    }

    /// `CpuTesselator::marching_cubes_enabled`.
    pub fn marching_cubes_enabled(&self) -> bool {
        self.0.marching_cubes_enabled()
    }

    /// `CpuTesselator::set_marching_cubes_enabled`: a change re-tesselates
    /// every chunk with voxels, and the chunks next to them, on the next
    /// `get_output_geometry_delta`.
    pub fn set_marching_cubes_enabled(&mut self, enabled: bool) {
        self.0.set_marching_cubes_enabled(enabled);
    }

    /// `Tesselator::update_input_geometry`: every block replaces the mask the
    /// tesselator had for it. `blocks` is an array of `PointI`s or of other
    /// objects with `x`, `y`, `z` and `i` properties, e.g. what r3v-voxelizer
    /// returns in compress mode.
    ///
    /// `RangeError` when a block is not aligned to the 4x4x2 grid (x and y
    /// multiples of 4, z of 2); none of the blocks is applied then.
    pub fn update_input_geometry(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "PointILike[]")] blocks: JsValue,
    ) -> Result<()> {
        let blocks = self::blocks(&blocks)?;
        Ok(self.0.update_input_geometry(blocks)?)
    }

    /// `Tesselator::get_output_geometry_delta`: the chunks changed since the
    /// last call, `undefined` when nothing changed.
    pub fn get_output_geometry_delta(&mut self) -> Result<Option<JsGeometryDelta>> {
        let delta = self.0.get_output_geometry_delta()?;
        Ok(delta.map(|delta| JsGeometryDelta(Some(delta))))
    }
}

/// Triangles of every chunk that changed (`Vec<TriangleChunk>`).
/// `WgpuRenderer.update_geometry` takes the chunks out of it, using it
/// afterwards throws.
#[wasm_bindgen(js_name = GeometryDelta)]
pub struct JsGeometryDelta(Option<Vec<TriangleChunk>>);

impl JsGeometryDelta {
    fn chunks(&self) -> Result<&Vec<TriangleChunk>> {
        self.0
            .as_ref()
            .ok_or_else(|| error("the delta was passed to WgpuRenderer.update_geometry"))
    }

    pub(super) fn take(&mut self) -> Result<Vec<TriangleChunk>> {
        self.chunks()?;
        Ok(self.0.take().unwrap_or_default())
    }
}

#[wasm_bindgen(js_class = GeometryDelta)]
impl JsGeometryDelta {
    /// Number of chunks.
    #[wasm_bindgen(getter)]
    pub fn length(&self) -> Result<usize> {
        Ok(self.chunks()?.len())
    }

    /// Chunk number `index`, as a copy.
    #[wasm_bindgen(unchecked_return_type = "TriangleChunk")]
    pub fn chunk(
        &self,
        #[wasm_bindgen(unchecked_param_type = "number")] index: JsValue,
    ) -> Result<JsValue> {
        let chunks = self.chunks()?;
        let index = self::index(&index, &|| "index".to_owned())?;
        let chunk = usize::try_from(index)
            .ok()
            .and_then(|index| chunks.get(index))
            .ok_or_else(|| {
                range_error(format!(
                    "chunk {index} does not exist, the delta has {}",
                    chunks.len()
                ))
            })?;
        Ok(triangle_chunk_object(chunk))
    }
}
