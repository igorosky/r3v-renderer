//! `tesselator::GreedyMeshTesselator` (trait `Tesselator`) and its output,
//! `Vec<QuadChunk>`.

use wasm_bindgen::prelude::*;

use crate::{
    bindings::checked::CheckedTesselator,
    geometry::{PointI, QuadChunk},
};

use super::{Result, array, error, f32_property, index, point_i, quad_chunk_object, range_error};

/// `tesselator::GreedyMeshTesselatorConfiguration`: `voxel_size` is the edge
/// length of a voxel in world units.
#[wasm_bindgen(js_name = GreedyMeshTesselatorConfiguration)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JsGreedyMeshTesselatorConfiguration {
    pub voxel_size: f32,
}

#[wasm_bindgen(js_class = GreedyMeshTesselatorConfiguration)]
impl JsGreedyMeshTesselatorConfiguration {
    #[wasm_bindgen(constructor)]
    pub fn new(voxel_size: f32) -> Self {
        Self { voxel_size }
    }
}

/// Turns blocks of voxels into quads (`tesselator::GreedyMeshTesselator`).
/// Configure it before use.
#[wasm_bindgen(js_name = GreedyMeshTesselator)]
pub struct JsGreedyMeshTesselator(CheckedTesselator);

/// `blocks` as `PointI`s: an array of `PointILike`s.
fn blocks(value: &JsValue) -> Result<Vec<PointI>> {
    array(value, &|| "blocks".to_owned())?
        .iter()
        .enumerate()
        .map(|(k, item)| point_i(&item, &|| format!("blocks[{k}]")))
        .collect()
}

#[wasm_bindgen(js_class = GreedyMeshTesselator)]
impl JsGreedyMeshTesselator {
    /// `GreedyMeshTesselator::new`.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self(CheckedTesselator::new())
    }

    /// `Tesselator::configure`. `RangeError` unless `voxel_size` is positive.
    pub fn configure(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "{ voxel_size: number }")] configuration: JsValue,
    ) -> Result<()> {
        let voxel_size =
            f32_property(&configuration, "voxel_size", &|| "configuration".to_owned())?;
        Ok(self.0.configure(voxel_size)?)
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

/// Quads of every chunk that changed (`Vec<QuadChunk>`).
/// `WgpuRenderer.update_geometry` takes the chunks out of it, using it
/// afterwards throws.
#[wasm_bindgen(js_name = GeometryDelta)]
pub struct JsGeometryDelta(Option<Vec<QuadChunk>>);

impl JsGeometryDelta {
    fn chunks(&self) -> Result<&Vec<QuadChunk>> {
        self.0
            .as_ref()
            .ok_or_else(|| error("the delta was passed to WgpuRenderer.update_geometry"))
    }

    pub(super) fn take(&mut self) -> Result<Vec<QuadChunk>> {
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
    #[wasm_bindgen(unchecked_return_type = "QuadChunk")]
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
        Ok(quad_chunk_object(chunk))
    }
}
