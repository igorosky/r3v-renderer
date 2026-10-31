//! `tesselator::CpuTesselator` (trait `Tesselator`) and its output,
//! `Vec<TriangleChunk>`.

use js_sys::BigUint64Array;
use wasm_bindgen::prelude::*;

use crate::{
    bindings::checked::CheckedTesselator,
    geometry::{PointI, TriangleChunk, VOXEL_GROUP_PAGES},
};

use super::{
    JsPointI, Result, What, array, describe, error, f32_property, flag_property, index, point_i,
    property, range_error, triangle_chunk_object, type_error,
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

/// `value` as the words of a group: a `BigUint64Array` holding one for every
/// page.
fn words(value: &JsValue, what: What) -> Result<Box<[u64; VOXEL_GROUP_PAGES]>> {
    let values = value.dyn_ref::<BigUint64Array>().ok_or_else(|| {
        type_error(format!(
            "{} is not a BigUint64Array (got {})",
            what(),
            describe(value)
        ))
    })?;
    if values.length() as usize != VOXEL_GROUP_PAGES {
        return Err(range_error(format!(
            "{} holds {} words, a group holds {VOXEL_GROUP_PAGES}",
            what(),
            values.length()
        )));
    }
    let mut words = Box::new([0; VOXEL_GROUP_PAGES]);
    values.copy_to(&mut words[..]);
    Ok(words)
}

type Group = (PointI, Box<[u64; VOXEL_GROUP_PAGES]>);

/// Bytes of a block in the wire layout: little endian int32 x, y, z and
/// uint32 mask, as `PointI` is laid out.
const BLOCK_BYTES: usize = 16;

/// Bytes of a group in the wire layout: its key as a block, then the little
/// endian words of its pages, as `VoxelGroup` is laid out.
const GROUP_BYTES: usize = BLOCK_BYTES + 8 * VOXEL_GROUP_PAGES;

/// A block from `BLOCK_BYTES` bytes of the wire layout.
fn wire_block(bytes: &[u8]) -> PointI {
    let word = |k: usize| <[u8; 4]>::try_from(&bytes[4 * k..4 * k + 4]).unwrap();
    PointI {
        x: i32::from_le_bytes(word(0)),
        y: i32::from_le_bytes(word(1)),
        z: i32::from_le_bytes(word(2)),
        i: u32::from_le_bytes(word(3)),
    }
}

/// `bytes` as `size` byte records of the wire layout.
fn records<'a>(bytes: &'a [u8], size: usize, what: &str) -> Result<std::slice::ChunksExact<'a, u8>> {
    if !bytes.len().is_multiple_of(size) {
        return Err(range_error(format!(
            "{} bytes are not a whole number of {size} byte {what}",
            bytes.len()
        )));
    }
    Ok(bytes.chunks_exact(size))
}

/// `groups` as keys and words: an array of `VoxelGroupLike`s.
fn groups(value: &JsValue) -> Result<Vec<Group>> {
    array(value, &|| "groups".to_owned())?
        .iter()
        .enumerate()
        .map(|(k, group)| {
            let what = || format!("groups[{k}]");
            let point = property(&group, "point", &what)?;
            let values = property(&group, "values", &what)?;
            Ok((
                point_i(&point, &|| format!("groups[{k}].point"))?,
                words(&values, &|| format!("groups[{k}].values"))?,
            ))
        })
        .collect()
}

/// Constructed voxels of a cube of 64x64x64 voxels (`geometry::VoxelGroup`):
/// `point` is the key of the group, voxel coordinates shifted right by 6 (its
/// `i` is ignored), `values` the occupancy of every 4x4x4 page of the group,
/// in z-order (bits of x, y and z interleaved from the lowest one). Bit
/// `x + 4 * y + 16 * z` of a word is set when voxel (x, y, z) of its page is
/// constructed. Same interface as r3v-voxelizer's `CompressedVoxelGroup`,
/// which can be passed on as it is.
#[wasm_bindgen(js_name = VoxelGroup)]
pub struct JsVoxelGroup {
    point: PointI,
    values: Box<[u64; VOXEL_GROUP_PAGES]>,
}

#[wasm_bindgen(js_class = VoxelGroup)]
impl JsVoxelGroup {
    /// `RangeError` unless `values` holds a word for every page of the group
    /// (4096).
    #[wasm_bindgen(constructor)]
    pub fn new(
        #[wasm_bindgen(unchecked_param_type = "PointILike")] point: JsValue,
        #[wasm_bindgen(unchecked_param_type = "BigUint64Array")] values: JsValue,
    ) -> Result<JsVoxelGroup> {
        Ok(Self {
            point: point_i(&point, &|| "point".to_owned())?,
            values: words(&values, &|| "values".to_owned())?,
        })
    }

    /// Key of the group, as a copy.
    #[wasm_bindgen(getter)]
    pub fn point(&self) -> JsPointI {
        let PointI { x, y, z, i } = self.point;
        JsPointI { x, y, z, i }
    }

    /// Words of the group, as a copy.
    #[wasm_bindgen(getter)]
    pub fn values(&self) -> Vec<u64> {
        self.values.to_vec()
    }
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

    /// `update_input_geometry` with the blocks in their wire layout: 16
    /// bytes each, little endian int32 x, y, z and uint32 mask, as the data
    /// of an r3v_interfaces VoxelMapUpdate DELTA (decompressed) holds them.
    ///
    /// `RangeError` when the length is not a multiple of 16 or a block is not
    /// aligned to the 4x4x2 grid; none of the blocks is applied then.
    pub fn update_input_bytes(&mut self, bytes: &[u8]) -> Result<()> {
        let blocks = records(bytes, BLOCK_BYTES, "blocks")?
            .map(wire_block)
            .collect();
        Ok(self.0.update_input_geometry(blocks)?)
    }

    /// `replace_input_geometry` with the groups in their wire layout: 32784
    /// bytes each, the key as a 16 byte block (mask ignored) followed by the
    /// 4096 little endian uint64 words of its pages, as the data of an
    /// r3v_interfaces VoxelMapUpdate FULL (decompressed) holds them. Every
    /// voxel the tesselator held before is dropped, as there.
    ///
    /// `RangeError` when the length is not a multiple of 32784 or a key
    /// coordinate is outside [-2^25, 2^25); none of the groups is applied
    /// then.
    pub fn replace_input_bytes(&mut self, bytes: &[u8]) -> Result<()> {
        let groups = records(bytes, GROUP_BYTES, "groups")?
            .map(|group| {
                let mut words = Box::new([0; VOXEL_GROUP_PAGES]);
                let (pages, _) = group[BLOCK_BYTES..].as_chunks::<8>();
                for (word, bytes) in words.iter_mut().zip(pages) {
                    *word = u64::from_le_bytes(*bytes);
                }
                (wire_block(&group[..BLOCK_BYTES]), words)
            })
            .collect::<Vec<Group>>();
        Ok(self
            .0
            .replace_input_geometry(groups.iter().map(|(key, values)| (key.clone(), &**values)))?)
    }

    /// `CpuTesselator::replace_input_geometry`: drops every voxel the
    /// tesselator holds and keeps only the ones of `groups`. `groups` is an
    /// array of `VoxelGroup`s or of other objects with `point` and `values`
    /// properties, e.g. what r3v-voxelizer's `get_all_points_compressed`
    /// returns. Groups sharing a key add up. Only the chunks whose voxels
    /// changed come out of the next `get_output_geometry_delta`, the ones left
    /// without voxels with no triangles.
    ///
    /// `TypeError` when `values` of a group is not a `BigUint64Array`,
    /// `RangeError` when it does not hold a word for every page (4096) or a
    /// key coordinate is outside [-2^25, 2^25); none of the groups is applied
    /// then.
    pub fn replace_input_geometry(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "VoxelGroupLike[]")] groups: JsValue,
    ) -> Result<()> {
        let groups = self::groups(&groups)?;
        Ok(self
            .0
            .replace_input_geometry(groups.iter().map(|(key, values)| (key.clone(), &**values)))?)
    }

    /// `Tesselator::get_output_geometry_delta`: the chunks changed since the
    /// last call, `undefined` when nothing changed. With a `camera_state`,
    /// only the chunks in its view; the others stay changed for a later call.
    pub fn get_output_geometry_delta(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "CameraStateLike | undefined")]
        camera_state: &JsValue,
    ) -> Result<Option<JsGeometryDelta>> {
        let camera_state = if camera_state.is_undefined() || camera_state.is_null() {
            None
        } else {
            Some(super::camera_state(camera_state, &|| {
                "camera_state".to_owned()
            })?)
        };
        let delta = self.0.get_output_geometry_delta(camera_state.as_ref())?;
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
