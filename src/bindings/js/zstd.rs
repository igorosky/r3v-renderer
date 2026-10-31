//! zstd decompression, for the data of the voxel stream (r3v_interfaces
//! VoxelMapUpdate), which r3v_ros compresses with zstd by default.

use std::io::Read;

use ruzstd::decoding::StreamingDecoder;
use wasm_bindgen::prelude::*;

use super::{Result, range_error};

/// Most bytes `decompress_zstd` returns, as r3v_ros decodes at most.
const MAX_DECOMPRESSED: u64 = 1 << 30;

/// Decompresses one zstd frame. `RangeError` when `data` is no valid zstd
/// frame or decompresses to more than 1 GiB.
#[wasm_bindgen]
pub fn decompress_zstd(data: &[u8]) -> Result<Vec<u8>> {
    let decoder = StreamingDecoder::new(data)
        .map_err(|e| range_error(format!("not a zstd frame: {e}")))?;
    let mut out = Vec::new();
    decoder
        .take(MAX_DECOMPRESSED + 1)
        .read_to_end(&mut out)
        .map_err(|e| range_error(format!("corrupt zstd data: {e}")))?;
    if out.len() as u64 > MAX_DECOMPRESSED {
        return Err(range_error(format!(
            "zstd data decompresses to more than {MAX_DECOMPRESSED} bytes"
        )));
    }
    Ok(out)
}
