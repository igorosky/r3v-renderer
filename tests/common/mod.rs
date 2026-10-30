// Shared helpers for the integration tests. Not every test binary uses every helper.
#![allow(dead_code)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};

pub const EPSILON: f32 = 1.0e-5;

#[track_caller]
pub fn close(actual: f32, expected: f32) {
    close_eps(actual, expected, EPSILON);
}

#[track_caller]
pub fn close_eps(actual: f32, expected: f32, epsilon: f32) {
    assert!(
        (actual - expected).abs() <= epsilon,
        "expected {expected}, got {actual}"
    );
}

#[track_caller]
pub fn close3(actual: [f32; 3], expected: [f32; 3]) {
    close3_eps(actual, expected, EPSILON);
}

#[track_caller]
pub fn close3_eps(actual: [f32; 3], expected: [f32; 3], epsilon: f32) {
    assert!(
        actual
            .iter()
            .zip(expected)
            .all(|(a, e)| (a - e).abs() <= epsilon),
        "expected {expected:?}, got {actual:?}"
    );
}

/// Temporary directory removed when dropped.
pub struct TempDir {
    path: PathBuf,
}

impl TempDir {
    pub fn new(prefix: &str) -> Self {
        static COUNTER: AtomicUsize = AtomicUsize::new(0);
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let count = COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "r3v-{prefix}-{}-{unique}-{count}",
            std::process::id()
        ));
        std::fs::create_dir_all(&path).unwrap();
        Self { path }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Writes `contents` to `relative`, creating parent directories, and returns the full path.
    pub fn write(&self, relative: &str, contents: impl AsRef<[u8]>) -> PathBuf {
        let path = self.path.join(relative);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, contents).unwrap();
        path
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.path);
    }
}

/// Small deterministic pseudo-random generator (64-bit LCG).
pub struct Lcg(u64);

impl Lcg {
    pub fn new(seed: u64) -> Self {
        Self(seed ^ 0x9E37_79B9_7F4A_7C15)
    }

    pub fn next_u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 32) as u32
    }

    /// Uniform integer in `low..high`.
    pub fn range(&mut self, low: i32, high: i32) -> i32 {
        low + (self.next_u32() % (high - low) as u32) as i32
    }

    pub fn chance(&mut self, numerator: u32, denominator: u32) -> bool {
        self.next_u32() % denominator < numerator
    }
}

/// Offset of a page in its voxel group packed as r3v-voxelizer packs it: bit
/// `j` of x, y and z goes to bit `3 * j`, `3 * j + 1` and `3 * j + 2`.
pub fn page_index(page: [i32; 3]) -> usize {
    let mut index = 0;
    for bit in 0..4 {
        for (axis, coordinate) in page.iter().enumerate() {
            index |= ((*coordinate as usize >> bit) & 1) << (3 * bit + axis);
        }
    }
    index
}

/// `voxels` as the groups r3v-voxelizer's `get_all_points_compressed`
/// returns, built voxel by voxel, sorted by key.
pub fn voxel_groups(
    voxels: impl IntoIterator<Item = [i32; 3]>,
) -> Vec<r3v_renderer::geometry::VoxelGroup> {
    use r3v_renderer::geometry::{PointI, VOXEL_GROUP_PAGES, VoxelGroup};
    let mut groups: std::collections::BTreeMap<[i32; 3], Box<[u64; VOXEL_GROUP_PAGES]>> =
        std::collections::BTreeMap::new();
    for voxel in voxels {
        let key = voxel.map(|c| c >> 6);
        let page = voxel.map(|c| (c >> 2) & 15);
        let local = voxel.map(|c| c & 3);
        groups
            .entry(key)
            .or_insert_with(|| Box::new([0; VOXEL_GROUP_PAGES]))[page_index(page)] |=
            1 << (local[0] + 4 * local[1] + 16 * local[2]);
    }
    groups
        .into_iter()
        .map(|(key, values)| VoxelGroup {
            point: PointI {
                x: key[0],
                y: key[1],
                z: key[2],
                i: 0,
            },
            values,
        })
        .collect()
}

/// Voxels of 4x4x2 blocks (`PointI` with bit `x + 4 * y + 16 * z` set for the
/// voxel at (x, y, z) of the block).
pub fn block_voxels<'a>(
    blocks: impl IntoIterator<Item = &'a r3v_renderer::geometry::PointI>,
) -> std::collections::BTreeSet<[i32; 3]> {
    let mut voxels = std::collections::BTreeSet::new();
    for block in blocks {
        for bit in 0..32 {
            if block.i >> bit & 1 != 0 {
                voxels.insert([
                    block.x + (bit & 3),
                    block.y + (bit >> 2 & 3),
                    block.z + (bit >> 4),
                ]);
            }
        }
    }
    voxels
}
