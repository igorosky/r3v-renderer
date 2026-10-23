use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::VecDeque;

#[path = "marching_cubes.rs"]
pub(crate) mod marching_cubes;
use marching_cubes::{
    MC_CUBE_CORNERS, MC_GRID_SIZE, MC_GRID_VOLUME, MC_PADDED_GRID_SIZE, MC_PADDED_GRID_VOLUME,
    MC_X_SUM_VOLUME, MC_XY_SUM_VOLUME, polygonize_cube,
};

#[cfg(feature = "multi-threaded")]
use rayon::prelude::*;

use crate::{
    Tesselator,
    geometry::{
        Axis, DiscretePoint, DiscreteQuad, DiscreteUnit, Point, PointI, RealTriangle, RealUnit,
        TriangleChunk, Vector2D, Vector3D, VoxelNormal,
    },
};

pub(crate) const SUBCHUNK_SIZE: Vector3D<usize> = Vector3D { x: 4, y: 4, z: 2 }; // corresponds to input geometry delta `PointI` bitfield
pub(crate) const CHUNK_WIDTH: usize = 16; // in voxels
const CHUNK_MASK: u32 = u32::MAX >> (32 - CHUNK_WIDTH);
const CHUNK_SUBCHUNKS: Vector3D<usize> = Vector3D {
    x: CHUNK_WIDTH / SUBCHUNK_SIZE.x,
    y: CHUNK_WIDTH / SUBCHUNK_SIZE.y,
    z: CHUNK_WIDTH / SUBCHUNK_SIZE.z,
};
pub(crate) const CHUNK_RATIO: usize = CHUNK_SUBCHUNKS.x * CHUNK_SUBCHUNKS.y * CHUNK_SUBCHUNKS.z;
type Columns<T> = [[[T; CHUNK_WIDTH]; CHUNK_WIDTH]; 3];
type SolidColumns = Columns<u64>;

const fn layer_mask(axis: Axis, layer: usize) -> u32 {
    let mut mask = 0;
    let mut bit = 0;
    while bit < u32::BITS {
        if PointI::voxel_local(bit).get(axis) == layer {
            mask |= 1 << bit;
        }
        bit += 1;
    }
    mask
}

const BOUNDARY_LAYERS: [[u32; 2]; 3] = [
    [
        layer_mask(Axis::X, 0),
        layer_mask(Axis::X, SUBCHUNK_SIZE.get(Axis::X) - 1),
    ],
    [
        layer_mask(Axis::Y, 0),
        layer_mask(Axis::Y, SUBCHUNK_SIZE.get(Axis::Y) - 1),
    ],
    [
        layer_mask(Axis::Z, 0),
        layer_mask(Axis::Z, SUBCHUNK_SIZE.get(Axis::Z) - 1),
    ],
];

const fn boundary_mask(offset: ChunkOffset) -> u32 {
    let mut mask = u32::MAX;
    let mut axis = 0;
    while axis < 3 {
        if offset[axis] != 0 {
            mask &= BOUNDARY_LAYERS[axis][(offset[axis] > 0) as usize];
        }
        axis += 1;
    }
    mask
}

fn subchunk_on_boundary(subchunk: Point<usize>, offset: ChunkOffset) -> bool {
    Axis::ALL
        .into_iter()
        .all(|axis| match offset[axis as usize] {
            0 => true,
            d if d < 0 => subchunk[axis] == 0,
            _ => subchunk[axis] == CHUNK_SUBCHUNKS[axis] - 1,
        })
}

struct Chunk {
    subchunks: [u32; CHUNK_RATIO],
    columns: Columns<u32>,
    solid_subchunks: usize,
}

impl Chunk {
    fn new() -> Box<Self> {
        Box::new(Self {
            subchunks: [0; CHUNK_RATIO],
            columns: [[[0; CHUNK_WIDTH]; CHUNK_WIDTH]; 3],
            solid_subchunks: 0,
        })
    }

    fn subchunk_index(subchunk: Point<usize>) -> usize {
        (subchunk.z * CHUNK_SUBCHUNKS.y + subchunk.y) * CHUNK_SUBCHUNKS.x + subchunk.x
    }

    fn subchunk_of(index: usize) -> Point<usize> {
        Point {
            x: index % CHUNK_SUBCHUNKS.x,
            y: index / CHUNK_SUBCHUNKS.x % CHUNK_SUBCHUNKS.y,
            z: index / (CHUNK_SUBCHUNKS.x * CHUNK_SUBCHUNKS.y),
        }
    }

    fn origin_of(point: DiscretePoint) -> DiscretePoint {
        point.map(|c| c.div_euclid(CHUNK_WIDTH as DiscreteUnit) * CHUNK_WIDTH as DiscreteUnit)
    }

    fn neighbour_of(chunk: DiscretePoint, axis: Axis, direction: DiscreteUnit) -> DiscretePoint {
        let mut neighbour = chunk;
        neighbour[axis] += direction * CHUNK_WIDTH as DiscreteUnit;
        neighbour
    }

    fn subchunk_mask(&self, subchunk: Point<usize>) -> u32 {
        self.subchunks[Self::subchunk_index(subchunk)]
    }

    fn boundary_voxels(
        &self,
        offset: ChunkOffset,
    ) -> impl Iterator<Item = (Point<usize>, u32)> + '_ {
        let mask = boundary_mask(offset);
        self.subchunks
            .iter()
            .enumerate()
            .map(|(index, &bits)| (Self::subchunk_of(index), bits))
            .filter(move |&(subchunk, _)| subchunk_on_boundary(subchunk, offset))
            .map(move |(subchunk, bits)| (subchunk, bits & mask))
    }

    fn has_voxel_on_boundary(&self, offset: ChunkOffset) -> bool {
        self.boundary_voxels(offset).any(|(_, bits)| bits != 0)
    }

    fn set_subchunk_mask(&mut self, subchunk: Point<usize>, mask: u32) {
        let previous = std::mem::replace(&mut self.subchunks[Self::subchunk_index(subchunk)], mask);
        self.solid_subchunks =
            self.solid_subchunks + (mask != 0) as usize - (previous != 0) as usize;
        let origin = Point::from_fn(|axis| subchunk[axis] * SUBCHUNK_SIZE[axis]);
        for normal in Axis::ALL {
            let (column, row) = normal.plane();
            let run = (1u32 << SUBCHUNK_SIZE[normal]) - 1;
            for r in 0..SUBCHUNK_SIZE[row] {
                for c in 0..SUBCHUNK_SIZE[column] {
                    let mut local = Point { x: 0, y: 0, z: 0 };
                    local[column] = c;
                    local[row] = r;
                    let mut bits = 0;
                    for n in 0..SUBCHUNK_SIZE[normal] {
                        local[normal] = n;
                        bits |= (mask >> PointI::voxel_bit(local) & 1) << n;
                    }
                    let voxels =
                        &mut self.columns[normal as usize][origin[row] + r][origin[column] + c];
                    *voxels = *voxels & !(run << origin[normal]) | bits << origin[normal];
                }
            }
        }
    }
}

pub struct CpuTesselatorConfiguration {
    pub voxel_size: f32,
    pub marching_cubes_enabled: bool,
}

pub(crate) type ChunkOffset = [i8; 3];

pub(crate) const NEIGHBOUR_OFFSETS: [ChunkOffset; 26] = {
    let mut offsets = [[0; 3]; 26];
    let mut n = 0;
    let mut i = 0;
    while i < 27 {
        let offset = [(i % 3) as i8 - 1, (i / 3 % 3) as i8 - 1, (i / 9) as i8 - 1];
        if i != 13 {
            offsets[n] = offset;
            n += 1;
        }
        i += 1;
    }
    offsets
};

pub(crate) fn neighbour_chunk(chunk: DiscretePoint, offset: ChunkOffset) -> DiscretePoint {
    let step = |d: i8| d as DiscreteUnit * CHUNK_WIDTH as DiscreteUnit;
    DiscretePoint {
        x: chunk.x + step(offset[0]),
        y: chunk.y + step(offset[1]),
        z: chunk.z + step(offset[2]),
    }
}

pub(crate) fn is_face_offset(offset: ChunkOffset) -> bool {
    offset.iter().filter(|&&d| d != 0).count() == 1
}

pub(crate) struct SubchunkChange {
    pub(crate) chunk: DiscretePoint,
    subchunk: Point<usize>,
    changed: u32,
}

impl SubchunkChange {
    /// The neighbouring chunks whose boundary a changed voxel lies on.
    pub(crate) fn touched_neighbours(&self) -> impl Iterator<Item = ChunkOffset> + use<> {
        let (subchunk, changed) = (self.subchunk, self.changed);
        NEIGHBOUR_OFFSETS.into_iter().filter(move |&offset| {
            subchunk_on_boundary(subchunk, offset) && changed & boundary_mask(offset) != 0
        })
    }
}

pub(crate) struct VoxelStore {
    chunks: FxHashMap<DiscretePoint, Box<Chunk>>,
}

impl VoxelStore {
    pub(crate) fn new() -> Self {
        Self {
            chunks: FxHashMap::default(),
        }
    }

    pub(crate) fn update(&mut self, geometry_delta: &[PointI]) -> Vec<SubchunkChange> {
        let mut changes = Vec::new();
        for subchunk in geometry_delta {
            let origin = DiscretePoint {
                x: subchunk.x,
                y: subchunk.y,
                z: subchunk.z,
            };
            debug_assert!(
                Axis::ALL
                    .iter()
                    .all(|&axis| origin[axis].rem_euclid(SUBCHUNK_SIZE[axis] as DiscreteUnit) == 0),
                "Subchunk origin {:?} is not aligned to {:?}",
                origin,
                SUBCHUNK_SIZE,
            );
            let chunk = Chunk::origin_of(origin);
            let local =
                Point::from_fn(|axis| (origin[axis] - chunk[axis]) as usize / SUBCHUNK_SIZE[axis]);
            let previous = self
                .chunks
                .get(&chunk)
                .map_or(0, |stored| stored.subchunk_mask(local));
            if previous == subchunk.i {
                continue;
            }
            let stored = self.chunks.entry(chunk).or_insert_with(Chunk::new);
            stored.set_subchunk_mask(local, subchunk.i);
            if stored.solid_subchunks == 0 {
                self.chunks.remove(&chunk);
            }
            changes.push(SubchunkChange {
                chunk,
                subchunk: local,
                changed: previous ^ subchunk.i,
            });
        }
        changes
    }

    pub(crate) fn chunks(&self) -> impl Iterator<Item = DiscretePoint> + '_ {
        self.chunks.keys().copied()
    }

    pub(crate) fn has_voxel_on_boundary(&self, chunk: DiscretePoint, offset: ChunkOffset) -> bool {
        self.chunks
            .get(&chunk)
            .is_some_and(|stored| stored.has_voxel_on_boundary(offset))
    }

    #[cfg_attr(not(feature = "gpu-tesselator"), allow(dead_code))]
    pub(crate) fn subchunk_masks(&self, chunk: DiscretePoint) -> Option<&[u32; CHUNK_RATIO]> {
        self.chunks.get(&chunk).map(|stored| &stored.subchunks)
    }

    fn get(&self, chunk: &DiscretePoint) -> Option<&Chunk> {
        self.chunks.get(chunk).map(Box::as_ref)
    }
}

#[derive(Default)]
pub(crate) struct DirtyQueue {
    order: VecDeque<DiscretePoint>,
    queued: FxHashSet<DiscretePoint>,
}

impl DirtyQueue {
    pub(crate) fn mark(&mut self, chunk: DiscretePoint) {
        if self.queued.insert(chunk) {
            self.order.push_back(chunk);
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    pub(crate) fn take(&mut self, limit: usize) -> Vec<DiscretePoint> {
        let taken: Vec<DiscretePoint> = self.order.drain(..limit.min(self.order.len())).collect();
        if self.order.is_empty() {
            self.queued.clear();
        } else {
            for chunk in &taken {
                self.queued.remove(chunk);
            }
        }
        taken
    }
}

pub struct CpuTesselator {
    config: Option<CpuTesselatorConfiguration>,
    voxels: VoxelStore,
    dirty: DirtyQueue,
    marching_cubes_enabled: bool,
}

impl Default for CpuTesselator {
    fn default() -> Self {
        Self::new()
    }
}

impl CpuTesselator {
    pub fn new() -> Self {
        Self {
            config: None,
            voxels: VoxelStore::new(),
            dirty: DirtyQueue::default(),
            marching_cubes_enabled: false,
        }
    }

    pub fn marching_cubes_enabled(&self) -> bool {
        self.marching_cubes_enabled
    }

    pub fn set_marching_cubes_enabled(&mut self, enabled: bool) {
        if self.marching_cubes_enabled == enabled {
            return;
        }
        self.marching_cubes_enabled = enabled;
        for chunk in self.voxels.chunks() {
            self.dirty.mark(chunk);
            for offset in NEIGHBOUR_OFFSETS {
                if self.voxels.has_voxel_on_boundary(chunk, offset) {
                    self.dirty.mark(neighbour_chunk(chunk, offset));
                }
            }
        }
    }

    fn solid_columns(&self, chunk: DiscretePoint) -> Box<SolidColumns> {
        let mut solid = Box::new([[[0u64; CHUNK_WIDTH]; CHUNK_WIDTH]; 3]);
        let curr = self.voxels.get(&chunk);
        for normal in Axis::ALL {
            let below = self.voxels.get(&Chunk::neighbour_of(chunk, normal, -1));
            let above = self.voxels.get(&Chunk::neighbour_of(chunk, normal, 1));
            let normal = normal as usize;
            for r in 0..CHUNK_WIDTH {
                for c in 0..CHUNK_WIDTH {
                    let column = |chunk: Option<&Chunk>| {
                        chunk.map_or(0, |chunk| chunk.columns[normal][r][c] as u64)
                    };
                    solid[normal][r][c] = column(curr) << 1
                        | column(below) >> (CHUNK_WIDTH - 1) & 1
                        | (column(above) & 1) << (CHUNK_WIDTH + 1);
                }
            }
        }
        solid
    }

    fn greedy_rectangles(
        mut rows: [u32; CHUNK_WIDTH],
    ) -> impl Iterator<Item = (Vector2D<DiscreteUnit>, Vector2D<DiscreteUnit>)> {
        let mut r = 0;
        std::iter::from_fn(move || {
            while rows.get(r)? == &0 {
                r += 1;
            }
            let start = rows[r].trailing_zeros();
            let width = (rows[r] >> start).trailing_ones();
            let run = (((1u64 << width) - 1) << start) as u32;
            let height = rows[r..]
                .iter()
                .take_while(|&&row| row & run == run)
                .count();
            rows[r..r + height].iter_mut().for_each(|row| *row &= !run);
            Some((
                Vector2D {
                    x: start as DiscreteUnit,
                    y: r as DiscreteUnit,
                },
                Vector2D {
                    x: width as DiscreteUnit,
                    y: height as DiscreteUnit,
                },
            ))
        })
    }

    fn chunk_quads(&self, chunk: DiscretePoint) -> impl Iterator<Item = DiscreteQuad> {
        let solid = self.solid_columns(chunk);
        VoxelNormal::ALL.into_iter().flat_map(move |normal| {
            let axis = normal.axis();
            let (column, row) = axis.plane();
            let mut planes = [[0u32; CHUNK_WIDTH]; CHUNK_WIDTH];
            let mut occupied_layers = 0u32;
            for (r, columns) in solid[axis as usize].iter().enumerate() {
                for (c, &voxels) in columns.iter().enumerate() {
                    let exposed = if normal.is_positive() {
                        voxels & !(voxels >> 1)
                    } else {
                        voxels & !(voxels << 1)
                    };
                    let mut faces = (exposed >> 1) as u32 & CHUNK_MASK;
                    occupied_layers |= faces;
                    while faces != 0 {
                        planes[faces.trailing_zeros() as usize][r] |= 1 << c;
                        faces &= faces - 1;
                    }
                }
            }
            std::iter::from_fn(move || {
                (occupied_layers != 0).then(|| {
                    let layer = occupied_layers.trailing_zeros() as usize;
                    occupied_layers &= occupied_layers - 1;
                    layer
                })
            })
            .flat_map(move |layer| {
                Self::greedy_rectangles(planes[layer]).map(move |(start, size)| {
                    let mut origin = chunk;
                    origin[axis] += layer as DiscreteUnit;
                    origin[column] += start.x;
                    origin[row] += start.y;
                    DiscreteQuad::voxel_face(normal, origin, size)
                })
            })
        })
    }

    fn tesselate_chunk(&self, chunk: DiscretePoint) -> Vec<RealTriangle> {
        let voxel_size = self
            .config
            .as_ref()
            .expect("Tesselator must be configured before generating output geometry")
            .voxel_size;
        if self.marching_cubes_enabled {
            return self.marching_cubes_chunk(chunk, voxel_size);
        }
        self.chunk_quads(chunk)
            .map(|quad| quad.to_real(voxel_size))
            .flat_map(|quad| quad.triangles())
            .collect()
    }

    fn marching_cubes_chunk(
        &self,
        chunk: DiscretePoint,
        voxel_size: RealUnit,
    ) -> Vec<RealTriangle> {
        let mut field = [0u8; MC_GRID_VOLUME];
        self.marching_cubes_field(chunk, &mut field);
        let grid_index = |x: usize, y: usize, z: usize| (z * MC_GRID_SIZE + y) * MC_GRID_SIZE + x;
        if !field.iter().any(|&value| value != 0) || field.iter().all(|&value| value != 0) {
            return Vec::new();
        }

        let mut triangles = Vec::new();
        for z in 0..CHUNK_WIDTH {
            for y in 0..CHUNK_WIDTH {
                for x in 0..CHUNK_WIDTH {
                    let mut case_index = 0usize;
                    let mut corner_values = [0u8; 8];
                    for (corner, offset) in MC_CUBE_CORNERS.iter().enumerate() {
                        let cx = x + offset[0];
                        let cy = y + offset[1];
                        let cz = z + offset[2];
                        let value = field[grid_index(cx, cy, cz)];
                        corner_values[corner] = value;
                        if value != 0 {
                            case_index |= 1 << corner;
                        }
                    }
                    if case_index == 0 || case_index == u8::MAX as usize {
                        continue;
                    }

                    let corners = MC_CUBE_CORNERS.map(|offset| {
                        [
                            (chunk.x + (x + offset[0]) as DiscreteUnit) as RealUnit,
                            (chunk.y + (y + offset[1]) as DiscreteUnit) as RealUnit,
                            (chunk.z + (z + offset[2]) as DiscreteUnit) as RealUnit,
                        ]
                    });
                    let values = corner_values.map(|value| value as RealUnit);
                    polygonize_cube(case_index, &corners, values, voxel_size, &mut triangles);
                }
            }
        }
        triangles
    }

    fn marching_cubes_field(&self, chunk: DiscretePoint, field: &mut [u8; MC_GRID_VOLUME]) {
        let source_chunks: [Option<&Chunk>; 27] = std::array::from_fn(|index| {
            let x = (index % 3) as DiscreteUnit - 1;
            let y = ((index / 3) % 3) as DiscreteUnit - 1;
            let z = (index / 9) as DiscreteUnit - 1;
            self.voxels.get(&DiscretePoint {
                x: chunk.x + x * CHUNK_WIDTH as DiscreteUnit,
                y: chunk.y + y * CHUNK_WIDTH as DiscreteUnit,
                z: chunk.z + z * CHUNK_WIDTH as DiscreteUnit,
            })
        });
        let mut voxels = [0u8; MC_PADDED_GRID_VOLUME];
        for (source_index, source) in source_chunks.into_iter().enumerate() {
            let Some(source) = source else {
                continue;
            };
            let offset = [
                (source_index % 3) as i8 - 1,
                (source_index / 3 % 3) as i8 - 1,
                (source_index / 9) as i8 - 1,
            ];
            let facing = offset.map(|d| -d);
            for (subchunk, mut mask) in source.boundary_voxels(facing) {
                while mask != 0 {
                    let bit = mask.trailing_zeros();
                    mask &= mask - 1;
                    let local = PointI::voxel_local(bit);
                    let padded = Point::from_fn(|axis| {
                        let voxel = subchunk[axis] * SUBCHUNK_SIZE[axis] + local[axis];
                        (voxel as isize + 1 + offset[axis as usize] as isize * CHUNK_WIDTH as isize)
                            as usize
                    });
                    voxels[(padded.z * MC_PADDED_GRID_SIZE + padded.y) * MC_PADDED_GRID_SIZE
                        + padded.x] = 1;
                }
            }
        }

        let mut x_sums = [0u8; MC_X_SUM_VOLUME];
        for z in 0..MC_PADDED_GRID_SIZE {
            for y in 0..MC_PADDED_GRID_SIZE {
                for x in 0..MC_GRID_SIZE {
                    let input = (z * MC_PADDED_GRID_SIZE + y) * MC_PADDED_GRID_SIZE + x;
                    let output = (z * MC_PADDED_GRID_SIZE + y) * MC_GRID_SIZE + x;
                    x_sums[output] = voxels[input] + voxels[input + 1];
                }
            }
        }

        let mut xy_sums = [0u8; MC_XY_SUM_VOLUME];
        for z in 0..MC_PADDED_GRID_SIZE {
            for y in 0..MC_GRID_SIZE {
                for x in 0..MC_GRID_SIZE {
                    let input = (z * MC_PADDED_GRID_SIZE + y) * MC_GRID_SIZE + x;
                    let output = (z * MC_GRID_SIZE + y) * MC_GRID_SIZE + x;
                    xy_sums[output] = x_sums[input] + x_sums[input + MC_GRID_SIZE];
                }
            }
        }

        for z in 0..MC_GRID_SIZE {
            for y in 0..MC_GRID_SIZE {
                for x in 0..MC_GRID_SIZE {
                    let index = (z * MC_GRID_SIZE + y) * MC_GRID_SIZE + x;
                    field[index] = xy_sums[index] + xy_sums[index + MC_GRID_SIZE * MC_GRID_SIZE];
                }
            }
        }
    }
}

impl Tesselator for CpuTesselator {
    type Configuration = CpuTesselatorConfiguration;
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<TriangleChunk>;

    fn update_input_geometry(&mut self, geometry_delta: &Self::InputGeometryDelta) {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before updating input geometry"
        );
        for change in self.voxels.update(geometry_delta) {
            self.dirty.mark(change.chunk);
            for offset in change.touched_neighbours() {
                if self.marching_cubes_enabled || is_face_offset(offset) {
                    self.dirty.mark(neighbour_chunk(change.chunk, offset));
                }
            }
        }
    }

    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before generating output geometry"
        );
        if self.dirty.is_empty() {
            return None;
        }
        let dirty = self.dirty.take(usize::MAX);
        #[cfg(feature = "multi-threaded")]
        let dirty = dirty.into_par_iter();
        #[cfg(not(feature = "multi-threaded"))]
        let dirty = dirty.into_iter();
        let output_delta = dirty
            .map(|chunk| TriangleChunk {
                index: chunk,
                triangles: self.tesselate_chunk(chunk),
            })
            .collect();
        Some(output_delta)
    }

    fn configure(&mut self, configuration: Self::Configuration) {
        let marching_cubes_enabled = configuration.marching_cubes_enabled;
        self.config = Some(configuration);
        self.set_marching_cubes_enabled(marching_cubes_enabled);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn origin() -> DiscretePoint {
        DiscretePoint { x: 0, y: 0, z: 0 }
    }

    /// The chunks a tesselator tesselates after setting the voxel at the
    /// corner of chunk 0 nearest negative infinity.
    fn dirtied_by_corner_voxel(marching_cubes_enabled: bool) -> FxHashSet<DiscretePoint> {
        let mut tesselator = CpuTesselator::new();
        tesselator.configure(CpuTesselatorConfiguration {
            voxel_size: 1.0,
            marching_cubes_enabled,
        });
        tesselator.update_input_geometry(&vec![PointI {
            x: 0,
            y: 0,
            z: 0,
            i: 1,
        }]);
        tesselator.dirty.take(usize::MAX).into_iter().collect()
    }

    fn chunks_at(offsets: impl IntoIterator<Item = ChunkOffset>) -> FxHashSet<DiscretePoint> {
        offsets
            .into_iter()
            .map(|offset| neighbour_chunk(origin(), offset))
            .collect()
    }

    #[test]
    fn boundaries_match_single_voxels() {
        let on_side = |voxel: Point<usize>, offset: ChunkOffset| {
            Axis::ALL
                .into_iter()
                .all(|axis| match offset[axis as usize] {
                    0 => true,
                    d if d < 0 => voxel[axis] == 0,
                    _ => voxel[axis] == CHUNK_WIDTH - 1,
                })
        };
        let all_offsets: Vec<ChunkOffset> = (0..27)
            .map(|i| [(i % 3) as i8 - 1, (i / 3 % 3) as i8 - 1, (i / 9) as i8 - 1])
            .collect();
        let mut chunk = Chunk::new();
        for index in 0..CHUNK_WIDTH * CHUNK_WIDTH * CHUNK_WIDTH {
            let voxel = Point {
                x: index % CHUNK_WIDTH,
                y: index / CHUNK_WIDTH % CHUNK_WIDTH,
                z: index / (CHUNK_WIDTH * CHUNK_WIDTH),
            };
            let subchunk = Point::from_fn(|axis| voxel[axis] / SUBCHUNK_SIZE[axis]);
            assert_eq!(
                Chunk::subchunk_of(Chunk::subchunk_index(subchunk)),
                subchunk
            );
            let bit = PointI::voxel_bit(Point::from_fn(|axis| voxel[axis] % SUBCHUNK_SIZE[axis]));
            chunk.set_subchunk_mask(subchunk, 1 << bit);
            for &offset in &all_offsets {
                let expected = on_side(voxel, offset);
                assert_eq!(
                    chunk.has_voxel_on_boundary(offset),
                    expected,
                    "voxel {voxel:?}, side {offset:?}"
                );
                let change = SubchunkChange {
                    chunk: origin(),
                    subchunk,
                    changed: 1 << bit,
                };
                if offset != [0, 0, 0] {
                    assert_eq!(
                        change.touched_neighbours().any(|touched| touched == offset),
                        expected,
                        "voxel {voxel:?}, side {offset:?}"
                    );
                }
            }
            chunk.set_subchunk_mask(subchunk, 0);
        }
    }

    #[test]
    fn neighbour_offsets_are_all_but_the_centre() {
        let unique: FxHashSet<ChunkOffset> = NEIGHBOUR_OFFSETS.into_iter().collect();
        assert_eq!(unique.len(), 26);
        assert!(!unique.contains(&[0, 0, 0]));
        assert!(
            unique
                .iter()
                .all(|offset| offset.iter().all(|d| (-1..=1).contains(d)))
        );
        assert_eq!(
            NEIGHBOUR_OFFSETS
                .into_iter()
                .filter(|&o| is_face_offset(o))
                .count(),
            6
        );
    }

    #[test]
    fn changes_touch_the_neighbours_on_their_boundary() {
        let mut voxels = VoxelStore::new();
        // A voxel in the middle of a chunk touches no neighbour
        let middle = voxels.update(&[PointI {
            x: 4,
            y: 4,
            z: 2,
            i: 1,
        }]);
        assert_eq!(middle.len(), 1);
        assert_eq!(middle[0].touched_neighbours().count(), 0);
        // The corner voxel touches the 3 faces, 3 edges and the corner it lies on
        let corner = voxels.update(&[PointI {
            x: 0,
            y: 0,
            z: 0,
            i: 1,
        }]);
        let touched: FxHashSet<ChunkOffset> = corner[0].touched_neighbours().collect();
        let expected: FxHashSet<ChunkOffset> = NEIGHBOUR_OFFSETS
            .into_iter()
            .filter(|offset| offset.iter().all(|&d| d <= 0))
            .collect();
        assert_eq!(touched, expected);
        // Setting the same voxels again changes nothing
        assert!(
            voxels
                .update(&[PointI {
                    x: 0,
                    y: 0,
                    z: 0,
                    i: 1
                }])
                .is_empty()
        );
    }

    #[test]
    fn emptied_chunks_are_dropped_but_reported() {
        let mut voxels = VoxelStore::new();
        voxels.update(&[PointI {
            x: 0,
            y: 0,
            z: 0,
            i: 1,
        }]);
        assert_eq!(voxels.chunks().collect::<Vec<_>>(), [origin()]);
        let changes = voxels.update(&[PointI {
            x: 0,
            y: 0,
            z: 0,
            i: 0,
        }]);
        assert_eq!(changes.len(), 1);
        assert_eq!(changes[0].chunk, origin());
        assert_eq!(voxels.chunks().count(), 0);
        assert!(voxels.subchunk_masks(origin()).is_none());
    }

    #[test]
    fn voxel_faces_dirty_only_neighbours_across_faces() {
        let mut expected = chunks_at([[-1, 0, 0], [0, -1, 0], [0, 0, -1]]);
        expected.insert(origin());
        assert_eq!(dirtied_by_corner_voxel(false), expected);
    }

    #[test]
    fn marching_cubes_dirty_neighbours_across_edges_and_corners() {
        let mut expected = chunks_at(
            NEIGHBOUR_OFFSETS
                .into_iter()
                .filter(|offset| offset.iter().all(|&d| d <= 0)),
        );
        expected.insert(origin());
        assert_eq!(dirtied_by_corner_voxel(true), expected);
    }

    #[test]
    fn dirty_queue_keeps_order_without_duplicates() {
        let chunk = |x| DiscretePoint { x, y: 0, z: 0 };
        let mut queue = DirtyQueue::default();
        for x in [3, 1, 3, 2, 1] {
            queue.mark(chunk(x));
        }
        assert_eq!(queue.take(2), [chunk(3), chunk(1)]);
        // Taken chunks can be queued again, queued ones still only once
        queue.mark(chunk(3));
        queue.mark(chunk(2));
        assert_eq!(queue.take(usize::MAX), [chunk(2), chunk(3)]);
        assert!(queue.is_empty());
    }
}
