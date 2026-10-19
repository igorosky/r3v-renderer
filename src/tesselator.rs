use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::VecDeque;
use std::sync::LazyLock;

#[cfg(feature = "multi-threaded")]
use rayon::prelude::*;

use crate::{
    Tesselator,
    geometry::{
        Axis, DiscretePoint, DiscreteQuad, DiscreteUnit, Point, PointI, RealTriangle, RealUnit,
        TriangleChunk, Vector2D, Vector3D, VoxelNormal, VoxelQuad,
    },
};

const SUBCHUNK_SIZE: Vector3D<usize> = Vector3D { x: 4, y: 4, z: 2 }; // corresponds to input geometry delta `PointI` bitfield
const CHUNK_WIDTH: usize = 16; // in voxels
const CHUNK_MASK: u32 = u32::MAX >> (32 - CHUNK_WIDTH);
const CHUNK_SUBCHUNKS: Vector3D<usize> = Vector3D {
    x: CHUNK_WIDTH / SUBCHUNK_SIZE.x,
    y: CHUNK_WIDTH / SUBCHUNK_SIZE.y,
    z: CHUNK_WIDTH / SUBCHUNK_SIZE.z,
};
const CHUNK_RATIO: usize = CHUNK_SUBCHUNKS.x * CHUNK_SUBCHUNKS.y * CHUNK_SUBCHUNKS.z;
const MC_GRID_SIZE: usize = CHUNK_WIDTH + 1;
const MC_PADDED_GRID_SIZE: usize = CHUNK_WIDTH + 2;
const MC_GRID_VOLUME: usize = MC_GRID_SIZE * MC_GRID_SIZE * MC_GRID_SIZE;
const MC_X_SUM_VOLUME: usize = MC_GRID_SIZE * MC_PADDED_GRID_SIZE * MC_PADDED_GRID_SIZE;
const MC_XY_SUM_VOLUME: usize = MC_GRID_SIZE * MC_GRID_SIZE * MC_PADDED_GRID_SIZE;
const MC_PADDED_GRID_VOLUME: usize =
    MC_PADDED_GRID_SIZE * MC_PADDED_GRID_SIZE * MC_PADDED_GRID_SIZE;
const MC_CUBE_CORNERS: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1],
];
const MC_CUBE_EDGES: [[usize; 2]; 12] = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 4],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
];
const MC_CUBE_FACES: [([usize; 4], [usize; 4]); 6] = [
    ([0, 1, 2, 3], [0, 1, 2, 3]),
    ([4, 5, 6, 7], [4, 5, 6, 7]),
    ([0, 1, 5, 4], [0, 9, 4, 8]),
    ([3, 2, 6, 7], [2, 10, 6, 11]),
    ([0, 3, 7, 4], [3, 11, 7, 8]),
    ([1, 2, 6, 5], [1, 10, 5, 9]),
];
const MC_ISO_LEVEL: RealUnit = 0.5;
const MC_TRIANGLE_TABLE_WIDTH: usize = 48;

struct MarchingCubesLookup {
    edge_masks: [u16; 256],
    triangle_edges: [[i8; MC_TRIANGLE_TABLE_WIDTH]; 256],
}

static MARCHING_CUBES_LOOKUP: LazyLock<MarchingCubesLookup> =
    LazyLock::new(MarchingCubesLookup::build);

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

    fn voxel(&self, local: Point<usize>) -> bool {
        let subchunk = Point::from_fn(|axis| local[axis] / SUBCHUNK_SIZE[axis]);
        let bit = PointI::voxel_bit(Point::from_fn(|axis| local[axis] % SUBCHUNK_SIZE[axis]));
        self.subchunk_mask(subchunk) & (1 << bit) != 0
    }

    fn has_voxel_on_boundary(&self, boundary: [i8; 3]) -> bool {
        for z in 0..CHUNK_WIDTH {
            if boundary[2] != 0 && z != if boundary[2] < 0 { 0 } else { CHUNK_WIDTH - 1 } {
                continue;
            }
            for y in 0..CHUNK_WIDTH {
                if boundary[1] != 0 && y != if boundary[1] < 0 { 0 } else { CHUNK_WIDTH - 1 } {
                    continue;
                }
                for x in 0..CHUNK_WIDTH {
                    if boundary[0] != 0 && x != if boundary[0] < 0 { 0 } else { CHUNK_WIDTH - 1 } {
                        continue;
                    }
                    if self.voxel(Point { x, y, z }) {
                        return true;
                    }
                }
            }
        }
        false
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

pub struct GreedyMeshTesselatorConfiguration {
    pub voxel_size: u16,
}

pub struct GreedyMeshTesselator {
    config: Option<GreedyMeshTesselatorConfiguration>,
    chunks: FxHashMap<DiscretePoint, Box<Chunk>>,
    dirty_queue: VecDeque<DiscretePoint>,
    in_queue: FxHashSet<DiscretePoint>,
    marching_cubes_enabled: bool,
}

impl Default for GreedyMeshTesselator {
    fn default() -> Self {
        Self::new()
    }
}

impl GreedyMeshTesselator {
    pub fn new() -> Self {
        Self {
            config: None,
            chunks: FxHashMap::default(),
            dirty_queue: VecDeque::new(),
            in_queue: FxHashSet::default(),
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
        let mut dirty = Vec::new();
        for (&chunk, stored) in &self.chunks {
            dirty.push(chunk);
            for z in -1..=1 {
                for y in -1..=1 {
                    for x in -1..=1 {
                        if x == 0 && y == 0 && z == 0 {
                            continue;
                        }
                        let boundary = [
                            if x < 0 {
                                -1
                            } else if x > 0 {
                                1
                            } else {
                                0
                            },
                            if y < 0 {
                                -1
                            } else if y > 0 {
                                1
                            } else {
                                0
                            },
                            if z < 0 {
                                -1
                            } else if z > 0 {
                                1
                            } else {
                                0
                            },
                        ];
                        if !stored.has_voxel_on_boundary(boundary) {
                            continue;
                        }
                        dirty.push(DiscretePoint {
                            x: chunk.x + x * CHUNK_WIDTH as DiscreteUnit,
                            y: chunk.y + y * CHUNK_WIDTH as DiscreteUnit,
                            z: chunk.z + z * CHUNK_WIDTH as DiscreteUnit,
                        });
                    }
                }
            }
        }
        for chunk in dirty {
            self.mark_dirty(chunk);
        }
    }

    fn mark_dirty(&mut self, chunk: DiscretePoint) {
        if self.in_queue.insert(chunk) {
            self.dirty_queue.push_back(chunk);
        }
    }

    fn mark_subchunk_dirty(&mut self, chunk: DiscretePoint, subchunk: Point<usize>, changed: u32) {
        self.mark_dirty(chunk);
        let mut directions = [[0; 3]; 3];
        let mut counts = [1; 3];
        for axis in Axis::ALL {
            let [low_layer, high_layer] = BOUNDARY_LAYERS[axis as usize];
            if subchunk[axis] == 0 && changed & low_layer != 0 {
                directions[axis as usize][counts[axis as usize]] = -1;
                counts[axis as usize] += 1;
            }
            if subchunk[axis] == CHUNK_SUBCHUNKS[axis] - 1 && changed & high_layer != 0 {
                directions[axis as usize][counts[axis as usize]] = 1;
                counts[axis as usize] += 1;
            }
        }
        for &z in &directions[2][..counts[2]] {
            for &y in &directions[1][..counts[1]] {
                for &x in &directions[0][..counts[0]] {
                    if x == 0 && y == 0 && z == 0 {
                        continue;
                    }
                    let changed_axes = (x != 0) as u8 + (y != 0) as u8 + (z != 0) as u8;
                    if self.marching_cubes_enabled || changed_axes == 1 {
                        self.mark_dirty(DiscretePoint {
                            x: chunk.x + x * CHUNK_WIDTH as DiscreteUnit,
                            y: chunk.y + y * CHUNK_WIDTH as DiscreteUnit,
                            z: chunk.z + z * CHUNK_WIDTH as DiscreteUnit,
                        });
                    }
                }
            }
        }
    }

    fn solid_columns(&self, chunk: DiscretePoint) -> Box<SolidColumns> {
        let mut solid = Box::new([[[0u64; CHUNK_WIDTH]; CHUNK_WIDTH]; 3]);
        let own = self.chunks.get(&chunk);
        for normal in Axis::ALL {
            let below = self.chunks.get(&Chunk::neighbour_of(chunk, normal, -1));
            let above = self.chunks.get(&Chunk::neighbour_of(chunk, normal, 1));
            let normal = normal as usize;
            for r in 0..CHUNK_WIDTH {
                for c in 0..CHUNK_WIDTH {
                    let column = |chunk: Option<&Box<Chunk>>| {
                        chunk.map_or(0, |chunk| chunk.columns[normal][r][c] as u64)
                    };
                    solid[normal][r][c] = column(own) << 1
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
                    VoxelQuad {
                        normal,
                        origin,
                        size,
                    }
                })
            })
        })
    }

    fn tesselate_chunk(&self, chunk: DiscretePoint) -> Vec<RealTriangle> {
        let voxel_size = self
            .config
            .as_ref()
            .expect("Tesselator must be configured before generating output geometry")
            .voxel_size as DiscreteUnit;
        if self.marching_cubes_enabled {
            return self.marching_cubes_chunk(chunk, voxel_size);
        }
        self.chunk_quads(chunk)
            .flat_map(|quad| quad.triangles())
            .map(|triangle| triangle.map(|coordinate| (coordinate * voxel_size) as RealUnit))
            .collect()
    }

    fn marching_cubes_chunk(
        &self,
        chunk: DiscretePoint,
        voxel_size: DiscreteUnit,
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
            self.chunks
                .get(&DiscretePoint {
                    x: chunk.x + x * CHUNK_WIDTH as DiscreteUnit,
                    y: chunk.y + y * CHUNK_WIDTH as DiscreteUnit,
                    z: chunk.z + z * CHUNK_WIDTH as DiscreteUnit,
                })
                .map(Box::as_ref)
        });
        let mut voxels = [0u8; MC_PADDED_GRID_VOLUME];
        let padded_coordinate = |chunk_offset: usize, local: usize| match chunk_offset {
            0 => (local == CHUNK_WIDTH - 1).then_some(0),
            1 => Some(local + 1),
            2 => (local == 0).then_some(MC_PADDED_GRID_SIZE - 1),
            _ => unreachable!(),
        };
        for (source_index, source) in source_chunks.into_iter().enumerate() {
            let Some(source) = source else {
                continue;
            };
            let chunk_x = source_index % 3;
            let chunk_y = source_index / 3 % 3;
            let chunk_z = source_index / 9;
            for (subchunk_index, &stored_mask) in source.subchunks.iter().enumerate() {
                let subchunk_x = subchunk_index % CHUNK_SUBCHUNKS.x;
                let subchunk_y = subchunk_index / CHUNK_SUBCHUNKS.x % CHUNK_SUBCHUNKS.y;
                let subchunk_z = subchunk_index / (CHUNK_SUBCHUNKS.x * CHUNK_SUBCHUNKS.y);
                let mut mask = stored_mask;
                for (axis, chunk_offset, subchunk_coordinate) in [
                    (0, chunk_x, subchunk_x),
                    (1, chunk_y, subchunk_y),
                    (2, chunk_z, subchunk_z),
                ] {
                    if chunk_offset == 1 {
                        continue;
                    }
                    let is_before = chunk_offset == 0;
                    let boundary_subchunk = if is_before {
                        CHUNK_SUBCHUNKS.get(Axis::ALL[axis]) - 1
                    } else {
                        0
                    };
                    if subchunk_coordinate != boundary_subchunk {
                        mask = 0;
                        break;
                    }
                    mask &= BOUNDARY_LAYERS[axis][is_before as usize];
                }
                while mask != 0 {
                    let bit = mask.trailing_zeros();
                    mask &= mask - 1;
                    let local = PointI::voxel_local(bit);
                    let voxel_x = subchunk_x * SUBCHUNK_SIZE.x + local.x;
                    let voxel_y = subchunk_y * SUBCHUNK_SIZE.y + local.y;
                    let voxel_z = subchunk_z * SUBCHUNK_SIZE.z + local.z;
                    let (Some(x), Some(y), Some(z)) = (
                        padded_coordinate(chunk_x, voxel_x),
                        padded_coordinate(chunk_y, voxel_y),
                        padded_coordinate(chunk_z, voxel_z),
                    ) else {
                        continue;
                    };
                    voxels[(z * MC_PADDED_GRID_SIZE + y) * MC_PADDED_GRID_SIZE + x] = 1;
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

fn polygonize_cube(
    case_index: usize,
    corners: &[[RealUnit; 3]; 8],
    values: [RealUnit; 8],
    voxel_size: DiscreteUnit,
    triangles: &mut Vec<RealTriangle>,
) {
    let lookup = &*MARCHING_CUBES_LOOKUP;
    let edge_mask = lookup.edge_masks[case_index];
    if edge_mask == 0 {
        return;
    }

    let mut edge_vertices = [[0.0; 3]; 12];
    for (edge, &[a, b]) in MC_CUBE_EDGES.iter().enumerate() {
        if edge_mask & (1 << edge) == 0 {
            continue;
        }
        let t = ((MC_ISO_LEVEL - values[a]) / (values[b] - values[a])).clamp(0.0, 1.0);
        edge_vertices[edge] = std::array::from_fn(|axis| {
            corners[a][axis] + (corners[b][axis] - corners[a][axis]) * t
        });
    }

    let scale = voxel_size as RealUnit;
    let mut edges = &lookup.triangle_edges[case_index][..];
    while let [a, b, c, rest @ ..] = edges {
        if *a < 0 {
            break;
        }
        let vertex = |edge: i8| edge_vertices[edge as usize].map(|coordinate| coordinate * scale);
        triangles.push(RealTriangle {
            v0: vertex(*a),
            v1: vertex(*b),
            v2: vertex(*c),
        });
        edges = rest;
    }
}

impl MarchingCubesLookup {
    fn build() -> Self {
        let mut lookup = Self {
            edge_masks: [0; 256],
            triangle_edges: [[-1; MC_TRIANGLE_TABLE_WIDTH]; 256],
        };
        for case_index in 0..256 {
            let (edge_mask, triangle_edges) = Self::build_case(case_index as u8);
            lookup.edge_masks[case_index] = edge_mask;
            lookup.triangle_edges[case_index] = triangle_edges;
        }
        lookup
    }

    fn build_case(case_index: u8) -> (u16, [i8; MC_TRIANGLE_TABLE_WIDTH]) {
        let mut edge_mask = 0u16;
        for (edge, [a, b]) in MC_CUBE_EDGES.iter().copied().enumerate() {
            if ((case_index >> a) & 1) != ((case_index >> b) & 1) {
                edge_mask |= 1 << edge;
            }
        }
        if edge_mask == 0 {
            return (edge_mask, [-1; MC_TRIANGLE_TABLE_WIDTH]);
        }

        let mut connections = [[0u8; 2]; 12];
        let mut connection_counts = [0u8; 12];
        let mut connect = |a: usize, b: usize| {
            let a_count = connection_counts[a] as usize;
            let b_count = connection_counts[b] as usize;
            if a_count < 2 && b_count < 2 {
                connections[a][a_count] = b as u8;
                connections[b][b_count] = a as u8;
                connection_counts[a] += 1;
                connection_counts[b] += 1;
            }
        };
        for (face_corners, face_edges) in MC_CUBE_FACES {
            let mut crossings = [0usize; 4];
            let mut crossing_count = 0;
            for edge in face_edges {
                if edge_mask & (1 << edge) != 0 {
                    crossings[crossing_count] = edge;
                    crossing_count += 1;
                }
            }
            match crossing_count {
                2 => connect(crossings[0], crossings[1]),
                4 => {
                    let inside_count = face_corners
                        .iter()
                        .filter(|&&corner| case_index & (1 << corner) != 0)
                        .count();
                    let face_inside = inside_count > 2;
                    for corner in 0..4 {
                        let corner_inside = case_index & (1 << face_corners[corner]) != 0;
                        if corner_inside != face_inside {
                            connect(face_edges[(corner + 3) % 4], face_edges[corner]);
                        }
                    }
                }
                _ => {}
            }
        }
        drop(connect);

        let inside_count = case_index.count_ones() as RealUnit;
        let outside_count = 8.0 - inside_count;
        let mut direction = [0.0; 3];
        for (corner, cube_corner) in MC_CUBE_CORNERS.iter().enumerate() {
            let sign = if case_index & (1 << corner) != 0 {
                -1.0 / inside_count
            } else {
                1.0 / outside_count
            };
            for axis in 0..3 {
                direction[axis] += cube_corner[axis] as RealUnit * sign;
            }
        }

        let edge_midpoint = |edge: usize| {
            let [a, b] = MC_CUBE_EDGES[edge];
            std::array::from_fn::<_, 3, _>(|axis| {
                (MC_CUBE_CORNERS[a][axis] + MC_CUBE_CORNERS[b][axis]) as RealUnit * 0.5
            })
        };
        let mut triangle_edges = [-1; MC_TRIANGLE_TABLE_WIDTH];
        let mut table_cursor = 0;
        let mut visited = [false; 12];
        for start in 0..12 {
            if edge_mask & (1 << start) == 0 || visited[start] {
                continue;
            }
            let mut polygon = [0usize; 12];
            let mut polygon_count = 0;
            let (mut previous, mut current) = (usize::MAX, start);
            loop {
                if visited[current] {
                    break;
                }
                visited[current] = true;
                polygon[polygon_count] = current;
                polygon_count += 1;
                if connection_counts[current] != 2 {
                    break;
                }
                let [first, second] = connections[current].map(usize::from);
                let next = if first == previous { second } else { first };
                previous = current;
                current = next;
                if current == start || polygon_count == 12 {
                    break;
                }
            }
            if polygon_count < 3 {
                continue;
            }

            let p0 = edge_midpoint(polygon[0]);
            let p1 = edge_midpoint(polygon[1]);
            let p2 = edge_midpoint(polygon[2]);
            let ab = std::array::from_fn::<_, 3, _>(|axis| p1[axis] - p0[axis]);
            let ac = std::array::from_fn::<_, 3, _>(|axis| p2[axis] - p0[axis]);
            let normal = [
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            ];
            let reverse =
                normal[0] * direction[0] + normal[1] * direction[1] + normal[2] * direction[2]
                    < 0.0;
            for index in 1..polygon_count - 1 {
                if table_cursor + 3 > MC_TRIANGLE_TABLE_WIDTH {
                    break;
                }
                triangle_edges[table_cursor] = polygon[0] as i8;
                triangle_edges[table_cursor + 1] =
                    polygon[if reverse { index + 1 } else { index }] as i8;
                triangle_edges[table_cursor + 2] =
                    polygon[if reverse { index } else { index + 1 }] as i8;
                table_cursor += 3;
            }
        }
        (edge_mask, triangle_edges)
    }
}

impl Tesselator for GreedyMeshTesselator {
    type Configuration = GreedyMeshTesselatorConfiguration;
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<TriangleChunk>;

    fn update_input_geometry(&mut self, geometry_delta: &Self::InputGeometryDelta) {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before updating input geometry"
        );
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
            self.mark_subchunk_dirty(chunk, local, previous ^ subchunk.i);
        }
    }

    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before generating output geometry"
        );
        if self.dirty_queue.is_empty() {
            return None;
        }
        self.in_queue.clear();
        let dirty: Vec<DiscretePoint> = self.dirty_queue.drain(..).collect();
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
        self.config = Some(configuration);
    }
}
