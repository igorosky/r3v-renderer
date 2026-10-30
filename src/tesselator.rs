use rustc_hash::{FxHashMap, FxHashSet};

#[path = "marching_cubes.rs"]
mod marching_cubes;
use marching_cubes::{MC_CUBE_CORNERS, MC_GRID_SIZE, polygonize_cube};

#[cfg(feature = "multi-threaded")]
use rayon::prelude::*;

use crate::{
    Tesselator,
    camera::Frustum,
    geometry::{
        Axis, DiscretePoint, DiscreteQuad, DiscreteUnit, Point, PointI, RealTriangle, RealUnit,
        TriangleChunk, Vector2D, Vector3D, VoxelNormal,
    },
};

pub(crate) const SUBCHUNK_SIZE: Vector3D<usize> = Vector3D { x: 4, y: 4, z: 2 }; // corresponds to input geometry delta `PointI` bitfield
const CHUNK_WIDTH: usize = 16; // in voxels
const CHUNK_MASK: u32 = u32::MAX >> (32 - CHUNK_WIDTH);
const CHUNK_SUBCHUNKS: Vector3D<usize> = Vector3D {
    x: CHUNK_WIDTH / SUBCHUNK_SIZE.x,
    y: CHUNK_WIDTH / SUBCHUNK_SIZE.y,
    z: CHUNK_WIDTH / SUBCHUNK_SIZE.z,
};
const CHUNK_RATIO: usize = CHUNK_SUBCHUNKS.x * CHUNK_SUBCHUNKS.y * CHUNK_SUBCHUNKS.z;
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

pub struct CpuTesselatorConfiguration {
    pub voxel_size: f32,
    pub marching_cubes_enabled: bool,
}

pub struct CpuTesselatorContext {
    pub camera_frustum: Option<Frustum>,
}

pub struct CpuTesselator {
    config: Option<CpuTesselatorConfiguration>,
    chunks: FxHashMap<DiscretePoint, Box<Chunk>>,
    dirty_set: FxHashSet<DiscretePoint>,
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
            chunks: FxHashMap::default(),
            dirty_set: FxHashSet::default(),
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
            for z in -1..=0 {
                for y in -1..=0 {
                    for x in -1..=0 {
                        if x == 0 && y == 0 && z == 0 {
                            continue;
                        }
                        if !stored.has_voxel_on_boundary([x as i8, y as i8, z as i8]) {
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
        self.dirty_set.insert(chunk);
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
            if !self.marching_cubes_enabled
                && subchunk[axis] == CHUNK_SUBCHUNKS[axis] - 1
                && changed & high_layer != 0
            {
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
        let curr: Option<&Box<Chunk>> = self.chunks.get(&chunk);
        for normal in Axis::ALL {
            let below = self.chunks.get(&Chunk::neighbour_of(chunk, normal, -1));
            let above = self.chunks.get(&Chunk::neighbour_of(chunk, normal, 1));
            let normal = normal as usize;
            for r in 0..CHUNK_WIDTH {
                for c in 0..CHUNK_WIDTH {
                    let column = |chunk: Option<&Box<Chunk>>| {
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
        let rows = self.marching_cubes_rows(chunk);
        let mut triangles = Vec::new();
        for z in 0..CHUNK_WIDTH {
            for y in 0..CHUNK_WIDTH {
                let quad = [
                    [rows[z][y], rows[z][y + 1]],
                    [rows[z + 1][y], rows[z + 1][y + 1]],
                ];
                let any = quad[0][0] | quad[0][1] | quad[1][0] | quad[1][1];
                let all = quad[0][0] & quad[0][1] & quad[1][0] & quad[1][1];
                let mut cubes = (any | any >> 1) & !(all & all >> 1) & CHUNK_MASK;
                while cubes != 0 {
                    let x = cubes.trailing_zeros() as usize;
                    cubes &= cubes - 1;
                    let mut case_index = 0usize;
                    for (corner, offset) in MC_CUBE_CORNERS.iter().enumerate() {
                        let solid = quad[offset[2]][offset[1]] >> (x + offset[0]) & 1;
                        case_index |= (solid as usize) << corner;
                    }

                    let corners = MC_CUBE_CORNERS.map(|offset| {
                        [
                            (chunk.x + (x + offset[0]) as DiscreteUnit) as RealUnit + 0.5,
                            (chunk.y + (y + offset[1]) as DiscreteUnit) as RealUnit + 0.5,
                            (chunk.z + (z + offset[2]) as DiscreteUnit) as RealUnit + 0.5,
                        ]
                    });
                    let values =
                        std::array::from_fn(|corner| (case_index >> corner & 1) as RealUnit);
                    polygonize_cube(case_index, &corners, values, voxel_size, &mut triangles);
                }
            }
        }
        triangles
    }

    fn marching_cubes_rows(&self, chunk: DiscretePoint) -> [[u32; MC_GRID_SIZE]; MC_GRID_SIZE] {
        let mut rows = [[0u32; MC_GRID_SIZE]; MC_GRID_SIZE];
        for source_index in 0..8 {
            let offset = Point {
                x: source_index & 1,
                y: source_index >> 1 & 1,
                z: source_index >> 2,
            };
            let Some(source) = self.chunks.get(&DiscretePoint {
                x: chunk.x + (offset.x * CHUNK_WIDTH) as DiscreteUnit,
                y: chunk.y + (offset.y * CHUNK_WIDTH) as DiscreteUnit,
                z: chunk.z + (offset.z * CHUNK_WIDTH) as DiscreteUnit,
            }) else {
                continue;
            };
            let columns = &source.columns[Axis::X as usize];
            let range = |offset: usize| 0..if offset == 0 { CHUNK_WIDTH } else { 1 };
            for z in range(offset.z) {
                for y in range(offset.y) {
                    let bits = columns[z][y];
                    rows[offset.z * CHUNK_WIDTH + z][offset.y * CHUNK_WIDTH + y] |= if offset.x == 0
                    {
                        bits
                    } else {
                        (bits & 1) << CHUNK_WIDTH
                    };
                }
            }
        }
        rows
    }
}

impl Tesselator for CpuTesselator {
    type Configuration = CpuTesselatorConfiguration;
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<TriangleChunk>;
    type TesselationContext = CpuTesselatorContext;

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

    fn get_output_geometry_delta(
        &mut self,
        context: &Self::TesselationContext,
    ) -> Option<Self::OutputGeometryDelta> {
        assert!(
            self.config.is_some(),
            "Tesselator must be configured before generating output geometry"
        );
        if self.dirty_set.is_empty() {
            return None;
        }
        let voxel_size = self.config.as_ref().unwrap().voxel_size;
        let chunk_extent = CHUNK_WIDTH as f32 * voxel_size;
        let dirty: Vec<DiscretePoint> = self
            .dirty_set
            .extract_if(|chunk| {
                let min = [chunk.x, chunk.y, chunk.z].map(|c| c as f32 * voxel_size);
                let max = min.map(|c| c + chunk_extent);
                context
                    .camera_frustum
                    .is_none_or(|frustum| frustum.intersects_box(min, max))
            })
            .collect();
        if dirty.is_empty() {
            return None;
        }
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
