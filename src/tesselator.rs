use std::collections::VecDeque;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::{
    Tesselator, geometry::{
        Axis, DiscretePoint, DiscreteQuad, DiscreteUnit, Point, PointI, RealTriangle, RealUnit, TriangleChunk, Vector2D, Vector3D,
        VoxelNormal, VoxelQuad,
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
    [layer_mask(Axis::X, 0), layer_mask(Axis::X, SUBCHUNK_SIZE.get(Axis::X) - 1)],
    [layer_mask(Axis::Y, 0), layer_mask(Axis::Y, SUBCHUNK_SIZE.get(Axis::Y) - 1)],
    [layer_mask(Axis::Z, 0), layer_mask(Axis::Z, SUBCHUNK_SIZE.get(Axis::Z) - 1)],
];

struct Chunk {
    subchunks: [u32; CHUNK_RATIO],
    columns: Columns<u32>,
    solid_subchunks: usize,
}

impl Chunk {
    fn new() -> Box<Self> {
        Box::new(Self { subchunks: [0; CHUNK_RATIO], columns: [[[0; CHUNK_WIDTH]; CHUNK_WIDTH]; 3], solid_subchunks: 0 })
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

    fn set_subchunk_mask(&mut self, subchunk: Point<usize>, mask: u32) {
        let previous = std::mem::replace(&mut self.subchunks[Self::subchunk_index(subchunk)], mask);
        self.solid_subchunks = self.solid_subchunks + (mask != 0) as usize - (previous != 0) as usize;
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
                    let voxels = &mut self.columns[normal as usize][origin[row] + r][origin[column] + c];
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
}

impl GreedyMeshTesselator {
    pub fn new() -> Self {
        Self {
            config: None,
            chunks: FxHashMap::default(),
            dirty_queue: VecDeque::new(),
            in_queue: FxHashSet::default(),
        }
    }

    fn mark_dirty(&mut self, chunk: DiscretePoint) {
        if self.in_queue.insert(chunk) {
            self.dirty_queue.push_back(chunk);
        }
    }

    fn mark_subchunk_dirty(&mut self, chunk: DiscretePoint, subchunk: Point<usize>, changed: u32) {
        self.mark_dirty(chunk);
        for axis in Axis::ALL {
            let [low_layer, high_layer] = BOUNDARY_LAYERS[axis as usize];
            if subchunk[axis] == 0 && changed & low_layer != 0 {
                self.mark_dirty(Chunk::neighbour_of(chunk, axis, -1));
            }
            if subchunk[axis] == CHUNK_SUBCHUNKS[axis] - 1 && changed & high_layer != 0 {
                self.mark_dirty(Chunk::neighbour_of(chunk, axis, 1));
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
                    let column = |chunk: Option<&Box<Chunk>>| chunk.map_or(0, |chunk| chunk.columns[normal][r][c] as u64);
                    solid[normal][r][c] = column(own) << 1
                        | column(below) >> (CHUNK_WIDTH - 1) & 1
                        | (column(above) & 1) << (CHUNK_WIDTH + 1);
                }
            }
        }
        solid
    }

    fn greedy_rectangles(mut rows: [u32; CHUNK_WIDTH]) -> impl Iterator<Item = (Vector2D<DiscreteUnit>, Vector2D<DiscreteUnit>)> {
        let mut r = 0;
        std::iter::from_fn(move || {
            while rows.get(r)? == &0 {
                r += 1;
            }
            let start = rows[r].trailing_zeros();
            let width = (rows[r] >> start).trailing_ones();
            let run = (((1u64 << width) - 1) << start) as u32;
            let height = rows[r..].iter().take_while(|&&row| row & run == run).count();
            rows[r..r + height].iter_mut().for_each(|row| *row &= !run);
            Some((
                Vector2D { x: start as DiscreteUnit, y: r as DiscreteUnit },
                Vector2D { x: width as DiscreteUnit, y: height as DiscreteUnit },
            ))
        })
    }

    fn chunk_quads(&self, chunk: DiscretePoint) -> impl Iterator<Item = DiscreteQuad> {
        let solid = self.solid_columns(chunk);
        VoxelNormal::ALL.into_iter().flat_map(move |normal| {
            let axis = normal.axis();
            let (column, row) = axis.plane();
            let mut planes = [[0u32; CHUNK_WIDTH]; CHUNK_WIDTH];
            for (r, columns) in solid[axis as usize].iter().enumerate() {
                for (c, &voxels) in columns.iter().enumerate() {
                    let exposed = if normal.is_positive() {
                        voxels & !(voxels >> 1)
                    } else {
                        voxels & !(voxels << 1)
                    };
                    let mut faces = (exposed >> 1) as u32 & CHUNK_MASK;
                    while faces != 0 {
                        planes[faces.trailing_zeros() as usize][r] |= 1 << c;
                        faces &= faces - 1;
                    }
                }
            }
            planes.into_iter().enumerate().flat_map(move |(layer, rows)| {
                Self::greedy_rectangles(rows).map(move |(start, size)| {
                    let mut origin = chunk;
                    origin[axis] += layer as DiscreteUnit;
                    origin[column] += start.x;
                    origin[row] += start.y;
                    VoxelQuad { normal, origin, size }
                })
            })
        })
    }

    fn tesselate_chunk(&self, chunk: DiscretePoint) -> Vec<RealTriangle> {
        let voxel_size = self.config.as_ref().expect("Tesselator must be configured before generating output geometry").voxel_size as DiscreteUnit;
        self.chunk_quads(chunk)
            .flat_map(|quad| quad.triangles())
            .map(|triangle| triangle.map(|coordinate| (coordinate * voxel_size) as RealUnit))
            .collect()
    }
}

impl Tesselator for GreedyMeshTesselator {
    type Configuration = GreedyMeshTesselatorConfiguration;
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<TriangleChunk>;

    fn update_input_geometry(
        &mut self, 
        geometry_delta: &Self::InputGeometryDelta
    ) {
        assert!(self.config.is_some(), "Tesselator must be configured before updating input geometry");
        for subchunk in geometry_delta {
            let origin = DiscretePoint { x: subchunk.x, y: subchunk.y, z: subchunk.z };
            debug_assert!(
                Axis::ALL.iter().all(|&axis| origin[axis].rem_euclid(SUBCHUNK_SIZE[axis] as DiscreteUnit) == 0),
                "Subchunk origin {:?} is not aligned to {:?}", origin, SUBCHUNK_SIZE,
            );
            let chunk = Chunk::origin_of(origin);
            let local = Point::from_fn(|axis| (origin[axis] - chunk[axis]) as usize / SUBCHUNK_SIZE[axis]);
            let previous = self.chunks.get(&chunk).map_or(0, |stored| stored.subchunk_mask(local));
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
        assert!(self.config.is_some(), "Tesselator must be configured before generating output geometry");
        if self.dirty_queue.is_empty() {
            return None;
        }
        let mut output_delta = Vec::with_capacity(self.dirty_queue.len());
        while let Some(chunk) = self.dirty_queue.pop_front() {
            self.in_queue.remove(&chunk);
            let triangles = self.tesselate_chunk(chunk);
            output_delta.push(TriangleChunk { index: chunk, triangles });
        }
        Some(output_delta)
    }
    
    fn configure(&mut self, configuration: Self::Configuration) {
        self.config = Some(configuration);
    }
}
