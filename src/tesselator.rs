use std::collections::VecDeque;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::{
    Tesselator, geometry::{
        DiscretePoint, PointI, RealTriangle, TriangleChunk,
    },
};

const FACE_FRONT:   [[i32; 3]; 6] = [[0,0,1], [1,0,1], [1,1,1], [0,0,1], [1,1,1], [0,1,1]]; // +Z
const FACE_BACK:    [[i32; 3]; 6] = [[1,0,0], [0,0,0], [0,1,0], [1,0,0], [0,1,0], [1,1,0]]; // -Z
const FACE_TOP:     [[i32; 3]; 6] = [[0,1,1], [1,1,1], [1,1,0], [0,1,1], [1,1,0], [0,1,0]]; // +Y
const FACE_BOTTOM:  [[i32; 3]; 6] = [[0,0,0], [1,0,0], [1,0,1], [0,0,0], [1,0,1], [0,0,1]]; // -Y
const FACE_RIGHT:   [[i32; 3]; 6] = [[1,0,1], [1,0,0], [1,1,0], [1,0,1], [1,1,0], [1,1,1]]; // +X
const FACE_LEFT:    [[i32; 3]; 6] = [[0,0,0], [0,0,1], [0,1,1], [0,0,0], [0,1,1], [0,1,0]]; // -X

const AXIS_X: usize = 0;
const AXIS_Y: usize = 1;
const AXIS_Z: usize = 2;

const BRICK_SIZE: [i32; 3] = [4, 4, 2];
const REGION_SIZE: i32 = 16;
const REGION_VOXELS: usize = REGION_SIZE as usize;
const REGION_MASK: u32 = u32::MAX >> (32 - REGION_SIZE);
const REGION_BRICKS: [usize; 3] = [
    (REGION_SIZE / BRICK_SIZE[0]) as usize,
    (REGION_SIZE / BRICK_SIZE[1]) as usize,
    (REGION_SIZE / BRICK_SIZE[2]) as usize,
];
const BRICKS_PER_REGION: usize = REGION_BRICKS[0] * REGION_BRICKS[1] * REGION_BRICKS[2];

struct FaceDirection {
    vertices: [[i32; 3]; 6],
    normal: usize,
    column: usize,
    row: usize,
}

const FACE_DIRECTIONS: [FaceDirection; 6] = [
    FaceDirection { vertices: FACE_RIGHT,  normal: AXIS_X, column: AXIS_Y, row: AXIS_Z },
    FaceDirection { vertices: FACE_LEFT,   normal: AXIS_X, column: AXIS_Y, row: AXIS_Z },
    FaceDirection { vertices: FACE_TOP,    normal: AXIS_Y, column: AXIS_X, row: AXIS_Z },
    FaceDirection { vertices: FACE_BOTTOM, normal: AXIS_Y, column: AXIS_X, row: AXIS_Z },
    FaceDirection { vertices: FACE_FRONT,  normal: AXIS_Z, column: AXIS_X, row: AXIS_Y },
    FaceDirection { vertices: FACE_BACK,   normal: AXIS_Z, column: AXIS_X, row: AXIS_Y },
];

const PLANE_AXES: [(usize, usize); 3] = [(AXIS_Y, AXIS_Z), (AXIS_X, AXIS_Z), (AXIS_X, AXIS_Y)];

#[derive(Debug, Clone, Copy)]
struct Quad {
    direction: usize,
    origin: [i32; 3],
    size: [i32; 3],
}

type Columns<T> = [[[T; REGION_VOXELS]; REGION_VOXELS]; 3];
type SolidColumns = Columns<u64>;

const fn voxel_bit(local: [i32; 3]) -> u32 {
    (local[0] | local[1] << 2 | local[2] << 4) as u32
}

const fn layer_mask(axis: usize, layer: i32) -> u32 {
    let mut mask = 0;
    let mut bit = 0;
    while bit < 32 {
        let local = [bit & 0b11, (bit >> 2) & 0b11, (bit >> 4) & 0b1];
        if local[axis] == layer {
            mask |= 1 << bit;
        }
        bit += 1;
    }
    mask
}

const BOUNDARY_LAYERS: [[u32; 2]; 3] = [
    [layer_mask(AXIS_X, 0), layer_mask(AXIS_X, BRICK_SIZE[AXIS_X] - 1)],
    [layer_mask(AXIS_Y, 0), layer_mask(AXIS_Y, BRICK_SIZE[AXIS_Y] - 1)],
    [layer_mask(AXIS_Z, 0), layer_mask(AXIS_Z, BRICK_SIZE[AXIS_Z] - 1)],
];

struct Region {
    bricks: [u32; BRICKS_PER_REGION],
    columns: Columns<u32>,
    solid_bricks: usize,
}

impl Region {
    fn new() -> Box<Self> {
        Box::new(Self { bricks: [0; BRICKS_PER_REGION], columns: [[[0; REGION_VOXELS]; REGION_VOXELS]; 3], solid_bricks: 0 })
    }

    fn brick_index(brick: [usize; 3]) -> usize {
        (brick[2] * REGION_BRICKS[1] + brick[1]) * REGION_BRICKS[0] + brick[0]
    }

    fn brick(&self, brick: [usize; 3]) -> u32 {
        self.bricks[Self::brick_index(brick)]
    }

    fn set_brick(&mut self, brick: [usize; 3], mask: u32) {
        let previous = std::mem::replace(&mut self.bricks[Self::brick_index(brick)], mask);
        self.solid_bricks = self.solid_bricks + (mask != 0) as usize - (previous != 0) as usize;
        let origin: [usize; 3] = std::array::from_fn(|axis| brick[axis] * BRICK_SIZE[axis] as usize);
        for (normal, (column, row)) in PLANE_AXES.into_iter().enumerate() {
            let run = (1u32 << BRICK_SIZE[normal]) - 1;
            for r in 0..BRICK_SIZE[row] {
                for c in 0..BRICK_SIZE[column] {
                    let mut local = [0; 3];
                    local[column] = c;
                    local[row] = r;
                    let mut bits = 0;
                    for n in 0..BRICK_SIZE[normal] {
                        local[normal] = n;
                        bits |= (mask >> voxel_bit(local) & 1) << n;
                    }
                    let voxels = &mut self.columns[normal][origin[row] + r as usize][origin[column] + c as usize];
                    *voxels = *voxels & !(run << origin[normal]) | bits << origin[normal];
                }
            }
        }
    }
}

pub struct GreedyMeshTesselator {
    regions: FxHashMap<DiscretePoint, Box<Region>>,
    dirty_queue: VecDeque<DiscretePoint>,
    in_queue: FxHashSet<DiscretePoint>,
    voxel_size: u16,
}

fn region_of(brick: [i32; 3]) -> [i32; 3] {
    brick.map(|c| c.div_euclid(REGION_SIZE) * REGION_SIZE)
}

impl GreedyMeshTesselator {
    pub fn new(voxel_size: u16) -> Self {
        Self {
            regions: FxHashMap::default(),
            dirty_queue: VecDeque::new(),
            in_queue: FxHashSet::default(),
            voxel_size
        }
    }

    fn mark_dirty(&mut self, region: DiscretePoint) {
        if self.in_queue.insert(region) {
            self.dirty_queue.push_back(region);
        }
    }

    fn mark_brick_dirty(&mut self, region: [i32; 3], brick: [i32; 3], changed: u32) {
        self.mark_dirty(DiscretePoint::from_array(region));
        for axis in 0..3 {
            let [low_layer, high_layer] = BOUNDARY_LAYERS[axis];
            let mut neighbour = region;
            if brick[axis] == region[axis] && changed & low_layer != 0 {
                neighbour[axis] -= REGION_SIZE;
                self.mark_dirty(DiscretePoint::from_array(neighbour));
            }
            neighbour = region;
            if brick[axis] + BRICK_SIZE[axis] == region[axis] + REGION_SIZE && changed & high_layer != 0 {
                neighbour[axis] += REGION_SIZE;
                self.mark_dirty(DiscretePoint::from_array(neighbour));
            }
        }
    }

    fn solid_columns(&self, region: [i32; 3]) -> Box<SolidColumns> {
        let mut solid = Box::new([[[0u64; REGION_VOXELS]; REGION_VOXELS]; 3]);
        let own = self.regions.get(&DiscretePoint::from_array(region));
        for normal in 0..3 {
            let neighbour = |offset: i32| {
                let mut origin = region;
                origin[normal] += offset * REGION_SIZE;
                self.regions.get(&DiscretePoint::from_array(origin))
            };
            let (below, above) = (neighbour(-1), neighbour(1));
            for r in 0..REGION_VOXELS {
                for c in 0..REGION_VOXELS {
                    let column = |region: Option<&Box<Region>>| region.map_or(0, |region| region.columns[normal][r][c] as u64);
                    solid[normal][r][c] = column(own) << 1
                        | column(below) >> (REGION_VOXELS - 1) & 1
                        | (column(above) & 1) << (REGION_VOXELS + 1);
                }
            }
        }
        solid
    }

    fn greedy_rectangles(rows: &mut [u32], mut emit: impl FnMut(i32, i32, i32, i32)) {
        for r in 0..rows.len() {
            while rows[r] != 0 {
                let start = rows[r].trailing_zeros();
                let width = (rows[r] >> start).trailing_ones();
                let run = (((1u64 << width) - 1) << start) as u32;
                let mut height = 1;
                while r + height < rows.len() && rows[r + height] & run == run {
                    rows[r + height] &= !run;
                    height += 1;
                }
                rows[r] &= !run;
                emit(start as i32, r as i32, width as i32, height as i32);
            }
        }
    }

    fn region_quads(&self, region: DiscretePoint) -> Vec<Quad> {
        let solid = self.solid_columns(region.to_array());
        let mut quads = Vec::new();
        let mut planes = [[0u32; REGION_SIZE as usize]; REGION_SIZE as usize];
        for (direction, &FaceDirection { normal, column, row, .. }) in FACE_DIRECTIONS.iter().enumerate() {
            let positive = direction % 2 == 0;
            for r in 0..REGION_SIZE as usize {
                for c in 0..REGION_SIZE as usize {
                    let voxels = solid[normal][r][c];
                    let faces = if positive { voxels & !(voxels >> 1) } else { voxels & !(voxels << 1) };
                    let mut faces = (faces >> 1) as u32 & REGION_MASK;
                    while faces != 0 {
                        let plane = faces.trailing_zeros() as usize;
                        faces &= faces - 1;
                        planes[plane][r] |= 1 << c;
                    }
                }
            }
            for (plane, rows) in planes.iter_mut().enumerate() {
                Self::greedy_rectangles(rows, |c, r, width, height| {
                    let mut origin = [0; 3];
                    origin[normal] = plane as i32;
                    origin[column] = c;
                    origin[row] = r;
                    let mut size = [1; 3];
                    size[column] = width;
                    size[row] = height;
                    quads.push(Quad { direction, origin, size });
                });
            }
        }
        quads
    }

    fn tesselate_region(&self, region: DiscretePoint) -> Vec<RealTriangle> {
        let region_origin = region.to_array();
        let quads = self.region_quads(region);
        let mut triangles = Vec::with_capacity(quads.len() * 2);
        for Quad { direction, origin, size } in quads {
            let corner = |v: [i32; 3]| -> [f32; 3] {
                std::array::from_fn(|axis| ((region_origin[axis] + origin[axis] + v[axis] * size[axis]) * self.voxel_size as i32) as f32)
            };
            for tri in FACE_DIRECTIONS[direction].vertices.chunks_exact(3) {
                triangles.push(RealTriangle {
                    v0: corner(tri[0]),
                    v1: corner(tri[1]),
                    v2: corner(tri[2]),
                });
            }
        }
        triangles
    }
}

impl Tesselator for GreedyMeshTesselator {
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<TriangleChunk>;

    fn update_input_geometry(
        &mut self, 
        geometry_delta: &Self::InputGeometryDelta
    ) {
        for brick in geometry_delta {
            let origin = [brick.x, brick.y, brick.z];
            debug_assert!(
                (0..3).all(|axis| origin[axis].rem_euclid(BRICK_SIZE[axis]) == 0),
                "Brick origin {:?} is not aligned to {:?}", origin, BRICK_SIZE,
            );
            let region = region_of(origin);
            let key = DiscretePoint::from_array(region);
            let local = std::array::from_fn(|axis| ((origin[axis] - region[axis]) / BRICK_SIZE[axis]) as usize);
            let previous = self.regions.get(&key).map_or(0, |region| region.brick(local));
            if previous == brick.i {
                continue;
            }
            let stored = self.regions.entry(key).or_insert_with(Region::new);
            stored.set_brick(local, brick.i);
            if stored.solid_bricks == 0 {
                self.regions.remove(&key);
            }
            self.mark_brick_dirty(region, origin, previous ^ brick.i);
        }
    }

    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        if self.dirty_queue.is_empty() {
            return None;
        }
        let mut output_delta = Vec::with_capacity(self.dirty_queue.len());
        while let Some(region) = self.dirty_queue.pop_front() {
            self.in_queue.remove(&region);
            let triangles = self.tesselate_region(region);
            output_delta.push(TriangleChunk { index: region, triangles });
        }
        Some(output_delta)
    }
}
