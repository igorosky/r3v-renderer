use std::collections::{HashMap, HashSet, VecDeque};

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
const REGION_SIZE: i32 = 32;
const PADDED_SIZE: usize = REGION_SIZE as usize + 2;

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

type SolidColumns = [[[u64; PADDED_SIZE]; PADDED_SIZE]; 3];

pub struct GreedyMeshTesselator {
    bricks: HashMap<DiscretePoint, u32>,
    dirty_queue: VecDeque<DiscretePoint>,
    in_queue: HashSet<DiscretePoint>,
}

fn region_of(brick: DiscretePoint) -> DiscretePoint {
    DiscretePoint::from_array(brick.to_array().map(|c| c.div_euclid(REGION_SIZE) * REGION_SIZE))
}

impl GreedyMeshTesselator {
    pub fn new() -> Self {
        Self {
            bricks: HashMap::new(),
            dirty_queue: VecDeque::new(),
            in_queue: HashSet::new(),
        }
    }

    fn mark_dirty(&mut self, region: DiscretePoint) {
        if self.in_queue.insert(region) {
            self.dirty_queue.push_back(region);
        }
    }

    fn mark_brick_dirty(&mut self, brick: DiscretePoint) {
        let region = region_of(brick);
        self.mark_dirty(region);
        let (brick, region_coords) = (brick.to_array(), region.to_array());
        for axis in 0..3 {
            let mut neighbour = region_coords;
            if brick[axis] == region_coords[axis] {
                neighbour[axis] -= REGION_SIZE;
                self.mark_dirty(DiscretePoint::from_array(neighbour));
            }
            neighbour = region_coords;
            if brick[axis] + BRICK_SIZE[axis] == region_coords[axis] + REGION_SIZE {
                neighbour[axis] += REGION_SIZE;
                self.mark_dirty(DiscretePoint::from_array(neighbour));
            }
        }
    }

    fn solid_columns(&self, region: [i32; 3]) -> Box<SolidColumns> {
        let mut solid = Box::new([[[0u64; PADDED_SIZE]; PADDED_SIZE]; 3]);
        let first = region.map(|c| c - 1);
        let mut brick = [0; 3];
        for bz in 0..(REGION_SIZE / BRICK_SIZE[2] + 2) {
            for by in 0..(REGION_SIZE / BRICK_SIZE[1] + 2) {
                for bx in 0..(REGION_SIZE / BRICK_SIZE[0] + 2) {
                    for (axis, b) in [bx, by, bz].into_iter().enumerate() {
                        brick[axis] = region[axis] + (b - 1) * BRICK_SIZE[axis];
                    }
                    let Some(&mask) = self.bricks.get(&DiscretePoint::from_array(brick)) else {
                        continue;
                    };
                    let mut bits = mask;
                    while bits != 0 {
                        let bit = bits.trailing_zeros() as i32;
                        bits &= bits - 1;
                        let local = [bit & 0b11, (bit >> 2) & 0b11, (bit >> 4) & 0b1];
                        let padded: [i32; 3] = std::array::from_fn(|axis| brick[axis] + local[axis] - first[axis]);
                        if padded.iter().any(|&p| p < 0 || p >= PADDED_SIZE as i32) {
                            continue;
                        }
                        for (normal, (column, row)) in PLANE_AXES.into_iter().enumerate() {
                            solid[normal][padded[row] as usize][padded[column] as usize] |= 1 << padded[normal];
                        }
                    }
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
                    let voxels = solid[normal][r + 1][c + 1];
                    let faces = if positive { voxels & !(voxels >> 1) } else { voxels & !(voxels << 1) };
                    let mut faces = (faces >> 1) as u32;
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
                std::array::from_fn(|axis| (region_origin[axis] + origin[axis] + v[axis] * size[axis]) as f32)
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
            let index = DiscretePoint { x: brick.x, y: brick.y, z: brick.z };
            debug_assert!(
                (0..3).all(|axis| index.to_array()[axis].rem_euclid(BRICK_SIZE[axis]) == 0),
                "Brick origin {:?} is not aligned to {:?}", index, BRICK_SIZE,
            );
            let previous = self.bricks.get(&index).copied().unwrap_or(0);
            if previous == brick.i {
                continue;
            }
            if brick.i == 0 {
                self.bricks.remove(&index);
            } else {
                self.bricks.insert(index, brick.i);
            }
            self.mark_brick_dirty(index);
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
