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

struct FaceDirection {
    vertices: [[i32; 3]; 6],
    normal: usize,
    column: usize,
    row: usize,
    planes: u32,
}

const FACE_DIRECTIONS: [FaceDirection; 6] = [
    FaceDirection { vertices: FACE_RIGHT,  normal: AXIS_X, column: AXIS_Y, row: AXIS_Z, planes: 4 },
    FaceDirection { vertices: FACE_LEFT,   normal: AXIS_X, column: AXIS_Y, row: AXIS_Z, planes: 4 },
    FaceDirection { vertices: FACE_TOP,    normal: AXIS_Y, column: AXIS_X, row: AXIS_Z, planes: 4 },
    FaceDirection { vertices: FACE_BOTTOM, normal: AXIS_Y, column: AXIS_X, row: AXIS_Z, planes: 4 },
    FaceDirection { vertices: FACE_FRONT,  normal: AXIS_Z, column: AXIS_X, row: AXIS_Y, planes: 2 },
    FaceDirection { vertices: FACE_BACK,   normal: AXIS_Z, column: AXIS_X, row: AXIS_Y, planes: 2 },
];

#[derive(Debug, Clone, Copy)]
struct Quad {
    direction: usize,
    origin: [i32; 3],
    size: [i32; 3],
}

struct VoxelChunk {
    pub index: DiscretePoint,
    pub mask: u32,
}

pub struct GreedyMeshTesselator {
    is_voxel_data_dirty: bool,
    chunks: HashMap<DiscretePoint, VoxelChunk>,
    dirty_queue: VecDeque<DiscretePoint>,
    in_queue: HashSet<DiscretePoint>,
}

impl GreedyMeshTesselator {
    pub fn new() -> Self {
        Self {
            is_voxel_data_dirty: true,
            chunks: HashMap::new(),
            dirty_queue: VecDeque::new(),
            in_queue: HashSet::new(),
        }
    }

    fn mark_dirty(&mut self, index: DiscretePoint) {
        if self.in_queue.insert(index) {
            self.dirty_queue.push_back(index);
        }
    }

    fn face_masks(mask: u32) -> [u32; 6] {
        [
            mask & !((mask >> 1)  & 0x77777777), // +X
            mask & !((mask << 1)  & 0xEEEEEEEE), // -X
            mask & !((mask >> 4)  & 0x0FFF0FFF), // +Y
            mask & !((mask << 4)  & 0xFFF0FFF0), // -Y
            mask & !((mask >> 16) & 0x0000FFFF), // +Z
            mask & !((mask << 16) & 0xFFFF0000), // -Z
        ]
    }

    fn plane_rows(faces: u32, normal: usize, plane: u32) -> [u8; 4] {
        let mut rows = [0u8; 4];
        match normal {
            AXIS_Z => for y in 0..4 {
                rows[y as usize] = ((faces >> (16 * plane + 4 * y)) & 0xF) as u8;
            },
            AXIS_Y => for z in 0..2 {
                rows[z as usize] = ((faces >> (16 * z + 4 * plane)) & 0xF) as u8;
            },
            _ => for z in 0..2 {
                let spread = (faces >> (16 * z + plane)) & 0x1111;
                rows[z as usize] = (((spread * 0x1248) >> 12) & 0xF) as u8;
            },
        }
        rows
    }

    fn greedy_rectangles(rows: &mut [u8; 4], mut emit: impl FnMut(i32, i32, i32, i32)) {
        for r in 0..rows.len() {
            while rows[r] != 0 {
                let start = rows[r].trailing_zeros();
                let width = (rows[r] >> start).trailing_ones();
                let run = (((1u16 << width) - 1) << start) as u8;
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

    fn greedy_quads(mask: u32) -> Vec<Quad> {
        let mut quads = Vec::new();
        for (direction, faces) in Self::face_masks(mask).into_iter().enumerate() {
            if faces == 0 {
                continue;
            }
            let FaceDirection { normal, column, row, planes, .. } = FACE_DIRECTIONS[direction];
            for plane in 0..planes {
                let mut rows = Self::plane_rows(faces, normal, plane);
                Self::greedy_rectangles(&mut rows, |c, r, width, height| {
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

    fn tesselate_chunk(chunk: &VoxelChunk) -> Vec<RealTriangle> {
        let chunk_origin = [chunk.index.x, chunk.index.y, chunk.index.z];
        let quads = Self::greedy_quads(chunk.mask);
        let mut chunk_triangles = Vec::with_capacity(quads.len() * 2);
        for Quad { direction, origin, size } in quads {
            let corner = |v: [i32; 3]| -> [f32; 3] {
                std::array::from_fn(|axis| (chunk_origin[axis] + origin[axis] + v[axis] * size[axis]) as f32)
            };
            for tri in FACE_DIRECTIONS[direction].vertices.chunks_exact(3) {
                chunk_triangles.push(RealTriangle {
                    v0: corner(tri[0]),
                    v1: corner(tri[1]),
                    v2: corner(tri[2]),
                });
            }
        }
        chunk_triangles
    }
}

impl Tesselator for GreedyMeshTesselator {
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<TriangleChunk>;

    fn update_input_geometry(
        &mut self, 
        geometry_delta: &Self::InputGeometryDelta
    ) {
        geometry_delta.iter().for_each(|chunk| {
            let index = DiscretePoint { x: chunk.x, y: chunk.y, z: chunk.z };
            match self.chunks.get_mut(&index) {
                Some(existing_chunk) => {
                    if existing_chunk.mask != chunk.i {
                        existing_chunk.mask = chunk.i;
                        self.mark_dirty(index);
                    }
                },
                None => {
                    self.chunks.insert(index, VoxelChunk { index, mask: chunk.i });
                    self.mark_dirty(index);
                }
            }
        });
    }

    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        if !self.is_voxel_data_dirty {
            return None;
        }
        self.is_voxel_data_dirty = false;
        let mut output_delta = Vec::new();
        while let Some(index) = self.dirty_queue.pop_front() {
            self.in_queue.remove(&index);
            if let Some(chunk) = self.chunks.get_mut(&index) {
                let triangles = Self::tesselate_chunk(chunk);
                output_delta.push(TriangleChunk { index, triangles });
            }
        }
        if output_delta.is_empty() {
            None
        } else {
            Some(output_delta)
        }
    }
}
