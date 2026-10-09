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
    
    fn voxel_world_origin(chunk_origin: &DiscretePoint, voxel_index: u32) -> DiscretePoint {
        let lx = (voxel_index & 0b11) as i32;
        let ly = ((voxel_index >> 2) & 0b11) as i32;
        let lz = ((voxel_index >> 4) & 0b1) as i32;
        DiscretePoint {
            x: chunk_origin.x + lx,
            y: chunk_origin.y + ly,
            z: chunk_origin.z + lz,
        }
    }

    fn tesselate_chunk(chunk: &VoxelChunk) -> Vec<RealTriangle> {
        let chunk_origin = chunk.index;
        let chunk_triangles = &mut Vec::new();

        let mask = chunk.mask;
        let right_faces  = mask & !((mask >> 1)  & 0x77777777);
        let left_faces   = mask & !((mask << 1)  & 0xEEEEEEEE);
        let top_faces    = mask & !((mask >> 4)  & 0x0FFF0FFF);
        let bottom_faces = mask & !((mask << 4)  & 0xFFF0FFF0);
        let front_faces  = mask & !((mask >> 16) & 0x0000FFFF);
        let back_faces   = mask & !((mask << 16) & 0xFFFF0000);

        for bit in 0..32 {
            let bit_mask = 1 << bit;
            if (mask & bit_mask) == 0 {
                continue;
            }
            let voxel_origin = Self::voxel_world_origin(&chunk_origin, bit);
            let mut push_face = |face_data: &[[i32; 3]; 6]| {
                for tri in face_data.chunks_exact(3) {
                    chunk_triangles.push(RealTriangle {
                        v0: [(voxel_origin.x + tri[0][0]) as f32, (voxel_origin.y + tri[0][1]) as f32, (voxel_origin.z + tri[0][2]) as f32],
                        v1: [(voxel_origin.x + tri[1][0]) as f32, (voxel_origin.y + tri[1][1]) as f32, (voxel_origin.z + tri[1][2]) as f32],
                        v2: [(voxel_origin.x + tri[2][0]) as f32, (voxel_origin.y + tri[2][1]) as f32, (voxel_origin.z + tri[2][2]) as f32],
                    });
                }
            };
            if (right_faces  & bit_mask) != 0 { push_face(&FACE_RIGHT);  }
            if (left_faces   & bit_mask) != 0 { push_face(&FACE_LEFT);   }
            if (top_faces    & bit_mask) != 0 { push_face(&FACE_TOP);    }
            if (bottom_faces & bit_mask) != 0 { push_face(&FACE_BOTTOM); }
            if (front_faces  & bit_mask) != 0 { push_face(&FACE_FRONT);  }
            if (back_faces   & bit_mask) != 0 { push_face(&FACE_BACK);   }
        }
        chunk_triangles.to_vec()
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
