use std::collections::HashSet;

use crate::{
    Tesselator, geometry::{
        TriangleChunk, DiscretePoint, PointI, RealTriangle,
    },
};

const FACE_FRONT:   [[i32; 3]; 6] = [[0,0,1], [1,0,1], [1,1,1], [0,0,1], [1,1,1], [0,1,1]]; // +Z
const FACE_BACK:    [[i32; 3]; 6] = [[1,0,0], [0,0,0], [0,1,0], [1,0,0], [0,1,0], [1,1,0]]; // -Z
const FACE_TOP:     [[i32; 3]; 6] = [[0,1,1], [1,1,1], [1,1,0], [0,1,1], [1,1,0], [0,1,0]]; // +Y
const FACE_BOTTOM:  [[i32; 3]; 6] = [[0,0,0], [1,0,0], [1,0,1], [0,0,0], [1,0,1], [0,0,1]]; // -Y
const FACE_RIGHT:   [[i32; 3]; 6] = [[1,0,1], [1,0,0], [1,1,0], [1,0,1], [1,1,0], [1,1,1]]; // +X
const FACE_LEFT:    [[i32; 3]; 6] = [[0,0,0], [0,0,1], [0,1,1], [0,0,0], [0,1,1], [0,1,0]]; // -X

pub struct GreedyMeshTesselator {
    is_voxel_data_dirty: bool,
    voxel_chunks: Vec<PointI>,
    emitted_chunks: HashSet<DiscretePoint>,
}

impl GreedyMeshTesselator {
    pub fn new() -> Self {
        Self {
            is_voxel_data_dirty: true,
            voxel_chunks: Vec::new(),
            emitted_chunks: HashSet::new(),
        }
    }

    fn voxel_world_origin(&self, chunk_origin: &DiscretePoint, voxel_index: u32) -> DiscretePoint {
        let lx = (voxel_index & 0b11) as i32;
        let ly = ((voxel_index >> 2) & 0b11) as i32;
        let lz = ((voxel_index >> 4) & 0b1) as i32;
        DiscretePoint {
            x: chunk_origin.x + lx,
            y: chunk_origin.y + ly,
            z: chunk_origin.z + lz,
        }
    }

    fn tesselate_chunk(&self, chunk: &PointI) -> Vec<RealTriangle> {
        let chunk_origin = DiscretePoint { x: chunk.x, y: chunk.y, z: chunk.z };
        let chunk_triangles = &mut Vec::new();

        let mask = chunk.i;
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
            let voxel_origin = self.voxel_world_origin(&chunk_origin, bit);
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
        self.voxel_chunks = geometry_delta.clone();
        self.is_voxel_data_dirty = true;
    }

    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        if !self.is_voxel_data_dirty {
            return None;
        }
        self.is_voxel_data_dirty = false;

        let mut deltas = Vec::new();
        let mut current_chunks = HashSet::new();
        for chunk in &self.voxel_chunks {
            let chunk_origin = DiscretePoint { x: chunk.x, y: chunk.y, z: chunk.z };
            let chunk_triangles = self.tesselate_chunk(chunk);
            if chunk_triangles.is_empty() {
                continue;
            }
            current_chunks.insert(chunk_origin);
            deltas.push(TriangleChunk {
                index: chunk_origin,
                triangles: chunk_triangles,
            });
        }
        deltas.extend(
            self.emitted_chunks
                .difference(&current_chunks)
                .map(|&index| TriangleChunk { index, triangles: Vec::new() }),
        );
        self.emitted_chunks = current_chunks;
        Some(deltas)
    }   
}
