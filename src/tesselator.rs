use crate::{
    Tesselator, geometry::{
        ChunkDelta, DiscretePoint, DiscreteTriangle, PointI, RealTriangle,
    },
};

pub struct GreedyMeshTesselator {
    is_voxel_data_dirty: bool,
    voxel_chunks: Vec<PointI>,
}

impl GreedyMeshTesselator {
    pub fn new() -> Self {
        Self {
            is_voxel_data_dirty: true,
            voxel_chunks: Vec::new(),
        }
    }

    fn voxel_world_origin(&self, chunk_origin: &DiscretePoint, voxel_index: u32) -> DiscretePoint {
        return chunk_origin.clone(); // TODO(@mariusz): Actually calculate the world origin of the voxel based on its index within the chunk.
    }

    fn voxel_chunk_index(&self, voxel_origin: &DiscretePoint) -> (DiscretePoint, u32) {
        return (voxel_origin.clone(), 0); // TODO(@mariusz): Actually calculate the chunk index and voxel index within the chunk based on the voxel's world origin.
    }

    fn tesselate_voxel(&self, voxel_origin: &DiscretePoint) -> Vec<DiscreteTriangle> {
        // TODO(@mariusz): Actually implement greedy meshing algorithm to generate triangles for the voxel.
        const CUBE_TESSELATION_OFFSETS: [[i32; 3]; 36] = [
            // Front face (Z = 1)
            [0, 0, 1], [1, 0, 1], [1, 1, 1],
            [0, 0, 1], [1, 1, 1], [0, 1, 1],

            // Back face (Z = 0)
            [1, 0, 0], [0, 0, 0], [0, 1, 0],
            [1, 0, 0], [0, 1, 0], [1, 1, 0],

            // Top face (Y = 1)
            [0, 1, 1], [1, 1, 1], [1, 1, 0],
            [0, 1, 1], [1, 1, 0], [0, 1, 0],

            // Bottom face (Y = 0)
            [0, 0, 0], [1, 0, 0], [1, 0, 1],
            [0, 0, 0], [1, 0, 1], [0, 0, 1],

            // Right face (X = 1)
            [1, 0, 1], [1, 0, 0], [1, 1, 0],
            [1, 0, 1], [1, 1, 0], [1, 1, 1],

            // Left face (X = 0)
            [0, 0, 0], [0, 0, 1], [0, 1, 1],
            [0, 0, 0], [0, 1, 1], [0, 1, 0],
        ];
        CUBE_TESSELATION_OFFSETS
            .chunks_exact(3)
            .map(|chunk| {
                DiscreteTriangle {
                    v0: [
                        voxel_origin.x + chunk[0][0],
                        voxel_origin.y + chunk[0][1],
                        voxel_origin.z + chunk[0][2],
                    ],
                    v1: [
                        voxel_origin.x + chunk[1][0],
                        voxel_origin.y + chunk[1][1],
                        voxel_origin.z + chunk[1][2],
                    ],
                    v2: [
                        voxel_origin.x + chunk[2][0],
                        voxel_origin.y + chunk[2][1],
                        voxel_origin.z + chunk[2][2],
                    ],
                }
        }).collect()
    }
}

impl Tesselator for GreedyMeshTesselator {
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<ChunkDelta>;

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
        Some(self.voxel_chunks.iter().filter_map(|chunk| {
            let chunk_origin = DiscretePoint { x: chunk.x, y: chunk.y, z: chunk.z };
            let mut chunk_triangles = Vec::new();
            for bit in 0..32 {
                let mask = 1 << bit;
                if chunk.i & mask != 0 {
                    let voxel_origin = self.voxel_world_origin(&chunk_origin, bit);
                    let voxel_triangles = self.tesselate_voxel(&voxel_origin);
                    chunk_triangles.extend(voxel_triangles.iter().map(|dt| RealTriangle {
                        v0: [dt.v0[0] as f32, dt.v0[1] as f32, dt.v0[2] as f32],
                        v1: [dt.v1[0] as f32, dt.v1[1] as f32, dt.v1[2] as f32],
                        v2: [dt.v2[0] as f32, dt.v2[1] as f32, dt.v2[2] as f32],
                    }));
                }
            }
            if chunk_triangles.is_empty() {
                None
            } else {
                Some(ChunkDelta::Add {
                    index: chunk_origin,
                    triangles: chunk_triangles,
                })
            }
        }).collect())
    }   
}
