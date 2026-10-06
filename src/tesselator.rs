use crate::{
    geometry::{
        PointI,
        ChunkDelta,
        ChunkIndex,
        Triangle,
    },
    Tesselator,
};

pub struct GreedyMeshTesselator {
}

impl GreedyMeshTesselator {
    pub fn new() -> Self {
        Self {
        }
    }
}

impl Tesselator for GreedyMeshTesselator {
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta = Vec<ChunkDelta>;

    fn update_input_geometry(
        &mut self, 
        geometry_delta: &Self::InputGeometryDelta
    ) {
        todo!()
    }

    fn get_output_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        return Some(vec![ // Placeholder
            ChunkDelta::Add {
                index: (0, 0, 0),
                triangles: vec![
                    Triangle {
                        v0: [0.0, 0.0, 0.0],
                        v1: [1.0, 0.0, 0.0],
                        v2: [1.0, 1.0, 0.0],
                    },
                    Triangle {
                        v0: [1.0, 1.0, 0.0],
                        v1: [0.0, 1.0, 0.0],
                        v2: [0.0, 0.0, 0.0],
                    },
                ],
            }
        ])
    }
}
