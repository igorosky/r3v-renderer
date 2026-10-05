use crate::{
    geometry::PointI,
    Tesselator,
};

pub struct GreedyMeshTesselator {
}

impl Tesselator for GreedyMeshTesselator {
    type InputGeometryDelta = Vec<PointI>;
    type OutputGeometryDelta;

    fn update_input_geometry(
        &mut self, 
        geometry_delta: &Self::InputGeometryDelta
    ) {
        todo!()
    }

    fn get_geometry_delta(&mut self) -> Option<Self::OutputGeometryDelta> {
        todo!();
    }
}
