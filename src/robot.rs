use std::collections::HashMap;

use crate::{
    Robot, geometry::RealTwist, stl, urdf,
};

pub struct UrdfRobot {
}

impl Robot for UrdfRobot {
    type RobotDescription = urdf::UrdfRobot;
    type SensorInput = HashMap<String, f32>;
    type TransformDescription = HashMap<String, RealTwist>;

    fn from_description(description: Self::RobotDescription) -> Self {
        todo!()
    }

    fn feed_sensor_input(&mut self, input: &Self::SensorInput) -> Self::TransformDescription {
        todo!()
    }
    
}
