use std::collections::{HashMap, VecDeque};

use crate::{
    Robot, geometry::{self, Quaternion, RealTwist, Vector3D}, urdf,
};

pub struct UrdfRobot {
    description: urdf::UrdfRobot,
    last_transforms: Option<RobotTransforms>,
}

impl UrdfRobot {
    pub fn description(&self) -> &urdf::UrdfRobot {
        &self.description
    }
}

#[derive(Debug, Clone)]
pub struct RobotTransforms {
    pub links: HashMap<String, RealTwist>,
    pub visuals: HashMap<(String, usize), RealTwist>,
}

impl RobotTransforms {
    pub fn transform_visual_point(
        &self,
        link: &str,
        visual_index: usize,
        point: [f32; 3],
    ) -> Option<[f32; 3]> {
        let transform = self.visuals.get(&(link.to_owned(), visual_index))?;
        let rotated = rotate_vector(
            transform.rotational,
            Vector3D {
                x: point[0],
                y: point[1],
                z: point[2],
            },
        );
        Some([
            rotated.x + transform.linear.x,
            rotated.y + transform.linear.y,
            rotated.z + transform.linear.z,
        ])
    }

    pub fn get_link_origin(&self, link: &str) -> Option<geometry::Point<f32>> {
        let transform = self.links.get(link)?;
        Some(geometry::Point {
            x: transform.linear.x,
            y: transform.linear.y,
            z: transform.linear.z,
        })
    }
}

impl Robot for UrdfRobot {
    type RobotDescription = urdf::UrdfRobot;
    type SensorInput = HashMap<String, f32>;
    type Transform = RobotTransforms;

    fn from_description(description: Self::RobotDescription) -> Self {
        Self {
            description,
            last_transforms: None,
        }
    }

    fn feed_sensor_input(&mut self, input: &Self::SensorInput) -> Self::Transform {
        let mut joints_by_parent: HashMap<&str, Vec<&urdf::UrdfJoint>> = HashMap::new();
        let mut child_links = std::collections::HashSet::new();
        for joint in &self.description.joints {
            joints_by_parent
                .entry(&joint.parent)
                .or_default()
                .push(joint);
            child_links.insert(joint.child.as_str());
        }

        let mut link_world = HashMap::new();
        let mut pending = VecDeque::new();
        for link in &self.description.links {
            if !child_links.contains(link.name.as_str()) {
                link_world.insert(link.name.clone(), identity_twist());
                pending.push_back(link.name.as_str());
            }
        }

        while let Some(parent_name) = pending.pop_front() {
            let parent_world = link_world[parent_name];
            if let Some(joints) = joints_by_parent.get(parent_name) {
                for joint in joints {
                    if link_world.contains_key(&joint.child) {
                        continue;
                    }
                    let value = input.get(&joint.name).copied().unwrap_or(0.0);
                    let child_world = compose_twists(parent_world, joint_transform(joint, value));
                    link_world.insert(joint.child.clone(), child_world);
                    pending.push_back(joint.child.as_str());
                }
            }
        }

        let mut visual_world = HashMap::new();
        for link in &self.description.links {
            let Some(link_transform) = link_world.get(&link.name).copied() else {
                continue;
            };
            for (visual_index, visual) in link.visuals.iter().enumerate() {
                let local = visual
                    .origin
                    .map(origin_transform)
                    .unwrap_or_else(identity_twist);
                visual_world.insert(
                    (link.name.clone(), visual_index),
                    compose_twists(link_transform, local),
                );
            }
        }

        let transforms = RobotTransforms {
            links: link_world,
            visuals: visual_world,
        };
        self.last_transforms = Some(transforms.clone());
        transforms
    }
    
    fn last_transform(&self) -> Option<&Self::Transform> {
        self.last_transforms.as_ref()
    }
}

fn identity_twist() -> RealTwist {
    RealTwist {
        linear: Vector3D {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        rotational: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    }
}

fn origin_transform(origin: urdf::UrdfOrigin) -> RealTwist {
    RealTwist {
        linear: Vector3D {
            x: origin.xyz.x,
            y: origin.xyz.y,
            z: origin.xyz.z,
        },
        rotational: tuple_to_quaternion(rpy_to_quaternion(
            origin.rpy.x,
            origin.rpy.y,
            origin.rpy.z,
        )),
    }
}

fn compose_twists(parent: RealTwist, child: RealTwist) -> RealTwist {
    let rotated = rotate_vector(parent.rotational, child.linear);
    let rotational = normalize_quaternion(multiply_quaternions(
        quaternion_to_tuple(parent.rotational),
        quaternion_to_tuple(child.rotational),
    ));

    RealTwist {
        linear: Vector3D {
            x: parent.linear.x + rotated.x,
            y: parent.linear.y + rotated.y,
            z: parent.linear.z + rotated.z,
        },
        rotational: tuple_to_quaternion(rotational),
    }
}

fn rotate_vector(rotation: Quaternion<f32>, point: Vector3D<f32>) -> Vector3D<f32> {
    let (x, y, z, w) = quaternion_to_tuple(rotation);
    let norm_squared = x * x + y * y + z * z + w * w;
    if norm_squared == 0.0 {
        return point;
    }

    let uv = cross((x, y, z), (point.x, point.y, point.z));
    let uuv = cross((x, y, z), uv);
    let scale = 2.0 / norm_squared;

    Vector3D {
        x: point.x + scale * (w * uv.0 + uuv.0),
        y: point.y + scale * (w * uv.1 + uuv.1),
        z: point.z + scale * (w * uv.2 + uuv.2),
    }
}

fn cross(a: (f32, f32, f32), b: (f32, f32, f32)) -> (f32, f32, f32) {
    (
        a.1 * b.2 - a.2 * b.1,
        a.2 * b.0 - a.0 * b.2,
        a.0 * b.1 - a.1 * b.0,
    )
}

fn quaternion_to_tuple(quaternion: Quaternion<f32>) -> (f32, f32, f32, f32) {
    (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
}

fn tuple_to_quaternion(value: (f32, f32, f32, f32)) -> Quaternion<f32> {
    Quaternion {
        x: value.0,
        y: value.1,
        z: value.2,
        w: value.3,
    }
}

fn normalize_quaternion((x, y, z, w): (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
    let length = (x * x + y * y + z * z + w * w).sqrt();
    if length == 0.0 {
        (0.0, 0.0, 0.0, 1.0)
    } else {
        (x / length, y / length, z / length, w / length)
    }
}

fn joint_transform(joint: &urdf::UrdfJoint, value: f32) -> RealTwist {
    let origin = joint
        .origin
        .map(origin_transform)
        .unwrap_or_else(identity_twist);
    let axis = joint.axis.unwrap_or(urdf::Vector3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    });

    let motion = match joint.joint_type.as_str() {
        "revolute" | "continuous" => RealTwist {
            linear: Vector3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotational: tuple_to_quaternion(axis_angle_to_quaternion(axis, value)),
        },
        "prismatic" => RealTwist {
            linear: Vector3D {
                x: axis.x * value,
                y: axis.y * value,
                z: axis.z * value,
            },
            rotational: tuple_to_quaternion(quaternion_identity()),
        },
        _ => identity_twist(),
    };
    compose_twists(origin, motion)
}

fn quaternion_identity() -> (f32, f32, f32, f32) {
    (0.0, 0.0, 0.0, 1.0)
}

fn axis_angle_to_quaternion(axis: urdf::Vector3, angle: f32) -> (f32, f32, f32, f32) {
    let length = (axis.x * axis.x + axis.y * axis.y + axis.z * axis.z).sqrt();
    if length == 0.0 {
        return quaternion_identity();
    }

    let half = angle * 0.5;
    let s = half.sin() / length;
    (axis.x * s, axis.y * s, axis.z * s, half.cos())
}

fn rpy_to_quaternion(roll: f32, pitch: f32, yaw: f32) -> (f32, f32, f32, f32) {
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (yaw * 0.5).sin_cos();

    (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
}

fn multiply_quaternions(
    (ax, ay, az, aw): (f32, f32, f32, f32),
    (bx, by, bz, bw): (f32, f32, f32, f32),
) -> (f32, f32, f32, f32) {
    (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
}
