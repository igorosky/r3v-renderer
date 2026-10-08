use super::{
    UrdfColor, UrdfGeometry, UrdfJoint, UrdfLink, UrdfMaterial, UrdfMesh, UrdfOrigin, UrdfReader,
    UrdfRobot, UrdfVisual, Vector3,
};

/// URDF reader backed by the `urdf-rs` crate.
#[derive(Clone, Copy, Debug, Default)]
pub struct UrdfRsReader;

impl UrdfReader for UrdfRsReader {
    fn read_str(&self, xml: &str) -> Result<UrdfRobot, String> {
        let parsed = urdf_rs::read_from_string(xml)
            .map_err(|err| format!("urdf-rs failed to parse URDF: {err}"))?;

        Ok(UrdfRobot {
            name: parsed.name,
            links: parsed
                .links
                .into_iter()
                .map(|link| UrdfLink {
                    name: link.name,
                    visuals: link
                        .visual
                        .into_iter()
                        .map(|visual| UrdfVisual {
                            name: visual.name,
                            origin: Some(convert_origin(visual.origin)),
                            geometry: Some(convert_geometry(visual.geometry)),
                            material: visual.material.map(convert_material),
                        })
                        .collect(),
                })
                .collect(),
            joints: parsed
                .joints
                .into_iter()
                .map(|joint| UrdfJoint {
                    name: joint.name,
                    joint_type: match joint.joint_type {
                        urdf_rs::JointType::Revolute => "revolute",
                        urdf_rs::JointType::Continuous => "continuous",
                        urdf_rs::JointType::Prismatic => "prismatic",
                        urdf_rs::JointType::Fixed => "fixed",
                        urdf_rs::JointType::Floating => "floating",
                        urdf_rs::JointType::Planar => "planar",
                        urdf_rs::JointType::Spherical => "spherical",
                    }
                    .to_string(),
                    parent: joint.parent.link,
                    child: joint.child.link,
                    origin: Some(convert_origin(joint.origin)),
                    axis: Some(convert_vector3(joint.axis.xyz.0)),
                })
                .collect(),
            materials: parsed.materials.into_iter().map(convert_material).collect(),
        })
    }
}

fn convert_vector3(value: [f64; 3]) -> Vector3 {
    Vector3 {
        x: value[0] as f32,
        y: value[1] as f32,
        z: value[2] as f32,
    }
}

fn convert_origin(origin: urdf_rs::Pose) -> UrdfOrigin {
    UrdfOrigin {
        xyz: convert_vector3(origin.xyz.0),
        rpy: convert_vector3(origin.rpy.0),
    }
}

fn convert_geometry(geometry: urdf_rs::Geometry) -> UrdfGeometry {
    match geometry {
        urdf_rs::Geometry::Mesh { filename, scale } => UrdfGeometry::Mesh(UrdfMesh {
            filename,
            scale: scale.map(|scale| convert_vector3(scale.0)),
        }),
        urdf_rs::Geometry::Box { size } => UrdfGeometry::Box {
            size: convert_vector3(size.0),
        },
        urdf_rs::Geometry::Sphere { radius } => UrdfGeometry::Sphere {
            radius: radius as f32,
        },
        urdf_rs::Geometry::Cylinder { radius, length } => UrdfGeometry::Cylinder {
            radius: radius as f32,
            length: length as f32,
        },
        urdf_rs::Geometry::Capsule { radius, length } => UrdfGeometry::Capsule {
            radius: radius as f32,
            length: length as f32,
        },
    }
}

fn convert_material(material: urdf_rs::Material) -> UrdfMaterial {
    UrdfMaterial {
        name: (!material.name.is_empty()).then_some(material.name),
        color: material.color.map(|color| UrdfColor {
            rgba: color.rgba.0.map(|channel| channel as f32),
        }),
    }
}
