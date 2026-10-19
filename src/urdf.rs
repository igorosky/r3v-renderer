use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UrdfOrigin {
    pub xyz: Vector3,
    pub rpy: Vector3,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfMesh {
    pub filename: String,
    pub scale: Option<Vector3>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum UrdfGeometry {
    Mesh(UrdfMesh),
    Box { size: Vector3 },
    Sphere { radius: f32 },
    Cylinder { radius: f32, length: f32 },
    Capsule { radius: f32, length: f32 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfColor {
    pub rgba: [f32; 4],
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfMaterial {
    pub name: Option<String>,
    pub color: Option<UrdfColor>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfVisual {
    pub name: Option<String>,
    pub origin: Option<UrdfOrigin>,
    pub geometry: Option<UrdfGeometry>,
    pub material: Option<UrdfMaterial>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfLink {
    pub name: String,
    pub visuals: Vec<UrdfVisual>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UrdfJointLimit {
    pub lower: f32,
    pub upper: f32,
    pub effort: f32,
    pub velocity: f32,
    pub acceleration: f32,
    pub deceleration: f32,
    pub jerk: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfJoint {
    pub name: String,
    pub joint_type: String,
    pub parent: String,
    pub child: String,
    pub origin: Option<UrdfOrigin>,
    pub axis: Option<Vector3>,
    pub limit: Option<UrdfJointLimit>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfRobot {
    pub name: String,
    pub links: Vec<UrdfLink>,
    pub joints: Vec<UrdfJoint>,
    pub materials: Vec<UrdfMaterial>,
}

pub fn read_file(path: &Path) -> Result<UrdfRobot, String> {
    let path = path
        .canonicalize()
        .map_err(|err| format!("Failed to resolve URDF path '{}': {err}", path.display()))?;
    let contents = fs::read_to_string(&path)
        .map_err(|err| format!("Failed to read URDF '{}': {err}", path.display()))?;
    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    read_str_with_base_dir(&contents, Some(base_dir))
}

pub fn read_str(xml: &str) -> Result<UrdfRobot, String> {
    read_str_with_base_dir(xml, None)
}

fn read_str_with_base_dir(xml: &str, base_dir: Option<&Path>) -> Result<UrdfRobot, String> {
    check_root_element(xml)?;
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
                        geometry: Some(convert_geometry(visual.geometry, base_dir)),
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
                limit: Some(convert_joint_limit(joint.limit)),
            })
            .collect(),
        materials: parsed.materials.into_iter().map(convert_material).collect(),
    })
}

fn check_root_element(xml: &str) -> Result<(), String> {
    let mut reader = quick_xml::Reader::from_str(xml);
    loop {
        match reader.read_event() {
            Ok(
                quick_xml::events::Event::Start(element) | quick_xml::events::Event::Empty(element),
            ) => {
                return match element.name().as_ref() {
                    b"robot" => Ok(()),
                    name => Err(format!(
                        "URDF root element must be <robot>, found <{}>",
                        String::from_utf8_lossy(name)
                    )),
                };
            }
            Ok(quick_xml::events::Event::Eof) => {
                return Err("URDF document has no root element".to_string());
            }
            Ok(_) => {}
            Err(err) => return Err(format!("Failed to parse URDF XML: {err}")),
        }
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

fn convert_joint_limit(limit: urdf_rs::JointLimit) -> UrdfJointLimit {
    UrdfJointLimit {
        lower: limit.lower as f32,
        upper: limit.upper as f32,
        effort: limit.effort as f32,
        velocity: limit.velocity as f32,
        acceleration: limit.acceleration as f32,
        deceleration: limit.deceleration as f32,
        jerk: limit.jerk as f32,
    }
}

fn convert_geometry(geometry: urdf_rs::Geometry, base_dir: Option<&Path>) -> UrdfGeometry {
    match geometry {
        urdf_rs::Geometry::Mesh { filename, scale } => UrdfGeometry::Mesh(UrdfMesh {
            filename: resolve_mesh_path(&filename, base_dir),
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

fn resolve_mesh_path(filename: &str, base_dir: Option<&Path>) -> String {
    if let Some((package, relative_path)) = filename
        .strip_prefix("package://")
        .and_then(|uri| uri.split_once('/'))
    {
        if let Some(package_dir) = base_dir.and_then(|base_dir| {
            base_dir
                .ancestors()
                .find(|ancestor| ancestor.file_name().is_some_and(|name| name == package))
        }) {
            return package_dir
                .join(relative_path)
                .to_string_lossy()
                .replace('\\', "/");
        }
        return filename.to_string();
    }

    if filename.starts_with("file://")
        || filename.starts_with("http://")
        || filename.starts_with("https://")
        || Path::new(filename).is_absolute()
    {
        return filename.to_string();
    }

    let Some(base_dir) = base_dir else {
        return filename.to_string();
    };

    base_dir
        .join(filename)
        .to_string_lossy()
        .replace('\\', "/")
        .to_string()
}

fn convert_material(material: urdf_rs::Material) -> UrdfMaterial {
    UrdfMaterial {
        name: (!material.name.is_empty()).then_some(material.name),
        color: material.color.map(|color| UrdfColor {
            rgba: color.rgba.0.map(|channel| channel as f32),
        }),
    }
}
