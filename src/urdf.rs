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
    pub texture: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfVisual {
    pub name: Option<String>,
    pub origin: UrdfOrigin,
    pub geometry: UrdfGeometry,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum UrdfJointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
    Spherical,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfJoint {
    pub name: String,
    pub joint_type: UrdfJointType,
    pub parent: String,
    pub child: String,
    pub origin: UrdfOrigin,
    pub axis: Vector3,
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
    let joint_has_limit = joints_with_limit(xml)?;
    let parsed = urdf_rs::read_from_string(xml)
        .map_err(|err| format!("urdf-rs failed to parse URDF: {err}"))?;
    if joint_has_limit.len() != parsed.joints.len() {
        return Err(format!(
            "Found {} <joint> elements but urdf-rs read {} joints",
            joint_has_limit.len(),
            parsed.joints.len()
        ));
    }

    let mut named_colors = std::collections::HashMap::new();
    let mut named_textures = std::collections::HashMap::new();
    let inline_materials = parsed
        .links
        .iter()
        .flat_map(|link| &link.visual)
        .filter_map(|visual| visual.material.as_ref());
    for material in parsed.materials.iter().chain(inline_materials) {
        if material.name.is_empty() {
            continue;
        }
        if let Some(color) = &material.color {
            named_colors
                .entry(material.name.clone())
                .or_insert(color.rgba.0);
        }
        if let Some(texture) = &material.texture {
            named_textures
                .entry(material.name.clone())
                .or_insert(texture.filename.clone());
        }
    }
    let convert_material = |mut material: urdf_rs::Material| {
        if material.color.is_none()
            && let Some(&rgba) = named_colors.get(&material.name)
        {
            material.color = Some(urdf_rs::Color {
                rgba: urdf_rs::Vec4(rgba),
            });
        }
        if material.texture.is_none()
            && let Some(filename) = named_textures.get(&material.name)
        {
            material.texture = Some(urdf_rs::Texture {
                filename: filename.clone(),
            });
        }
        convert_material(material, base_dir)
    };

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
                        origin: convert_origin(visual.origin),
                        geometry: convert_geometry(visual.geometry, base_dir),
                        material: visual.material.map(convert_material),
                    })
                    .collect(),
            })
            .collect(),
        joints: parsed
            .joints
            .into_iter()
            .zip(joint_has_limit)
            .map(|(joint, has_limit)| UrdfJoint {
                name: joint.name,
                joint_type: match joint.joint_type {
                    urdf_rs::JointType::Revolute => UrdfJointType::Revolute,
                    urdf_rs::JointType::Continuous => UrdfJointType::Continuous,
                    urdf_rs::JointType::Prismatic => UrdfJointType::Prismatic,
                    urdf_rs::JointType::Fixed => UrdfJointType::Fixed,
                    urdf_rs::JointType::Floating => UrdfJointType::Floating,
                    urdf_rs::JointType::Planar => UrdfJointType::Planar,
                    urdf_rs::JointType::Spherical => UrdfJointType::Spherical,
                },
                parent: joint.parent.link,
                child: joint.child.link,
                origin: convert_origin(joint.origin),
                axis: convert_vector3(joint.axis.xyz.0),
                limit: has_limit.then(|| convert_joint_limit(joint.limit)),
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

fn joints_with_limit(xml: &str) -> Result<Vec<bool>, String> {
    use quick_xml::events::Event;

    let mut reader = quick_xml::Reader::from_str(xml);
    let mut joints = Vec::new();
    let mut depth = 0;
    let mut in_joint = false;
    loop {
        let event = reader
            .read_event()
            .map_err(|err| format!("Failed to parse URDF XML: {err}"))?;
        match event {
            Event::Start(element) => {
                depth += 1;
                match (depth, element.name().as_ref()) {
                    (2, b"joint") => {
                        joints.push(false);
                        in_joint = true;
                    }
                    (3, b"limit") if in_joint => *joints.last_mut().unwrap() = true,
                    _ => {}
                }
            }
            Event::Empty(element) => match (depth + 1, element.name().as_ref()) {
                (2, b"joint") => joints.push(false),
                (3, b"limit") if in_joint => *joints.last_mut().unwrap() = true,
                _ => {}
            },
            Event::End(_) => {
                if depth == 2 {
                    in_joint = false;
                }
                depth -= 1;
            }
            Event::Eof => return Ok(joints),
            _ => {}
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

fn convert_material(material: urdf_rs::Material, base_dir: Option<&Path>) -> UrdfMaterial {
    UrdfMaterial {
        name: (!material.name.is_empty()).then_some(material.name),
        color: material.color.map(|color| UrdfColor {
            rgba: color.rgba.0.map(|channel| channel as f32),
        }),
        texture: material
            .texture
            .map(|texture| resolve_mesh_path(&texture.filename, base_dir)),
    }
}
