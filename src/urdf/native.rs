use std::path::Path;

use super::UrdfReader;
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn parse(value: &str) -> Result<Self, String> {
        let values: Vec<&str> = value.split_whitespace().collect();
        if values.len() != 3 {
            return Err(format!("Expected 3 components for vector, got '{value}'"));
        }

        Ok(Self {
            x: parse_float(values[0])?,
            y: parse_float(values[1])?,
            z: parse_float(values[2])?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UrdfOrigin {
    pub xyz: Vector3,
    pub rpy: Vector3,
}

impl UrdfOrigin {
    pub fn from_attributes(attributes: &BytesStart<'_>) -> Result<Self, String> {
        let xyz = required_attr(attributes, b"xyz")?;
        let rpy = required_attr(attributes, b"rpy").unwrap_or_else(|_| "0 0 0".to_string());
        Ok(Self {
            xyz: Vector3::parse(&xyz)?,
            rpy: Vector3::parse(&rpy)?,
        })
    }
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

impl UrdfColor {
    pub fn parse(value: &str) -> Result<Self, String> {
        let values: Vec<&str> = value.split_whitespace().collect();
        if values.len() != 4 {
            return Err(format!("Expected RGBA with 4 channels, got '{value}'"));
        }

        Ok(Self {
            rgba: [
                parse_float(values[0])?,
                parse_float(values[1])?,
                parse_float(values[2])?,
                parse_float(values[3])?,
            ],
        })
    }
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

impl UrdfVisual {
    pub fn mesh_filename(&self) -> Option<&str> {
        match self.geometry.as_ref() {
            Some(UrdfGeometry::Mesh(mesh)) => Some(mesh.filename.as_str()),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfLink {
    pub name: String,
    pub visuals: Vec<UrdfVisual>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfJoint {
    pub name: String,
    pub joint_type: String,
    pub parent: String,
    pub child: String,
    pub origin: Option<UrdfOrigin>,
    pub axis: Option<Vector3>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfRobot {
    pub name: String,
    pub links: Vec<UrdfLink>,
    pub joints: Vec<UrdfJoint>,
    pub materials: Vec<UrdfMaterial>,
}

impl UrdfRobot {
    /// Parses URDF with the native reader for backward compatibility.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        NativeUrdfReader.read_file(path.as_ref())
    }

    /// Parses URDF with the native reader for backward compatibility.
    pub fn from_str(xml: &str) -> Result<Self, String> {
        NativeUrdfReader.read_str(xml)
    }

    pub fn link_by_name(&self, name: &str) -> Option<&UrdfLink> {
        self.links.iter().find(|link| link.name == name)
    }

    pub fn joint_by_name(&self, name: &str) -> Option<&UrdfJoint> {
        self.joints.iter().find(|joint| joint.name == name)
    }
}

#[derive(Clone, Copy, Debug, Default)]
/// URDF reader implemented with this crate's native XML parser.
pub struct NativeUrdfReader;

impl UrdfReader for NativeUrdfReader {
    fn read_str(&self, xml: &str) -> Result<UrdfRobot, String> {
        let mut reader = Reader::from_str(xml);
        let mut buffer = Vec::new();
        let mut robot = UrdfRobot {
            name: String::new(),
            links: Vec::new(),
            joints: Vec::new(),
            materials: Vec::new(),
        };

        loop {
            match reader.read_event_into(&mut buffer) {
                Ok(Event::Start(start)) => {
                    let tag = start.name().as_ref().to_vec();
                    if tag == b"robot" {
                        robot.name = required_attr(&start, b"name")?;
                    } else if tag == b"link" {
                        robot.links.push(parse_link(&mut reader, &start)?);
                    } else if tag == b"joint" {
                        robot.joints.push(parse_joint(&mut reader, &start)?);
                    } else if tag == b"material" {
                        robot.materials.push(parse_material(&mut reader, &start)?);
                    } else {
                        skip_element(&mut reader, &start)?;
                    }
                }
                Ok(Event::Empty(start)) => {
                    let tag = start.name().as_ref().to_vec();
                    if tag == b"link" {
                        robot.links.push(parse_link_empty(&start)?);
                    } else if tag == b"joint" {
                        robot.joints.push(parse_joint_empty(&start)?);
                    } else if tag == b"material" {
                        robot.materials.push(parse_material_empty(&start)?);
                    }
                }
                Ok(Event::Eof) => break,
                Ok(_) => {}
                Err(err) => return Err(format!("Failed to parse URDF XML: {err}")),
            }
        }

        if robot.name.is_empty() {
            return Err("URDF robot element is missing a name attribute.".to_string());
        }

        Ok(robot)
    }
}

fn parse_link(reader: &mut Reader<&[u8]>, start: &BytesStart<'_>) -> Result<UrdfLink, String> {
    let name = required_attr(start, b"name")?;
    let mut visuals = Vec::new();

    loop {
        match reader.read_event() {
            Ok(Event::Start(element)) => {
                let tag = element.name().as_ref().to_vec();
                if tag == b"visual" {
                    visuals.push(parse_visual(reader, &element)?);
                } else {
                    skip_element(reader, &element)?;
                }
            }
            Ok(Event::Empty(element)) => {
                let tag = element.name().as_ref().to_vec();
                if tag == b"visual" {
                    visuals.push(parse_visual_empty(&element)?);
                }
            }
            Ok(Event::End(end)) if end.name().as_ref() == b"link" => break,
            Ok(Event::Eof) => return Err("Unexpected end of file while parsing link.".to_string()),
            Ok(_) => {}
            Err(err) => return Err(format!("Failed to parse link '{name}': {err}")),
        }
    }

    Ok(UrdfLink { name, visuals })
}

fn parse_visual(reader: &mut Reader<&[u8]>, start: &BytesStart<'_>) -> Result<UrdfVisual, String> {
    let name = get_attr(start, b"name");
    let mut origin = None;
    let mut geometry = None;
    let mut material = None;

    loop {
        match reader.read_event() {
            Ok(Event::Start(element)) => {
                let tag = element.name().as_ref().to_vec();
                match tag.as_slice() {
                    b"origin" => origin = Some(UrdfOrigin::from_attributes(&element)?),
                    b"geometry" => geometry = Some(parse_geometry(reader, &element)?),
                    b"material" => material = Some(parse_material_reference(reader, &element)?),
                    _ => skip_element(reader, &element)?,
                }
            }
            Ok(Event::Empty(element)) => {
                let tag = element.name().as_ref().to_vec();
                match tag.as_slice() {
                    b"origin" => origin = Some(UrdfOrigin::from_attributes(&element)?),
                    b"geometry" => geometry = Some(parse_geometry_empty(&element)?),
                    b"material" => material = Some(parse_material_reference_empty(&element)?),
                    _ => {}
                }
            }
            Ok(Event::End(end)) if end.name().as_ref() == b"visual" => break,
            Ok(Event::Eof) => {
                return Err("Unexpected end of file while parsing visual.".to_string());
            }
            Ok(_) => {}
            Err(err) => return Err(format!("Failed to parse visual: {err}")),
        }
    }

    Ok(UrdfVisual {
        name,
        origin,
        geometry,
        material,
    })
}

fn parse_geometry(
    reader: &mut Reader<&[u8]>,
    _start: &BytesStart<'_>,
) -> Result<UrdfGeometry, String> {
    let mut geometry = None;

    loop {
        match reader.read_event() {
            Ok(Event::Start(element)) => {
                let tag = element.name().as_ref().to_vec();
                let parsed = match tag.as_slice() {
                    b"mesh" => parse_mesh(&element)?,
                    b"box" => {
                        let size = required_attr(&element, b"size")?;
                        UrdfGeometry::Box {
                            size: Vector3::parse(&size)?,
                        }
                    }
                    b"sphere" => {
                        let radius = required_attr(&element, b"radius")?;
                        UrdfGeometry::Sphere {
                            radius: parse_float(&radius)?,
                        }
                    }
                    b"cylinder" => {
                        let radius = required_attr(&element, b"radius")?;
                        let length = required_attr(&element, b"length")?;
                        UrdfGeometry::Cylinder {
                            radius: parse_float(&radius)?,
                            length: parse_float(&length)?,
                        }
                    }
                    b"capsule" => {
                        let radius = required_attr(&element, b"radius")?;
                        let length = required_attr(&element, b"length")?;
                        UrdfGeometry::Capsule {
                            radius: parse_float(&radius)?,
                            length: parse_float(&length)?,
                        }
                    }
                    _ => {
                        skip_element(reader, &element)?;
                        continue;
                    }
                };
                geometry = Some(parsed);
            }
            Ok(Event::Empty(element)) => {
                let tag = element.name().as_ref().to_vec();
                let parsed = match tag.as_slice() {
                    b"mesh" => parse_mesh(&element)?,
                    b"box" => {
                        let size = required_attr(&element, b"size")?;
                        UrdfGeometry::Box {
                            size: Vector3::parse(&size)?,
                        }
                    }
                    b"sphere" => {
                        let radius = required_attr(&element, b"radius")?;
                        UrdfGeometry::Sphere {
                            radius: parse_float(&radius)?,
                        }
                    }
                    b"cylinder" => {
                        let radius = required_attr(&element, b"radius")?;
                        let length = required_attr(&element, b"length")?;
                        UrdfGeometry::Cylinder {
                            radius: parse_float(&radius)?,
                            length: parse_float(&length)?,
                        }
                    }
                    b"capsule" => {
                        let radius = required_attr(&element, b"radius")?;
                        let length = required_attr(&element, b"length")?;
                        UrdfGeometry::Capsule {
                            radius: parse_float(&radius)?,
                            length: parse_float(&length)?,
                        }
                    }
                    _ => continue,
                };
                geometry = Some(parsed);
            }
            Ok(Event::End(end)) if end.name().as_ref() == b"geometry" => break,
            Ok(Event::Eof) => {
                return Err("Unexpected end of file while parsing geometry.".to_string());
            }
            Ok(_) => {}
            Err(err) => return Err(format!("Failed to parse geometry: {err}")),
        }
    }

    geometry.ok_or_else(|| "Geometry element is missing a valid shape.".to_string())
}

fn parse_mesh(start: &BytesStart<'_>) -> Result<UrdfGeometry, String> {
    let filename = required_attr(start, b"filename")?;
    let scale = get_attr(start, b"scale")
        .map(|value| Vector3::parse(&value))
        .transpose()?;
    Ok(UrdfGeometry::Mesh(UrdfMesh { filename, scale }))
}

fn parse_joint(reader: &mut Reader<&[u8]>, start: &BytesStart<'_>) -> Result<UrdfJoint, String> {
    let name = required_attr(start, b"name")?;
    let joint_type = required_attr(start, b"type")?;
    let mut parent = None;
    let mut child = None;
    let mut origin = None;
    let mut axis = None;

    loop {
        match reader.read_event() {
            Ok(Event::Start(element)) => {
                let tag = element.name().as_ref().to_vec();
                match tag.as_slice() {
                    b"origin" => origin = Some(UrdfOrigin::from_attributes(&element)?),
                    b"parent" => parent = Some(required_attr(&element, b"link")?),
                    b"child" => child = Some(required_attr(&element, b"link")?),
                    b"axis" => {
                        let xyz = required_attr(&element, b"xyz")?;
                        axis = Some(Vector3::parse(&xyz)?);
                    }
                    _ => skip_element(reader, &element)?,
                }
            }
            Ok(Event::Empty(element)) => {
                let tag = element.name().as_ref().to_vec();
                match tag.as_slice() {
                    b"origin" => origin = Some(UrdfOrigin::from_attributes(&element)?),
                    b"parent" => parent = Some(required_attr(&element, b"link")?),
                    b"child" => child = Some(required_attr(&element, b"link")?),
                    b"axis" => {
                        let xyz = required_attr(&element, b"xyz")?;
                        axis = Some(Vector3::parse(&xyz)?);
                    }
                    _ => {}
                }
            }
            Ok(Event::End(end)) if end.name().as_ref() == b"joint" => break,
            Ok(Event::Eof) => return Err("Unexpected end of file while parsing joint.".to_string()),
            Ok(_) => {}
            Err(err) => return Err(format!("Failed to parse joint '{name}': {err}")),
        }
    }

    Ok(UrdfJoint {
        name: name.clone(),
        joint_type,
        parent: parent.ok_or_else(|| format!("Joint '{name}' is missing a parent link."))?,
        child: child.ok_or_else(|| format!("Joint '{name}' is missing a child link."))?,
        origin,
        axis,
    })
}

fn parse_link_empty(start: &BytesStart<'_>) -> Result<UrdfLink, String> {
    Ok(UrdfLink {
        name: required_attr(start, b"name")?,
        visuals: Vec::new(),
    })
}

fn parse_joint_empty(start: &BytesStart<'_>) -> Result<UrdfJoint, String> {
    let name = required_attr(start, b"name")?;
    let joint_type = required_attr(start, b"type")?;
    Ok(UrdfJoint {
        name,
        joint_type,
        parent: String::new(),
        child: String::new(),
        origin: None,
        axis: None,
    })
}

fn parse_material(
    reader: &mut Reader<&[u8]>,
    start: &BytesStart<'_>,
) -> Result<UrdfMaterial, String> {
    let name = get_attr(start, b"name");
    let mut color = None;

    loop {
        match reader.read_event() {
            Ok(Event::Start(element)) => {
                let tag = element.name().as_ref().to_vec();
                if tag == b"color" {
                    let rgba = required_attr(&element, b"rgba")?;
                    color = Some(UrdfColor::parse(&rgba)?);
                } else {
                    skip_element(reader, &element)?;
                }
            }
            Ok(Event::Empty(element)) => {
                let tag = element.name().as_ref().to_vec();
                if tag == b"color" {
                    let rgba = required_attr(&element, b"rgba")?;
                    color = Some(UrdfColor::parse(&rgba)?);
                }
            }
            Ok(Event::End(end)) if end.name().as_ref() == b"material" => break,
            Ok(Event::Eof) => {
                return Err("Unexpected end of file while parsing material.".to_string());
            }
            Ok(_) => {}
            Err(err) => return Err(format!("Failed to parse material: {err}")),
        }
    }

    Ok(UrdfMaterial { name, color })
}

fn parse_material_empty(start: &BytesStart<'_>) -> Result<UrdfMaterial, String> {
    Ok(UrdfMaterial {
        name: get_attr(start, b"name"),
        color: get_attr(start, b"rgba")
            .map(|rgba| UrdfColor::parse(&rgba))
            .transpose()?,
    })
}

fn parse_material_reference(
    reader: &mut Reader<&[u8]>,
    start: &BytesStart<'_>,
) -> Result<UrdfMaterial, String> {
    let mut material = UrdfMaterial {
        name: get_attr(start, b"name"),
        color: None,
    };

    loop {
        match reader.read_event() {
            Ok(Event::Start(element)) => {
                let tag = element.name().as_ref().to_vec();
                if tag == b"color" {
                    let rgba = required_attr(&element, b"rgba")?;
                    material.color = Some(UrdfColor::parse(&rgba)?);
                } else {
                    skip_element(reader, &element)?;
                }
            }
            Ok(Event::Empty(element)) => {
                let tag = element.name().as_ref().to_vec();
                if tag == b"color" {
                    let rgba = required_attr(&element, b"rgba")?;
                    material.color = Some(UrdfColor::parse(&rgba)?);
                }
            }
            Ok(Event::End(end)) if end.name().as_ref() == b"material" => break,
            Ok(Event::Eof) => {
                return Err("Unexpected end of file while parsing material reference.".to_string());
            }
            Ok(_) => {}
            Err(err) => return Err(format!("Failed to parse material reference: {err}")),
        }
    }

    Ok(material)
}

fn parse_geometry_empty(start: &BytesStart<'_>) -> Result<UrdfGeometry, String> {
    parse_mesh(start)
}

fn parse_material_reference_empty(start: &BytesStart<'_>) -> Result<UrdfMaterial, String> {
    Ok(UrdfMaterial {
        name: get_attr(start, b"name"),
        color: get_attr(start, b"rgba")
            .map(|rgba| UrdfColor::parse(&rgba))
            .transpose()?,
    })
}

fn parse_visual_empty(start: &BytesStart<'_>) -> Result<UrdfVisual, String> {
    Ok(UrdfVisual {
        name: get_attr(start, b"name"),
        origin: None,
        geometry: None,
        material: None,
    })
}

fn get_attr(start: &BytesStart<'_>, key: &[u8]) -> Option<String> {
    start.attributes().find_map(|attribute| {
        let attribute = attribute.ok()?;
        if attribute.key.as_ref() == key {
            Some(String::from_utf8_lossy(&attribute.value).to_string())
        } else {
            None
        }
    })
}

fn required_attr(start: &BytesStart<'_>, key: &[u8]) -> Result<String, String> {
    get_attr(start, key).ok_or_else(|| {
        let name = String::from_utf8_lossy(start.name().as_ref()).to_string();
        format!(
            "Missing required attribute '{:?}' on <{}> element.",
            String::from_utf8_lossy(key),
            name
        )
    })
}

fn skip_element(reader: &mut Reader<&[u8]>, start: &BytesStart<'_>) -> Result<(), String> {
    let target_name = start.name().as_ref().to_vec();
    let mut depth = 1usize;

    loop {
        match reader.read_event() {
            Ok(Event::Start(_)) => depth += 1,
            Ok(Event::End(end)) if end.name().as_ref() == target_name => {
                depth -= 1;
                if depth == 0 {
                    return Ok(());
                }
            }
            Ok(Event::End(_)) => {
                depth = depth.checked_sub(1).unwrap_or(0);
            }
            Ok(Event::Eof) => {
                let name = String::from_utf8_lossy(&target_name).to_string();
                return Err(format!("Unexpected end of file while skipping <{name}>."));
            }
            Ok(_) => {}
            Err(err) => return Err(format!("Failed to skip XML element: {err}")),
        }
    }
}

fn parse_float(raw: &str) -> Result<f32, String> {
    raw.trim()
        .parse::<f32>()
        .map_err(|_| format!("Failed to parse float value '{raw}'"))
}
