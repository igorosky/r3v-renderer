use std::{
    fs,
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};

use crate::{
    StlIoReader, StlReader, UrdfReader, UrdfRsReader,
    stl::NativeStlReader,
    urdf::{NativeUrdfReader, UrdfGeometry},
};

fn temporary_file(name: &str) -> PathBuf {
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock should be after the Unix epoch")
        .as_nanos();
    std::env::temp_dir().join(format!(
        "r3v_renderer_{name}_{}_{}.tmp",
        std::process::id(),
        timestamp
    ))
}

#[test]
fn urdf_readers_produce_compatible_models() {
    let xml = r#"<robot name="demo_robot">
        <material name="red"><color rgba="1 0 0 1"/></material>
        <link name="base">
            <visual>
                <origin xyz="1 2 3" rpy="0 0 1.5708"/>
                <geometry>
                    <mesh filename="package://demo/meshes/base.stl" scale="1 1 1"/>
                </geometry>
                <material name="red"/>
            </visual>
        </link>
        <link name="arm"/>
        <joint name="base_to_arm" type="revolute">
            <origin xyz="0 0 1" rpy="0 0 0"/>
            <parent link="base"/>
            <child link="arm"/>
            <axis xyz="0 1 0"/>
        </joint>
    </robot>"#;

    let native = NativeUrdfReader
        .read_str(xml)
        .expect("native parser should parse URDF");
    let library = UrdfRsReader
        .read_str(xml)
        .expect("urdf-rs should parse URDF");

    assert_eq!(native.name, library.name);
    assert_eq!(native.links.len(), library.links.len());
    assert_eq!(native.links[0].name, library.links[0].name);
    assert_eq!(
        native.links[0].visuals[0].mesh_filename(),
        library.links[0].visuals[0].mesh_filename()
    );
    assert_eq!(native.joints[0].parent, library.joints[0].parent);
    assert_eq!(native.joints[0].child, library.joints[0].child);
    assert_eq!(native.joints[0].joint_type, library.joints[0].joint_type);
    assert_eq!(library.joints[0].axis.expect("joint axis").y, 1.0);
    assert!(matches!(
        native.links[0].visuals[0].geometry,
        Some(UrdfGeometry::Mesh(_))
    ));
}

#[test]
fn stl_readers_parse_ascii_files_compatibly() {
    let path = temporary_file("ascii_stl");
    let content = r#"solid test_mesh
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid test_mesh
"#;
    fs::write(&path, content).expect("write ASCII STL fixture");

    let native = NativeStlReader
        .read_file(&path)
        .expect("native parser should parse ASCII STL");
    let library = StlIoReader
        .read_file(&path)
        .expect("stl_io should parse ASCII STL");

    assert_eq!(native, library);
    assert_eq!(native.triangle_count(), 1);
    assert_eq!(native.vertex_count(), 3);
    fs::remove_file(path).expect("remove ASCII STL fixture");
}

#[test]
fn stl_readers_parse_binary_files_compatibly() {
    let path = temporary_file("binary_stl");
    let mut bytes = vec![0_u8; 84 + 50];
    bytes[0..10].copy_from_slice(b"binary STL");
    bytes[80..84].copy_from_slice(&1_u32.to_le_bytes());

    let normal = [0.0_f32, 0.0, 1.0];
    let vertices = [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let mut offset = 84;
    for value in normal.into_iter().chain(vertices.into_iter().flatten()) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        offset += 4;
    }
    fs::write(&path, bytes).expect("write binary STL fixture");

    let native = NativeStlReader
        .read_file(&path)
        .expect("native parser should parse binary STL");
    let library = StlIoReader
        .read_file(&path)
        .expect("stl_io should parse binary STL");

    assert_eq!(native, library);
    assert_eq!(native.triangle_count(), 1);
    fs::remove_file(path).expect("remove binary STL fixture");
}
