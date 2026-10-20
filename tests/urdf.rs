mod common;

use std::path::Path;

use common::TempDir;
use r3v_renderer::urdf::{UrdfGeometry, UrdfOrigin, UrdfRobot, Vector3, read_file, read_str};

const ZERO: Vector3 = Vector3 {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

fn v(x: f32, y: f32, z: f32) -> Vector3 {
    Vector3 { x, y, z }
}

/// Origin as the URDF spec defines it: an omitted `<origin>` is the identity.
fn origin_or_identity(origin: Option<UrdfOrigin>) -> (Vector3, Vector3) {
    origin.map_or((ZERO, ZERO), |o| (o.xyz, o.rpy))
}

fn mesh_filename(robot: &UrdfRobot) -> &str {
    match &robot.links[0].visuals[0].geometry {
        Some(UrdfGeometry::Mesh(mesh)) => &mesh.filename,
        other => panic!("expected mesh geometry, got: {other:?}"),
    }
}

fn mesh_robot(filename: &str) -> String {
    format!(
        r#"<robot name="demo"><link name="base"><visual><geometry><mesh filename="{filename}"/></geometry></visual></link></robot>"#
    )
}

fn canonical(path: &Path) -> String {
    path.canonicalize()
        .unwrap()
        .to_string_lossy()
        .replace('\\', "/")
}

#[test]
fn reads_robot_name_and_links_in_order() {
    let robot = read_str(
        r#"<robot name="my_robot">
            <link name="base"/>
            <link name="arm"/>
            <link name="hand"/>
        </robot>"#,
    )
    .unwrap();
    assert_eq!(robot.name, "my_robot");
    let names: Vec<_> = robot.links.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["base", "arm", "hand"]);
    assert!(robot.links.iter().all(|l| l.visuals.is_empty()));
    assert!(robot.joints.is_empty());
    assert!(robot.materials.is_empty());
}

#[test]
fn reads_every_primitive_geometry() {
    let robot = read_str(
        r#"<robot name="shapes">
            <link name="base">
                <visual name="box"><geometry><box size="1 2 3"/></geometry></visual>
                <visual name="sphere"><geometry><sphere radius="0.5"/></geometry></visual>
                <visual name="cylinder"><geometry><cylinder radius="0.25" length="2"/></geometry></visual>
                <visual name="capsule"><geometry><capsule radius="0.125" length="1.5"/></geometry></visual>
                <visual><geometry><mesh filename="part.stl" scale="0.001 0.002 0.003"/></geometry></visual>
                <visual><geometry><mesh filename="other.stl"/></geometry></visual>
            </link>
        </robot>"#,
    )
    .unwrap();
    let visuals = &robot.links[0].visuals;
    assert_eq!(visuals.len(), 6);

    let names: Vec<_> = visuals.iter().map(|v| v.name.as_deref()).collect();
    assert_eq!(
        names,
        [
            Some("box"),
            Some("sphere"),
            Some("cylinder"),
            Some("capsule"),
            None,
            None
        ]
    );

    assert_eq!(
        visuals[0].geometry,
        Some(UrdfGeometry::Box {
            size: v(1.0, 2.0, 3.0)
        })
    );
    assert_eq!(
        visuals[1].geometry,
        Some(UrdfGeometry::Sphere { radius: 0.5 })
    );
    assert_eq!(
        visuals[2].geometry,
        Some(UrdfGeometry::Cylinder {
            radius: 0.25,
            length: 2.0
        })
    );
    assert_eq!(
        visuals[3].geometry,
        Some(UrdfGeometry::Capsule {
            radius: 0.125,
            length: 1.5
        })
    );
    match &visuals[4].geometry {
        Some(UrdfGeometry::Mesh(mesh)) => {
            assert_eq!(mesh.filename, "part.stl");
            assert_eq!(mesh.scale, Some(v(0.001, 0.002, 0.003)));
        }
        other => panic!("expected mesh, got {other:?}"),
    }
    match &visuals[5].geometry {
        Some(UrdfGeometry::Mesh(mesh)) => {
            assert_eq!(mesh.filename, "other.stl");
            // No scale given: either omitted or the identity scale.
            assert!(mesh.scale.is_none_or(|s| s == v(1.0, 1.0, 1.0)));
        }
        other => panic!("expected mesh, got {other:?}"),
    }
}

#[test]
fn reads_visual_origin_and_defaults_missing_origin_to_identity() {
    let robot = read_str(
        r#"<robot name="origins">
            <link name="base">
                <visual>
                    <origin xyz="1 -2 3.5" rpy="0.1 -0.2 0.3"/>
                    <geometry><sphere radius="1"/></geometry>
                </visual>
                <visual><geometry><sphere radius="1"/></geometry></visual>
                <visual>
                    <origin xyz="4 5 6"/>
                    <geometry><sphere radius="1"/></geometry>
                </visual>
            </link>
        </robot>"#,
    )
    .unwrap();
    let visuals = &robot.links[0].visuals;
    assert_eq!(
        origin_or_identity(visuals[0].origin),
        (v(1.0, -2.0, 3.5), v(0.1, -0.2, 0.3))
    );
    assert_eq!(origin_or_identity(visuals[1].origin), (ZERO, ZERO));
    assert_eq!(
        origin_or_identity(visuals[2].origin),
        (v(4.0, 5.0, 6.0), ZERO)
    );
}

#[test]
fn reads_materials_inline_and_top_level() {
    let robot = read_str(
        r#"<robot name="materials">
            <material name="red"><color rgba="1 0 0 1"/></material>
            <material name="glass"><color rgba="0.2 0.4 0.6 0.5"/></material>
            <link name="base">
                <visual>
                    <geometry><sphere radius="1"/></geometry>
                    <material name="blue"><color rgba="0 0 1 1"/></material>
                </visual>
                <visual>
                    <geometry><sphere radius="1"/></geometry>
                    <material name="red"/>
                </visual>
                <visual>
                    <geometry><sphere radius="1"/></geometry>
                    <material name=""><color rgba="0 1 0 1"/></material>
                </visual>
                <visual><geometry><sphere radius="1"/></geometry></visual>
            </link>
        </robot>"#,
    )
    .unwrap();

    assert_eq!(robot.materials.len(), 2);
    assert_eq!(robot.materials[0].name.as_deref(), Some("red"));
    assert_eq!(
        robot.materials[0].color.as_ref().unwrap().rgba,
        [1.0, 0.0, 0.0, 1.0]
    );
    assert_eq!(robot.materials[1].name.as_deref(), Some("glass"));
    assert_eq!(
        robot.materials[1].color.as_ref().unwrap().rgba,
        [0.2, 0.4, 0.6, 0.5]
    );

    let visuals = &robot.links[0].visuals;
    let blue = visuals[0].material.as_ref().unwrap();
    assert_eq!(blue.name.as_deref(), Some("blue"));
    assert_eq!(blue.color.as_ref().unwrap().rgba, [0.0, 0.0, 1.0, 1.0]);

    let reference = visuals[1].material.as_ref().unwrap();
    assert_eq!(reference.name.as_deref(), Some("red"));

    let unnamed = visuals[2].material.as_ref().unwrap();
    assert_eq!(unnamed.name, None);
    assert_eq!(unnamed.color.as_ref().unwrap().rgba, [0.0, 1.0, 0.0, 1.0]);

    assert!(visuals[3].material.is_none());
}

#[test]
fn maps_every_joint_type_to_its_urdf_name() {
    let types = [
        "revolute",
        "continuous",
        "prismatic",
        "fixed",
        "floating",
        "planar",
        "spherical",
    ];
    let mut xml = String::from(r#"<robot name="joints"><link name="base"/>"#);
    for (i, joint_type) in types.iter().enumerate() {
        xml += &format!(
            r#"<link name="l{i}"/>
            <joint name="j{i}" type="{joint_type}">
                <parent link="base"/><child link="l{i}"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>"#
        );
    }
    xml += "</robot>";
    let robot = read_str(&xml).unwrap();
    assert_eq!(robot.joints.len(), types.len());
    for (i, (joint, joint_type)) in robot.joints.iter().zip(types).enumerate() {
        assert_eq!(joint.name, format!("j{i}"));
        assert_eq!(joint.joint_type, joint_type);
        assert_eq!(joint.parent, "base");
        assert_eq!(joint.child, format!("l{i}"));
    }
}

#[test]
fn reads_joint_origin_axis_and_limits() {
    let robot = read_str(
        r#"<robot name="arm">
            <link name="base"/><link name="arm"/>
            <joint name="shoulder" type="revolute">
                <parent link="base"/><child link="arm"/>
                <origin xyz="0.1 0.2 0.3" rpy="0 1.5 -0.5"/>
                <axis xyz="0 1 0"/>
                <limit lower="-2.5" upper="1.25" effort="40" velocity="3.5"/>
            </joint>
        </robot>"#,
    )
    .unwrap();
    let joint = &robot.joints[0];
    assert_eq!(
        origin_or_identity(joint.origin),
        (v(0.1, 0.2, 0.3), v(0.0, 1.5, -0.5))
    );
    assert_eq!(joint.axis, Some(v(0.0, 1.0, 0.0)));
    let limit = joint.limit.unwrap();
    assert_eq!(limit.lower, -2.5);
    assert_eq!(limit.upper, 1.25);
    assert_eq!(limit.effort, 40.0);
    assert_eq!(limit.velocity, 3.5);
}

#[test]
fn joint_without_origin_or_axis_uses_spec_defaults() {
    let robot = read_str(
        r#"<robot name="arm">
            <link name="base"/><link name="arm"/>
            <joint name="spin" type="continuous">
                <parent link="base"/><child link="arm"/>
            </joint>
        </robot>"#,
    )
    .unwrap();
    let joint = &robot.joints[0];
    assert_eq!(origin_or_identity(joint.origin), (ZERO, ZERO));
    // URDF: the default axis is (1, 0, 0).
    assert_eq!(joint.axis.unwrap_or(v(1.0, 0.0, 0.0)), v(1.0, 0.0, 0.0));
}

#[test]
fn read_str_leaves_mesh_paths_unresolved() {
    for filename in [
        "meshes/part.stl",
        "../part.stl",
        "package://pkg/meshes/part.stl",
        "file:///opt/part.stl",
        "https://example.com/part.stl",
        "/abs/part.stl",
    ] {
        let robot = read_str(&mesh_robot(filename)).unwrap();
        assert_eq!(mesh_filename(&robot), filename);
    }
}

#[test]
fn read_str_rejects_invalid_documents() {
    for xml in [
        "",
        "not xml at all",
        r#"<robot name="broken"><link name="base">"#,
        r#"<not_a_robot name="x"/>"#,
        r#"<robot name="x"><link name="base"><visual><geometry><sphere radius="abc"/></geometry></visual></link></robot>"#,
    ] {
        assert!(read_str(xml).is_err(), "should reject: {xml:?}");
    }
}

#[test]
fn read_file_resolves_relative_mesh_paths() {
    let dir = TempDir::new("urdf");
    let mesh = dir.write("meshes/cube.stl", b"dummy");
    let urdf = dir.write("robot.urdf", mesh_robot("meshes/cube.stl"));

    let robot = read_file(&urdf).unwrap();
    assert_eq!(
        canonical(Path::new(mesh_filename(&robot))),
        canonical(&mesh)
    );
}

#[test]
fn read_file_resolves_parent_relative_mesh_paths() {
    let dir = TempDir::new("urdf");
    let mesh = dir.write("meshes/cube.stl", b"dummy");
    let urdf = dir.write("urdf/robot.urdf", mesh_robot("../meshes/cube.stl"));

    let robot = read_file(&urdf).unwrap();
    assert_eq!(
        canonical(Path::new(mesh_filename(&robot))),
        canonical(&mesh)
    );
}

#[test]
fn read_file_resolves_package_mesh_paths() {
    let dir = TempDir::new("urdf");
    let mesh = dir.write("description/meshes/cube.stl", b"dummy");
    let urdf = dir.write(
        "description/urdf/robot.urdf",
        mesh_robot("package://description/meshes/cube.stl"),
    );

    let robot = read_file(&urdf).unwrap();
    assert_eq!(mesh_filename(&robot), canonical(&mesh));
}

#[test]
fn read_file_resolves_package_in_nested_directory() {
    let dir = TempDir::new("urdf");
    let mesh = dir.write("ws/src/my_pkg/meshes/visual/cube.stl", b"dummy");
    let urdf = dir.write(
        "ws/src/my_pkg/urdf/robots/v2/robot.urdf",
        mesh_robot("package://my_pkg/meshes/visual/cube.stl"),
    );

    let robot = read_file(&urdf).unwrap();
    assert_eq!(mesh_filename(&robot), canonical(&mesh));
}

#[test]
fn read_file_leaves_unknown_package_and_absolute_uris() {
    let dir = TempDir::new("urdf");
    for filename in [
        "package://unknown_pkg/meshes/cube.stl",
        "file:///opt/cube.stl",
        "http://example.com/cube.stl",
        "https://example.com/cube.stl",
    ] {
        let urdf = dir.write("robot.urdf", mesh_robot(filename));
        let robot = read_file(&urdf).unwrap();
        assert_eq!(mesh_filename(&robot), filename);
    }
}

#[test]
fn read_file_keeps_absolute_mesh_paths() {
    let dir = TempDir::new("urdf");
    let mesh = dir.write("elsewhere/cube.stl", b"dummy");
    let absolute = canonical(&mesh);
    let urdf = dir.write("robot/robot.urdf", mesh_robot(&absolute));

    let robot = read_file(&urdf).unwrap();
    assert_eq!(mesh_filename(&robot), absolute);
}

#[test]
fn read_file_reads_joint_limits() {
    let dir = TempDir::new("urdf");
    let urdf = dir.write(
        "robot.urdf",
        r#"<robot name="demo"><link name="base"/><link name="arm"/><joint name="joint1" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/></joint></robot>"#,
    );

    let robot = read_file(&urdf).unwrap();
    let limit = robot.joints[0].limit.as_ref().unwrap();
    assert_eq!(limit.lower, -1.57);
    assert_eq!(limit.upper, 1.57);
    assert_eq!(limit.effort, 10.0);
    assert_eq!(limit.velocity, 2.0);
}

#[test]
fn read_file_and_read_str_agree_on_non_mesh_content() {
    let xml = r#"<robot name="same">
        <material name="red"><color rgba="1 0 0 1"/></material>
        <link name="base"><visual><origin xyz="1 2 3"/><geometry><box size="1 1 1"/></geometry></visual></link>
        <link name="arm"/>
        <joint name="j" type="prismatic"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
            <limit lower="0" upper="1" effort="1" velocity="1"/></joint>
    </robot>"#;
    let dir = TempDir::new("urdf");
    let urdf = dir.write("robot.urdf", xml);
    assert_eq!(read_file(&urdf).unwrap(), read_str(xml).unwrap());
}

#[test]
fn read_file_reports_missing_and_invalid_files() {
    let dir = TempDir::new("urdf");
    let missing = dir.path().join("missing.urdf");
    let error = read_file(&missing).unwrap_err();
    assert!(error.contains("missing.urdf"), "{error}");

    let invalid = dir.write("invalid.urdf", "<robot");
    assert!(read_file(&invalid).is_err());

    // A directory is not a URDF file.
    assert!(read_file(dir.path()).is_err());
}
