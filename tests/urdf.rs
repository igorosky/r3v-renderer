use r3v_renderer::urdf::{UrdfGeometry, read_file};

#[test]
fn read_file_resolves_relative_mesh_paths() {
    let unique = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let root = std::env::temp_dir().join(format!("r3v-urdf-{unique}"));
    let meshes = root.join("meshes");
    std::fs::create_dir_all(&meshes).unwrap();
    std::fs::write(meshes.join("cube.stl"), b"dummy").unwrap();

    let urdf_path = root.join("robot.urdf");
    std::fs::write(
        &urdf_path,
        r#"<robot name="demo"><link name="base"><visual><geometry><mesh filename="meshes/cube.stl"/></geometry></visual></link></robot>"#,
    )
    .unwrap();

    let robot = read_file(&urdf_path).unwrap();
    let geometry = &robot.links[0].visuals[0].geometry;
    match geometry {
        Some(UrdfGeometry::Mesh(mesh)) => {
            assert!(mesh.filename.ends_with("meshes/cube.stl"));
        }
        other => panic!("expected mesh geometry, got: {other:?}"),
    }

    std::fs::remove_dir_all(root).unwrap();
}

#[test]
fn read_file_resolves_package_mesh_paths() {
    let unique = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let root = std::env::temp_dir().join(format!("r3v-package-{unique}"));
    let package_dir = root.join("description");
    let meshes = package_dir.join("meshes");
    let urdf_dir = package_dir.join("urdf");
    std::fs::create_dir_all(&meshes).unwrap();
    std::fs::create_dir_all(&urdf_dir).unwrap();
    std::fs::write(meshes.join("cube.stl"), b"dummy").unwrap();

    let urdf_path = urdf_dir.join("robot.urdf");
    std::fs::write(
        &urdf_path,
        r#"<robot name="demo"><link name="base"><visual><geometry><mesh filename="package://description/meshes/cube.stl"/></geometry></visual></link></robot>"#,
    )
    .unwrap();

    let robot = read_file(&urdf_path).unwrap();
    let geometry = &robot.links[0].visuals[0].geometry;
    match geometry {
        Some(UrdfGeometry::Mesh(mesh)) => {
            let expected = meshes
                .join("cube.stl")
                .canonicalize()
                .unwrap()
                .to_string_lossy()
                .replace('\\', "/");
            assert_eq!(mesh.filename, expected);
        }
        other => panic!("expected mesh geometry, got: {other:?}"),
    }

    std::fs::remove_dir_all(root).unwrap();
}

#[test]
fn read_file_reads_joint_limits() {
    let unique = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let root = std::env::temp_dir().join(format!("r3v-urdf-limit-{unique}"));
    let urdf_path = root.join("robot.urdf");
    std::fs::create_dir_all(&root).unwrap();
    std::fs::write(
        &urdf_path,
        r#"<robot name="demo"><link name="base"/><link name="arm"/><joint name="joint1" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/></joint></robot>"#,
    )
    .unwrap();

    let robot = read_file(&urdf_path).unwrap();
    let limit = robot.joints[0].limit.as_ref().unwrap();
    assert_eq!(limit.lower, -1.57);
    assert_eq!(limit.upper, 1.57);
    assert_eq!(limit.effort, 10.0);
    assert_eq!(limit.velocity, 2.0);

    std::fs::remove_dir_all(root).unwrap();
}
