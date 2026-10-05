use r3v_renderer::urdf::UrdfRobot;

fn main() {
    let path = "scorpio8.urdf.xacro";
    match UrdfRobot::from_file(path) {
        Ok(robot) => {
            println!("robot name: {}", robot.name);
            println!("links: {}", robot.links.len());
            println!("joints: {}", robot.joints.len());
            println!("materials: {}", robot.materials.len());

            let first_mesh = robot
                .links
                .iter()
                .flat_map(|link| &link.visuals)
                .filter_map(|visual| visual.mesh_filename())
                .next();

            println!("first mesh: {:?}", first_mesh);

            if let Some(joint) = robot.joints.first() {
                println!("first joint: {} -> {} ({})", joint.parent, joint.child, joint.joint_type);
            }
        }
        Err(err) => {
            eprintln!("URDF parse failed: {err}");
            std::process::exit(1);
        }
    }
}
