use std::path::Path;

use r3v_renderer::{stl::resolve_stl_reference, urdf::UrdfRobot};

fn main() {
    let urdf_path = Path::new("examples/test_data/sample_robot.urdf");

    let robot = match UrdfRobot::from_file(urdf_path) {
        Ok(robot) => robot,
        Err(err) => {
            eprintln!("URDF parse failed: {err}");
            std::process::exit(1);
        }
    };

    let mut mesh_count = 0usize;
    let mut triangle_total = 0usize;

    for link in &robot.links {
        for visual in &link.visuals {
            if let Some(mesh_ref) = visual.mesh_filename() {
                let mesh_path = resolve_stl_reference(urdf_path, mesh_ref);
                println!("Reading mesh: {} -> {}", mesh_ref, mesh_path.display());

                match r3v_renderer::StlMesh::from_file(&mesh_path) {
                    Ok(mesh) => {
                        mesh_count += 1;
                        triangle_total += mesh.triangle_count();
                        println!("  OK: {} triangles, {} vertices", mesh.triangle_count(), mesh.vertex_count());
                    }
                    Err(err) => {
                        eprintln!("  FAILED: {err}");
                        std::process::exit(1);
                    }
                }
            }
        }
    }

    if mesh_count == 0 {
        eprintln!("No mesh references were found in the URDF.");
        std::process::exit(1);
    }

    println!("Summary: {mesh_count} STL file(s) parsed successfully, {triangle_total} triangles loaded.");
    println!("Status: STL files referenced by the URDF are readable and ready for rendering pipeline input.");
}
