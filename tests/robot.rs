use std::collections::HashMap;

use r3v_renderer::{Robot, robot::UrdfRobot, urdf};

fn close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 1.0e-5,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn computes_link_transforms_independent_of_joint_order() {
    let description = urdf::read_str(
        r#"<robot name="chain">
            <link name="base"/>
            <link name="middle"/>
            <link name="tip"/>
            <joint name="middle_to_tip" type="fixed">
                <parent link="middle"/><child link="tip"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
            </joint>
            <joint name="base_to_middle" type="fixed">
                <parent link="base"/><child link="middle"/>
                <origin xyz="0 0 0" rpy="0 0 1.57079632679"/>
            </joint>
        </robot>"#,
    )
    .unwrap();
    let mut robot = UrdfRobot::from_description(description);

    let transforms = robot.feed_sensor_input(&HashMap::new());
    let tip = transforms.links.get("tip").unwrap();
    close(tip.linear.x, 0.0);
    close(tip.linear.y, 1.0);
    close(tip.linear.z, 0.0);
}

#[test]
fn includes_joint_motion_and_visual_origin_in_world_point() {
    let description = urdf::read_str(
        r#"<robot name="visual">
            <link name="base"/>
            <link name="arm">
                <visual>
                    <origin xyz="1 0 0" rpy="0 0 0"/>
                    <geometry><box size="1 1 1"/></geometry>
                </visual>
            </link>
            <joint name="shoulder" type="revolute">
                <parent link="base"/><child link="arm"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" effort="10" velocity="2"/>
            </joint>
        </robot>"#,
    )
    .unwrap();
    let mut robot = UrdfRobot::from_description(description);
    let transforms = robot.feed_sensor_input(&HashMap::from([(
        "shoulder".to_owned(),
        std::f32::consts::FRAC_PI_2,
    )]));

    let arm = transforms.links.get("arm").unwrap();
    close(arm.linear.x, 1.0);
    close(arm.linear.y, 0.0);

    let point = transforms
        .transform_visual_point("arm", 0, [0.0, 0.0, 0.0])
        .unwrap();
    close(point[0], 1.0);
    close(point[1], 1.0);
    close(point[2], 0.0);
}
