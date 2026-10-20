mod common;

use std::collections::HashMap;
use std::f32::consts::{FRAC_PI_2, PI, TAU};

use common::{close, close3, close3_eps};
use r3v_renderer::{
    Robot,
    robot::{RobotTransforms, UrdfRobot},
    urdf,
};

// Reference rotation math, built from rotation matrices (independent of quaternions).

type Mat3 = [[f32; 3]; 3];

fn mat_mul(a: Mat3, b: Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

fn apply(m: Mat3, v: [f32; 3]) -> [f32; 3] {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}

fn add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn rot_x(a: f32) -> Mat3 {
    let (s, c) = a.sin_cos();
    [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
}

fn rot_y(a: f32) -> Mat3 {
    let (s, c) = a.sin_cos();
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}

fn rot_z(a: f32) -> Mat3 {
    let (s, c) = a.sin_cos();
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

/// URDF rpy: fixed-axis roll about X, then pitch about Y, then yaw about Z.
fn rpy(roll: f32, pitch: f32, yaw: f32) -> Mat3 {
    mat_mul(rot_z(yaw), mat_mul(rot_y(pitch), rot_x(roll)))
}

/// Rodrigues' rotation formula around a (not necessarily unit) axis.
fn axis_angle(axis: [f32; 3], angle: f32) -> Mat3 {
    let length = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let [x, y, z] = axis.map(|c| c / length);
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Asserts that the frame of `link`'s first visual (which must have an identity origin) maps
/// points like the rigid transform (`rotation`, `translation`).
#[track_caller]
fn assert_frame(transforms: &RobotTransforms, link: &str, rotation: Mat3, translation: [f32; 3]) {
    let origin = transforms.get_link_origin(link).unwrap();
    close3_eps([origin.x, origin.y, origin.z], translation, 1.0e-4);
    for point in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.3, -1.2, 2.0],
    ] {
        let actual = transforms.transform_visual_point(link, 0, point).unwrap();
        close3_eps(actual, add(apply(rotation, point), translation), 1.0e-4);
    }
}

fn robot(xml: &str) -> UrdfRobot {
    UrdfRobot::from_description(urdf::read_str(xml).unwrap())
}

fn inputs(values: &[(&str, f32)]) -> HashMap<String, f32> {
    values
        .iter()
        .map(|(name, value)| (name.to_string(), *value))
        .collect()
}

const MARKER: &str = "<visual><geometry><sphere radius=\"0.1\"/></geometry></visual>";

/// A two-link robot with a single joint and a marker visual on the child link.
fn single_joint(joint_type: &str, origin: &str, axis: &str) -> UrdfRobot {
    robot(&format!(
        r#"<robot name="single">
            <link name="base"/>
            <link name="child">{MARKER}</link>
            <joint name="joint" type="{joint_type}">
                <parent link="base"/><child link="child"/>
                {origin}
                {axis}
                <limit lower="-10" upper="10" effort="1" velocity="1"/>
            </joint>
        </robot>"#
    ))
}

#[test]
fn last_transform_tracks_latest_feed() {
    let mut robot = single_joint("revolute", "", r#"<axis xyz="0 0 1"/>"#);
    assert!(robot.last_transform().is_none());

    let first = robot.feed_sensor_input(&inputs(&[("joint", 0.5)]));
    let stored = robot.last_transform().unwrap();
    assert_eq!(stored.links, first.links);
    assert_eq!(stored.visuals, first.visuals);

    let second = robot.feed_sensor_input(&inputs(&[("joint", -0.5)]));
    assert_ne!(first.links, second.links);
    let stored = robot.last_transform().unwrap();
    assert_eq!(stored.links, second.links);
    assert_eq!(stored.visuals, second.visuals);
}

#[test]
fn description_is_kept() {
    let xml = r#"<robot name="kept"><link name="base"/></robot>"#;
    let robot = robot(xml);
    assert_eq!(robot.description(), &urdf::read_str(xml).unwrap());
}

#[test]
fn single_link_is_at_world_origin() {
    let mut robot = robot(&format!(
        r#"<robot name="one"><link name="base">{MARKER}</link></robot>"#
    ));
    let transforms = robot.feed_sensor_input(&HashMap::new());
    assert_eq!(transforms.links.len(), 1);
    assert_eq!(transforms.visuals.len(), 1);
    assert_frame(&transforms, "base", IDENTITY, [0.0, 0.0, 0.0]);
}

#[test]
fn robot_without_links_has_no_transforms() {
    let mut robot = robot(r#"<robot name="empty"/>"#);
    let transforms = robot.feed_sensor_input(&HashMap::new());
    assert!(transforms.links.is_empty());
    assert!(transforms.visuals.is_empty());
}

#[test]
fn missing_inputs_mean_zero_position() {
    let mut robot = single_joint(
        "revolute",
        r#"<origin xyz="1 2 3" rpy="0 0 0"/>"#,
        r#"<axis xyz="0 0 1"/>"#,
    );
    let transforms = robot.feed_sensor_input(&HashMap::new());
    assert_frame(&transforms, "child", IDENTITY, [1.0, 2.0, 3.0]);
}

#[test]
fn unknown_inputs_are_ignored() {
    let mut robot = single_joint("revolute", "", r#"<axis xyz="0 0 1"/>"#);
    let baseline = robot.feed_sensor_input(&HashMap::new());
    let transforms = robot.feed_sensor_input(&inputs(&[("no_such_joint", 1.0), ("child", 2.0)]));
    assert_eq!(baseline.links, transforms.links);
    assert_eq!(baseline.visuals, transforms.visuals);
}

#[test]
fn revolute_and_continuous_rotate_about_axis() {
    for joint_type in ["revolute", "continuous"] {
        for axis in [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.3, -0.4, 0.8],
        ] {
            let mut robot = single_joint(
                joint_type,
                r#"<origin xyz="0.5 0 0" rpy="0 0 0"/>"#,
                &format!(r#"<axis xyz="{} {} {}"/>"#, axis[0], axis[1], axis[2]),
            );
            for angle in [0.0, 0.3, FRAC_PI_2, -FRAC_PI_2, PI, -2.5] {
                let transforms = robot.feed_sensor_input(&inputs(&[("joint", angle)]));
                assert_frame(
                    &transforms,
                    "child",
                    axis_angle(axis, angle),
                    [0.5, 0.0, 0.0],
                );
            }
        }
    }
}

#[test]
fn revolute_full_turn_is_identity() {
    let mut robot = single_joint("continuous", "", r#"<axis xyz="0 1 0"/>"#);
    for angle in [TAU, -TAU, 2.0 * TAU] {
        let transforms = robot.feed_sensor_input(&inputs(&[("joint", angle)]));
        assert_frame(&transforms, "child", IDENTITY, [0.0, 0.0, 0.0]);
    }
}

#[test]
fn rotation_does_not_depend_on_axis_length() {
    let mut unit = single_joint("revolute", "", r#"<axis xyz="0 0 1"/>"#);
    let mut long = single_joint("revolute", "", r#"<axis xyz="0 0 5"/>"#);
    let mut short = single_joint("revolute", "", r#"<axis xyz="0 0 0.01"/>"#);
    let values = inputs(&[("joint", 0.7)]);
    let expected = unit.feed_sensor_input(&values);
    for robot in [&mut long, &mut short] {
        let transforms = robot.feed_sensor_input(&values);
        let point = [1.0, 2.0, 3.0];
        close3(
            transforms
                .transform_visual_point("child", 0, point)
                .unwrap(),
            expected.transform_visual_point("child", 0, point).unwrap(),
        );
    }
}

#[test]
fn default_axis_is_x() {
    let mut robot = single_joint("revolute", "", "");
    let transforms = robot.feed_sensor_input(&inputs(&[("joint", 0.4)]));
    assert_frame(&transforms, "child", rot_x(0.4), [0.0, 0.0, 0.0]);

    let mut robot = single_joint("prismatic", "", "");
    let transforms = robot.feed_sensor_input(&inputs(&[("joint", 0.4)]));
    assert_frame(&transforms, "child", IDENTITY, [0.4, 0.0, 0.0]);
}

#[test]
fn prismatic_translates_along_axis() {
    for (axis, value, expected) in [
        ([0.0, 0.0, 1.0], 0.5, [0.0, 0.0, 0.5]),
        ([0.0, 0.0, 1.0], -0.25, [0.0, 0.0, -0.25]),
        ([1.0, 0.0, 0.0], 2.0, [2.0, 0.0, 0.0]),
        ([0.0, -1.0, 0.0], 1.0, [0.0, -1.0, 0.0]),
        ([0.6, 0.8, 0.0], 1.0, [0.6, 0.8, 0.0]),
    ] {
        let mut robot = single_joint(
            "prismatic",
            "",
            &format!(r#"<axis xyz="{} {} {}"/>"#, axis[0], axis[1], axis[2]),
        );
        let transforms = robot.feed_sensor_input(&inputs(&[("joint", value)]));
        assert_frame(&transforms, "child", IDENTITY, expected);
    }
}

#[test]
fn prismatic_distance_does_not_depend_on_axis_length() {
    // The joint value is a distance in metres along the axis direction.
    let mut robot = single_joint("prismatic", "", r#"<axis xyz="0 2 0"/>"#);
    let transforms = robot.feed_sensor_input(&inputs(&[("joint", 0.5)]));
    assert_frame(&transforms, "child", IDENTITY, [0.0, 0.5, 0.0]);

    let mut robot = single_joint("prismatic", "", r#"<axis xyz="3 4 0"/>"#);
    let transforms = robot.feed_sensor_input(&inputs(&[("joint", 1.0)]));
    assert_frame(&transforms, "child", IDENTITY, [0.6, 0.8, 0.0]);
}

#[test]
fn zero_axis_produces_no_motion_and_no_nan() {
    for joint_type in ["revolute", "continuous", "prismatic"] {
        let mut robot = single_joint(
            joint_type,
            r#"<origin xyz="1 0 0" rpy="0 0 0"/>"#,
            r#"<axis xyz="0 0 0"/>"#,
        );
        let transforms = robot.feed_sensor_input(&inputs(&[("joint", 1.0)]));
        assert_frame(&transforms, "child", IDENTITY, [1.0, 0.0, 0.0]);
    }
}

#[test]
fn non_actuated_joint_types_ignore_input() {
    for joint_type in ["fixed", "floating", "planar", "spherical"] {
        let mut robot = single_joint(
            joint_type,
            r#"<origin xyz="0 1 0" rpy="0 0 0.5"/>"#,
            r#"<axis xyz="0 0 1"/>"#,
        );
        let transforms = robot.feed_sensor_input(&inputs(&[("joint", 1.3)]));
        assert_frame(&transforms, "child", rot_z(0.5), [0.0, 1.0, 0.0]);
    }
}

#[test]
fn joint_origin_rpy_follows_urdf_convention() {
    for (r, p, y) in [
        (0.7, 0.0, 0.0),
        (0.0, 0.7, 0.0),
        (0.0, 0.0, 0.7),
        (0.3, -0.5, 1.1),
        (-1.2, 0.4, -2.8),
        (PI, FRAC_PI_2 - 0.01, -PI),
    ] {
        let mut robot = single_joint(
            "fixed",
            &format!(r#"<origin xyz="1 -2 0.5" rpy="{r} {p} {y}"/>"#),
            "",
        );
        let transforms = robot.feed_sensor_input(&HashMap::new());
        assert_frame(&transforms, "child", rpy(r, p, y), [1.0, -2.0, 0.5]);
    }
}

#[test]
fn joint_axis_is_expressed_in_joint_frame() {
    // Joint frame yawed by 90 degrees: its x axis is the parent's y axis.
    let mut robot = single_joint(
        "prismatic",
        r#"<origin xyz="0 0 0" rpy="0 0 1.5707963"/>"#,
        r#"<axis xyz="1 0 0"/>"#,
    );
    let transforms = robot.feed_sensor_input(&inputs(&[("joint", 2.0)]));
    assert_frame(&transforms, "child", rot_z(FRAC_PI_2), [0.0, 2.0, 0.0]);

    // Joint frame rolled by 90 degrees: its z axis is the parent's -y axis.
    let mut robot = single_joint(
        "revolute",
        r#"<origin xyz="0 0 1" rpy="1.5707963 0 0"/>"#,
        r#"<axis xyz="0 0 1"/>"#,
    );
    let transforms = robot.feed_sensor_input(&inputs(&[("joint", 0.6)]));
    assert_frame(
        &transforms,
        "child",
        mat_mul(rot_x(FRAC_PI_2), rot_z(0.6)),
        [0.0, 0.0, 1.0],
    );
}

#[test]
fn transforms_compose_along_a_chain() {
    let mut robot = robot(&format!(
        r#"<robot name="chain">
            <link name="base"/>
            <link name="upper">{MARKER}</link>
            <link name="lower">{MARKER}</link>
            <link name="tip">{MARKER}</link>
            <joint name="shoulder" type="revolute">
                <parent link="base"/><child link="upper"/>
                <origin xyz="0 0 1" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
            <joint name="elbow" type="revolute">
                <parent link="upper"/><child link="lower"/>
                <origin xyz="2 0 0" rpy="0 0 0"/>
                <axis xyz="0 1 0"/>
                <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
            <joint name="slide" type="prismatic">
                <parent link="lower"/><child link="tip"/>
                <origin xyz="1 0 0" rpy="0.2 0 0"/>
                <axis xyz="1 0 0"/>
                <limit lower="0" upper="1" effort="1" velocity="1"/>
            </joint>
        </robot>"#
    ));
    let (a, b, d) = (0.8, -0.6, 0.3);
    let transforms =
        robot.feed_sensor_input(&inputs(&[("shoulder", a), ("elbow", b), ("slide", d)]));

    let upper_r = rot_z(a);
    let upper_t = [0.0, 0.0, 1.0];
    assert_frame(&transforms, "upper", upper_r, upper_t);

    let lower_r = mat_mul(upper_r, rot_y(b));
    let lower_t = add(upper_t, apply(upper_r, [2.0, 0.0, 0.0]));
    assert_frame(&transforms, "lower", lower_r, lower_t);

    let tip_r = mat_mul(lower_r, rot_x(0.2));
    let tip_t = add(
        lower_t,
        apply(
            lower_r,
            add([1.0, 0.0, 0.0], apply(rot_x(0.2), [d, 0.0, 0.0])),
        ),
    );
    assert_frame(&transforms, "tip", tip_r, tip_t);
}

#[test]
fn branches_and_multiple_roots_are_all_placed() {
    let mut robot = robot(&format!(
        r#"<robot name="tree">
            <link name="base">{MARKER}</link>
            <link name="left">{MARKER}</link>
            <link name="right">{MARKER}</link>
            <link name="loose">{MARKER}</link>
            <joint name="to_left" type="fixed">
                <parent link="base"/><child link="left"/>
                <origin xyz="0 1 0" rpy="0 0 0"/>
            </joint>
            <joint name="to_right" type="continuous">
                <parent link="base"/><child link="right"/>
                <origin xyz="0 -1 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
            </joint>
        </robot>"#
    ));
    let transforms = robot.feed_sensor_input(&inputs(&[("to_right", 1.0)]));
    assert_eq!(transforms.links.len(), 4);
    assert_frame(&transforms, "base", IDENTITY, [0.0, 0.0, 0.0]);
    assert_frame(&transforms, "loose", IDENTITY, [0.0, 0.0, 0.0]);
    assert_frame(&transforms, "left", IDENTITY, [0.0, 1.0, 0.0]);
    assert_frame(&transforms, "right", rot_z(1.0), [0.0, -1.0, 0.0]);
}

#[test]
fn computes_link_transforms_independent_of_joint_order() {
    let mut robot = robot(&format!(
        r#"<robot name="chain">
            <link name="base"/>
            <link name="middle"/>
            <link name="tip">{MARKER}</link>
            <joint name="middle_to_tip" type="fixed">
                <parent link="middle"/><child link="tip"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
            </joint>
            <joint name="base_to_middle" type="fixed">
                <parent link="base"/><child link="middle"/>
                <origin xyz="0 0 0" rpy="0 0 1.57079632679"/>
            </joint>
        </robot>"#
    ));
    let transforms = robot.feed_sensor_input(&HashMap::new());
    assert_frame(&transforms, "tip", rot_z(FRAC_PI_2), [0.0, 1.0, 0.0]);
}

#[test]
fn malformed_trees_do_not_hang_or_panic() {
    // Cycle: no root link.
    let mut cycle = robot(
        r#"<robot name="cycle">
            <link name="a"/><link name="b"/>
            <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
            <joint name="ba" type="fixed"><parent link="b"/><child link="a"/></joint>
        </robot>"#,
    );
    let transforms = cycle.feed_sensor_input(&HashMap::new());
    assert!(transforms.links.values().all(|t| t.linear.x.is_finite()));

    // Joint whose parent link does not exist.
    let mut dangling = robot(
        r#"<robot name="dangling">
            <link name="base"/><link name="orphan"/>
            <joint name="j" type="fixed"><parent link="ghost"/><child link="orphan"/></joint>
        </robot>"#,
    );
    let transforms = dangling.feed_sensor_input(&HashMap::new());
    assert!(transforms.get_link_origin("base").is_some());
}

#[test]
fn visual_origin_is_applied_in_link_frame() {
    let mut robot = robot(
        r#"<robot name="visual">
            <link name="base"/>
            <link name="arm">
                <visual>
                    <origin xyz="1 0 0" rpy="0 0 0"/>
                    <geometry><box size="1 1 1"/></geometry>
                </visual>
                <visual>
                    <origin xyz="0 0 2" rpy="0 1.5707963 0"/>
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
    );
    let transforms = robot.feed_sensor_input(&inputs(&[("shoulder", FRAC_PI_2)]));

    let arm = transforms.get_link_origin("arm").unwrap();
    close3([arm.x, arm.y, arm.z], [1.0, 0.0, 0.0]);

    let link_r = rot_z(FRAC_PI_2);
    let link_t = [1.0, 0.0, 0.0];
    for point in [[0.0, 0.0, 0.0], [0.5, -0.5, 0.25]] {
        let first = transforms.transform_visual_point("arm", 0, point).unwrap();
        let expected = add(link_t, apply(link_r, add([1.0, 0.0, 0.0], point)));
        close3_eps(first, expected, 1.0e-4);

        let second = transforms.transform_visual_point("arm", 1, point).unwrap();
        let local = add([0.0, 0.0, 2.0], apply(rot_y(FRAC_PI_2), point));
        close3_eps(second, add(link_t, apply(link_r, local)), 1.0e-4);
    }
}

#[test]
fn visual_lookups_for_unknown_entries_return_none() {
    let mut robot = single_joint("fixed", "", "");
    let transforms = robot.feed_sensor_input(&HashMap::new());
    assert!(
        transforms
            .transform_visual_point("child", 0, [0.0; 3])
            .is_some()
    );
    assert!(
        transforms
            .transform_visual_point("child", 1, [0.0; 3])
            .is_none()
    );
    assert!(
        transforms
            .transform_visual_point("base", 0, [0.0; 3])
            .is_none()
    );
    assert!(
        transforms
            .transform_visual_point("nope", 0, [0.0; 3])
            .is_none()
    );
    assert!(transforms.get_link_origin("nope").is_none());
    assert!(transforms.get_link_origin("base").is_some());
}

#[test]
fn link_rotations_are_unit_quaternions() {
    let mut robot = single_joint(
        "revolute",
        r#"<origin xyz="0 0 0" rpy="0.3 0.2 0.1"/>"#,
        r#"<axis xyz="1 2 3"/>"#,
    );
    let transforms = robot.feed_sensor_input(&inputs(&[("joint", 12.345)]));
    for twist in transforms.links.values().chain(transforms.visuals.values()) {
        let q = twist.rotational;
        close((q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt(), 1.0);
    }
}
