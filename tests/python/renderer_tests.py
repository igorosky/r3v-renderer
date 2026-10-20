"""
Test cases for the Python bindings: the Python facing API (classes,
properties, argument conversion, exceptions) and a port of the C binding
tests in tests/c/renderer_tests.c.
"""

import array
import math
import sys

import r3v_renderer as rr

from harness import (
    add_block, check, close, lit, main, make_renderer, make_tesselator, raises, render_now, temp_path, write_file,
)

U32_MAX = 2**32 - 1
HALF_PI = math.pi / 2
# Pitch limit of the orbit camera (src/camera.rs).
SAFE_PITCH = HALF_PI - 0.01
IDENTITY = rr.Twist()

# A triangle facing +Z, around the origin.
FRONT_TRIANGLE = rr.Triangle((-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (0.0, 1.0, 0.0))

ROBOT_URDF = """
<robot name="test_bot">
  <material name="red"><color rgba="1 0 0 1"/></material>
  <link name="base">
    <visual name="body">
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/base.stl" scale="2 2 2"/></geometry>
    </visual>
  </link>
  <link name="arm">
    <visual>
      <geometry><box size="1 2 3"/></geometry>
      <material name="red"/>
    </visual>
    <visual><geometry><cylinder radius="0.5" length="2"/></geometry></visual>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="2"/>
  </joint>
</robot>
"""


class VoxelizerPointI:
    """Stands in for r3v_voxelizer.PointI: the same interface, another class."""

    def __init__(self, x, y, z, i):
        self._values = (x, y, z, i)

    def x(self):
        return self._values[0]

    def y(self):
        return self._values[1]

    def z(self):
        return self._values[2]

    def i(self):
        return self._values[3]


def project(state, point):
    """view_proj * (point, 1), divided by w."""
    v = (*point, 1.0)
    clip = [sum(state.view_proj[column][row] * v[column] for column in range(4)) for row in range(4)]
    return tuple(c / clip[3] for c in clip[:3])


# ------------------------------------------------------------ Python API

CLASSES = [
    "CameraState", "GeometryDelta", "GreedyMeshTesselator", "GreedyMeshTesselatorConfiguration", "OrbitCamera",
    "PointI", "Quad", "QuadChunk", "RobotTransforms", "StlMesh", "Triangle", "Twist", "UrdfBox", "UrdfCapsule",
    "UrdfColor", "UrdfCylinder", "UrdfDescription", "UrdfJoint", "UrdfJointLimit", "UrdfLink", "UrdfMaterial",
    "UrdfMesh", "UrdfOrigin", "UrdfRobot", "UrdfSphere", "UrdfVisual", "WgpuRenderer", "WgpuRendererConfiguration",
]


def module_exports_its_classes():
    public = sorted(name for name in dir(rr) if not name.startswith("_"))
    check(public == CLASSES, f"module exports {public}")
    for name in CLASSES:
        cls = getattr(rr, name)
        check(isinstance(cls, type), f"r3v_renderer.{name} is no class")
        check(cls.__module__ == "r3v_renderer", f"{name} belongs to {cls.__module__}")
        check(cls.__name__ == name, f"{name} is named {cls.__name__}")


def point_i_has_the_voxelizer_interface():
    p = rr.PointI(4, -8, 2, U32_MAX)
    check((p.x(), p.y(), p.z(), p.i()) == (4, -8, 2, U32_MAX))
    p.set_x(8)
    p.set_y(4)
    p.set_z(-2)
    p.set_i(1)
    check((p.x(), p.y(), p.z(), p.i()) == (8, 4, -2, 1))
    check(rr.PointI(x=1, y=2, z=3, i=4) == rr.PointI(1, 2, 3, 4))
    check(rr.PointI(1, 2, 3, 4) != rr.PointI(1, 2, 3, 5))
    check(repr(rr.PointI(1, 2, 3, 4)) == "PointI(1, 2, 3, 4)")
    with raises(TypeError):
        rr.PointI(1.5, 0, 0, 0)
    with raises(OverflowError):
        rr.PointI(0, 0, 0, -1)
    with raises(OverflowError):
        rr.PointI(2**31, 0, 0, 0)


def value_classes_have_properties():
    twist = rr.Twist()
    check(twist.linear == (0.0, 0.0, 0.0) and twist.rotational == (0.0, 0.0, 0.0, 1.0))
    twist.linear = (1.0, 2.0, 3.0)
    check(twist == rr.Twist(linear=(1, 2, 3)))
    check(repr(twist) == "Twist(linear=(1.0, 2.0, 3.0), rotational=(0.0, 0.0, 0.0, 1.0))")

    triangle = rr.Triangle((0, 0, 0), (1, 0, 0), (0, 1, 0))
    check(triangle.v1 == (1.0, 0.0, 0.0))
    triangle.v2 = (0, 2, 0)
    check(triangle == rr.Triangle((0, 0, 0), (1, 0, 0), (0, 2, 0)))

    configuration = rr.GreedyMeshTesselatorConfiguration(0.5)
    configuration.voxel_size = 0.25
    check(configuration == rr.GreedyMeshTesselatorConfiguration(voxel_size=0.25))
    configuration = rr.WgpuRendererConfiguration(width=640, height=480)
    check((configuration.width, configuration.height) == (640, 480))
    with raises(OverflowError):
        rr.WgpuRendererConfiguration(-1, 1)

    state = rr.CameraState([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], (0, 0, 5), (0, 0, -1))
    check(state.view_proj[3] == [0.0, 0.0, 0.0, 1.0])
    check(state == rr.CameraState(state.view_proj, state.position, state.forward))
    with raises(ValueError, "length 3"):
        rr.Twist(linear=(1, 2))


def output_classes_can_not_be_created():
    for cls in (rr.Quad, rr.QuadChunk, rr.GeometryDelta, rr.RobotTransforms, rr.UrdfDescription, rr.UrdfRobot,
                rr.UrdfLink, rr.UrdfVisual, rr.UrdfMesh):
        with raises(TypeError):
            cls()


# ---------------------------------------------------------------- camera


def camera_default_looks_down_negative_z():
    camera = rr.OrbitCamera()
    close(camera.target, (0, 0, 0))
    close(camera.distance, 5)
    close((camera.yaw, camera.pitch), (0, 0))
    close((camera.near, camera.far), (0.1, 100))
    check(camera.new_uniform_available)
    close(camera.forward(), (0, 0, -1))
    state = camera.get_state(1.0)
    close(state.position, (0, 0, 5))
    close(state.forward, (0, 0, -1))
    check(repr(camera).startswith("OrbitCamera(target=(0.0, 0.0, 0.0), distance=5.0"))


def camera_new_takes_degrees():
    camera = rr.OrbitCamera((1, 2, 3), 2, 90, 0)
    close(camera.yaw, HALF_PI)
    state = camera.get_state(1.0)
    close(state.position, (3, 2, 3))
    close(state.forward, (-1, 0, 0))
    check(rr.OrbitCamera(target=(1, 2, 3), distance=2, yaw_deg=90) == camera)


def camera_view_proj_is_column_major():
    state = rr.OrbitCamera().get_state(2.0)
    x, y, z = project(state, (0, 0, 0))
    close((x, y), (0, 0))
    check(0 < z < 1)
    check(project(state, (1, 0, 0))[0] > 0)
    check(project(state, (0, 1, 0))[1] > 0)


def camera_mutations_match_rust():
    camera = rr.OrbitCamera()
    camera.new_uniform_available = False
    camera.rotate(0.5, 0.25)
    close((camera.yaw, camera.pitch), (0.5, 0.25))
    check(camera.new_uniform_available)
    camera.rotate(0, 10)
    close(camera.pitch, SAFE_PITCH)
    camera.zoom(-100)
    close(camera.distance, 0.1)

    camera = rr.OrbitCamera()
    camera.pan(1, 2)
    close(camera.target, (1, 2, 0))
    camera.look_at((4, 5, 6), 0, 90, 90)
    close(camera.target, (4, 5, 6))
    close(camera.distance, 0.1)
    close((camera.yaw, camera.pitch), (HALF_PI, SAFE_PITCH))


def camera_fields_are_writable():
    camera = rr.OrbitCamera()
    camera.target = (1, 0, 0)
    camera.distance = 2
    camera.yaw = HALF_PI
    close(camera.get_state(1.0).position, (3, 0, 0))


def camera_with_clip_planes_returns_a_copy():
    camera = rr.OrbitCamera()
    clipped = camera.with_clip_planes(0.5, 50)
    close((clipped.near, clipped.far), (0.5, 50))
    close((camera.near, camera.far), (0.1, 100))


def camera_get_state_checks_arguments():
    camera = rr.OrbitCamera()
    for aspect_ratio in (0, -1, math.nan, math.inf):
        with raises(ValueError, "aspect ratio"):
            camera.get_state(aspect_ratio)
    for near, far in ((0, 100), (1, 1), (0.1, math.inf)):
        with raises(ValueError, "clip planes"):
            camera.with_clip_planes(near, far).get_state(1.0)


# ------------------------------------------------------------------- stl


def stl_read_ascii_file():
    path = temp_path("triangle.stl")
    write_file(path, "solid t\nfacet normal 0 0 1\n outer loop\n  vertex 0 0 0\n  vertex 1 0 0\n"
                     "  vertex 0 1 0\n endloop\nendfacet\nendsolid t\n")
    mesh = rr.StlMesh.read_file(path)
    check(mesh.triangles == [rr.Triangle((0, 0, 0), (1, 0, 0), (0, 1, 0))])
    check(repr(mesh) == "StlMesh(<1 triangles>)")


def stl_missing_file_fails():
    with raises(RuntimeError, "missing.stl"):
        rr.StlMesh.read_file(temp_path("missing.stl"))
    with raises(TypeError):
        rr.StlMesh.read_file(None)


def stl_triangles_are_copies():
    other = rr.Triangle((0, 0, 0), (1, 0, 0), (0, 0, 7))
    mesh = rr.StlMesh([FRONT_TRIANGLE, other])
    triangles = mesh.triangles
    check(triangles == [FRONT_TRIANGLE, other])
    # Changing a copy does not change the mesh, assigning does.
    triangles[0].v0 = (42, 0, 0)
    check(mesh.triangles[0] == FRONT_TRIANGLE)
    mesh.triangles = [rr.Triangle(*(tuple(3 * c for c in v) for v in (t.v0, t.v1, t.v2))) for t in triangles]
    close(mesh.triangles[1].v2, (0, 0, 21))
    check(len(rr.StlMesh([]).triangles) == 0)
    with raises(TypeError):
        rr.StlMesh([(0, 0, 0)])


# ------------------------------------------------------------ tesselator


def tesselator_requires_configure():
    tesselator = rr.GreedyMeshTesselator()
    with raises(RuntimeError, "configure"):
        tesselator.update_input_geometry([rr.PointI(0, 0, 0, 1)])
    with raises(RuntimeError, "configure"):
        tesselator.get_output_geometry_delta()


def tesselator_checks_voxel_size():
    tesselator = rr.GreedyMeshTesselator()
    for voxel_size in (0, -1, math.nan, math.inf):
        with raises(ValueError, "voxel size"):
            tesselator.configure(rr.GreedyMeshTesselatorConfiguration(voxel_size))
    with raises(RuntimeError):
        tesselator.update_input_geometry([])


def tesselator_rejects_unaligned_blocks():
    tesselator = make_tesselator(1.0)
    for unaligned in (rr.PointI(1, 0, 0, 1), rr.PointI(0, 2, 0, 1), rr.PointI(0, 0, -1, 1)):
        # A valid block first: none of them may be applied.
        with raises(ValueError, "blocks[1]"):
            tesselator.update_input_geometry([rr.PointI(4, 4, 2, 1), unaligned])
    check(tesselator.get_output_geometry_delta() is None)


def tesselator_full_block_is_a_box():
    tesselator = make_tesselator(0.5)
    # Inside its chunk, so the neighbouring chunks stay untouched.
    block = rr.PointI(4, 4, 2, U32_MAX)
    tesselator.update_input_geometry([block])
    delta = tesselator.get_output_geometry_delta()
    check(len(delta) == 1)
    check(repr(delta) == "GeometryDelta(<1 chunks>)")
    chunk = delta[0]
    check(chunk.index == (0, 0, 0))
    check(len(chunk.quads) == 6)
    # Voxels 4..8 x 4..8 x 2..4 at 0.5 world units each.
    lo, hi = (2, 2, 1), (4, 4, 2)
    for quad in chunk.quads:
        for corner in (quad.v0, quad.v1, quad.v2, quad.v3):
            check(all(lo[a] - 1e-4 <= corner[a] <= hi[a] + 1e-4 for a in range(3)), f"{quad}")
    check(delta[-1].index == chunk.index)
    check([c.index for c in delta] == [(0, 0, 0)])
    with raises(IndexError, "chunk 1"):
        delta[1]
    with raises(IndexError):
        delta[-2]

    # Nothing changed since, sending the same block changes nothing either.
    check(tesselator.get_output_geometry_delta() is None)
    tesselator.update_input_geometry([block])
    check(tesselator.get_output_geometry_delta() is None)

    # Clearing it leaves the chunk without quads.
    block.set_i(0)
    tesselator.update_input_geometry([block])
    delta = tesselator.get_output_geometry_delta()
    check([len(c.quads) for c in delta] == [0])


def tesselator_takes_voxelizer_points():
    tesselator = make_tesselator(1.0)
    # Any iterable of objects with x(), y(), z() and i(), also mixed.
    blocks = (b for b in [VoxelizerPointI(4, 4, 2, U32_MAX), rr.PointI(8, 4, 2, U32_MAX)])
    tesselator.update_input_geometry(blocks)
    delta = tesselator.get_output_geometry_delta()
    # Two blocks side by side make one box.
    check([len(c.quads) for c in delta] == [6])
    tesselator.update_input_geometry(())
    check(tesselator.get_output_geometry_delta() is None)


def tesselator_rejects_other_objects():
    tesselator = make_tesselator(1.0)
    with raises(TypeError, "blocks[1] is a tuple, not a PointI"):
        tesselator.update_input_geometry([rr.PointI(0, 0, 0, 1), (0, 0, 0, 1)])
    with raises(TypeError):
        tesselator.update_input_geometry(5)
    with raises(OverflowError):
        tesselator.update_input_geometry([VoxelizerPointI(0, 0, 0, -1)])
    check(tesselator.get_output_geometry_delta() is None)


# -------------------------------------------------------------- renderer


def renderer_configure_checks_size():
    renderer = make_renderer(8, 8)
    for width, height in ((0, 8), (8, 0), (8193, 8), (8, 8193)):
        with raises(ValueError, "frame size"):
            renderer.configure(rr.WgpuRendererConfiguration(width, height))
    renderer.configure(rr.WgpuRendererConfiguration(8192, 1))


def renderer_render_checks_arguments():
    renderer = rr.WgpuRenderer()
    state = rr.OrbitCamera().get_state(1.0)
    with raises(RuntimeError, "configure"):
        renderer.render(state, bytearray(64))
    renderer.configure(rr.WgpuRendererConfiguration(4, 4))
    with raises(ValueError, "64 bytes"):
        renderer.render(state, bytearray(63))
    with raises(ValueError, "writable"):
        renderer.render(state, bytes(64))
    with raises(ValueError, "contiguous"):
        renderer.render(state, memoryview(bytearray(128))[::2])
    with raises(BufferError):
        renderer.render(state, array.array("f", [0.0] * 64))
    with raises(TypeError):
        renderer.render(state, "x" * 64)
    with raises(TypeError):
        renderer.render(rr.OrbitCamera(), bytearray(64))


def renderer_writes_any_byte_buffer():
    renderer = make_renderer(4, 4)
    renderer.upload_asset(rr.StlMesh([FRONT_TRIANGLE]), 1)
    state = rr.OrbitCamera().get_state(1.0)
    renderer.render(state, bytearray(64))
    for out in (bytearray(64), array.array("B", [0] * 64), memoryview(bytearray(64))):
        renderer.render(state, out)
        check(lit(bytes(out), 4, 2, 2), f"nothing rendered into {type(out).__name__}")
    # A larger buffer is fine, only the frame is written.
    larger = bytearray(b"\xab" * 80)
    renderer.render(state, larger)
    check(larger[64:] == b"\xab" * 16)


def renderer_first_frame_is_zeros():
    renderer = make_renderer(16, 16)
    state = rr.OrbitCamera().get_state(1.0)
    frame = bytearray(b"\xab" * (16 * 16 * 4))
    renderer.render(state, frame)
    check(not any(frame))
    # The next one is the cleared first frame: black, opaque.
    renderer.render(state, frame)
    check(not lit(frame, 16, 8, 8))
    check(frame[3] == 255)


def renderer_draws_voxels():
    w = h = 32
    renderer = make_renderer(w, h)
    # Voxels 4..8 x 4..8 x 2..4.
    tesselator = add_block(renderer, 1.0, 4, 4, 2)
    camera = rr.OrbitCamera((6, 6, 3), 10)
    frame = render_now(renderer, camera, w, h)
    check(lit(frame, w, w // 2, h // 2))
    check(not lit(frame, w, 0, 0))
    check(not lit(frame, w, w - 1, h - 1))

    # Clearing the block clears the frame.
    tesselator.update_input_geometry([rr.PointI(4, 4, 2, 0)])
    renderer.update_geometry(tesselator.get_output_geometry_delta())
    check(not lit(render_now(renderer, camera, w, h), w, w // 2, h // 2))


def renderer_update_geometry_takes_the_delta():
    renderer = make_renderer(4, 4)
    tesselator = make_tesselator(1.0)
    tesselator.update_input_geometry([rr.PointI(0, 0, 0, 1)])
    delta = tesselator.get_output_geometry_delta()
    renderer.update_geometry(delta)
    check(repr(delta) == "GeometryDelta(<passed on>)")
    for use in (lambda: renderer.update_geometry(delta), lambda: len(delta), lambda: delta[0]):
        with raises(ValueError, "update_geometry"):
            use()
    with raises(TypeError):
        renderer.update_geometry(None)


def renderer_draws_assets():
    w = h = 32
    renderer = make_renderer(w, h)
    mesh = rr.StlMesh([FRONT_TRIANGLE])
    ids = renderer.upload_asset(mesh, 2)
    check(ids == [0, 1])
    check(renderer.upload_asset(mesh, 1) == [2])
    renderer.set_asset_visibility(2, False)

    camera = rr.OrbitCamera()
    frame = render_now(renderer, camera, w, h)
    check(lit(frame, w, w // 2, h // 2))
    check(not lit(frame, w, 0, 0))

    for instance in ids:
        renderer.set_asset_visibility(instance, False)
    check(not lit(render_now(renderer, camera, w, h), w, w // 2, h // 2))
    renderer.set_asset_visibility(ids[1], True)
    check(lit(render_now(renderer, camera, w, h), w, w // 2, h // 2))

    renderer.set_asset_transform(ids[1], rr.Twist(linear=(100, 0, 0)))
    check(not lit(render_now(renderer, camera, w, h), w, w // 2, h // 2))
    # Turned half a turn around Y it shows its back, which is culled.
    renderer.set_asset_transform(ids[1], rr.Twist(rotational=(0, 1, 0, 0)))
    check(not lit(render_now(renderer, camera, w, h), w, w // 2, h // 2))
    renderer.set_asset_transform(ids[1], IDENTITY)
    check(lit(render_now(renderer, camera, w, h), w, w // 2, h // 2))


def renderer_rejects_unknown_instances():
    renderer = make_renderer(4, 4)
    with raises(IndexError, "instance 0"):
        renderer.set_asset_transform(0, IDENTITY)
    (instance,) = renderer.upload_asset(rr.StlMesh([FRONT_TRIANGLE]), 1)
    renderer.set_asset_transform(instance, IDENTITY)
    with raises(IndexError):
        renderer.set_asset_transform(instance + 1, IDENTITY)
    with raises(IndexError):
        renderer.set_asset_visibility(instance + 1, False)
    with raises(OverflowError):
        renderer.set_asset_visibility(-1, False)


def renderer_upload_checks_arguments():
    renderer = make_renderer(4, 4)
    mesh = rr.StlMesh([FRONT_TRIANGLE])
    check(renderer.upload_asset(mesh, 0) == [])
    with raises(ValueError, "instances"):
        renderer.upload_asset(mesh, 2**32)
    with raises(OverflowError):
        renderer.upload_asset(mesh, -1)
    with raises(TypeError):
        renderer.upload_asset([FRONT_TRIANGLE], 1)
    # Failed uploads take no ids.
    check(renderer.upload_asset(mesh, 1) == [0])


def renderer_resize_restarts_readback():
    renderer = make_renderer(32, 32)
    renderer.upload_asset(rr.StlMesh([FRONT_TRIANGLE]), 1)
    camera = rr.OrbitCamera()
    check(lit(render_now(renderer, camera, 32, 32), 32, 16, 16))
    renderer.configure(rr.WgpuRendererConfiguration(16, 8))
    state = camera.get_state(2.0)
    frame = bytearray(b"\xab" * (16 * 8 * 4))
    renderer.render(state, frame)
    check(not any(frame))
    renderer.render(state, frame)
    check(lit(frame, 16, 8, 4))


# ----------------------------------------------------------------- robot


def urdf_description_reads_everything():
    description = rr.UrdfDescription.read_str(ROBOT_URDF)
    check(description.name == "test_bot")
    check(repr(description) == "UrdfDescription(name=\"test_bot\", <2 links>, <1 joints>)")
    base, arm = description.links
    check((base.name, arm.name) == ("base", "arm"))
    check((len(base.visuals), len(arm.visuals)) == (1, 2))

    (body,) = base.visuals
    check(body.name == "body")
    close(body.origin.xyz, (0, 0, 0.5))
    check(isinstance(body.geometry, rr.UrdfMesh))
    # read_str has no directory to resolve against.
    check(body.geometry.filename == "meshes/base.stl")
    close(body.geometry.scale, (2, 2, 2))
    check(body.material is None)

    box, cylinder = arm.visuals
    check(box.name is None)
    check(isinstance(box.geometry, rr.UrdfBox))
    close(box.geometry.size, (1, 2, 3))
    check(box.material.name == "red")
    check(isinstance(cylinder.geometry, rr.UrdfCylinder))
    close((cylinder.geometry.radius, cylinder.geometry.length), (0.5, 2))

    (joint,) = description.joints
    check((joint.name, joint.joint_type, joint.parent, joint.child) == ("shoulder", "revolute", "base", "arm"))
    close(joint.origin.xyz, (0, 0, 1))
    close(joint.axis, (0, 0, 1))
    close((joint.limit.lower, joint.limit.upper, joint.limit.effort, joint.limit.velocity), (-1.5, 1.5, 10, 2))

    (material,) = description.materials
    check(material.name == "red")
    close(material.color.rgba, (1, 0, 0, 1))

    # Plain data: equal when read twice, read only.
    check(rr.UrdfDescription.read_str(ROBOT_URDF) == description)
    check(description.links[1].visuals[0] == box)
    with raises(AttributeError):
        joint.name = "elbow"


def urdf_read_rejects_bad_input():
    with raises(RuntimeError, "<robot>"):
        rr.UrdfDescription.read_str('<link name="base"/>')
    with raises(RuntimeError, "missing.urdf"):
        rr.UrdfDescription.read_file(temp_path("missing.urdf"))
    with raises(TypeError):
        rr.UrdfDescription.read_str(None)


def urdf_read_file_resolves_meshes():
    path = temp_path("robot.urdf")
    write_file(path, ROBOT_URDF)
    filename = rr.UrdfDescription.read_file(path).links[0].visuals[0].geometry.filename
    check(filename.startswith("/") and filename.endswith("/meshes/base.stl"), filename)


def urdf_robot_moves_links():
    description = rr.UrdfDescription.read_str(ROBOT_URDF)
    robot = rr.UrdfRobot.from_description(description)
    check(robot.description() == description)
    check(robot.last_transform() is None)

    transforms = robot.feed_sensor_input({"shoulder": HALF_PI})
    links = transforms.links
    check(sorted(links) == ["arm", "base"])
    close(links["base"].linear, (0, 0, 0))
    close(links["arm"].linear, (0, 0, 1))
    close(links["arm"].rotational, (0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    visuals = transforms.visuals
    check(sorted(visuals) == [("arm", 0), ("arm", 1), ("base", 0)])
    close(visuals[("base", 0)].linear, (0, 0, 0.5))

    close(transforms.get_link_origin("arm"), (0, 0, 1))
    close(transforms.transform_visual_point("arm", 0, (1, 0, 0)), (0, 1, 1))
    check(transforms.get_link_origin("nope") is None)
    check(transforms.transform_visual_point("arm", 5, (0, 0, 0)) is None)

    close(robot.last_transform().links["arm"].rotational, links["arm"].rotational)
    # Joints left out are at 0.
    robot.feed_sensor_input({})
    close(robot.last_transform().links["arm"].rotational, (0, 0, 0, 1))
    with raises(TypeError):
        robot.feed_sensor_input({"shoulder": "far"})


TEST_CASES = [
    module_exports_its_classes,
    point_i_has_the_voxelizer_interface,
    value_classes_have_properties,
    output_classes_can_not_be_created,
    camera_default_looks_down_negative_z,
    camera_new_takes_degrees,
    camera_view_proj_is_column_major,
    camera_mutations_match_rust,
    camera_fields_are_writable,
    camera_with_clip_planes_returns_a_copy,
    camera_get_state_checks_arguments,
    stl_read_ascii_file,
    stl_missing_file_fails,
    stl_triangles_are_copies,
    tesselator_requires_configure,
    tesselator_checks_voxel_size,
    tesselator_rejects_unaligned_blocks,
    tesselator_full_block_is_a_box,
    tesselator_takes_voxelizer_points,
    tesselator_rejects_other_objects,
    renderer_configure_checks_size,
    renderer_render_checks_arguments,
    renderer_writes_any_byte_buffer,
    renderer_first_frame_is_zeros,
    renderer_draws_voxels,
    renderer_update_geometry_takes_the_delta,
    renderer_draws_assets,
    renderer_rejects_unknown_instances,
    renderer_upload_checks_arguments,
    renderer_resize_restarts_readback,
    urdf_description_reads_everything,
    urdf_read_rejects_bad_input,
    urdf_read_file_resolves_meshes,
    urdf_robot_moves_links,
]

if __name__ == "__main__":
    sys.exit(main(TEST_CASES))
