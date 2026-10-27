/**
 * Test cases for the JS bindings: the JS facing API (classes, properties,
 * argument conversion, exceptions, Promises) and a port of the Python binding
 * tests in tests/python/renderer_tests.py.
 *
 * Test cases keep the snake_case names of the C and Python suites.
 */

import process from "node:process";

import {
    addBlock, check, close, fmt, lit, main, makeRenderer, makeTesselator, pixelIs, renderNow, rr, same, throws,
} from "./harness.mjs";

const U32_MAX = 2 ** 32 - 1;
const HALF_PI = Math.PI / 2;
// Pitch limit of the orbit camera (src/camera.rs).
const SAFE_PITCH = HALF_PI - 0.01;
const IDENTITY = new rr.Twist();

// A triangle facing +Z, around the origin.
const FRONT_TRIANGLE = new rr.Triangle([-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 1.0, 0.0]);

const ROBOT_URDF = `
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
`;

/** Stands in for r3v-voxelizer's PointI: the same properties, read through
 * getters on the prototype as for every wasm-bindgen class, another class. */
class VoxelizerPointI {
    #values;

    constructor(x, y, z, i) {
        this.#values = [x, y, z, i];
    }

    get x() {
        return this.#values[0];
    }

    get y() {
        return this.#values[1];
    }

    get z() {
        return this.#values[2];
    }

    get i() {
        return this.#values[3];
    }
}

/** view_proj * (point, 1), divided by w. */
function project(state, point) {
    const v = [...point, 1.0];
    const viewProj = state.view_proj;
    const clip = [0, 1, 2, 3].map((row) => [0, 1, 2, 3].reduce((sum, column) => sum + viewProj[column][row] * v[column], 0));
    return clip.slice(0, 3).map((c) => c / clip[3]);
}

/** Binary STL of triangles given as [normal, v0, v1, v2]. */
function binaryStl(triangles) {
    const bytes = new Uint8Array(84 + 50 * triangles.length);
    const view = new DataView(bytes.buffer);
    view.setUint32(80, triangles.length, true);
    triangles.forEach((triangle, k) => {
        triangle.flat().forEach((value, j) => view.setFloat32(84 + 50 * k + 4 * j, value, true));
    });
    return bytes;
}

/** z of (v1 - v0) x (v2 - v0): positive when the triangle faces +Z. */
function facingZ({ v0, v1, v2 }) {
    return (v1[0] - v0[0]) * (v2[1] - v0[1]) - (v1[1] - v0[1]) * (v2[0] - v0[0]);
}

// ---------------------------------------------------------------- JS API

const CLASSES = [
    "CameraState", "CpuTesselator", "CpuTesselatorConfiguration", "GeometryDelta", "OrbitCamera", "PointI",
    "RobotTransforms", "StlMesh", "Triangle", "Twist", "UrdfDescription", "UrdfRobot", "WgpuRenderer",
    "WgpuRendererConfiguration",
];

function module_exports_its_classes() {
    // Besides what wasm-bindgen's --target web adds to load the module.
    const loader = ["default", "initSync", "__wasm"];
    const exported = Object.keys(rr).filter((name) => !loader.includes(name)).sort();
    check(same(exported, CLASSES), `module exports ${fmt(exported)}`);
    for (const name of CLASSES) {
        check(typeof rr[name] === "function", `r3v_renderer.${name} is no class`);
        check(rr[name].name === name, `${name} is named ${rr[name].name}`);
    }
}

function point_i_has_the_voxelizer_interface() {
    const p = new rr.PointI(4, -8, 2, U32_MAX);
    check(same([p.x, p.y, p.z, p.i], [4, -8, 2, U32_MAX]));
    p.x = 8;
    p.y = 4;
    p.z = -2;
    p.i = 1;
    check(same([p.x, p.y, p.z, p.i], [8, 4, -2, 1]));
    const extremes = new rr.PointI(-(2 ** 31), 2 ** 31 - 1, 0, U32_MAX);
    check(same([extremes.x, extremes.y], [-(2 ** 31), 2 ** 31 - 1]));
    // Converted the way wasm-bindgen converts every integer, as in
    // r3v-voxelizer: truncated and wrapped.
    const wrapped = new rr.PointI(1.9, 0, 0, -1);
    check(same([wrapped.x, wrapped.i], [1, U32_MAX]));
}

function value_classes_have_properties() {
    const twist = new rr.Twist();
    check(same(twist.linear, [0, 0, 0]) && same(twist.rotational, [0, 0, 0, 1]));
    twist.linear = [1, 2, 3];
    check(same(twist.linear, [1, 2, 3]));
    // Vectors are copies: changing one does not change the twist.
    twist.linear[0] = 42;
    check(twist.linear[0] === 1);
    check(same(new rr.Twist([1, 2, 3], new Float32Array([0, 0, 0, 1])).linear, [1, 2, 3]));

    const triangle = new rr.Triangle([0, 0, 0], [1, 0, 0], [0, 1, 0]);
    check(same(triangle.v1, [1, 0, 0]));
    triangle.v2 = [0, 2, 0];
    check(same(triangle.v2, [0, 2, 0]));

    const tesselatorConfiguration = new rr.CpuTesselatorConfiguration(0.5);
    check(tesselatorConfiguration.marching_cubes_enabled === false);
    tesselatorConfiguration.voxel_size = 0.25;
    tesselatorConfiguration.marching_cubes_enabled = true;
    check(tesselatorConfiguration.voxel_size === 0.25 && tesselatorConfiguration.marching_cubes_enabled === true);
    check(new rr.CpuTesselatorConfiguration(1, true).marching_cubes_enabled === true);
    const rendererConfiguration = new rr.WgpuRendererConfiguration(640, 480);
    check(same([rendererConfiguration.width, rendererConfiguration.height], [640, 480]));
    check(same(rendererConfiguration.background_color, [0, 0, 0]));
    check(rendererConfiguration.color_depth_range === 50);
    rendererConfiguration.background_color = [1, 0.5, 0];
    rendererConfiguration.color_depth_range = 10;
    check(same(rendererConfiguration.background_color, [1, 0.5, 0]));
    check(rendererConfiguration.color_depth_range === 10);
    const colored = new rr.WgpuRendererConfiguration(4, 4, [0, 0, 1], 5);
    check(same(colored.background_color, [0, 0, 1]) && colored.color_depth_range === 5);

    const identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    const state = new rr.CameraState(identity, [0, 0, 5], [0, 0, -1]);
    check(same(state.view_proj, identity));
    check(same([state.position, state.forward], [[0, 0, 5], [0, 0, -1]]));
}

async function value_classes_check_their_arguments() {
    await throws(() => new rr.Twist([1, 2]), TypeError, "linear must be an array of 3 numbers");
    await throws(() => new rr.Twist([0, 0, 0], [0, 0, 0, "1"]), TypeError, "rotational[3] is not a number");
    await throws(() => new rr.Triangle([0, 0, 0], [1, 0, 0]), TypeError, "v2");
    await throws(() => { new rr.Triangle([0, 0, 0], [1, 0, 0], [0, 1, 0]).v0 = 5; }, TypeError, "v0");
    await throws(() => new rr.CameraState([[1, 0, 0, 0]], [0, 0, 5], [0, 0, -1]), TypeError, "view_proj");
    await throws(() => new rr.OrbitCamera("origin"), TypeError, "target");
}

async function classes_without_constructor_can_not_be_used() {
    const uses = [
        [rr.GeometryDelta, (o) => o.length],
        [rr.RobotTransforms, (o) => o.links],
        [rr.UrdfDescription, (o) => o.name],
        [rr.UrdfRobot, (o) => o.last_transform()],
        [rr.WgpuRenderer, (o) => o.configure({ width: 1, height: 1 })],
    ];
    for (const [cls, use] of uses) {
        await throws(() => use(new cls()), Error);
    }
}

async function freed_objects_can_not_be_used() {
    const point = new rr.PointI(1, 2, 3, 4);
    point.free();
    await throws(() => point.x, Error);
    const triangle = new rr.Triangle([0, 0, 0], [1, 0, 0], [0, 1, 0]);
    triangle[Symbol.dispose]();
    await throws(() => new rr.StlMesh([triangle]), Error);
}

// ---------------------------------------------------------------- camera

function camera_default_looks_down_negative_z() {
    const camera = new rr.OrbitCamera();
    close(camera.target, [0, 0, 0]);
    close(camera.distance, 5);
    close([camera.yaw, camera.pitch], [0, 0]);
    close([camera.near, camera.far], [0.1, 100]);
    check(camera.new_uniform_available);
    close(camera.forward(), [0, 0, -1]);
    const state = camera.get_state(1.0);
    close(state.position, [0, 0, 5]);
    close(state.forward, [0, 0, -1]);
}

function camera_new_takes_degrees() {
    const camera = new rr.OrbitCamera([1, 2, 3], 2, 90, 0);
    close(camera.yaw, HALF_PI);
    const state = camera.get_state(1.0);
    close(state.position, [3, 2, 3]);
    close(state.forward, [-1, 0, 0]);
    // Left out arguments take their defaults.
    close(new rr.OrbitCamera([1, 2, 3]).distance, 5);
}

function camera_view_proj_is_column_major() {
    const state = new rr.OrbitCamera().get_state(2.0);
    const [x, y, z] = project(state, [0, 0, 0]);
    close([x, y], [0, 0]);
    check(z > 0 && z < 1);
    check(project(state, [1, 0, 0])[0] > 0);
    check(project(state, [0, 1, 0])[1] > 0);
}

function camera_mutations_match_rust() {
    let camera = new rr.OrbitCamera();
    camera.new_uniform_available = false;
    camera.rotate(0.5, 0.25);
    close([camera.yaw, camera.pitch], [0.5, 0.25]);
    check(camera.new_uniform_available);
    camera.rotate(0, 10);
    close(camera.pitch, SAFE_PITCH);
    camera.zoom(-100);
    close(camera.distance, 0.1);

    camera = new rr.OrbitCamera();
    camera.pan(1, 2);
    close(camera.target, [1, 2, 0]);
    camera.look_at([4, 5, 6], 0, 90, 90);
    close(camera.target, [4, 5, 6]);
    close(camera.distance, 0.1);
    close([camera.yaw, camera.pitch], [HALF_PI, SAFE_PITCH]);
}

function camera_fields_are_writable() {
    const camera = new rr.OrbitCamera();
    camera.target = [1, 0, 0];
    camera.distance = 2;
    camera.yaw = HALF_PI;
    close(camera.get_state(1.0).position, [3, 0, 0]);
}

function camera_with_clip_planes_returns_a_copy() {
    const camera = new rr.OrbitCamera();
    const clipped = camera.with_clip_planes(0.5, 50);
    close([clipped.near, clipped.far], [0.5, 50]);
    close([camera.near, camera.far], [0.1, 100]);
}

async function camera_get_state_checks_arguments() {
    const camera = new rr.OrbitCamera();
    for (const aspectRatio of [0, -1, NaN, Infinity]) {
        await throws(() => camera.get_state(aspectRatio), RangeError, "aspect ratio");
    }
    for (const [near, far] of [[0, 100], [1, 1], [0.1, Infinity]]) {
        await throws(() => camera.with_clip_planes(near, far).get_state(1.0), RangeError, "clip planes");
    }
}

// ------------------------------------------------------------------- stl

function stl_read_ascii_bytes() {
    const ascii = "solid t\nfacet normal 0 0 1\n outer loop\n  vertex 0 0 0\n  vertex 1 0 0\n"
        + "  vertex 0 1 0\n endloop\nendfacet\nendsolid t\n";
    const bytes = new TextEncoder().encode(ascii);
    for (const input of [bytes, bytes.buffer]) {
        const triangles = rr.StlMesh.read_bytes(input).triangles;
        check(triangles.length === 1);
        check(triangles[0] instanceof rr.Triangle);
        check(same([triangles[0].v0, triangles[0].v1, triangles[0].v2], [[0, 0, 0], [1, 0, 0], [0, 1, 0]]));
    }
}

function stl_read_binary_bytes() {
    // Declared to face -Z, so it is wound the other way round.
    const bytes = binaryStl([[[0, 0, -1], [0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 1], [0, 0, 0], [2, 0, 0], [0, 2, 0]]]);
    const [down, up] = rr.StlMesh.read_bytes(bytes).triangles;
    check(facingZ(down) < 0, `${fmt([down.v0, down.v1, down.v2])} does not face -Z`);
    check(facingZ(up) > 0, `${fmt([up.v0, up.v1, up.v2])} does not face +Z`);
}

async function stl_read_rejects_bad_input() {
    await throws(() => rr.StlMesh.read_bytes(new Uint8Array(10)), Error, "STL");
    for (const input of ["solid", null, [1, 2, 3], new Float32Array(4)]) {
        await throws(() => rr.StlMesh.read_bytes(input), TypeError, "Uint8Array");
    }
}

async function stl_triangles_are_copies() {
    const other = new rr.Triangle([0, 0, 0], [1, 0, 0], [0, 0, 7]);
    // Triangles and plain objects alike.
    const mesh = new rr.StlMesh([FRONT_TRIANGLE, { v0: [0, 0, 0], v1: [1, 0, 0], v2: [0, 0, 7] }]);
    const triangles = mesh.triangles;
    check(same(triangles.map((t) => [t.v0, t.v1, t.v2]), [FRONT_TRIANGLE, other].map((t) => [t.v0, t.v1, t.v2])));
    // Changing a copy does not change the mesh, assigning does.
    triangles[0].v0 = [42, 0, 0];
    check(same(mesh.triangles[0].v0, FRONT_TRIANGLE.v0));
    mesh.triangles = triangles.map((t) => ({ v0: t.v0.map((c) => 3 * c), v1: t.v1.map((c) => 3 * c), v2: t.v2.map((c) => 3 * c) }));
    close(mesh.triangles[1].v2, [0, 0, 21]);
    check(new rr.StlMesh([]).triangles.length === 0);
    await throws(() => new rr.StlMesh([[0, 0, 0]]), TypeError, "triangles[0].v0");
    await throws(() => new rr.StlMesh(FRONT_TRIANGLE), TypeError, "triangles is not an array");
}

// ------------------------------------------------------------ tesselator

async function tesselator_requires_configure() {
    const tesselator = new rr.CpuTesselator();
    await throws(() => tesselator.update_input_geometry([new rr.PointI(0, 0, 0, 1)]), Error, "configure");
    await throws(() => tesselator.get_output_geometry_delta(), Error, "configure");
}

async function tesselator_checks_voxel_size() {
    const tesselator = new rr.CpuTesselator();
    for (const voxelSize of [0, -1, NaN, Infinity]) {
        await throws(() => tesselator.configure(new rr.CpuTesselatorConfiguration(voxelSize)), RangeError, "voxel size");
    }
    await throws(() => tesselator.configure({}), TypeError, "configuration.voxel_size");
    await throws(() => tesselator.update_input_geometry([]), Error, "configure");
    // A plain object works too.
    tesselator.configure({ voxel_size: 1 });
    tesselator.update_input_geometry([]);
}

async function tesselator_rejects_unaligned_blocks() {
    const tesselator = makeTesselator(1.0);
    for (const unaligned of [new rr.PointI(1, 0, 0, 1), new rr.PointI(0, 2, 0, 1), new rr.PointI(0, 0, -1, 1)]) {
        // A valid block first: none of them may be applied.
        await throws(() => tesselator.update_input_geometry([new rr.PointI(4, 4, 2, 1), unaligned]), RangeError, "blocks[1]");
    }
    check(tesselator.get_output_geometry_delta() === undefined);
}

async function tesselator_full_block_is_a_box() {
    const tesselator = makeTesselator(0.5);
    // Inside its chunk, so the neighbouring chunks stay untouched.
    const block = new rr.PointI(4, 4, 2, U32_MAX);
    tesselator.update_input_geometry([block]);
    const delta = tesselator.get_output_geometry_delta();
    check(delta instanceof rr.GeometryDelta);
    check(delta.length === 1);
    const chunk = delta.chunk(0);
    check(same(chunk.index, [0, 0, 0]));
    // Two triangles for each of the six faces, as plain objects.
    check(chunk.triangles.length === 12);
    check(Object.getPrototypeOf(chunk.triangles[0]) === Object.prototype);
    check(same(structuredClone(chunk), chunk));
    // Voxels 4..8 x 4..8 x 2..4 at 0.5 world units each.
    const [lo, hi] = [[2, 2, 1], [4, 4, 2]];
    for (const triangle of chunk.triangles) {
        for (const corner of [triangle.v0, triangle.v1, triangle.v2]) {
            check([0, 1, 2].every((a) => lo[a] - 1e-4 <= corner[a] && corner[a] <= hi[a] + 1e-4), fmt(triangle));
        }
    }
    await throws(() => delta.chunk(1), RangeError, "chunk 1");
    await throws(() => delta.chunk(-1), RangeError);
    await throws(() => delta.chunk("0"), TypeError);

    // Nothing changed since, sending the same block changes nothing either.
    check(tesselator.get_output_geometry_delta() === undefined);
    tesselator.update_input_geometry([block]);
    check(tesselator.get_output_geometry_delta() === undefined);

    // Clearing it leaves the chunk without triangles.
    block.i = 0;
    tesselator.update_input_geometry([block]);
    const cleared = tesselator.get_output_geometry_delta();
    check(cleared.length === 1 && cleared.chunk(0).triangles.length === 0);
}

async function tesselator_toggles_marching_cubes() {
    const tesselator = makeTesselator(1.0);
    check(tesselator.marching_cubes_enabled() === false);
    tesselator.update_input_geometry([new rr.PointI(4, 4, 2, U32_MAX)]);
    check(tesselator.get_output_geometry_delta().chunk(0).triangles.length === 12);

    // Switching it on tesselates the chunk again, smoothed: marching cubes puts
    // vertices between voxel corners.
    tesselator.set_marching_cubes_enabled(true);
    check(tesselator.marching_cubes_enabled() === true);
    const delta = tesselator.get_output_geometry_delta();
    check(delta.length === 1);
    const triangles = delta.chunk(0).triangles;
    check(triangles.length !== 0 && triangles.length !== 12, `${triangles.length} triangles`);
    const corners = triangles.flatMap((triangle) => [...triangle.v0, ...triangle.v1, ...triangle.v2]);
    check(corners.some((corner) => corner !== Math.floor(corner)));
    tesselator.set_marching_cubes_enabled(true);
    check(tesselator.get_output_geometry_delta() === undefined);

    // The configuration switches it too, off when left out.
    tesselator.configure({ voxel_size: 1, marching_cubes_enabled: true });
    check(tesselator.marching_cubes_enabled() === true);
    tesselator.configure({ voxel_size: 1 });
    check(tesselator.marching_cubes_enabled() === false);
    check(tesselator.get_output_geometry_delta().chunk(0).triangles.length === 12);
    await throws(() => tesselator.configure({ voxel_size: 1, marching_cubes_enabled: 1 }), TypeError,
        "configuration.marching_cubes_enabled is not a boolean");
}

function tesselator_takes_voxelizer_points() {
    const tesselator = makeTesselator(1.0);
    // Anything with x, y, z and i properties, also mixed.
    tesselator.update_input_geometry([
        new VoxelizerPointI(4, 4, 2, U32_MAX),
        new rr.PointI(8, 4, 2, U32_MAX),
        { x: 4, y: 8, z: 2, i: U32_MAX },
        new VoxelizerPointI(8, 8, 2, U32_MAX),
    ]);
    const delta = tesselator.get_output_geometry_delta();
    // Four blocks side by side make one box, away from the chunk's sides
    // (blocks on a side touch the neighbouring chunk too).
    check(delta.length === 1 && delta.chunk(0).triangles.length === 12);
    tesselator.update_input_geometry([]);
    check(tesselator.get_output_geometry_delta() === undefined);
}

async function tesselator_rejects_other_objects() {
    const tesselator = makeTesselator(1.0);
    await throws(() => tesselator.update_input_geometry([new rr.PointI(0, 0, 0, 1), [0, 0, 0, 1]]), TypeError,
        "blocks[1].x is not a number");
    await throws(() => tesselator.update_input_geometry([null]), TypeError, "blocks[0] is not an object");
    await throws(() => tesselator.update_input_geometry(5), TypeError, "blocks is not an array");
    await throws(() => tesselator.update_input_geometry([{ x: 0, y: 0, z: 0, i: -1 }]), RangeError, "blocks[0].i");
    await throws(() => tesselator.update_input_geometry([{ x: 0.5, y: 0, z: 0, i: 1 }]), RangeError, "blocks[0].x");
    await throws(() => tesselator.update_input_geometry([{ x: 2 ** 31, y: 0, z: 0, i: 1 }]), RangeError, "blocks[0].x");
    check(tesselator.get_output_geometry_delta() === undefined);
}

// -------------------------------------------------------------- renderer

async function renderer_configure_checks_size() {
    const renderer = await makeRenderer(8, 8);
    for (const [width, height] of [[0, 8], [8, 0], [8193, 8], [8, 8193]]) {
        await throws(() => renderer.configure(new rr.WgpuRendererConfiguration(width, height)), RangeError, "frame size");
    }
    await throws(() => renderer.configure({ width: -1, height: 8 }), RangeError, "configuration.width");
    await throws(() => renderer.configure({ width: 8, height: 1.5 }), RangeError, "configuration.height");
    await throws(() => renderer.configure(null), TypeError, "configuration");
    // A plain object works too.
    renderer.configure({ width: 8192, height: 1 });
}

async function renderer_configure_checks_colors() {
    const renderer = await makeRenderer(8, 8);
    for (const color_depth_range of [0, -1, NaN, Infinity]) {
        await throws(() => renderer.configure({ width: 8, height: 8, color_depth_range }), RangeError, "color depth range");
    }
    for (const background_color of [[-0.1, 0, 0], [0, 1.1, 0], [0, 0, NaN]]) {
        await throws(() => renderer.configure({ width: 8, height: 8, background_color }), RangeError, "background color");
    }
    await throws(() => renderer.configure({ width: 8, height: 8, background_color: [0, 0] }), TypeError,
        "configuration.background_color");
    await throws(() => renderer.configure({ width: 8, height: 8, color_depth_range: "5" }), TypeError,
        "configuration.color_depth_range");
}

async function renderer_clears_to_background_color() {
    const [w, h] = [8, 8];
    const renderer = await makeRenderer(w, h);
    const camera = new rr.OrbitCamera();
    renderer.configure(new rr.WgpuRendererConfiguration(w, h, [1, 0, 0]));
    const frame = await renderNow(renderer, camera, w, h);
    check(pixelIs(frame, w, 0, 0, [0, 0, 255]));
    check(pixelIs(frame, w, w - 1, h - 1, [0, 0, 255]));

    // At the same size it applies to the next frame; components are sRGB.
    renderer.configure({ width: w, height: h, background_color: [0, 0.5, 1] });
    check(pixelIs(await renderNow(renderer, camera, w, h), w, w / 2, h / 2, [255, 128, 0]));
}

async function renderer_color_depth_range_recolors_voxels() {
    const [w, h] = [32, 32];
    const renderer = await makeRenderer(w, h);
    addBlock(renderer, 1.0, 4, 4, 2);
    const camera = new rr.OrbitCamera([6, 6, 3], 10);
    const centre = 4 * ((h / 2) * w + w / 2);
    // Near the start of the 50 m range: turbo's blue (BGRA).
    let frame = await renderNow(renderer, camera, w, h);
    check(frame[centre] > frame[centre + 2]);

    // The block is beyond a 5 m range: turbo's red end.
    renderer.configure({ width: w, height: h, color_depth_range: 5 });
    frame = await renderNow(renderer, camera, w, h);
    check(frame[centre + 2] > frame[centre]);
}

async function renderer_render_checks_arguments() {
    const renderer = await rr.WgpuRenderer.create();
    const state = new rr.OrbitCamera().get_state(1.0);
    await throws(() => renderer.render(state, new Uint8Array(64)), Error, "configure");
    renderer.configure(new rr.WgpuRendererConfiguration(4, 4));
    await throws(() => renderer.render(state, new Uint8Array(63)), RangeError, "64 bytes");
    for (const out of [new Float32Array(64), new Array(64).fill(0), "x".repeat(64), null]) {
        await throws(() => renderer.render(state, out), TypeError, "Uint8Array");
    }
    await throws(() => renderer.render(new rr.OrbitCamera(), new Uint8Array(64)), TypeError, "camera_state.view_proj");
    // The errors are rejections, render never throws.
    const rejected = renderer.render(state, new Uint8Array(1));
    check(rejected instanceof Promise);
    await throws(() => rejected, RangeError);
    await renderer.render(state, new Uint8Array(64));
}

async function renderer_writes_any_byte_array() {
    const renderer = await makeRenderer(4, 4);
    renderer.upload_asset(new rr.StlMesh([FRONT_TRIANGLE]), 1);
    const state = new rr.OrbitCamera().get_state(1.0);
    await renderer.render(state, new Uint8Array(64));
    // A view on a part of a larger buffer only writes that part.
    const backing = new Uint8Array(96).fill(0xab);
    const view = new Uint8Array(backing.buffer, 16, 64);
    for (const out of [new Uint8Array(64), new Uint8ClampedArray(64), Buffer.alloc(64), view]) {
        await renderer.render(state, out);
        check(lit(out, 4, 2, 2), `nothing rendered into ${out.constructor.name}`);
    }
    check(backing.subarray(0, 16).every((b) => b === 0xab) && backing.subarray(80).every((b) => b === 0xab));
    // A larger array is fine, only the frame is written.
    const larger = new Uint8Array(80).fill(0xab);
    await renderer.render(state, larger);
    check(larger.subarray(64).every((b) => b === 0xab));
}

async function renderer_render_rgba_swaps_channels() {
    const [w, h] = [32, 32];
    const renderer = await makeRenderer(w, h);
    addBlock(renderer, 1.0, 4, 4, 2);
    const state = new rr.OrbitCamera([6, 6, 3], 10).get_state(w / h);
    const bgra = new Uint8Array(w * h * 4);
    const rgba = new Uint8ClampedArray(w * h * 4);
    await renderer.render(state, bgra);
    await renderer.render(state, bgra);
    // The same scene once more.
    await renderer.render_rgba(state, rgba);
    let colored = false;
    for (let p = 0; p < bgra.length; p += 4) {
        check(rgba[p] === bgra[p + 2] && rgba[p + 1] === bgra[p + 1] && rgba[p + 2] === bgra[p]
            && rgba[p + 3] === bgra[p + 3], `pixel ${p / 4}: BGRA ${bgra.subarray(p, p + 4)}, RGBA ${rgba.subarray(p, p + 4)}`);
        colored ||= bgra[p] !== bgra[p + 2];
    }
    check(colored, "every pixel has blue = red, the swap can not be seen");
}

async function renderer_first_frame_is_zeros() {
    const renderer = await makeRenderer(16, 16);
    const state = new rr.OrbitCamera().get_state(1.0);
    const frame = new Uint8Array(16 * 16 * 4).fill(0xab);
    await renderer.render(state, frame);
    check(frame.every((b) => b === 0));
    // The next one is the cleared first frame: black, opaque.
    await renderer.render(state, frame);
    check(!lit(frame, 16, 8, 8));
    check(frame[3] === 255);
}

async function renderer_draws_voxels() {
    const [w, h] = [32, 32];
    const renderer = await makeRenderer(w, h);
    // Voxels 4..8 x 4..8 x 2..4.
    const tesselator = addBlock(renderer, 1.0, 4, 4, 2);
    const camera = new rr.OrbitCamera([6, 6, 3], 10);
    const frame = await renderNow(renderer, camera, w, h);
    check(lit(frame, w, w / 2, h / 2));
    check(!lit(frame, w, 0, 0));
    check(!lit(frame, w, w - 1, h - 1));

    // Clearing the block clears the frame.
    tesselator.update_input_geometry([new rr.PointI(4, 4, 2, 0)]);
    renderer.update_geometry(tesselator.get_output_geometry_delta());
    check(!lit(await renderNow(renderer, camera, w, h), w, w / 2, h / 2));
}

async function renderer_update_geometry_takes_the_delta() {
    const renderer = await makeRenderer(4, 4);
    const tesselator = makeTesselator(1.0);
    tesselator.update_input_geometry([new rr.PointI(0, 0, 0, 1)]);
    const delta = tesselator.get_output_geometry_delta();
    renderer.update_geometry(delta);
    for (const use of [() => renderer.update_geometry(delta), () => delta.length, () => delta.chunk(0)]) {
        await throws(use, Error, "update_geometry");
    }
    await throws(() => renderer.update_geometry(null), Error);
    await throws(() => renderer.update_geometry({ length: 0 }), Error);
}

async function renderer_draws_assets() {
    const [w, h] = [32, 32];
    const renderer = await makeRenderer(w, h);
    const mesh = new rr.StlMesh([FRONT_TRIANGLE]);
    const ids = renderer.upload_asset(mesh, 2);
    check(Array.isArray(ids) && same(ids, [0, 1]));
    check(same(renderer.upload_asset(mesh, 1), [2]));
    renderer.set_asset_visibility(2, false);

    const camera = new rr.OrbitCamera();
    check(lit(await renderNow(renderer, camera, w, h), w, w / 2, h / 2));
    check(!lit(await renderNow(renderer, camera, w, h), w, 0, 0));

    for (const instance of ids) {
        renderer.set_asset_visibility(instance, false);
    }
    check(!lit(await renderNow(renderer, camera, w, h), w, w / 2, h / 2));
    renderer.set_asset_visibility(ids[1], true);
    check(lit(await renderNow(renderer, camera, w, h), w, w / 2, h / 2));

    renderer.set_asset_transform(ids[1], new rr.Twist([100, 0, 0]));
    check(!lit(await renderNow(renderer, camera, w, h), w, w / 2, h / 2));
    // Turned half a turn around Y it shows its back, which is culled. A plain
    // object, e.g. a twist of RobotTransforms, works too.
    renderer.set_asset_transform(ids[1], { linear: [0, 0, 0], rotational: [0, 1, 0, 0] });
    check(!lit(await renderNow(renderer, camera, w, h), w, w / 2, h / 2));
    renderer.set_asset_transform(ids[1], IDENTITY);
    check(lit(await renderNow(renderer, camera, w, h), w, w / 2, h / 2));
}

async function renderer_rejects_unknown_instances() {
    const renderer = await makeRenderer(4, 4);
    await throws(() => renderer.set_asset_transform(0, IDENTITY), RangeError, "instance 0");
    const [instance] = renderer.upload_asset(new rr.StlMesh([FRONT_TRIANGLE]), 1);
    renderer.set_asset_transform(instance, IDENTITY);
    await throws(() => renderer.set_asset_transform(instance + 1, IDENTITY), RangeError, "instance 1");
    await throws(() => renderer.set_asset_visibility(instance + 1, false), RangeError, "instance 1");
    for (const id of [-1, 0.5, NaN, 2 ** 53]) {
        await throws(() => renderer.set_asset_visibility(id, false), RangeError, "instance_id");
    }
    for (const id of ["0", null, undefined, 0n]) {
        await throws(() => renderer.set_asset_visibility(id, false), TypeError, "instance_id");
    }
    await throws(() => renderer.set_asset_transform(instance, { linear: [0, 0, 0] }), TypeError, "transform.rotational");
}

async function renderer_upload_checks_arguments() {
    const renderer = await makeRenderer(4, 4);
    const mesh = new rr.StlMesh([FRONT_TRIANGLE]);
    check(same(renderer.upload_asset(mesh, 0), []));
    await throws(() => renderer.upload_asset(mesh, 2 ** 32), RangeError, "instances");
    await throws(() => renderer.upload_asset(mesh, -1), RangeError, "instances");
    await throws(() => renderer.upload_asset(mesh, "1"), TypeError, "instances");
    await throws(() => renderer.upload_asset([FRONT_TRIANGLE], 1), Error);
    // Failed uploads take no ids.
    check(same(renderer.upload_asset(mesh, 1), [0]));
}

async function renderer_resize_restarts_readback() {
    const renderer = await makeRenderer(32, 32);
    renderer.upload_asset(new rr.StlMesh([FRONT_TRIANGLE]), 1);
    const camera = new rr.OrbitCamera();
    check(lit(await renderNow(renderer, camera, 32, 32), 32, 16, 16));
    renderer.configure(new rr.WgpuRendererConfiguration(16, 8));
    const state = camera.get_state(2.0);
    const frame = new Uint8Array(16 * 8 * 4).fill(0xab);
    await renderer.render(state, frame);
    check(frame.every((b) => b === 0));
    await renderer.render(state, frame);
    check(lit(frame, 16, 8, 4));
}

async function renderer_is_busy_while_rendering() {
    const renderer = await makeRenderer(4, 4);
    const state = new rr.OrbitCamera().get_state(1.0);
    const tesselator = makeTesselator(1.0);
    tesselator.update_input_geometry([new rr.PointI(0, 0, 0, 1)]);
    const delta = tesselator.get_output_geometry_delta();
    const chunks = delta.length;
    const mesh = new rr.StlMesh([FRONT_TRIANGLE]);

    // throws() calls each of them right away, before the render settles.
    const pending = renderer.render(state, new Uint8Array(64));
    const busy = [
        () => renderer.render(state, new Uint8Array(64)),
        () => renderer.render_rgba(state, new Uint8Array(64)),
        () => renderer.configure(new rr.WgpuRendererConfiguration(8, 8)),
        () => renderer.update_geometry(delta),
        () => renderer.upload_asset(mesh, 1),
        () => renderer.set_asset_visibility(0, false),
    ].map((call) => throws(call, Error, "still running"));
    await pending;
    await Promise.all(busy);

    // The delta was not taken, the renderer works again.
    check(delta.length === chunks);
    renderer.update_geometry(delta);
    check(same(renderer.upload_asset(mesh, 1), [0]));
    await renderer.render(state, new Uint8Array(64));
}

async function renderer_survives_a_detached_out() {
    const renderer = await makeRenderer(4, 4);
    const state = new rr.OrbitCamera().get_state(1.0);
    const detach = (array) => structuredClone(array.buffer, { transfer: [array.buffer] });

    const detached = new Uint8Array(64);
    detach(detached);
    await throws(() => renderer.render(state, detached), RangeError, "has 0");

    // Transferred while the frame is being rendered.
    const out = new Uint8ClampedArray(64);
    const pending = renderer.render(state, out);
    detach(out);
    await throws(() => pending, Error, "detached");
    await renderer.render(state, new Uint8Array(64));
}

// ----------------------------------------------------------------- robot

function urdf_description_reads_everything() {
    const description = rr.UrdfDescription.read_str(ROBOT_URDF);
    check(description.name === "test_bot");
    const [base, arm] = description.links;
    check(same([base.name, arm.name], ["base", "arm"]));
    check(same([base.visuals.length, arm.visuals.length], [1, 2]));

    const [body] = base.visuals;
    check(body.name === "body");
    close(body.origin.xyz, [0, 0, 0.5]);
    check(body.geometry.type === "mesh");
    // read_str has no directory to resolve against.
    check(body.geometry.filename === "meshes/base.stl");
    close(body.geometry.scale, [2, 2, 2]);
    check(body.material === null);

    const [box, cylinder] = arm.visuals;
    check(box.name === null);
    check(box.geometry.type === "box");
    close(box.geometry.size, [1, 2, 3]);
    check(box.material.name === "red");
    check(cylinder.geometry.type === "cylinder");
    close([cylinder.geometry.radius, cylinder.geometry.length], [0.5, 2]);

    const [joint] = description.joints;
    check(same([joint.name, joint.joint_type, joint.parent, joint.child], ["shoulder", "revolute", "base", "arm"]));
    close(joint.origin.xyz, [0, 0, 1]);
    close(joint.axis, [0, 0, 1]);
    const { limit } = joint;
    close([limit.lower, limit.upper, limit.effort, limit.velocity], [-1.5, 1.5, 10, 2]);

    const [material] = description.materials;
    check(material.name === "red");
    close(material.color.rgba, [1, 0, 0, 1]);

    // Made anew on every read, so changing what was read changes nothing.
    check(same(rr.UrdfDescription.read_str(ROBOT_URDF).links, description.links));
    check(description.links !== description.links);
    base.name = "elbow";
    check(description.links[0].name === "base");
}

function urdf_data_is_plain() {
    const description = rr.UrdfDescription.read_str(ROBOT_URDF);
    const links = description.links;
    check(Object.getPrototypeOf(links[0].visuals[0].geometry) === Object.prototype);
    // What wasm-bindgen classes lose: their data lives in wasm memory.
    check(structuredClone(links)[0].visuals[0].geometry.filename === "meshes/base.stl");
    check(JSON.parse(JSON.stringify(description.joints))[0].limit.upper === 1.5);
    const transforms = rr.UrdfRobot.from_description(description).feed_sensor_input({ shoulder: 1 });
    close(structuredClone(transforms.links).arm.linear, [0, 0, 1]);
    close(JSON.parse(JSON.stringify(transforms.visuals)).base[0].linear, [0, 0, 0.5]);
}

async function urdf_read_rejects_bad_input() {
    await throws(() => rr.UrdfDescription.read_str('<link name="base"/>'), Error, "<robot>");
    for (const xml of [null, 5, new TextEncoder().encode(ROBOT_URDF)]) {
        await throws(() => rr.UrdfDescription.read_str(xml), TypeError, "xml");
    }
}

async function urdf_robot_moves_links() {
    const description = rr.UrdfDescription.read_str(ROBOT_URDF);
    const robot = rr.UrdfRobot.from_description(description);
    check(same(robot.description().links, description.links));
    check(robot.last_transform() === undefined);

    const transforms = robot.feed_sensor_input({ shoulder: HALF_PI });
    check(transforms instanceof rr.RobotTransforms);
    const links = transforms.links;
    check(same(Object.keys(links), ["arm", "base"]));
    close(links.base.linear, [0, 0, 0]);
    close(links.arm.linear, [0, 0, 1]);
    close(links.arm.rotational, [0, 0, Math.SQRT1_2, Math.SQRT1_2]);
    const visuals = transforms.visuals;
    check(same(Object.keys(visuals), ["arm", "base"]));
    check(same([visuals.arm.length, visuals.base.length], [2, 1]));
    close(visuals.base[0].linear, [0, 0, 0.5]);

    close(transforms.get_link_origin("arm"), [0, 0, 1]);
    close(transforms.transform_visual_point("arm", 0, [1, 0, 0]), [0, 1, 1]);
    check(transforms.get_link_origin("nope") === undefined);
    check(transforms.transform_visual_point("arm", 5, [0, 0, 0]) === undefined);
    await throws(() => transforms.transform_visual_point("arm", -1, [0, 0, 0]), RangeError, "visual_index");
    await throws(() => transforms.get_link_origin(5), TypeError, "link");

    close(robot.last_transform().links.arm.rotational, links.arm.rotational);
    // A Map works too. Joints left out are at 0.
    robot.feed_sensor_input(new Map([["shoulder", -HALF_PI]]));
    close(robot.last_transform().links.arm.rotational, [0, 0, -Math.SQRT1_2, Math.SQRT1_2]);
    robot.feed_sensor_input({});
    close(robot.last_transform().links.arm.rotational, [0, 0, 0, 1]);
    await throws(() => robot.feed_sensor_input({ shoulder: "far" }), TypeError, "shoulder");
    await throws(() => robot.feed_sensor_input(new Map([[1, 0]])), TypeError, "joint name");
    await throws(() => robot.feed_sensor_input(5), TypeError, "input");
}

const TEST_CASES = [
    module_exports_its_classes,
    point_i_has_the_voxelizer_interface,
    value_classes_have_properties,
    value_classes_check_their_arguments,
    classes_without_constructor_can_not_be_used,
    freed_objects_can_not_be_used,
    camera_default_looks_down_negative_z,
    camera_new_takes_degrees,
    camera_view_proj_is_column_major,
    camera_mutations_match_rust,
    camera_fields_are_writable,
    camera_with_clip_planes_returns_a_copy,
    camera_get_state_checks_arguments,
    stl_read_ascii_bytes,
    stl_read_binary_bytes,
    stl_read_rejects_bad_input,
    stl_triangles_are_copies,
    tesselator_requires_configure,
    tesselator_checks_voxel_size,
    tesselator_rejects_unaligned_blocks,
    tesselator_full_block_is_a_box,
    tesselator_toggles_marching_cubes,
    tesselator_takes_voxelizer_points,
    tesselator_rejects_other_objects,
    renderer_configure_checks_size,
    renderer_configure_checks_colors,
    renderer_clears_to_background_color,
    renderer_color_depth_range_recolors_voxels,
    renderer_render_checks_arguments,
    renderer_writes_any_byte_array,
    renderer_render_rgba_swaps_channels,
    renderer_first_frame_is_zeros,
    renderer_draws_voxels,
    renderer_update_geometry_takes_the_delta,
    renderer_draws_assets,
    renderer_rejects_unknown_instances,
    renderer_upload_checks_arguments,
    renderer_resize_restarts_readback,
    renderer_is_busy_while_rendering,
    renderer_survives_a_detached_out,
    urdf_description_reads_everything,
    urdf_data_is_plain,
    urdf_read_rejects_bad_input,
    urdf_robot_moves_links,
];

process.exit(await main(TEST_CASES));
