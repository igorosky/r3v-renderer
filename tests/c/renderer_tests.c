/*
 * Test cases for the C bindings. The Rust tests cover the components
 * themselves; these check what the C API adds: conversions, ownership,
 * argument checks and status codes.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "harness.h"

#define HALF_PI 1.5707964f
/* Pitch limit of the orbit camera (src/camera.rs). */
#define SAFE_PITCH (HALF_PI - 0.01f)

/* A triangle facing +Z, around the origin. */
static const R3vTriangle FRONT_TRIANGLE = {{-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};

static const char *ROBOT_URDF =
    "<robot name=\"test_bot\">"
    "  <material name=\"red\"><color rgba=\"1 0 0 1\"/></material>"
    "  <link name=\"base\">"
    "    <visual name=\"body\">"
    "      <origin xyz=\"0 0 0.5\" rpy=\"0 0 0\"/>"
    "      <geometry><mesh filename=\"meshes/base.stl\" scale=\"2 2 2\"/></geometry>"
    "    </visual>"
    "  </link>"
    "  <link name=\"arm\">"
    "    <visual>"
    "      <geometry><box size=\"1 2 3\"/></geometry>"
    "      <material name=\"red\"/>"
    "    </visual>"
    "    <visual><geometry><cylinder radius=\"0.5\" length=\"2\"/></geometry></visual>"
    "  </link>"
    "  <joint name=\"shoulder\" type=\"revolute\">"
    "    <parent link=\"base\"/>"
    "    <child link=\"arm\"/>"
    "    <origin xyz=\"0 0 1\" rpy=\"0 0 0\"/>"
    "    <axis xyz=\"0 0 1\"/>"
    "    <limit lower=\"-1.5\" upper=\"1.5\" effort=\"10\" velocity=\"2\"/>"
    "  </joint>"
    "</robot>";

/* view_proj * (p, 1), divided by w. */
static R3vVec3 project(const R3vCameraState *state, R3vVec3 p) {
    const float v[4] = {p.x, p.y, p.z, 1.0f};
    float clip[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            clip[row] += state->view_proj[column][row] * v[column];
        }
    }
    return vec3(clip[0] / clip[3], clip[1] / clip[3], clip[2] / clip[3]);
}

static R3vCpuTesselator *make_tesselator(float voxel_size) {
    R3vCpuTesselator *tesselator = r3v_cpu_tesselator_create();
    CHECK(tesselator != NULL);
    R3vCpuTesselatorConfiguration configuration = {voxel_size, false};
    CHECK_OK(r3v_cpu_tesselator_configure(tesselator, configuration));
    return tesselator;
}

static R3vUrdfDescription *robot_description(void) {
    R3vUrdfDescription *description = r3v_urdf_read_str(ROBOT_URDF);
    if (description == NULL) {
        fail(__FILE__, __LINE__, "URDF not read: %s", r3v_renderer_last_error());
    }
    return description;
}

/* ---------------------------------------------------------------- errors */

static void last_error_starts_empty(void) {
    const char *error = r3v_renderer_last_error();
    CHECK(error != NULL);
    CHECK(strcmp(error, "") == 0);
}

static void last_error_survives_success(void) {
    CHECK_STATUS(r3v_orbit_camera_zoom(NULL, 1.0f), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`camera` is NULL");
    R3vOrbitCamera camera = r3v_orbit_camera_default();
    CHECK_OK(r3v_orbit_camera_zoom(&camera, 1.0f));
    CHECK_ERROR_CONTAINS("`camera` is NULL");
}

static void drop_null_is_ignored(void) {
    r3v_wgpu_renderer_drop(NULL);
    r3v_cpu_tesselator_drop(NULL);
    r3v_geometry_delta_drop(NULL);
    r3v_stl_mesh_drop(NULL);
    r3v_urdf_description_drop(NULL);
    r3v_urdf_robot_drop(NULL);
    r3v_robot_transforms_drop(NULL);
    CHECK(strcmp(r3v_renderer_last_error(), "") == 0);
}

static void getters_of_null_are_empty(void) {
    CHECK(r3v_stl_mesh_triangle_count(NULL) == 0);
    CHECK(r3v_stl_mesh_triangles(NULL) == NULL);
    CHECK(r3v_geometry_delta_chunk_count(NULL) == 0);
    CHECK(r3v_urdf_description_name(NULL) == NULL);
    CHECK(r3v_urdf_description_link_count(NULL) == 0);
    CHECK(r3v_urdf_description_joint_count(NULL) == 0);
    CHECK(r3v_urdf_description_material_count(NULL) == 0);
    CHECK(r3v_urdf_robot_last_transform(NULL) == NULL);
}

/* ---------------------------------------------------------------- camera */

static void camera_default_looks_down_negative_z(void) {
    R3vOrbitCamera camera = r3v_orbit_camera_default();
    CHECK_VEC3(camera.target, 0.0f, 0.0f, 0.0f);
    CHECK_CLOSE(camera.distance, 5.0f);
    CHECK_CLOSE(camera.yaw, 0.0f);
    CHECK_CLOSE(camera.pitch, 0.0f);
    CHECK_CLOSE(camera.near, 0.1f);
    CHECK_CLOSE(camera.far, 100.0f);
    CHECK(camera.new_uniform_available);
    R3vVec3 forward = r3v_orbit_camera_forward(camera);
    CHECK_VEC3(forward, 0.0f, 0.0f, -1.0f);
    R3vCameraState state;
    CHECK_OK(r3v_orbit_camera_get_state(camera, 1.0f, &state));
    CHECK_VEC3(state.position, 0.0f, 0.0f, 5.0f);
    CHECK_VEC3(state.forward, 0.0f, 0.0f, -1.0f);
}

static void camera_new_takes_degrees(void) {
    R3vOrbitCamera camera = r3v_orbit_camera_new(vec3(1.0f, 2.0f, 3.0f), 2.0f, 90.0f, 0.0f);
    CHECK_CLOSE(camera.yaw, HALF_PI);
    CHECK_CLOSE(camera.pitch, 0.0f);
    R3vCameraState state;
    CHECK_OK(r3v_orbit_camera_get_state(camera, 1.0f, &state));
    CHECK_VEC3(state.position, 3.0f, 2.0f, 3.0f);
    CHECK_VEC3(state.forward, -1.0f, 0.0f, 0.0f);
}

static void camera_view_proj_is_column_major(void) {
    R3vCameraState state;
    CHECK_OK(r3v_orbit_camera_get_state(r3v_orbit_camera_default(), 2.0f, &state));
    R3vVec3 target = project(&state, vec3(0.0f, 0.0f, 0.0f));
    CHECK_CLOSE(target.x, 0.0f);
    CHECK_CLOSE(target.y, 0.0f);
    CHECK(target.z > 0.0f && target.z < 1.0f);
    /* The screen's right and up follow world +X and +Y from this camera. */
    CHECK(project(&state, vec3(1.0f, 0.0f, 0.0f)).x > 0.0f);
    CHECK(project(&state, vec3(0.0f, 1.0f, 0.0f)).y > 0.0f);
}

static void camera_mutations_match_rust(void) {
    R3vOrbitCamera camera = r3v_orbit_camera_default();
    camera.new_uniform_available = false;
    CHECK_OK(r3v_orbit_camera_rotate(&camera, 0.5f, 0.25f));
    CHECK_CLOSE(camera.yaw, 0.5f);
    CHECK_CLOSE(camera.pitch, 0.25f);
    CHECK(camera.new_uniform_available);
    CHECK_OK(r3v_orbit_camera_rotate(&camera, 0.0f, 10.0f));
    CHECK_CLOSE(camera.pitch, SAFE_PITCH);
    CHECK_OK(r3v_orbit_camera_zoom(&camera, -100.0f));
    CHECK_CLOSE(camera.distance, 0.1f);

    camera = r3v_orbit_camera_default();
    CHECK_OK(r3v_orbit_camera_pan(&camera, 1.0f, 2.0f));
    CHECK_VEC3(camera.target, 1.0f, 2.0f, 0.0f);

    CHECK_OK(r3v_orbit_camera_look_at(&camera, vec3(4.0f, 5.0f, 6.0f), 0.0f, 90.0f, 90.0f));
    CHECK_VEC3(camera.target, 4.0f, 5.0f, 6.0f);
    CHECK_CLOSE(camera.distance, 0.1f);
    CHECK_CLOSE(camera.yaw, HALF_PI);
    CHECK_CLOSE(camera.pitch, SAFE_PITCH);

    CHECK_STATUS(r3v_orbit_camera_rotate(NULL, 1.0f, 1.0f), R3V_INVALID_ARGUMENT);
    CHECK_STATUS(r3v_orbit_camera_pan(NULL, 1.0f, 1.0f), R3V_INVALID_ARGUMENT);
    CHECK_STATUS(r3v_orbit_camera_look_at(NULL, vec3(0.0f, 0.0f, 0.0f), 1.0f, 0.0f, 0.0f),
                 R3V_INVALID_ARGUMENT);
}

static void camera_with_clip_planes(void) {
    R3vOrbitCamera camera = r3v_orbit_camera_with_clip_planes(r3v_orbit_camera_default(), 0.5f, 50.0f);
    CHECK_CLOSE(camera.near, 0.5f);
    CHECK_CLOSE(camera.far, 50.0f);
    CHECK_CLOSE(camera.distance, 5.0f);
}

static void camera_get_state_checks_arguments(void) {
    R3vOrbitCamera camera = r3v_orbit_camera_default();
    R3vCameraState state;
    CHECK_STATUS(r3v_orbit_camera_get_state(camera, 0.0f, &state), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("aspect ratio");
    CHECK_STATUS(r3v_orbit_camera_get_state(camera, -1.0f, &state), R3V_INVALID_ARGUMENT);
    CHECK_STATUS(r3v_orbit_camera_get_state(camera, NAN, &state), R3V_INVALID_ARGUMENT);
    CHECK_STATUS(r3v_orbit_camera_get_state(camera, 1.0f, NULL), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`out` is NULL");

    R3vOrbitCamera bad = camera;
    bad.near = 0.0f;
    CHECK_STATUS(r3v_orbit_camera_get_state(bad, 1.0f, &state), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("clip planes");
    bad = camera;
    bad.far = bad.near;
    CHECK_STATUS(r3v_orbit_camera_get_state(bad, 1.0f, &state), R3V_INVALID_ARGUMENT);
    bad = camera;
    bad.far = INFINITY;
    CHECK_STATUS(r3v_orbit_camera_get_state(bad, 1.0f, &state), R3V_INVALID_ARGUMENT);
}

/* ------------------------------------------------------------------- stl */

static void stl_read_ascii_file(void) {
    const char *path = temp_path("triangle.stl");
    write_file(path,
               "solid t\n"
               "facet normal 0 0 1\n"
               "  outer loop\n"
               "    vertex 0 0 0\n"
               "    vertex 1 0 0\n"
               "    vertex 0 1 0\n"
               "  endloop\n"
               "endfacet\n"
               "endsolid t\n");
    R3vStlMesh *mesh = r3v_stl_read_file(path);
    if (mesh == NULL) {
        fail(__FILE__, __LINE__, "STL not read: %s", r3v_renderer_last_error());
    }
    CHECK(r3v_stl_mesh_triangle_count(mesh) == 1);
    const R3vTriangle *triangle = r3v_stl_mesh_triangles(mesh);
    CHECK(triangle != NULL);
    CHECK_CLOSE(triangle->v0[0], 0.0f);
    CHECK_CLOSE(triangle->v1[0], 1.0f);
    CHECK_CLOSE(triangle->v2[1], 1.0f);
    r3v_stl_mesh_drop(mesh);
}

static void stl_missing_file_fails(void) {
    CHECK(r3v_stl_read_file(temp_path("missing.stl")) == NULL);
    CHECK_ERROR_CONTAINS("missing.stl");
    CHECK(r3v_stl_read_file(NULL) == NULL);
    CHECK_ERROR_CONTAINS("`path` is NULL");
}

static void stl_from_triangles_is_writable(void) {
    R3vTriangle triangles[2] = {FRONT_TRIANGLE, FRONT_TRIANGLE};
    triangles[1].v2[2] = 7.0f;
    R3vStlMesh *mesh = r3v_stl_mesh_from_triangles(triangles, ARRAY_LEN(triangles));
    CHECK(mesh != NULL);
    CHECK(r3v_stl_mesh_triangle_count(mesh) == 2);
    R3vTriangle *stored = r3v_stl_mesh_triangles(mesh);
    CHECK(memcmp(stored, triangles, sizeof triangles) == 0);
    /* A copy: changing the input does not change the mesh, but writing
     * through the pointer does. */
    triangles[0].v0[0] = 42.0f;
    CHECK_CLOSE(stored[0].v0[0], -1.0f);
    stored[0].v0[0] *= 3.0f;
    CHECK_CLOSE(r3v_stl_mesh_triangles(mesh)[0].v0[0], -3.0f);
    r3v_stl_mesh_drop(mesh);
}

static void stl_empty_mesh(void) {
    R3vStlMesh *mesh = r3v_stl_mesh_from_triangles(NULL, 0);
    CHECK(mesh != NULL);
    CHECK(r3v_stl_mesh_triangle_count(mesh) == 0);
    CHECK(r3v_stl_mesh_triangles(mesh) == NULL);
    r3v_stl_mesh_drop(mesh);
    CHECK(r3v_stl_mesh_from_triangles(NULL, 3) == NULL);
    CHECK_ERROR_CONTAINS("`triangles` is NULL");
}

/* ------------------------------------------------------------ tesselator */

static void tesselator_requires_configure(void) {
    R3vCpuTesselator *tesselator = r3v_cpu_tesselator_create();
    CHECK(tesselator != NULL);
    R3vPointI block = {0, 0, 0, 1};
    CHECK_STATUS(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1),
                 R3V_NOT_CONFIGURED);
    R3vGeometryDelta *delta = (R3vGeometryDelta *)&block;
    CHECK_STATUS(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta),
                 R3V_NOT_CONFIGURED);
    CHECK(delta == NULL);
    r3v_cpu_tesselator_drop(tesselator);
}

static void tesselator_checks_voxel_size(void) {
    R3vCpuTesselator *tesselator = r3v_cpu_tesselator_create();
    const float sizes[] = {0.0f, -1.0f, NAN, INFINITY};
    for (size_t k = 0; k < ARRAY_LEN(sizes); ++k) {
        R3vCpuTesselatorConfiguration configuration = {sizes[k], false};
        CHECK_STATUS(r3v_cpu_tesselator_configure(tesselator, configuration),
                     R3V_INVALID_ARGUMENT);
    }
    CHECK_ERROR_CONTAINS("voxel size");
    R3vPointI block = {0, 0, 0, 1};
    CHECK_STATUS(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1),
                 R3V_NOT_CONFIGURED);
    r3v_cpu_tesselator_drop(tesselator);
}

static void tesselator_rejects_unaligned_blocks(void) {
    R3vCpuTesselator *tesselator = make_tesselator(1.0f);
    const R3vPointI unaligned[] = {{1, 0, 0, 1}, {0, 2, 0, 1}, {0, 0, -1, 1}};
    for (size_t k = 0; k < ARRAY_LEN(unaligned); ++k) {
        /* A valid block first: none of them may be applied. */
        R3vPointI blocks[2] = {{4, 4, 2, 1}, unaligned[k]};
        CHECK_STATUS(r3v_cpu_tesselator_update_input_geometry(tesselator, blocks, 2),
                     R3V_INVALID_ARGUMENT);
        CHECK_ERROR_CONTAINS("blocks[1]");
    }
    R3vGeometryDelta *delta = NULL;
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK(delta == NULL);
    CHECK_STATUS(r3v_cpu_tesselator_update_input_geometry(tesselator, NULL, 1),
                 R3V_INVALID_ARGUMENT);
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, NULL, 0));
    r3v_cpu_tesselator_drop(tesselator);
}

static void tesselator_full_block_is_a_box(void) {
    R3vCpuTesselator *tesselator = make_tesselator(0.5f);
    /* Inside its chunk, so the neighbouring chunks stay untouched. */
    R3vPointI block = {4, 4, 2, UINT32_MAX};
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1));
    R3vGeometryDelta *delta = NULL;
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK(delta != NULL);
    CHECK(r3v_geometry_delta_chunk_count(delta) == 1);
    R3vTriangleChunk chunk;
    CHECK_OK(r3v_geometry_delta_chunk(delta, 0, &chunk));
    CHECK(chunk.index.x == 0 && chunk.index.y == 0 && chunk.index.z == 0);
    /* Two triangles for each of the six faces. */
    CHECK(chunk.triangle_count == 12);
    CHECK(chunk.triangles != NULL);
    /* Voxels 4..8 x 4..8 x 2..4 at 0.5 world units each. */
    const float lo[3] = {2.0f, 2.0f, 1.0f}, hi[3] = {4.0f, 4.0f, 2.0f};
    for (size_t t = 0; t < chunk.triangle_count; ++t) {
        const float *corners[3] = {chunk.triangles[t].v0, chunk.triangles[t].v1,
                                   chunk.triangles[t].v2};
        for (size_t c = 0; c < 3; ++c) {
            for (size_t axis = 0; axis < 3; ++axis) {
                CHECK(corners[c][axis] >= lo[axis] - 1e-4f && corners[c][axis] <= hi[axis] + 1e-4f);
            }
        }
    }
    CHECK_STATUS(r3v_geometry_delta_chunk(delta, 1, &chunk), R3V_OUT_OF_RANGE);
    CHECK_ERROR_CONTAINS("chunk 1");
    CHECK_STATUS(r3v_geometry_delta_chunk(delta, 0, NULL), R3V_INVALID_ARGUMENT);
    r3v_geometry_delta_drop(delta);

    /* Nothing changed since, sending the same block changes nothing either. */
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK(delta == NULL);
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1));
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK(delta == NULL);

    /* Clearing it leaves the chunk without triangles. */
    block.i = 0;
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1));
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK(delta != NULL);
    CHECK(r3v_geometry_delta_chunk_count(delta) == 1);
    CHECK_OK(r3v_geometry_delta_chunk(delta, 0, &chunk));
    CHECK(chunk.triangle_count == 0);
    CHECK(chunk.triangles == NULL);
    r3v_geometry_delta_drop(delta);
    r3v_cpu_tesselator_drop(tesselator);
}

/* The next delta of `tesselator`, which has to hold exactly one chunk, and
 * that chunk in `*chunk`. */
static R3vGeometryDelta *single_chunk_delta(R3vCpuTesselator *tesselator,
                                            R3vTriangleChunk *chunk) {
    R3vGeometryDelta *delta = NULL;
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK(delta != NULL);
    CHECK(r3v_geometry_delta_chunk_count(delta) == 1);
    CHECK_OK(r3v_geometry_delta_chunk(delta, 0, chunk));
    return delta;
}

static void tesselator_toggles_marching_cubes(void) {
    R3vCpuTesselator *tesselator = make_tesselator(1.0f);
    CHECK(!r3v_cpu_tesselator_marching_cubes_enabled(tesselator));
    R3vPointI block = {4, 4, 2, UINT32_MAX};
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1));
    R3vTriangleChunk chunk;
    R3vGeometryDelta *delta = single_chunk_delta(tesselator, &chunk);
    CHECK(chunk.triangle_count == 12);
    r3v_geometry_delta_drop(delta);

    /* Switching it on tesselates the chunk again, smoothed: marching cubes
     * puts vertices between voxel corners. */
    CHECK_OK(r3v_cpu_tesselator_set_marching_cubes_enabled(tesselator, true));
    CHECK(r3v_cpu_tesselator_marching_cubes_enabled(tesselator));
    delta = single_chunk_delta(tesselator, &chunk);
    CHECK(chunk.triangle_count > 0 && chunk.triangle_count != 12);
    bool fractional = false;
    for (size_t t = 0; t < chunk.triangle_count; ++t) {
        const R3vTriangle *triangle = &chunk.triangles[t];
        for (size_t axis = 0; axis < 3; ++axis) {
            const float corners[3] = {triangle->v0[axis], triangle->v1[axis], triangle->v2[axis]};
            for (size_t c = 0; c < 3; ++c) {
                fractional = fractional || corners[c] != floorf(corners[c]);
            }
        }
    }
    CHECK(fractional);
    r3v_geometry_delta_drop(delta);

    /* The configuration switches it too. */
    R3vCpuTesselatorConfiguration configuration = {1.0f, false};
    CHECK_OK(r3v_cpu_tesselator_configure(tesselator, configuration));
    CHECK(!r3v_cpu_tesselator_marching_cubes_enabled(tesselator));
    delta = single_chunk_delta(tesselator, &chunk);
    CHECK(chunk.triangle_count == 12);
    r3v_geometry_delta_drop(delta);

    CHECK_STATUS(r3v_cpu_tesselator_set_marching_cubes_enabled(NULL, true), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`tesselator` is NULL");
    CHECK(!r3v_cpu_tesselator_marching_cubes_enabled(NULL));
    r3v_cpu_tesselator_drop(tesselator);
}

static void tesselator_null_arguments(void) {
    R3vGeometryDelta *delta = NULL;
    CHECK_STATUS(r3v_cpu_tesselator_get_output_geometry_delta(NULL, &delta),
                 R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`tesselator` is NULL");
    R3vCpuTesselator *tesselator = make_tesselator(1.0f);
    CHECK_STATUS(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, NULL),
                 R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`out` is NULL");
    r3v_cpu_tesselator_drop(tesselator);
}

/* -------------------------------------------------------------- renderer */

static void renderer_configure_checks_size(void) {
    R3vWgpuRenderer *renderer = make_renderer(8, 8);
    const R3vWgpuRendererConfiguration bad[] = {
        renderer_configuration(0, 8),
        renderer_configuration(8, 0),
        renderer_configuration(8193, 8),
        renderer_configuration(8, 8193),
    };
    for (size_t k = 0; k < ARRAY_LEN(bad); ++k) {
        CHECK_STATUS(r3v_wgpu_renderer_configure(renderer, bad[k]), R3V_INVALID_ARGUMENT);
    }
    CHECK_ERROR_CONTAINS("frame size");
    R3vWgpuRendererConfiguration largest = renderer_configuration(8192, 1);
    CHECK_OK(r3v_wgpu_renderer_configure(renderer, largest));
    CHECK_STATUS(r3v_wgpu_renderer_configure(NULL, largest), R3V_INVALID_ARGUMENT);
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_configure_checks_colors(void) {
    R3vWgpuRenderer *renderer = make_renderer(8, 8);
    const float bad_ranges[] = {0.0f, -1.0f, NAN, INFINITY};
    for (size_t k = 0; k < ARRAY_LEN(bad_ranges); ++k) {
        R3vWgpuRendererConfiguration configuration = renderer_configuration(8, 8);
        configuration.color_depth_range = bad_ranges[k];
        CHECK_STATUS(r3v_wgpu_renderer_configure(renderer, configuration), R3V_INVALID_ARGUMENT);
        CHECK_ERROR_CONTAINS("color depth range");
    }
    /* Leaving out the depth range leaves it 0. */
    R3vWgpuRendererConfiguration partial = {.width = 8, .height = 8};
    CHECK_STATUS(r3v_wgpu_renderer_configure(renderer, partial), R3V_INVALID_ARGUMENT);
    const R3vVec3 bad_colors[] = {{-0.1f, 0.0f, 0.0f}, {0.0f, 1.1f, 0.0f}, {0.0f, 0.0f, NAN}};
    for (size_t k = 0; k < ARRAY_LEN(bad_colors); ++k) {
        R3vWgpuRendererConfiguration configuration = renderer_configuration(8, 8);
        configuration.background_color = bad_colors[k];
        CHECK_STATUS(r3v_wgpu_renderer_configure(renderer, configuration), R3V_INVALID_ARGUMENT);
        CHECK_ERROR_CONTAINS("background color");
    }
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_clears_to_background_color(void) {
    enum { W = 8, H = 8 };
    R3vWgpuRenderer *renderer = make_renderer(W, H);
    R3vOrbitCamera camera = r3v_orbit_camera_default();
    static uint8_t frame[W * H * 4];
    R3vWgpuRendererConfiguration configuration = renderer_configuration(W, H);
    configuration.background_color = vec3(1.0f, 0.0f, 0.0f);
    CHECK_OK(r3v_wgpu_renderer_configure(renderer, configuration));
    render_now(renderer, camera, W, H, frame);
    CHECK(pixel_is(frame, W, 0, 0, 0, 0, 255));
    CHECK(pixel_is(frame, W, W - 1, H - 1, 0, 0, 255));

    /* At the same size it applies to the next frame; components are sRGB. */
    configuration.background_color = vec3(0.0f, 0.5f, 1.0f);
    CHECK_OK(r3v_wgpu_renderer_configure(renderer, configuration));
    render_now(renderer, camera, W, H, frame);
    CHECK(pixel_is(frame, W, W / 2, H / 2, 255, 128, 0));
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_color_depth_range_recolors_voxels(void) {
    enum { W = 32, H = 32 };
    R3vWgpuRenderer *renderer = make_renderer(W, H);
    R3vCpuTesselator *tesselator = add_block(renderer, 1.0f, 4, 4, 2);
    R3vOrbitCamera camera = r3v_orbit_camera_new(vec3(6.0f, 6.0f, 3.0f), 10.0f, 0.0f, 0.0f);
    static uint8_t frame[W * H * 4];
    const uint8_t *centre = frame + 4 * ((H / 2) * W + W / 2);
    render_now(renderer, camera, W, H, frame);
    /* Near the start of the 50 m range: turbo's blue (BGRA). */
    CHECK(centre[0] > centre[2]);

    /* The block is beyond a 5 m range: turbo's red end. */
    R3vWgpuRendererConfiguration configuration = renderer_configuration(W, H);
    configuration.color_depth_range = 5.0f;
    CHECK_OK(r3v_wgpu_renderer_configure(renderer, configuration));
    render_now(renderer, camera, W, H, frame);
    CHECK(centre[2] > centre[0]);
    r3v_cpu_tesselator_drop(tesselator);
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_render_checks_arguments(void) {
    R3vWgpuRenderer *renderer = r3v_wgpu_renderer_create();
    CHECK(renderer != NULL);
    R3vCameraState state;
    CHECK_OK(r3v_orbit_camera_get_state(r3v_orbit_camera_default(), 1.0f, &state));
    uint8_t frame[4 * 4 * 4];
    CHECK_STATUS(r3v_wgpu_renderer_render(renderer, &state, frame, sizeof frame),
                 R3V_NOT_CONFIGURED);
    CHECK_OK(r3v_wgpu_renderer_configure(renderer, renderer_configuration(4, 4)));
    CHECK_STATUS(r3v_wgpu_renderer_render(renderer, &state, frame, sizeof frame - 1),
                 R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("64 bytes");
    CHECK_STATUS(r3v_wgpu_renderer_render(renderer, &state, NULL, sizeof frame),
                 R3V_INVALID_ARGUMENT);
    CHECK_STATUS(r3v_wgpu_renderer_render(renderer, NULL, frame, sizeof frame),
                 R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`camera_state` is NULL");
    CHECK_STATUS(r3v_wgpu_renderer_render(NULL, &state, frame, sizeof frame),
                 R3V_INVALID_ARGUMENT);
    /* A larger buffer is fine. */
    uint8_t larger[sizeof frame + 16];
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, larger, sizeof larger));
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_first_frame_is_zeros(void) {
    R3vWgpuRenderer *renderer = make_renderer(16, 16);
    R3vCameraState state;
    CHECK_OK(r3v_orbit_camera_get_state(r3v_orbit_camera_default(), 1.0f, &state));
    uint8_t frame[16 * 16 * 4];
    memset(frame, 0xAB, sizeof frame);
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, frame, sizeof frame));
    for (size_t k = 0; k < sizeof frame; ++k) {
        CHECK(frame[k] == 0);
    }
    /* The next one is the cleared first frame: black, opaque. */
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, frame, sizeof frame));
    CHECK(!lit(frame, 16, 8, 8));
    CHECK(frame[3] == 255);
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_draws_voxels(void) {
    enum { W = 32, H = 32 };
    R3vWgpuRenderer *renderer = make_renderer(W, H);
    /* Voxels 4..8 x 4..8 x 2..4. */
    R3vCpuTesselator *tesselator = add_block(renderer, 1.0f, 4, 4, 2);
    R3vOrbitCamera camera = r3v_orbit_camera_new(vec3(6.0f, 6.0f, 3.0f), 10.0f, 0.0f, 0.0f);
    static uint8_t frame[W * H * 4];
    render_now(renderer, camera, W, H, frame);
    CHECK(lit(frame, W, W / 2, H / 2));
    /* Coloured by distance: a few metres along the view is turbo's blue (BGRA). */
    const uint8_t *centre = frame + 4 * ((H / 2) * W + W / 2);
    CHECK(centre[0] > centre[2]);
    CHECK(!lit(frame, W, 0, 0));
    CHECK(!lit(frame, W, W - 1, H - 1));

    /* Clearing the block clears the frame. */
    R3vPointI cleared = {4, 4, 2, 0};
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, &cleared, 1));
    R3vGeometryDelta *delta = NULL;
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK_OK(r3v_wgpu_renderer_update_geometry(renderer, delta));
    render_now(renderer, camera, W, H, frame);
    CHECK(!lit(frame, W, W / 2, H / 2));

    r3v_cpu_tesselator_drop(tesselator);
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_update_geometry_checks_arguments(void) {
    R3vWgpuRenderer *renderer = make_renderer(4, 4);
    CHECK_STATUS(r3v_wgpu_renderer_update_geometry(renderer, NULL), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`delta` is NULL");
    /* The delta is taken (and freed) even when the renderer is NULL. */
    R3vCpuTesselator *tesselator = make_tesselator(1.0f);
    R3vPointI block = {0, 0, 0, 1};
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1));
    R3vGeometryDelta *delta = NULL;
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK_STATUS(r3v_wgpu_renderer_update_geometry(NULL, delta), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`renderer` is NULL");
    r3v_cpu_tesselator_drop(tesselator);
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_draws_assets(void) {
    enum { W = 32, H = 32 };
    R3vWgpuRenderer *renderer = make_renderer(W, H);
    R3vStlMesh *mesh = r3v_stl_mesh_from_triangles(&FRONT_TRIANGLE, 1);
    CHECK(mesh != NULL);
    uint64_t ids[2] = {99, 99};
    CHECK_OK(r3v_wgpu_renderer_upload_asset(renderer, mesh, 2, ids));
    CHECK(ids[0] == 0 && ids[1] == 1);
    uint64_t more = 99;
    CHECK_OK(r3v_wgpu_renderer_upload_asset(renderer, mesh, 1, &more));
    CHECK(more == 2);
    /* The renderer copied the mesh. */
    r3v_stl_mesh_drop(mesh);
    CHECK_OK(r3v_wgpu_renderer_set_asset_visibility(renderer, 2, false));

    R3vOrbitCamera camera = r3v_orbit_camera_default();
    static uint8_t frame[W * H * 4];
    render_now(renderer, camera, W, H, frame);
    CHECK(lit(frame, W, W / 2, H / 2));
    CHECK(!lit(frame, W, 0, 0));

    CHECK_OK(r3v_wgpu_renderer_set_asset_visibility(renderer, ids[0], false));
    CHECK_OK(r3v_wgpu_renderer_set_asset_visibility(renderer, ids[1], false));
    render_now(renderer, camera, W, H, frame);
    CHECK(!lit(frame, W, W / 2, H / 2));

    CHECK_OK(r3v_wgpu_renderer_set_asset_visibility(renderer, ids[1], true));
    render_now(renderer, camera, W, H, frame);
    CHECK(lit(frame, W, W / 2, H / 2));

    R3vTwist away = {{100.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    CHECK_OK(r3v_wgpu_renderer_set_asset_transform(renderer, ids[1], away));
    render_now(renderer, camera, W, H, frame);
    CHECK(!lit(frame, W, W / 2, H / 2));

    /* Turned half a turn around Y it shows its back, which is culled. */
    R3vTwist back = {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}};
    CHECK_OK(r3v_wgpu_renderer_set_asset_transform(renderer, ids[1], back));
    render_now(renderer, camera, W, H, frame);
    CHECK(!lit(frame, W, W / 2, H / 2));
    R3vTwist identity = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    CHECK_OK(r3v_wgpu_renderer_set_asset_transform(renderer, ids[1], identity));
    render_now(renderer, camera, W, H, frame);
    CHECK(lit(frame, W, W / 2, H / 2));

    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_colors_assets(void) {
    enum { W = 32, H = 32 };
    R3vWgpuRenderer *renderer = make_renderer(W, H);
    R3vStlMesh *mesh = r3v_stl_mesh_from_triangles(&FRONT_TRIANGLE, 1);
    uint64_t id;
    CHECK_OK(r3v_wgpu_renderer_upload_asset(renderer, mesh, 1, &id));
    r3v_stl_mesh_drop(mesh);
    R3vOrbitCamera camera = r3v_orbit_camera_default();
    static uint8_t frame[W * H * 4];
    const uint8_t *center = frame + ((H / 2) * W + W / 2) * 4;

    render_now(renderer, camera, W, H, frame);
    uint8_t white = center[2];
    CHECK(white != 0 && center[0] == white && center[1] == white);

    R3vVec3 red = {1.0f, 0.0f, 0.0f};
    CHECK_OK(r3v_wgpu_renderer_set_asset_color(renderer, id, red));
    render_now(renderer, camera, W, H, frame);
    CHECK(center[2] == white && center[1] == 0 && center[0] == 0);

    R3vVec3 too_bright = {1.5f, 0.0f, 0.0f};
    CHECK_STATUS(r3v_wgpu_renderer_set_asset_color(renderer, id, too_bright),
                 R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("between 0 and 1");
    R3vVec3 not_a_number = {NAN, 0.0f, 0.0f};
    CHECK_STATUS(r3v_wgpu_renderer_set_asset_color(renderer, id, not_a_number),
                 R3V_INVALID_ARGUMENT);
    CHECK_STATUS(r3v_wgpu_renderer_set_asset_color(renderer, id + 1, red), R3V_OUT_OF_RANGE);
    render_now(renderer, camera, W, H, frame);
    CHECK(center[2] == white && center[1] == 0 && center[0] == 0);

    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_rejects_unknown_instances(void) {
    R3vWgpuRenderer *renderer = make_renderer(4, 4);
    R3vTwist identity = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    CHECK_STATUS(r3v_wgpu_renderer_set_asset_transform(renderer, 0, identity), R3V_OUT_OF_RANGE);
    CHECK_ERROR_CONTAINS("instance 0");
    R3vStlMesh *mesh = r3v_stl_mesh_from_triangles(&FRONT_TRIANGLE, 1);
    uint64_t id;
    CHECK_OK(r3v_wgpu_renderer_upload_asset(renderer, mesh, 1, &id));
    CHECK_OK(r3v_wgpu_renderer_set_asset_transform(renderer, id, identity));
    CHECK_STATUS(r3v_wgpu_renderer_set_asset_transform(renderer, id + 1, identity),
                 R3V_OUT_OF_RANGE);
    CHECK_STATUS(r3v_wgpu_renderer_set_asset_visibility(renderer, id + 1, false),
                 R3V_OUT_OF_RANGE);
    CHECK_STATUS(r3v_wgpu_renderer_set_asset_visibility(renderer, UINT64_MAX, false),
                 R3V_OUT_OF_RANGE);
    r3v_stl_mesh_drop(mesh);
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_upload_checks_arguments(void) {
    R3vWgpuRenderer *renderer = make_renderer(4, 4);
    R3vStlMesh *mesh = r3v_stl_mesh_from_triangles(&FRONT_TRIANGLE, 1);
    /* No instances need no ids. */
    CHECK_OK(r3v_wgpu_renderer_upload_asset(renderer, mesh, 0, NULL));
    CHECK_STATUS(r3v_wgpu_renderer_upload_asset(renderer, mesh, 2, NULL), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`out_ids` is NULL");
    uint64_t id;
    CHECK_STATUS(r3v_wgpu_renderer_upload_asset(renderer, NULL, 1, &id), R3V_INVALID_ARGUMENT);
    CHECK_ERROR_CONTAINS("`mesh` is NULL");
    CHECK_STATUS(r3v_wgpu_renderer_upload_asset(renderer, mesh, (size_t)UINT32_MAX + 1, &id),
                 R3V_INVALID_ARGUMENT);
    /* Failed uploads take no ids. */
    CHECK_OK(r3v_wgpu_renderer_upload_asset(renderer, mesh, 1, &id));
    CHECK(id == 0);
    r3v_stl_mesh_drop(mesh);
    r3v_wgpu_renderer_drop(renderer);
}

static void renderer_resize_restarts_readback(void) {
    R3vWgpuRenderer *renderer = make_renderer(32, 32);
    R3vStlMesh *mesh = r3v_stl_mesh_from_triangles(&FRONT_TRIANGLE, 1);
    uint64_t id;
    CHECK_OK(r3v_wgpu_renderer_upload_asset(renderer, mesh, 1, &id));
    r3v_stl_mesh_drop(mesh);
    R3vOrbitCamera camera = r3v_orbit_camera_default();
    static uint8_t frame[32 * 32 * 4];
    render_now(renderer, camera, 32, 32, frame);
    CHECK(lit(frame, 32, 16, 16));

    CHECK_OK(r3v_wgpu_renderer_configure(renderer, renderer_configuration(16, 8)));
    R3vCameraState state;
    CHECK_OK(r3v_orbit_camera_get_state(camera, 2.0f, &state));
    memset(frame, 0xAB, sizeof frame);
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, frame, 16 * 8 * 4));
    for (size_t k = 0; k < 16 * 8 * 4; ++k) {
        CHECK(frame[k] == 0);
    }
    /* Only the frame was written. */
    CHECK(frame[16 * 8 * 4] == 0xAB);
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, frame, 16 * 8 * 4));
    CHECK(lit(frame, 16, 8, 4));
    r3v_wgpu_renderer_drop(renderer);
}

/* ----------------------------------------------------------------- robot */

static void urdf_description_reads_everything(void) {
    R3vUrdfDescription *description = robot_description();
    CHECK(strcmp(r3v_urdf_description_name(description), "test_bot") == 0);

    CHECK(r3v_urdf_description_link_count(description) == 2);
    R3vUrdfLink link;
    CHECK_OK(r3v_urdf_description_link(description, 0, &link));
    CHECK(strcmp(link.name, "base") == 0);
    CHECK(link.visual_count == 1);
    CHECK_OK(r3v_urdf_description_link(description, 1, &link));
    CHECK(strcmp(link.name, "arm") == 0);
    CHECK(link.visual_count == 2);

    R3vUrdfVisual visual;
    CHECK_OK(r3v_urdf_description_visual(description, 0, 0, &visual));
    CHECK(strcmp(visual.name, "body") == 0);
    CHECK(visual.has_origin);
    CHECK_VEC3(visual.origin.xyz, 0.0f, 0.0f, 0.5f);
    CHECK(visual.has_geometry);
    CHECK(visual.geometry.kind == R3V_URDF_GEOMETRY_MESH);
    /* read_str has no directory to resolve against. */
    CHECK(strcmp(visual.geometry.mesh_filename, "meshes/base.stl") == 0);
    CHECK(visual.geometry.has_mesh_scale);
    CHECK_VEC3(visual.geometry.mesh_scale, 2.0f, 2.0f, 2.0f);
    CHECK(!visual.has_material);

    CHECK_OK(r3v_urdf_description_visual(description, 1, 0, &visual));
    CHECK(visual.name == NULL);
    CHECK(visual.geometry.kind == R3V_URDF_GEOMETRY_BOX);
    CHECK(visual.geometry.mesh_filename == NULL);
    CHECK_VEC3(visual.geometry.box_size, 1.0f, 2.0f, 3.0f);
    CHECK(visual.has_material);
    CHECK(strcmp(visual.material.name, "red") == 0);

    CHECK_OK(r3v_urdf_description_visual(description, 1, 1, &visual));
    CHECK(visual.geometry.kind == R3V_URDF_GEOMETRY_CYLINDER);
    CHECK_CLOSE(visual.geometry.radius, 0.5f);
    CHECK_CLOSE(visual.geometry.length, 2.0f);

    CHECK(r3v_urdf_description_joint_count(description) == 1);
    R3vUrdfJoint joint;
    CHECK_OK(r3v_urdf_description_joint(description, 0, &joint));
    CHECK(strcmp(joint.name, "shoulder") == 0);
    CHECK(strcmp(joint.joint_type, "revolute") == 0);
    CHECK(strcmp(joint.parent, "base") == 0);
    CHECK(strcmp(joint.child, "arm") == 0);
    CHECK(joint.has_origin);
    CHECK_VEC3(joint.origin.xyz, 0.0f, 0.0f, 1.0f);
    CHECK(joint.has_axis);
    CHECK_VEC3(joint.axis, 0.0f, 0.0f, 1.0f);
    CHECK(joint.has_limit);
    CHECK_CLOSE(joint.limit.lower, -1.5f);
    CHECK_CLOSE(joint.limit.upper, 1.5f);
    CHECK_CLOSE(joint.limit.effort, 10.0f);
    CHECK_CLOSE(joint.limit.velocity, 2.0f);

    CHECK(r3v_urdf_description_material_count(description) == 1);
    R3vUrdfMaterial material;
    CHECK_OK(r3v_urdf_description_material(description, 0, &material));
    CHECK(strcmp(material.name, "red") == 0);
    CHECK(material.has_color);
    CHECK_CLOSE(material.rgba[0], 1.0f);
    CHECK_CLOSE(material.rgba[1], 0.0f);
    CHECK_CLOSE(material.rgba[3], 1.0f);

    r3v_urdf_description_drop(description);
}

static void urdf_description_checks_indices(void) {
    R3vUrdfDescription *description = robot_description();
    R3vUrdfLink link;
    CHECK_STATUS(r3v_urdf_description_link(description, 2, &link), R3V_OUT_OF_RANGE);
    CHECK_ERROR_CONTAINS("link 2");
    R3vUrdfVisual visual;
    CHECK_STATUS(r3v_urdf_description_visual(description, 0, 1, &visual), R3V_OUT_OF_RANGE);
    CHECK_ERROR_CONTAINS("visual 1");
    CHECK_STATUS(r3v_urdf_description_visual(description, 5, 0, &visual), R3V_OUT_OF_RANGE);
    CHECK_ERROR_CONTAINS("link 5");
    R3vUrdfJoint joint;
    CHECK_STATUS(r3v_urdf_description_joint(description, 1, &joint), R3V_OUT_OF_RANGE);
    R3vUrdfMaterial material;
    CHECK_STATUS(r3v_urdf_description_material(description, 1, &material), R3V_OUT_OF_RANGE);
    CHECK_STATUS(r3v_urdf_description_link(description, 0, NULL), R3V_INVALID_ARGUMENT);
    CHECK_STATUS(r3v_urdf_description_link(NULL, 0, &link), R3V_INVALID_ARGUMENT);
    r3v_urdf_description_drop(description);
}

static void urdf_read_rejects_bad_input(void) {
    CHECK(r3v_urdf_read_str("<link name=\"base\"/>") == NULL);
    CHECK_ERROR_CONTAINS("<robot>");
    CHECK(r3v_urdf_read_str(NULL) == NULL);
    CHECK_ERROR_CONTAINS("`xml` is NULL");
    CHECK(r3v_urdf_read_file(temp_path("missing.urdf")) == NULL);
    CHECK_ERROR_CONTAINS("missing.urdf");
}

static void urdf_read_file_resolves_meshes(void) {
    const char *path = temp_path("robot.urdf");
    write_file(path, ROBOT_URDF);
    R3vUrdfDescription *description = r3v_urdf_read_file(path);
    if (description == NULL) {
        fail(__FILE__, __LINE__, "URDF not read: %s", r3v_renderer_last_error());
    }
    R3vUrdfVisual visual;
    CHECK_OK(r3v_urdf_description_visual(description, 0, 0, &visual));
    const char *filename = visual.geometry.mesh_filename;
    const char *suffix = "/meshes/base.stl";
    CHECK(filename[0] == '/');
    CHECK(strlen(filename) > strlen(suffix));
    CHECK(strcmp(filename + strlen(filename) - strlen(suffix), suffix) == 0);
    r3v_urdf_description_drop(description);
}

static void urdf_robot_moves_links(void) {
    R3vUrdfDescription *description = robot_description();
    R3vUrdfRobot *robot = r3v_urdf_robot_from_description(description);
    CHECK(robot != NULL);
    /* The robot has its own copy. */
    r3v_urdf_description_drop(description);
    CHECK(r3v_urdf_robot_last_transform(robot) == NULL);

    const char *names[] = {"shoulder"};
    const float values[] = {HALF_PI};
    R3vRobotTransforms *transforms = r3v_urdf_robot_feed_sensor_input(robot, names, values, 1);
    if (transforms == NULL) {
        fail(__FILE__, __LINE__, "no transforms: %s", r3v_renderer_last_error());
    }

    R3vTwist twist;
    CHECK_OK(r3v_robot_transforms_link(transforms, "base", &twist));
    CHECK_VEC3(twist.linear, 0.0f, 0.0f, 0.0f);
    CHECK_CLOSE(twist.rotational.w, 1.0f);
    CHECK_OK(r3v_robot_transforms_link(transforms, "arm", &twist));
    CHECK_VEC3(twist.linear, 0.0f, 0.0f, 1.0f);
    CHECK_CLOSE(twist.rotational.z, sqrtf(0.5f));
    CHECK_CLOSE(twist.rotational.w, sqrtf(0.5f));
    CHECK_OK(r3v_robot_transforms_visual(transforms, "base", 0, &twist));
    CHECK_VEC3(twist.linear, 0.0f, 0.0f, 0.5f);

    R3vVec3 point;
    CHECK_OK(r3v_robot_transforms_get_link_origin(transforms, "arm", &point));
    CHECK_VEC3(point, 0.0f, 0.0f, 1.0f);
    CHECK_OK(r3v_robot_transforms_transform_visual_point(transforms, "arm", 0,
                                                         vec3(1.0f, 0.0f, 0.0f), &point));
    CHECK_VEC3(point, 0.0f, 1.0f, 1.0f);

    CHECK_STATUS(r3v_robot_transforms_link(transforms, "nope", &twist), R3V_NOT_FOUND);
    CHECK_ERROR_CONTAINS("\"nope\"");
    CHECK_STATUS(r3v_robot_transforms_visual(transforms, "arm", 5, &twist), R3V_NOT_FOUND);
    CHECK_STATUS(r3v_robot_transforms_get_link_origin(transforms, "nope", &point), R3V_NOT_FOUND);
    CHECK_STATUS(r3v_robot_transforms_transform_visual_point(transforms, "base", 1, point, &point),
                 R3V_NOT_FOUND);
    CHECK_STATUS(r3v_robot_transforms_link(transforms, NULL, &twist), R3V_INVALID_ARGUMENT);
    CHECK_STATUS(r3v_robot_transforms_link(NULL, "arm", &twist), R3V_INVALID_ARGUMENT);

    /* The last transform is the robot's own copy of what feed returned. */
    const R3vRobotTransforms *last = r3v_urdf_robot_last_transform(robot);
    CHECK(last != NULL && last != transforms);
    CHECK_OK(r3v_robot_transforms_link(last, "arm", &twist));
    CHECK_CLOSE(twist.rotational.w, sqrtf(0.5f));
    r3v_robot_transforms_drop(transforms);

    /* Joints left out are at 0. */
    transforms = r3v_urdf_robot_feed_sensor_input(robot, NULL, NULL, 0);
    CHECK(transforms != NULL);
    CHECK_OK(r3v_robot_transforms_link(r3v_urdf_robot_last_transform(robot), "arm", &twist));
    CHECK_CLOSE(twist.rotational.w, 1.0f);
    r3v_robot_transforms_drop(transforms);
    r3v_urdf_robot_drop(robot);
}

static void urdf_robot_feed_checks_arguments(void) {
    R3vUrdfDescription *description = robot_description();
    R3vUrdfRobot *robot = r3v_urdf_robot_from_description(description);
    r3v_urdf_description_drop(description);
    const float values[] = {1.0f, 2.0f};
    CHECK(r3v_urdf_robot_feed_sensor_input(robot, NULL, values, 2) == NULL);
    CHECK_ERROR_CONTAINS("`joint_names` is NULL");
    const char *names[] = {"shoulder", NULL};
    CHECK(r3v_urdf_robot_feed_sensor_input(robot, names, values, 2) == NULL);
    CHECK_ERROR_CONTAINS("`joint_names[1]` is NULL");
    CHECK(r3v_urdf_robot_feed_sensor_input(robot, names, NULL, 1) == NULL);
    CHECK_ERROR_CONTAINS("`joint_values` is NULL");
    CHECK(r3v_urdf_robot_feed_sensor_input(NULL, names, values, 1) == NULL);
    CHECK_ERROR_CONTAINS("`robot` is NULL");
    CHECK(r3v_urdf_robot_last_transform(robot) == NULL);
    CHECK(r3v_urdf_robot_from_description(NULL) == NULL);
    r3v_urdf_robot_drop(robot);
}

#define TEST(fn) {#fn, fn}

const TestCase TEST_CASES[] = {
    TEST(last_error_starts_empty),
    TEST(last_error_survives_success),
    TEST(drop_null_is_ignored),
    TEST(getters_of_null_are_empty),
    TEST(camera_default_looks_down_negative_z),
    TEST(camera_new_takes_degrees),
    TEST(camera_view_proj_is_column_major),
    TEST(camera_mutations_match_rust),
    TEST(camera_with_clip_planes),
    TEST(camera_get_state_checks_arguments),
    TEST(stl_read_ascii_file),
    TEST(stl_missing_file_fails),
    TEST(stl_from_triangles_is_writable),
    TEST(stl_empty_mesh),
    TEST(tesselator_requires_configure),
    TEST(tesselator_checks_voxel_size),
    TEST(tesselator_rejects_unaligned_blocks),
    TEST(tesselator_full_block_is_a_box),
    TEST(tesselator_toggles_marching_cubes),
    TEST(tesselator_null_arguments),
    TEST(renderer_configure_checks_size),
    TEST(renderer_configure_checks_colors),
    TEST(renderer_clears_to_background_color),
    TEST(renderer_color_depth_range_recolors_voxels),
    TEST(renderer_render_checks_arguments),
    TEST(renderer_first_frame_is_zeros),
    TEST(renderer_draws_voxels),
    TEST(renderer_update_geometry_checks_arguments),
    TEST(renderer_draws_assets),
    TEST(renderer_colors_assets),
    TEST(renderer_rejects_unknown_instances),
    TEST(renderer_upload_checks_arguments),
    TEST(renderer_resize_restarts_readback),
    TEST(urdf_description_reads_everything),
    TEST(urdf_description_checks_indices),
    TEST(urdf_read_rejects_bad_input),
    TEST(urdf_read_file_resolves_meshes),
    TEST(urdf_robot_moves_links),
    TEST(urdf_robot_feed_checks_arguments),
};
const size_t TEST_CASE_COUNT = ARRAY_LEN(TEST_CASES);
