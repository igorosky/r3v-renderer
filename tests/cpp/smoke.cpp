// Uses every component once through the C++ header, so a missing symbol
// fails to link. The C tests (tests/c) check the behaviour.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "r3v-renderer.hpp"

using namespace r3v::renderer;

#define CHECK(cond)                                                                      \
    do {                                                                                 \
        if (!(cond)) {                                                                   \
            std::fprintf(stderr, "%s:%d: check failed: %s (last error: %s)\n", __FILE__, \
                         __LINE__, #cond, r3v_renderer_last_error());                    \
            std::exit(1);                                                                \
        }                                                                                \
    } while (0)

#define CHECK_OK(call) CHECK((call) == R3vStatus::R3V_OK)

int main() {
    CHECK(std::strcmp(r3v_renderer_last_error(), "") == 0);

    R3vUrdfDescription *description = r3v_urdf_read_str(
        "<robot name=\"bot\"><link name=\"base\"/></robot>");
    CHECK(description != nullptr);
    CHECK(std::strcmp(r3v_urdf_description_name(description), "bot") == 0);
    R3vUrdfRobot *robot = r3v_urdf_robot_from_description(description);
    R3vRobotTransforms *transforms = r3v_urdf_robot_feed_sensor_input(robot, nullptr, nullptr, 0);
    R3vVec3 origin{};
    CHECK_OK(r3v_robot_transforms_get_link_origin(transforms, "base", &origin));
    r3v_robot_transforms_drop(transforms);
    r3v_urdf_robot_drop(robot);
    r3v_urdf_description_drop(description);

    R3vCpuTesselator *tesselator = r3v_cpu_tesselator_create();
    CHECK_OK(r3v_cpu_tesselator_configure(tesselator, R3vCpuTesselatorConfiguration{1.0f, false}));
    const R3vPointI block{4, 4, 2, 0xFFFFFFFFu};
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1));
    R3vGeometryDelta *delta = nullptr;
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK(r3v_geometry_delta_chunk_count(delta) == 1);

    const uint32_t width = 16, height = 16;
    R3vWgpuRenderer *renderer = r3v_wgpu_renderer_create();
    CHECK(renderer != nullptr);
    CHECK_OK(r3v_wgpu_renderer_configure(renderer, R3vWgpuRendererConfiguration{width, height, {0.0f, 0.0f, 0.0f}, 50.0f}));
    CHECK_OK(r3v_wgpu_renderer_update_geometry(renderer, delta));
    const R3vTriangle triangle{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    R3vStlMesh *mesh = r3v_stl_mesh_from_triangles(&triangle, 1);
    uint64_t id = 0;
    CHECK_OK(r3v_wgpu_renderer_upload_asset(renderer, mesh, 1, &id));
    CHECK_OK(r3v_wgpu_renderer_set_asset_color(renderer, id, R3vVec3{1.0f, 0.5f, 0.0f}));
    CHECK_OK(r3v_wgpu_renderer_set_asset_visibility(renderer, id, false));
    r3v_stl_mesh_drop(mesh);

    R3vOrbitCamera camera = r3v_orbit_camera_default();
    CHECK_OK(r3v_orbit_camera_look_at(&camera, R3vVec3{6, 6, 3}, 10, 0, 0));
    R3vCameraState state{};
    CHECK_OK(r3v_orbit_camera_get_state(camera, 1.0f, &state));
    std::vector<uint8_t> frame(width * height * 4);
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, frame.data(), frame.size()));
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, frame.data(), frame.size()));
    const uint8_t *center = &frame[(height / 2 * width + width / 2) * 4];
    CHECK(center[0] != 0 || center[1] != 0 || center[2] != 0);

    r3v_wgpu_renderer_drop(renderer);
    r3v_cpu_tesselator_drop(tesselator);
    return 0;
}
