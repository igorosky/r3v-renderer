/*
 * Renders a staircase of voxels and a triangle mesh with the C API and
 * writes the frame to a binary PPM image.
 *
 *     cargo build --release --features c
 *     cc -std=c11 -I bindings examples/c/voxels_to_ppm.c \
 *         -L target/release -lr3v_renderer -o voxels_to_ppm
 *     ./voxels_to_ppm stairs.ppm
 *
 * Run it with target/release in the library path (LD_LIBRARY_PATH on Linux,
 * DYLD_LIBRARY_PATH on macOS).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "r3v-renderer.h"

#define WIDTH 320
#define HEIGHT 240
#define STEPS 8

/* Prints the last error and exits unless `status` is R3V_OK. */
static void check(R3vStatus status, const char *what) {
    if (status != R3V_OK) {
        fprintf(stderr, "%s failed: %s\n", what, r3v_renderer_last_error());
        exit(1);
    }
}

static void *check_handle(void *handle, const char *what) {
    if (handle == NULL) {
        fprintf(stderr, "%s failed: %s\n", what, r3v_renderer_last_error());
        exit(1);
    }
    return handle;
}

/* Feeds a staircase of full 4x4x2 blocks through the tesselator into the
 * renderer: step i is i / 2 + 1 blocks high (Y is up) and 4 blocks deep. */
static void add_stairs(R3vWgpuRenderer *renderer) {
    R3vCpuTesselator *tesselator =
        check_handle(r3v_cpu_tesselator_create(), "creating the tesselator");
    R3vCpuTesselatorConfiguration configuration = {0.25f, false};
    check(r3v_cpu_tesselator_configure(tesselator, configuration), "configure");

    R3vPointI blocks[STEPS * (STEPS / 2) * 4];
    size_t count = 0;
    for (int32_t step = 0; step < STEPS; ++step) {
        for (int32_t level = 0; level <= step / 2; ++level) {
            for (int32_t depth = 0; depth < 4; ++depth) {
                R3vPointI block = {4 * step, 4 * level, 2 * depth, UINT32_MAX};
                blocks[count++] = block;
            }
        }
    }
    check(r3v_cpu_tesselator_update_input_geometry(tesselator, blocks, count),
          "update_input_geometry");

    R3vGeometryDelta *delta = NULL;
    check(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, NULL, &delta),
          "get_output_geometry_delta");
    if (delta != NULL) {
        /* Takes the delta. */
        check(r3v_wgpu_renderer_update_geometry(renderer, delta), "update_geometry");
    }
    r3v_cpu_tesselator_drop(tesselator);
}

/* Two instances of one triangle above the stairs. */
static void add_flags(R3vWgpuRenderer *renderer) {
    const R3vTriangle triangle = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
    R3vStlMesh *mesh =
        check_handle(r3v_stl_mesh_from_triangles(&triangle, 1), "creating the mesh");
    uint64_t ids[2];
    check(r3v_wgpu_renderer_upload_asset(renderer, mesh, 2, ids), "upload_asset");
    /* The renderer keeps a copy. */
    r3v_stl_mesh_drop(mesh);

    for (size_t k = 0; k < 2; ++k) {
        R3vTwist twist = {{1.0f + 4.0f * (float)k, 3.0f, 2.5f}, {0.0f, 0.0f, 0.0f, 1.0f}};
        check(r3v_wgpu_renderer_set_asset_transform(renderer, ids[k], twist),
              "set_asset_transform");
    }
}

static void write_ppm(const char *path, const uint8_t *bgra) {
    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        perror(path);
        exit(1);
    }
    fprintf(file, "P6\n%d %d\n255\n", WIDTH, HEIGHT);
    for (size_t pixel = 0; pixel < (size_t)WIDTH * HEIGHT; ++pixel) {
        const uint8_t rgb[3] = {bgra[4 * pixel + 2], bgra[4 * pixel + 1], bgra[4 * pixel]};
        fwrite(rgb, 1, sizeof rgb, file);
    }
    if (fclose(file) != 0) {
        perror(path);
        exit(1);
    }
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s <output.ppm>\n", argv[0]);
        return 2;
    }

    R3vWgpuRenderer *renderer = check_handle(r3v_wgpu_renderer_create(), "creating the renderer");
    /* Black background, the colormap spanning 50 m from the camera. */
    R3vWgpuRendererConfiguration configuration = {WIDTH, HEIGHT, {0.0f, 0.0f, 0.0f}, 50.0f,
                                                  r3v_terrain_coloring_default()};
    check(r3v_wgpu_renderer_configure(renderer, configuration), "configure");
    add_stairs(renderer);
    add_flags(renderer);

    R3vOrbitCamera camera = r3v_orbit_camera_new((R3vVec3){4.0f, 1.0f, 1.0f}, 12.0f, 30.0f, 25.0f);
    R3vCameraState state;
    check(r3v_orbit_camera_get_state(camera, (float)WIDTH / HEIGHT, &state), "get_state");

    /* Frames come back one call late, the first call gives zeros. */
    static uint8_t frame[WIDTH * HEIGHT * 4];
    check(r3v_wgpu_renderer_render(renderer, &state, frame, sizeof frame), "render");
    check(r3v_wgpu_renderer_render(renderer, &state, frame, sizeof frame), "render");
    r3v_wgpu_renderer_drop(renderer);

    write_ppm(argv[1], frame);
    return 0;
}
