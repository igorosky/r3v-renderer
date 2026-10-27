#include "harness.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------- checks */

void fail(const char *file, int line, const char *fmt, ...) {
    va_list args;
    fprintf(stderr, "%s:%d: ", file, line);
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fputc('\n', stderr);
    exit(1);
}

void check_status(R3vStatus got, R3vStatus expected, const char *call, const char *file,
                  int line) {
    if (got != expected) {
        fail(file, line, "%s returned %d, expected %d (last error: %s)", call, (int)got,
             (int)expected, r3v_renderer_last_error());
    }
}

void check_error_contains(const char *needle, const char *file, int line) {
    const char *error = r3v_renderer_last_error();
    if (error == NULL || strstr(error, needle) == NULL) {
        fail(file, line, "last error \"%s\" does not contain \"%s\"", error ? error : "(NULL)",
             needle);
    }
}

void check_close(float got, float expected, const char *what, const char *file, int line) {
    if (!(fabsf(got - expected) <= 1e-4f)) {
        fail(file, line, "%s is %f, expected %f", what, (double)got, (double)expected);
    }
}

/* ------------------------------------------------------------- helpers */

R3vVec3 vec3(float x, float y, float z) {
    R3vVec3 v = {x, y, z};
    return v;
}

const char *temp_path(const char *name) {
    static char path[4096];
    const char *dir = getenv("R3V_TEST_TMPDIR");
    CHECK(dir != NULL);
    int len = snprintf(path, sizeof path, "%s/%s", dir, name);
    CHECK(len > 0 && (size_t)len < sizeof path);
    return path;
}

void write_file(const char *path, const char *contents) {
    FILE *file = fopen(path, "w");
    CHECK(file != NULL);
    CHECK(fputs(contents, file) >= 0);
    CHECK(fclose(file) == 0);
}

R3vWgpuRendererConfiguration renderer_configuration(uint32_t width, uint32_t height) {
    R3vWgpuRendererConfiguration configuration = {width, height, {0.0f, 0.0f, 0.0f}, 50.0f};
    return configuration;
}

R3vWgpuRenderer *make_renderer(uint32_t width, uint32_t height) {
    R3vWgpuRenderer *renderer = r3v_wgpu_renderer_create();
    if (renderer == NULL) {
        fail(__FILE__, __LINE__, "no renderer: %s", r3v_renderer_last_error());
    }
    CHECK_OK(r3v_wgpu_renderer_configure(renderer, renderer_configuration(width, height)));
    return renderer;
}

R3vCpuTesselator *add_block(R3vWgpuRenderer *renderer, float voxel_size, int32_t x, int32_t y,
                            int32_t z) {
    R3vCpuTesselator *tesselator = r3v_cpu_tesselator_create();
    CHECK(tesselator != NULL);
    R3vCpuTesselatorConfiguration configuration = {voxel_size, false};
    CHECK_OK(r3v_cpu_tesselator_configure(tesselator, configuration));
    R3vPointI block = {x, y, z, UINT32_MAX};
    CHECK_OK(r3v_cpu_tesselator_update_input_geometry(tesselator, &block, 1));
    R3vGeometryDelta *delta = NULL;
    CHECK_OK(r3v_cpu_tesselator_get_output_geometry_delta(tesselator, &delta));
    CHECK(delta != NULL);
    CHECK_OK(r3v_wgpu_renderer_update_geometry(renderer, delta));
    return tesselator;
}

void render_now(R3vWgpuRenderer *renderer, R3vOrbitCamera camera, uint32_t width,
                uint32_t height, uint8_t *frame) {
    R3vCameraState state;
    CHECK_OK(r3v_orbit_camera_get_state(camera, (float)width / (float)height, &state));
    size_t len = (size_t)width * height * 4;
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, frame, len));
    CHECK_OK(r3v_wgpu_renderer_render(renderer, &state, frame, len));
}

bool lit(const uint8_t *frame, uint32_t width, uint32_t x, uint32_t y) {
    const uint8_t *pixel = frame + ((size_t)y * width + x) * 4;
    return pixel[0] != 0 || pixel[1] != 0 || pixel[2] != 0;
}

bool pixel_is(const uint8_t *frame, uint32_t width, uint32_t x, uint32_t y, uint8_t b, uint8_t g,
              uint8_t r) {
    const uint8_t *pixel = frame + ((size_t)y * width + x) * 4;
    const uint8_t expected[4] = {b, g, r, 255};
    for (size_t k = 0; k < 4; ++k) {
        if (abs((int)pixel[k] - (int)expected[k]) > 1) {
            return false;
        }
    }
    return true;
}

/* ---------------------------------------------------------------- main */

static int usage(const char *prog) {
    fprintf(stderr, "usage: %s list | run <test>\n", prog);
    return 2;
}

int main(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "list") == 0) {
        for (size_t t = 0; t < TEST_CASE_COUNT; ++t) {
            printf("%s\n", TEST_CASES[t].name);
        }
        return 0;
    }
    if (argc == 3 && strcmp(argv[1], "run") == 0) {
        for (size_t t = 0; t < TEST_CASE_COUNT; ++t) {
            if (strcmp(TEST_CASES[t].name, argv[2]) == 0) {
                TEST_CASES[t].fn();
                return 0;
            }
        }
        fprintf(stderr, "unknown test %s\n", argv[2]);
        return 2;
    }
    return usage(argv[0]);
}
