/*
 * Minimal test harness for the C bindings.
 *
 * Every test case runs in its own process (see tests/c_bindings.rs), so a
 * failed check prints a message and exits right away.
 */
#ifndef RENDERER_C_TEST_HARNESS_H
#define RENDERER_C_TEST_HARNESS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "r3v-renderer.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

typedef void (*TestFn)(void);

typedef struct TestCase {
    const char *name;
    TestFn fn;
} TestCase;

/* Defined by the test file. */
extern const TestCase TEST_CASES[];
extern const size_t TEST_CASE_COUNT;

/* ------------------------------------------------------------- checks */

void fail(const char *file, int line, const char *fmt, ...);

#define CHECK(cond)                                              \
    do {                                                         \
        if (!(cond)) {                                           \
            fail(__FILE__, __LINE__, "check failed: %s", #cond); \
        }                                                        \
    } while (0)

/* Checks that `call` returned `expected`, showing the last error if not. */
void check_status(R3vStatus got, R3vStatus expected, const char *call, const char *file, int line);
#define CHECK_STATUS(call, expected) check_status((call), (expected), #call, __FILE__, __LINE__)
#define CHECK_OK(call) CHECK_STATUS(call, R3V_OK)

/* Checks that the last error contains `needle`. */
void check_error_contains(const char *needle, const char *file, int line);
#define CHECK_ERROR_CONTAINS(needle) check_error_contains((needle), __FILE__, __LINE__)

/* Checks that `got` is within 1e-4 of `expected`. */
void check_close(float got, float expected, const char *what, const char *file, int line);
#define CHECK_CLOSE(got, expected) check_close((got), (expected), #got, __FILE__, __LINE__)
#define CHECK_VEC3(v, ex, ey, ez) \
    do {                          \
        CHECK_CLOSE((v).x, (ex)); \
        CHECK_CLOSE((v).y, (ey)); \
        CHECK_CLOSE((v).z, (ez)); \
    } while (0)

/* ------------------------------------------------------------- helpers */

R3vVec3 vec3(float x, float y, float z);

/* `name` inside a directory private to this test run. The returned string
 * is valid until the next call. */
const char *temp_path(const char *name);
void write_file(const char *path, const char *contents);

/* Creates a renderer configured to width x height. Fails the test on
 * failure. */
R3vWgpuRenderer *make_renderer(uint32_t width, uint32_t height);

/* A configured tesselator holding one full 4x4x2 block at voxel (x, y, z),
 * with the delta it produced passed to `renderer`. */
R3vCpuTesselator *add_block(R3vWgpuRenderer *renderer, float voxel_size, int32_t x, int32_t y,
                            int32_t z);

/* Renders twice, so `frame` (width * height BGRA pixels) shows the scene
 * as it is now: frames are read back one call late. */
void render_now(R3vWgpuRenderer *renderer, R3vOrbitCamera camera, uint32_t width,
                uint32_t height, uint8_t *frame);

/* Whether the pixel at (x, y) is not black (alpha ignored). */
bool lit(const uint8_t *frame, uint32_t width, uint32_t x, uint32_t y);

#endif /* RENDERER_C_TEST_HARNESS_H */
