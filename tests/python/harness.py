"""
Minimal test harness for the Python bindings.

Every test case runs in its own interpreter process (see
tests/python_bindings.rs), so a failed check raises and ends the process.

usage: renderer_tests.py list | run <test>
"""

import contextlib
import math
import os
import sys

import r3v_renderer as rr

# ---------------------------------------------------------------- checks


def fail(message):
    raise AssertionError(message)


def check(cond, message="check failed"):
    if not cond:
        fail(message)


@contextlib.contextmanager
def raises(exc_type, contains=None):
    """Fails unless the block raises exc_type, with `contains` in its message
    when given. Other exceptions propagate."""
    try:
        yield
    except exc_type as error:
        if contains is not None and contains not in str(error):
            fail(f"{exc_type.__name__} {str(error)!r} does not contain {contains!r}")
        return
    fail(f"expected {exc_type.__name__}, nothing was raised")


def close(got, expected, eps=1e-4):
    """Checks that got is within eps of expected, element wise for tuples."""
    if isinstance(expected, (tuple, list)):
        check(len(got) == len(expected), f"{got} is not like {expected}")
        for g, e in zip(got, expected):
            close(g, e, eps)
    elif not math.isclose(got, expected, abs_tol=eps):
        fail(f"{got} is not {expected}")


# ------------------------------------------------------------- helpers


def temp_path(name):
    """name inside a directory private to this test run."""
    return os.path.join(os.environ["R3V_TEST_TMPDIR"], name)


def write_file(path, contents):
    with open(path, "w", encoding="utf-8") as file:
        file.write(contents)


def make_renderer(width, height):
    renderer = rr.WgpuRenderer()
    renderer.configure(rr.WgpuRendererConfiguration(width, height))
    return renderer


def make_tesselator(voxel_size):
    tesselator = rr.GreedyMeshTesselator()
    tesselator.configure(rr.GreedyMeshTesselatorConfiguration(voxel_size))
    return tesselator


def add_block(renderer, voxel_size, x, y, z):
    """A tesselator holding one full 4x4x2 block at voxel (x, y, z), with the
    delta it produced passed to renderer."""
    tesselator = make_tesselator(voxel_size)
    tesselator.update_input_geometry([rr.PointI(x, y, z, 2**32 - 1)])
    renderer.update_geometry(tesselator.get_output_geometry_delta())
    return tesselator


def render_now(renderer, camera, width, height):
    """Renders twice, so the returned frame shows the scene as it is now:
    frames are read back one call late."""
    state = camera.get_state(width / height)
    frame = bytearray(width * height * 4)
    renderer.render(state, frame)
    renderer.render(state, frame)
    return frame


def lit(frame, width, x, y):
    """Whether the pixel at (x, y) is not black (alpha ignored)."""
    offset = (y * width + x) * 4
    return any(frame[offset:offset + 3])


# ------------------------------------------------------------------ main


def main(test_cases):
    """test_cases: test functions, run by name."""
    by_name = {fn.__name__: fn for fn in test_cases}
    args = sys.argv[1:]
    if args == ["list"]:
        for name in by_name:
            print(name)
        return 0
    if len(args) == 2 and args[0] == "run":
        fn = by_name.get(args[1])
        if fn is None:
            print(f"unknown test {args[1]}", file=sys.stderr)
            return 2
        fn()
        return 0
    print(f"usage: {sys.argv[0]} list | run <test>", file=sys.stderr)
    return 2
