"""
Renders a staircase of voxels and a triangle mesh with the Python bindings
and writes the frame to a binary PPM image.

    cargo build --release --features python
    cp target/release/libr3v_renderer.so r3v_renderer.so   # .dylib on macOS
    python3 examples/python/voxels_to_ppm.py stairs.ppm

The directory holding r3v_renderer.so has to be on the module search path
(e.g. PYTHONPATH).
"""

import sys

import r3v_renderer as rr

WIDTH, HEIGHT = 320, 240
STEPS = 8


def add_stairs(renderer):
    """Feeds a staircase of full 4x4x2 blocks through the tesselator into the
    renderer: step i is i // 2 + 1 blocks high (Y is up) and 4 blocks deep.
    The blocks could just as well come from r3v_voxelizer in compress mode."""
    tesselator = rr.GreedyMeshTesselator()
    tesselator.configure(rr.GreedyMeshTesselatorConfiguration(voxel_size=0.25))
    tesselator.update_input_geometry(
        rr.PointI(4 * step, 4 * level, 2 * depth, 2**32 - 1)
        for step in range(STEPS)
        for level in range(step // 2 + 1)
        for depth in range(4)
    )
    delta = tesselator.get_output_geometry_delta()
    if delta is not None:
        renderer.update_geometry(delta)


def add_flags(renderer):
    """Two instances of one triangle above the stairs."""
    mesh = rr.StlMesh([rr.Triangle((0, 0, 0), (1, 0, 0), (0, 1, 0))])
    for k, instance in enumerate(renderer.upload_asset(mesh, 2)):
        renderer.set_asset_transform(instance, rr.Twist(linear=(1 + 4 * k, 3, 2.5)))


def write_ppm(path, bgra):
    rgb = bytearray(WIDTH * HEIGHT * 3)
    rgb[0::3] = bgra[2::4]
    rgb[1::3] = bgra[1::4]
    rgb[2::3] = bgra[0::4]
    with open(path, "wb") as file:
        file.write(b"P6\n%d %d\n255\n" % (WIDTH, HEIGHT))
        file.write(rgb)


def main(args):
    if len(args) != 1:
        print(f"usage: {sys.argv[0]} <output.ppm>", file=sys.stderr)
        return 2

    renderer = rr.WgpuRenderer()
    renderer.configure(rr.WgpuRendererConfiguration(WIDTH, HEIGHT))
    add_stairs(renderer)
    add_flags(renderer)

    camera = rr.OrbitCamera(target=(4, 1, 1), distance=12, yaw_deg=30, pitch_deg=25)
    state = camera.get_state(WIDTH / HEIGHT)

    # Frames come back one call late, the first call gives zeros.
    frame = bytearray(WIDTH * HEIGHT * 4)
    renderer.render(state, frame)
    renderer.render(state, frame)

    write_ppm(args[0], frame)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
