# r3v-renderer

Renders voxel geometry (for example the output of r3v-voxelizer) and mesh assets such as URDF robots with wgpu.

## C/C++

Build the library with the `c` or `cpp` feature:

```bash
cargo build --release --features c     # For C
cargo build --release --features cpp   # For C++
```

The library (`libr3v_renderer.so`, `.dylib` or `.dll`) ends up in `target/release`. Its headers are in `bindings`: `r3v-renderer.h` for C and `r3v-renderer.hpp` for C++, with the same API in the `r3v::renderer` namespace. They document every function.

Every Rust component has a C counterpart with the same methods, named `r3v_<component>_<method>`:

| Rust | C |
|---|---|
| `camera::OrbitCamera` | `R3vOrbitCamera`, a plain struct |
| `tesselator::GreedyMeshTesselator` | `R3vGreedyMeshTesselator *`; its output is `R3vGeometryDelta *` |
| `renderer::WgpuAccelerated` | `R3vWgpuRenderer *` |
| `stl::StlMesh`, `stl::read_file` | `R3vStlMesh *`, `r3v_stl_read_file` |
| `urdf::UrdfRobot`, `urdf::read_file`, `urdf::read_str` | `R3vUrdfDescription *`, `r3v_urdf_read_file`, `r3v_urdf_read_str` |
| `robot::UrdfRobot`, `robot::RobotTransforms` | `R3vUrdfRobot *`, `R3vRobotTransforms *` |

### Conventions

- **Errors.** No call aborts the process; panics inside the library are caught as well.
  - A function that can fail returns an `R3vStatus`. A function that creates an object returns NULL on failure.
  - `r3v_renderer_last_error()` gives the reason for the last failure on the calling thread.
  - Invalid arguments (NULL pointers, unknown instance ids, a buffer that is too small, rendering before `configure`) are reported this way and change nothing.
- **Ownership.**
  - Every object created by the library has a `*_drop` function, and dropping NULL is fine.
  - `r3v_wgpu_renderer_update_geometry` takes ownership of the delta passed to it.
  - The library never allocates memory for the caller to free. Outputs such as instance ids and frames go into buffers the caller provides.
- **Threads.** An object may be moved to another thread, but must not be used from two threads at once.
- **Frames.** Frames are BGRA, 8 bit sRGB, with rows top to bottom and no padding.
  - `r3v_wgpu_renderer_render` returns the frame rendered by the previous call.
  - On the first call, and after the frame size changes, it returns zeros instead.

### Voxels from r3v-voxelizer

The tesselator takes blocks of 4x4x2 voxels in the format r3v-voxelizer outputs with `compress` on. `R3vPointI` has the same layout as its `PointI`, so its output can be passed on unchanged:

```c
VoxelsUpdate update = add_points(voxelizer, sensor, points, count);
r3v_greedy_mesh_tesselator_update_input_geometry(tesselator, (const R3vPointI *)update.data, update.len);
drop_update(update);

R3vGeometryDelta *delta = NULL;
r3v_greedy_mesh_tesselator_get_output_geometry_delta(tesselator, &delta);
if (delta != NULL) {
    r3v_wgpu_renderer_update_geometry(renderer, delta);
}
```

[examples/c/voxels_to_ppm.c](examples/c/voxels_to_ppm.c) renders voxels and a mesh to an image. Its header comment shows how to build and run it.

### Tests

`./test.sh c` runs the C test suite (`tests/c`), the example and a C++ test (`tests/cpp`). They need a C and a C++ compiler and a GPU; a software Vulkan driver such as lavapipe works.

## Python

Build the extension module with the `python` feature, which can not be combined with `c` or `cpp`:

```bash
cargo build --release --features python
```

Python imports the library from `target/release` as the module `r3v_renderer` once it is renamed: `libr3v_renderer.so` (or `.dylib` on macOS) becomes `r3v_renderer.so`, and its directory has to be on the module search path. There are no stubs for the module yet.

The classes have the names of the C types without `R3v`. For example, `OrbitCamera`, `GreedyMeshTesselator`, `WgpuRenderer`, `StlMesh`, `UrdfDescription` and `UrdfRobot` work like their Rust counterparts. Their docstrings describe every method.

- **Data.** Blocks of voxels, triangles and quads are lists of objects, as in r3v-voxelizer. Rendered frames are the exception: `WgpuRenderer.render` writes them to a writable buffer of bytes, such as a `bytearray` or a numpy `uint8` array.
- **Errors.** Errors are exceptions:
  - `ValueError` for invalid arguments; nothing is changed when one is raised.
  - `IndexError` for unknown instance ids.
  - `RuntimeError` when a component is used before `configure`, or when a file can not be read or there is no GPU.
- **Optional values.** Optional values are `None`. For example, `get_output_geometry_delta()` returns `None` when nothing changed, and a URDF visual without a material has `material` set to `None`.
- **Threads.** The GPU work and the tesselation run without the GIL, so other Python threads keep running meanwhile.
- **Frames.** Frames follow the same rules as in C (see Conventions above).

`GreedyMeshTesselator.update_input_geometry` takes the output of r3v-voxelizer's `add_points` in compress mode as it is. It accepts any iterable of objects with `x()`, `y()`, `z()` and `i()` methods:

```python
tesselator.update_input_geometry(voxelizer.add_points(sensor, points))
delta = tesselator.get_output_geometry_delta()
if delta is not None:
    renderer.update_geometry(delta)

frame = bytearray(width * height * 4)
renderer.render(camera.get_state(width / height), frame)
```

[examples/python/voxels_to_ppm.py](examples/python/voxels_to_ppm.py) is the Python version of the C example.

`./test.sh python` runs the Python test suite (`tests/python`) and the example. They need `python3` (or the interpreter given in `PYO3_PYTHON`) and a GPU.
