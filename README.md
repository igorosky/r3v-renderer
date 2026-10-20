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
