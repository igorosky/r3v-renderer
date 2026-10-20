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

## JavaScript

Build the module for the web with wasm-pack and the `js` feature, which can not be combined with `c`, `cpp`, `python` or `multi-threaded`:

```bash
wasm-pack build --release --target web --features js
```

wasm-pack writes the module `r3v_renderer`, with TypeScript types, to `pkg`. Rendering needs WebGPU.

The classes and methods have the same names as in Python, and the TypeScript types document every method. JavaScript differs in these points:

- **Async.** `WgpuRenderer.create()` takes the place of the constructor and returns a Promise; `render` returns one too. Until it settles the renderer is busy: other calls throw and another `render` rejects.
- **Data in.** Arguments are read by property name, so the classes and plain objects work alike. For example, `update_input_geometry` takes an array of anything with `x`, `y`, `z` and `i`, `set_asset_transform` takes `{linear, rotational}` and `configure` takes `{width, height}`. Vectors are arrays of numbers.
- **Data out.** Quads (`GeometryDelta.chunk(i)`), URDF descriptions (`links`, `joints`, `materials`) and robot transforms are plain objects and arrays, made anew on every read. `structuredClone`, `postMessage` and `JSON.stringify` keep them, unlike objects of the classes, whose data stays in wasm memory. `RobotTransforms.visuals` is keyed by link name, then by visual index.
- **Errors.** Errors are exceptions, or rejections for `create`, `render` and `render_rgba`:
  - `TypeError` for an argument of the wrong type.
  - `RangeError` for an invalid value or an unknown instance id; nothing is changed when one is thrown.
  - `Error` when a component is used before `configure` or while it renders, or when parsing fails or there is no WebGPU.
- **Numbers.** Instance ids, counts and the coordinates of blocks have to be integers, otherwise a `RangeError` is thrown. Integer arguments of constructors and integer properties of the classes are converted the way wasm-bindgen converts every integer, as in r3v-voxelizer: `-1` passed as an unsigned value becomes `4294967295` and `1.9` becomes `1`.
- **Optional values.** Methods return `undefined` when there is nothing, for example `get_output_geometry_delta()` when nothing changed. Plain data has `null` for a missing part, such as a URDF visual without a material.
- **Files.** There is no file system, so there is no `read_file`. `StlMesh.read_bytes` takes the contents of an STL file and `UrdfDescription.read_str` those of a URDF file.
- **Memory.** Objects of the classes are freed when they are garbage collected, or right away with `free()` (or `using`).
- **Frames.** Frames follow the same rules as in C (see Conventions above). `render_rgba` gives RGBA instead of BGRA, as `ImageData` takes it.

`GreedyMeshTesselator.update_input_geometry` takes the output of r3v-voxelizer's `add_points` in compress mode as it is:

```js
import init, * as rr from "./pkg/r3v_renderer.js";
await init();

const renderer = await rr.WgpuRenderer.create();
renderer.configure({ width, height });

tesselator.update_input_geometry(voxelizer.add_points(sensor, points));
const delta = tesselator.get_output_geometry_delta();
if (delta !== undefined) {
    renderer.update_geometry(delta);
}

const frame = new Uint8ClampedArray(width * height * 4);
await renderer.render_rgba(camera.get_state(width / height), frame);
context.putImageData(new ImageData(frame, width, height), 0, 0);
```

[examples/js/voxels_to_ppm.mjs](examples/js/voxels_to_ppm.mjs) is the JavaScript version of the C example, for Node.js. Node.js has no WebGPU; the example takes it from the `webgpu` package (Dawn), as its header comment shows.

`./test.sh js` runs the JavaScript test suite (`tests/js`) and the example in Node.js. They need wasm-pack, the `wasm32-unknown-unknown` target, Node.js, the `webgpu` package (`npm ci` in `tests/js`) and a GPU.
