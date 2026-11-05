# Guide

[Back to README](../README.md)

- [The building blocks](#the-building-blocks)
- [Voxel input](#voxel-input)
- [Tesselator](#tesselator)
- [Renderer](#renderer)
- [Meshes](#meshes)
- [Robots](#robots)
- [Grids](#grids)
- [Camera](#camera)

## The building blocks

The crate has four traits in `src/lib.rs`, each with one implementation:

| Trait | Implementation | Job |
|---|---|---|
| `Tesselator` | `CpuTesselator` (`tesselator`) | Turns voxels into triangle chunks |
| `Renderer` | `WgpuAccelerated` (`renderer`) | Holds geometry, meshes and grids on the GPU and renders frames |
| `Robot` | `UrdfRobot` (`robot`) | Computes link poses of a URDF robot from joint values |
| `Camera` | `OrbitCamera` (`camera`) | A camera orbiting a target point |

A frame is made like this:

1. Voxels go into the tesselator (`update_input_geometry` or
   `replace_input_geometry`).
2. The tesselator gives triangle chunks for what changed
   (`get_output_geometry_delta`) and they go to the renderer
   (`update_geometry`).
3. Meshes are uploaded once (`upload_asset`) and moved every frame
   (`set_asset_transform`).
4. `render(camera_state, buffer)` draws the frame.

The world is Y up. Coordinates are in meters if you set the voxel size and
mesh sizes in meters.

## Voxel input

The tesselator takes voxels in two forms. Both match what
[r3v-voxelizer](https://github.com/igorosky/r3v-voxelizer) returns with
`compress` on.

**Blocks (`PointI`)**, for incremental updates (r3v-voxelizer `add_points`):

- one `PointI { x, y, z, i }` per 4x4x2 block of voxels,
- `x`, `y`, `z` is the block origin in voxels: `x` and `y` multiples of 4, `z` a multiple of 2,
- `i` is a 32 bit occupancy mask, bit `lx | ly << 2 | lz << 4` for voxel `(lx, ly, lz)` inside the block,
- a block replaces whatever mask was stored at its origin, so `i = 0` clears it and `u32::MAX` fills it.

**Groups (`VoxelGroup`)**, for the whole map at once (r3v-voxelizer
`get_all_points_compressed`):

- one group per cube of 64x64x64 voxels,
- `point` is the group key: voxel coordinates shifted right by 6 (`point.i` is ignored, keys outside `[-2^25, 2^25)` wrap),
- `values[k]` is the occupancy of the 4x4x4 page whose offset in the group, with the bits of x, y and z interleaved from the lowest one, is `k`,
- bit `x + 4 * y + 16 * z` of a value is set when voxel `(x, y, z)` of the page is occupied.

`VoxelGroup::blocks()` splits a group into `PointI` blocks.

## Tesselator

```rust
let mut tesselator = CpuTesselator::new();
tesselator.configure(CpuTesselatorConfiguration {
    voxel_size: 0.1,               // voxel edge length in world units
    marching_cubes_enabled: false, // true smooths the surface
});
```

- `update_input_geometry(&blocks)` applies a list of blocks.
- `replace_input_geometry(&groups)` drops every voxel the tesselator holds and
  keeps only the given groups. Only chunks that really changed are rebuilt.
- `get_output_geometry_delta(&context)` returns the triangles of the chunks
  that changed (16x16x16 voxels each), or `None` when nothing changed.
- `set_marching_cubes_enabled(bool)` switches smoothing at runtime and
  rebuilds everything.

The context can hold the camera frustum
(`CpuTesselatorContext { camera_frustum: Some(state.frustum()) }`). Then only
visible chunks are rebuilt and the rest wait for a later call. To rebuild
everything, pass `None` and drain the deltas:

```rust
let all = CpuTesselatorContext { camera_frustum: None };
while let Some(delta) = tesselator.get_output_geometry_delta(&all) {
    renderer.update_geometry(delta);
}
```

With the `multi-threaded` feature the tesselator uses rayon.

## Renderer

`WgpuAccelerated::new()` returns `None` when there is no GPU adapter
(`new_async()` is the async version). Configure it before the first frame:

```rust
renderer.configure(WgpuAcceleratedConfiguration {
    width: 1280,
    height: 720,
    background_color: [0.0, 0.0, 0.0],             // sRGB, 0..1
    color_depth_range: DEFAULT_COLOR_DEPTH_RANGE,  // 50 m
    terrain_coloring: TerrainColoring::default(),
});
```

`configure` can be called again at any time, for example to resize.

### Frames

- `render(&camera_state, &mut buffer)` writes `width * height * 4` bytes of
  BGRA, 8 bit sRGB, rows from the top, no padding.
- Frames are read back one call late. The first call, and the first call after
  a size change, gives zeros.

### Voxel colors

By default voxels are colored by their distance along the camera's forward
axis, with the turbo colormap running once over `color_depth_range`.

`TerrainColoring` switches to coloring by height with a hypsometric colormap
(blue below the neutral height, then green, yellow, brown and white above it):

| Field | Meaning |
|---|---|
| `height_coloring_enabled` | Color by height instead of distance |
| `relative_to_robot` | Neutral height is `robot_height` (true) or 0 (false) |
| `robot_height` | Height of the robot, used with `relative_to_robot` |
| `height_offset` | Added to the neutral height |
| `height_knee` | Distance from the neutral height after which colors change slower (logarithmic), default 0.5 |
| `height_gain` | How quickly colors run toward the ends of the colormap, default 0.35 |
| `height_bands` | 0 for a continuous colormap, otherwise that many flat color bands |

### Frustum culling

The renderer skips chunks outside the camera view. It is on by default,
`set_frustum_culling(false)` turns it off and `drawn_chunks()` returns
`(drawn, total)` of the last frame.

## Meshes

Meshes are drawn white unless they get a color or a texture.

```rust
let mesh = r3v_renderer::mesh::read_file(Path::new("part.stl"))?; // STL or OBJ
let ids = renderer.upload_asset(mesh, 2);                       // two instances
renderer.set_asset_transform(ids[0], twist);                    // RealTwist: position + quaternion
renderer.set_asset_color(ids[1], [1.0, 0.5, 0.0]);
renderer.set_asset_visibility(ids[1], false);
```

- `stl::read_file` / `read_bytes` and `obj::read_file` / `read_bytes` load a
  specific format.
- `shapes::tessellate(&geometry)` turns a URDF box, sphere, cylinder or capsule
  into a mesh.
- `upload_texture(width, height, rgba)` and `set_asset_texture(id, Some(texture))`
  put a texture on a mesh with UV coordinates (Rust only).

## Robots

```rust
let description = r3v_renderer::urdf::read_file(Path::new("robot.urdf"))?;
let mut robot = UrdfRobot::from_description(description);

let joints: HashMap<String, f32> = HashMap::from([("arm_joint".into(), 0.5)]);
let transforms = robot.feed_sensor_input(&joints);
// transforms.links[link] and transforms.visuals[(link, visual_index)] are RealTwists
```

- `urdf::read_file` resolves `package://` and `file://` mesh paths relative to
  the URDF directory. `urdf::read_str` parses a string.
- Missing joints count as 0, unknown names are ignored.
- The robot does not draw itself. Upload a mesh per visual (file or primitive)
  and set its transform from `transforms.visuals` every frame. The demo
  (`src/main.rs`, `RobotScene`) shows the full pattern, including the Z up to Y
  up rotation.

## Grids

Line grids in the rendered world: a lattice of cells, or a plane when one of
the sizes is 0, placed in the world or relative to a named frame (for example
a robot link).

```rust
let grid = renderer.create_grid(GridDescription {
    size: [10.0, 0.0, 10.0],      // a plane on the ground
    resolution: [1.0, 1.0, 1.0],  // 1 m cells
    color: [0.5, 0.5, 0.5],
    frame: Some("base_link".into()),
    pose: RealTwist::identity(),
});
renderer.set_frame_transform("base_link", base_link_pose); // keep it up to date
```

`GridDescription::default()` is a 10 x 10 m gray ground plane in the world.
A grid attached to a frame is drawn only while the renderer knows the frame's
pose. `update_grid`, `remove_grid` and `remove_frame` change or remove them.

## Camera

```rust
let mut camera = OrbitCamera::new(Point3::new(0.0, 0.0, 0.0), 10.0, 30.0, 25.0)
    .with_clip_planes(0.1, 1000.0);
camera.rotate(5.0, 0.0); // degrees
camera.zoom(-1.0);
camera.pan(0.5, 0.0);
let state = camera.get_state(width as f32 / height as f32);
```

`OrbitCamera::new(target, distance, yaw_deg, pitch_deg)` looks at `target`
from `distance`. Its fields are public, `look_at` sets all four at once.
`CameraState` holds the view projection matrix, the position and the forward
vector, and `state.frustum()` gives the frustum for culling.
