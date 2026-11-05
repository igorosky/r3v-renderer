# Demo

[Back to README](../README.md)

The `demo` feature builds an interactive window on top of the library. It is
a good way to see what the renderer does and to try the settings.

```bash
cargo run --release --features demo -- path/to/robot.urdf
```

## What it shows

- A generated voxel world of 512 x 64 x 512 voxels (0.2 m each) with hills,
  mountain ranges, lakes with waves, trees and moving objects (balls, snakes,
  windmills, gyroscopes, pistons).
- A URDF rover driving in a loop with animated joints. Only voxels within
  20 m of its "lidar" are sent to the tesselator, so the map is revealed as it
  drives.
- An overlay with FPS and frustum culling statistics.

## Robot

The only argument is the path of a URDF file. Mesh paths in it
(`package://<pkg>/...` or relative) are resolved against the directory of the
URDF file, so keep the meshes next to it (for example `meshes/*.STL`).

Without an argument the demo loads `assets/scorpio9.urdf.xacro`. The `assets/`
directory is not part of the repository; when the file is missing, the demo
prints `Running without a robot` and goes on without one.

## Controls

The demo prints this list on start too.

| Input | Action |
|---|---|
| Left drag | Orbit |
| Right or middle drag | Pan |
| Scroll | Zoom |
| W / A / S / D | Move horizontally |
| Q / E | Move down / up |
| Arrow keys | Orbit |
| Shift | Move faster |
| O | Auto orbit on / off |
| C | Marching cubes smoothing on / off |
| V | Frustum culling on / off |
| R | Reset camera |
| F | Follow the robot on / off |
| M | Add / remove the robot arm |
| B | Cycle background color |
| [ / ] | Shorten / lengthen the colormap depth range |
| H | Color the terrain by height on / off |
| G | Measure heights from the world's 0 or from the robot |
| J / K | Shorten / lengthen the height knee |
| U / I | Lower / raise the height gain |
| T | Cycle height color bands |
| L | Cycle grids: none, world lattice, around the robot |
| P | Save a snapshot (`output_frame_NN.png`) |
| Esc | Quit |
