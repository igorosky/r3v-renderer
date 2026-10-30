#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

namespace r3v {
namespace renderer {

/// Result of a function that can fail. Anything but `R3V_OK` comes with a
/// message from `r3v_renderer_last_error`.
enum class R3vStatus {
  R3V_OK = 0,
  /// A NULL or otherwise invalid argument. Nothing was changed.
  R3V_INVALID_ARGUMENT,
  /// The object has to be configured first.
  R3V_NOT_CONFIGURED,
  /// An index or instance id past the end.
  R3V_OUT_OF_RANGE,
  /// Nothing with the given name.
  R3V_NOT_FOUND,
  /// The operation failed for another reason, e.g. reading a file.
  R3V_FAILED,
  /// A bug or an unrecoverable GPU error inside the library. The object the
  /// function was called on should be dropped.
  R3V_PANIC,
};

/// Which kind of `urdf::UrdfGeometry` an `R3vUrdfGeometry` is.
enum class R3vUrdfGeometryKind {
  R3V_URDF_GEOMETRY_MESH,
  R3V_URDF_GEOMETRY_BOX,
  R3V_URDF_GEOMETRY_SPHERE,
  R3V_URDF_GEOMETRY_CYLINDER,
  R3V_URDF_GEOMETRY_CAPSULE,
};

/// Turns blocks of voxels into triangles (`tesselator::CpuTesselator`).
struct R3vCpuTesselator;

/// Triangles of every chunk that changed (`Vec<TriangleChunk>`). Owned by the
/// caller until it is passed to `r3v_wgpu_renderer_update_geometry` or
/// dropped.
struct R3vGeometryDelta;

/// Transforms of every link and visual for one set of joint positions
/// (`robot::RobotTransforms`).
struct R3vRobotTransforms;

/// Triangle mesh for `r3v_wgpu_renderer_upload_asset` (`stl::StlMesh`).
struct R3vStlMesh;

/// Robot description read from URDF (`urdf::UrdfRobot`). Every string it
/// hands out stays valid until it is dropped.
struct R3vUrdfDescription;

/// Robot computing link transforms from joint positions
/// (`robot::UrdfRobot`).
struct R3vUrdfRobot;

/// Renders voxel geometry and mesh instances on the GPU
/// (`renderer::WgpuAccelerated`).
struct R3vWgpuRenderer;

/// 3D vector, also used for points.
struct R3vVec3 {
  float x;
  float y;
  float z;
};

/// Camera circling `target` (`camera::OrbitCamera`). Change it through the
/// `r3v_orbit_camera_*` functions or by writing the fields directly.
struct R3vOrbitCamera {
  R3vVec3 target;
  float distance;
  /// Radians.
  float yaw;
  /// Radians.
  float pitch;
  /// Near clip plane distance.
  float near;
  /// Far clip plane distance.
  float far;
  /// Set by every change of the camera, never cleared by the library.
  bool new_uniform_available;
};

/// What the renderer needs to know about a camera (`camera::CameraState`).
struct R3vCameraState {
  /// Projection times view, column-major: `view_proj[column][row]`.
  float view_proj[4][4];
  /// Eye position.
  R3vVec3 position;
  /// Unit view direction.
  R3vVec3 forward;
};

/// `renderer::WgpuAcceleratedConfiguration`.
struct R3vWgpuRendererConfiguration {
  /// Frame width in pixels.
  uint32_t width;
  /// Frame height in pixels.
  uint32_t height;
  /// Opaque background color, sRGB: `x`, `y`, `z` are red, green and blue
  /// between 0 and 1.
  R3vVec3 background_color;
  /// Distance along the camera's forward axis over which the voxel colormap
  /// runs once. Has to be positive.
  float color_depth_range;
};

/// Rotation quaternion.
struct R3vQuat {
  float x;
  float y;
  float z;
  float w;
};

/// Rigid transform (`geometry::RealTwist`): rotation by `rotational`, then
/// translation by `linear`.
struct R3vTwist {
  R3vVec3 linear;
  R3vQuat rotational;
};

/// Link of a robot description (`urdf::UrdfLink`).
struct R3vUrdfLink {
  const char *name;
  /// Read the visuals with `r3v_urdf_description_visual`.
  size_t visual_count;
};

/// Position and orientation (`urdf::UrdfOrigin`).
struct R3vUrdfOrigin {
  R3vVec3 xyz;
  /// Roll, pitch and yaw in radians.
  R3vVec3 rpy;
};

/// Shape of a visual (`urdf::UrdfGeometry`). Only the fields of its `kind`
/// are set, the others are zero.
struct R3vUrdfGeometry {
  R3vUrdfGeometryKind kind;
  /// Mesh: file name, resolved against the URDF file's directory and
  /// `package://` ancestors when read with `r3v_urdf_read_file`.
  const char *mesh_filename;
  /// Mesh: whether `mesh_scale` was given.
  bool has_mesh_scale;
  R3vVec3 mesh_scale;
  /// Box: edge lengths.
  R3vVec3 box_size;
  /// Sphere, cylinder and capsule.
  float radius;
  /// Cylinder and capsule.
  float length;
};

/// Material (`urdf::UrdfMaterial`).
struct R3vUrdfMaterial {
  /// NULL when the material has no name.
  const char *name;
  /// Whether `rgba` was given.
  bool has_color;
  float rgba[4];
};

/// Visual of a link (`urdf::UrdfVisual`). The `has_*` flags tell which
/// optional parts are there.
struct R3vUrdfVisual {
  /// NULL when the visual has no name.
  const char *name;
  bool has_origin;
  R3vUrdfOrigin origin;
  bool has_geometry;
  R3vUrdfGeometry geometry;
  bool has_material;
  R3vUrdfMaterial material;
};

/// Limits of a joint (`urdf::UrdfJointLimit`).
struct R3vUrdfJointLimit {
  float lower;
  float upper;
  float effort;
  float velocity;
  float acceleration;
  float deceleration;
  float jerk;
};

/// Joint of a robot description (`urdf::UrdfJoint`). The `has_*` flags tell
/// which optional parts are there.
struct R3vUrdfJoint {
  const char *name;
  /// "revolute", "continuous", "prismatic", "fixed", "floating", "planar"
  /// or "spherical".
  const char *joint_type;
  /// Parent link name.
  const char *parent;
  /// Child link name.
  const char *child;
  bool has_origin;
  R3vUrdfOrigin origin;
  bool has_axis;
  R3vVec3 axis;
  bool has_limit;
  R3vUrdfJointLimit limit;
};

/// Triangle (`geometry::RealTriangle`), counter-clockwise when seen from
/// its front side.
struct R3vTriangle {
  float v0[3];
  float v1[3];
  float v2[3];
};

/// `tesselator::CpuTesselatorConfiguration`.
struct R3vCpuTesselatorConfiguration {
  /// Edge length of a voxel in world units.
  float voxel_size;
  /// Smooths the surface with marching cubes instead of meshing the voxel
  /// faces.
  bool marching_cubes_enabled;
};

/// One 4x4x2 block of voxels (`geometry::PointI`): `x`, `y`, `z` are the
/// voxel origin of the block, `i` its occupancy mask with bit
/// `x + 4 * y + 16 * z` for the voxel at (`x`, `y`, `z`) inside the block.
/// Same layout as `PointI` of r3v-voxelizer, whose compress mode output can
/// be passed on as it is.
struct R3vPointI {
  int32_t x;
  int32_t y;
  int32_t z;
  uint32_t i;
};

/// Integer point, e.g. the origin of a chunk in voxels.
struct R3vPoint3I {
  int32_t x;
  int32_t y;
  int32_t z;
};

/// All triangles of one chunk of 16x16x16 voxels (`geometry::TriangleChunk`).
/// They replace what the chunk had before; no triangles means it is empty
/// now.
struct R3vTriangleChunk {
  /// Origin of the chunk in voxels.
  R3vPoint3I index;
  /// In world units. NULL when `triangle_count` is 0, otherwise valid until
  /// the delta is dropped or passed on.
  const R3vTriangle *triangles;
  size_t triangle_count;
};

extern "C" {

/// Message of the last failure on the calling thread, NUL terminated and
/// valid until the next failure on this thread. Empty before the first
/// failure, never NULL. Successful calls leave it as it is.
const char *r3v_renderer_last_error();

/// `OrbitCamera::new`: angles in degrees, clip planes at 0.1 and 100.
R3vOrbitCamera r3v_orbit_camera_new(R3vVec3 target, float distance, float yaw_deg, float pitch_deg);

/// `OrbitCamera::default`: looks at the origin from 5 units along +Z.
R3vOrbitCamera r3v_orbit_camera_default();

/// `OrbitCamera::with_clip_planes`.
R3vOrbitCamera r3v_orbit_camera_with_clip_planes(R3vOrbitCamera camera, float near, float far);

/// `OrbitCamera::forward`: unit vector from the eye towards `target`.
R3vVec3 r3v_orbit_camera_forward(R3vOrbitCamera camera);

/// `Camera::look_at`: angles in degrees. The distance is kept at 0.1 or more
/// and the pitch short of the poles.
R3vStatus r3v_orbit_camera_look_at(R3vOrbitCamera *camera,
                                   R3vVec3 target,
                                   float distance,
                                   float yaw_deg,
                                   float pitch_deg);

/// `Camera::rotate`: deltas in radians, unlike the degrees of `look_at`. The
/// pitch is kept short of the poles.
R3vStatus r3v_orbit_camera_rotate(R3vOrbitCamera *camera, float delta_yaw, float delta_pitch);

/// `Camera::zoom`: changes the distance, which is kept at 0.1 or more.
R3vStatus r3v_orbit_camera_zoom(R3vOrbitCamera *camera, float delta_distance);

/// `Camera::pan`: moves `target` along the screen's right and up axes.
R3vStatus r3v_orbit_camera_pan(R3vOrbitCamera *camera, float delta_x, float delta_y);

/// `Camera::get_state` for a view of `aspect_ratio` (width / height). Fails
/// with `R3V_INVALID_ARGUMENT` unless `aspect_ratio` > 0, `near` > 0 and
/// `far` > `near` (all finite).
R3vStatus r3v_orbit_camera_get_state(R3vOrbitCamera camera,
                                     float aspect_ratio,
                                     R3vCameraState *out);

/// `WgpuAccelerated::new`: blocks until a device of the default GPU adapter
/// is ready. NULL when there is no adapter or no device. Configure it before
/// rendering.
R3vWgpuRenderer *r3v_wgpu_renderer_create();

/// `Renderer::configure`: sets the frame size, the background color and the
/// colormap depth range. Both sides have to be between 1 and 8192.
R3vStatus r3v_wgpu_renderer_configure(R3vWgpuRenderer *renderer,
                                      R3vWgpuRendererConfiguration configuration);

/// `Renderer::update_geometry`: replaces the triangles of every chunk in
/// `delta`. Takes ownership of `delta`, which is freed on failure too.
R3vStatus r3v_wgpu_renderer_update_geometry(R3vWgpuRenderer *renderer, R3vGeometryDelta *delta);

/// `Renderer::upload_asset`: uploads a copy of `mesh` with `instances`
/// instances and writes their ids to `out_ids`, which has room for
/// `instances` ids (NULL when `instances` is 0). Instances start visible,
/// with the identity transform. Ids count up from 0 over all uploads.
R3vStatus r3v_wgpu_renderer_upload_asset(R3vWgpuRenderer *renderer,
                                         const R3vStlMesh *mesh,
                                         size_t instances,
                                         uint64_t *out_ids);

/// `Renderer::set_asset_transform`. `R3V_OUT_OF_RANGE` for an id that was
/// never returned by `r3v_wgpu_renderer_upload_asset`.
R3vStatus r3v_wgpu_renderer_set_asset_transform(R3vWgpuRenderer *renderer,
                                                uint64_t instance_id,
                                                R3vTwist transform);

/// `Renderer::set_asset_visibility`. `R3V_OUT_OF_RANGE` for an id that was
/// never returned by `r3v_wgpu_renderer_upload_asset`.
R3vStatus r3v_wgpu_renderer_set_asset_visibility(R3vWgpuRenderer *renderer,
                                                 uint64_t instance_id,
                                                 bool visible);

R3vStatus r3v_wgpu_renderer_set_asset_color(R3vWgpuRenderer *renderer,
                                            uint64_t instance_id,
                                            R3vVec3 color);

/// `Renderer::render`: renders a frame seen by `camera_state` and writes the
/// previous one to `out` (frames are read back one call late). After
/// `r3v_wgpu_renderer_configure` changed the frame size, and on the first
/// call, `out` is filled with zeros instead.
///
/// Pixels are BGRA, 8 bit sRGB, rows top to bottom without padding: `out`
/// needs `width * height * 4` bytes, `out_len` is its size. Blocks until the
/// GPU is done.
R3vStatus r3v_wgpu_renderer_render(R3vWgpuRenderer *renderer,
                                   const R3vCameraState *camera_state,
                                   uint8_t *out,
                                   size_t out_len);

/// Frees a renderer. NULL is ignored.
void r3v_wgpu_renderer_drop(R3vWgpuRenderer *renderer);

/// `urdf::read_file`: reads a URDF file. Relative mesh file names are
/// resolved against its directory. NULL on failure.
R3vUrdfDescription *r3v_urdf_read_file(const char *path);

/// `urdf::read_str`: reads URDF from a string. Mesh file names are kept as
/// they are. NULL on failure.
R3vUrdfDescription *r3v_urdf_read_str(const char *xml);

/// Name of the robot, NULL for NULL.
const char *r3v_urdf_description_name(const R3vUrdfDescription *description);

/// Number of links, 0 for NULL.
size_t r3v_urdf_description_link_count(const R3vUrdfDescription *description);

/// Link number `link`.
R3vStatus r3v_urdf_description_link(const R3vUrdfDescription *description,
                                    size_t link,
                                    R3vUrdfLink *out);

/// Visual number `visual` of link number `link`.
R3vStatus r3v_urdf_description_visual(const R3vUrdfDescription *description,
                                      size_t link,
                                      size_t visual,
                                      R3vUrdfVisual *out);

/// Number of joints, 0 for NULL.
size_t r3v_urdf_description_joint_count(const R3vUrdfDescription *description);

/// Joint number `joint`.
R3vStatus r3v_urdf_description_joint(const R3vUrdfDescription *description,
                                     size_t joint,
                                     R3vUrdfJoint *out);

/// Number of top level materials, 0 for NULL.
size_t r3v_urdf_description_material_count(const R3vUrdfDescription *description);

/// Top level material number `material`.
R3vStatus r3v_urdf_description_material(const R3vUrdfDescription *description,
                                        size_t material,
                                        R3vUrdfMaterial *out);

/// Frees a description. NULL is ignored.
void r3v_urdf_description_drop(R3vUrdfDescription *description);

/// `Robot::from_description` with a copy of `description`. NULL on failure.
R3vUrdfRobot *r3v_urdf_robot_from_description(const R3vUrdfDescription *description);

/// `Robot::feed_sensor_input`: transforms for the joints named in
/// `joint_names` at `joint_values` (radians or meters), `count` of each.
/// Joints left out are at 0. Drop the result with
/// `r3v_robot_transforms_drop`. NULL on failure.
R3vRobotTransforms *r3v_urdf_robot_feed_sensor_input(R3vUrdfRobot *robot,
                                                     const char *const *joint_names,
                                                     const float *joint_values,
                                                     size_t count);

/// `Robot::last_transform`: what the last `r3v_urdf_robot_feed_sensor_input`
/// returned, owned by the robot and valid until the next feed or until the
/// robot is dropped. Do not drop it. NULL before the first feed and for NULL.
const R3vRobotTransforms *r3v_urdf_robot_last_transform(const R3vUrdfRobot *robot);

/// Frees a robot. NULL is ignored.
void r3v_urdf_robot_drop(R3vUrdfRobot *robot);

/// World transform of the link named `link` (`RobotTransforms::links`).
R3vStatus r3v_robot_transforms_link(const R3vRobotTransforms *transforms,
                                    const char *link,
                                    R3vTwist *out);

/// World transform of visual number `visual_index` of the link named `link`
/// (`RobotTransforms::visuals`).
R3vStatus r3v_robot_transforms_visual(const R3vRobotTransforms *transforms,
                                      const char *link,
                                      size_t visual_index,
                                      R3vTwist *out);

/// `RobotTransforms::get_link_origin`: world position of the link named
/// `link`.
R3vStatus r3v_robot_transforms_get_link_origin(const R3vRobotTransforms *transforms,
                                               const char *link,
                                               R3vVec3 *out);

/// `RobotTransforms::transform_visual_point`: `point` given in the frame of
/// visual number `visual_index` of the link named `link`, in world space.
R3vStatus r3v_robot_transforms_transform_visual_point(const R3vRobotTransforms *transforms,
                                                      const char *link,
                                                      size_t visual_index,
                                                      R3vVec3 point,
                                                      R3vVec3 *out);

/// Frees transforms returned by `r3v_urdf_robot_feed_sensor_input`, never the
/// ones from `r3v_urdf_robot_last_transform`. NULL is ignored.
void r3v_robot_transforms_drop(R3vRobotTransforms *transforms);

/// `stl::read_file`: reads an ASCII or binary STL file and winds every
/// triangle to match the normal declared for it. NULL on failure.
R3vStlMesh *r3v_stl_read_file(const char *path);

/// A mesh of the `count` triangles at `triangles` (copied), which may be NULL
/// when `count` is 0. NULL on failure.
R3vStlMesh *r3v_stl_mesh_from_triangles(const R3vTriangle *triangles, size_t count);

/// Number of triangles of the mesh, 0 for NULL.
size_t r3v_stl_mesh_triangle_count(const R3vStlMesh *mesh);

/// The triangles of the mesh, writable in place (e.g. to scale it) and valid
/// until the mesh is dropped. NULL for NULL or an empty mesh.
R3vTriangle *r3v_stl_mesh_triangles(R3vStlMesh *mesh);

/// Frees a mesh. NULL is ignored.
void r3v_stl_mesh_drop(R3vStlMesh *mesh);

/// `CpuTesselator::new`, with marching cubes disabled. Configure it before
/// use.
R3vCpuTesselator *r3v_cpu_tesselator_create();

/// `Tesselator::configure`. `voxel_size` has to be positive.
R3vStatus r3v_cpu_tesselator_configure(R3vCpuTesselator *tesselator,
                                       R3vCpuTesselatorConfiguration configuration);

/// `CpuTesselator::marching_cubes_enabled`, false for NULL.
bool r3v_cpu_tesselator_marching_cubes_enabled(const R3vCpuTesselator *tesselator);

/// `CpuTesselator::set_marching_cubes_enabled`: a change re-tesselates every
/// chunk with voxels, and the chunks next to them, on the next
/// `r3v_cpu_tesselator_get_output_geometry_delta`.
R3vStatus r3v_cpu_tesselator_set_marching_cubes_enabled(R3vCpuTesselator *tesselator, bool enabled);

/// `Tesselator::update_input_geometry`: every block replaces the mask the
/// tesselator had for it. `blocks` may be NULL when `count` is 0.
///
/// `R3V_INVALID_ARGUMENT` when a block is not aligned to the 4x4x2 grid (`x`
/// and `y` multiples of 4, `z` of 2); none of the blocks is applied then.
R3vStatus r3v_cpu_tesselator_update_input_geometry(R3vCpuTesselator *tesselator,
                                                   const R3vPointI *blocks,
                                                   size_t count);

/// `Tesselator::get_output_geometry_delta`: the chunks changed since the last
/// call, or NULL in `*out` when nothing changed. With a `camera_state`, only
/// the chunks in its view; the others stay changed for a later call. NULL
/// `camera_state` returns every changed chunk.
R3vStatus r3v_cpu_tesselator_get_output_geometry_delta(R3vCpuTesselator *tesselator,
                                                       const R3vCameraState *camera_state,
                                                       R3vGeometryDelta **out);

/// Frees a tesselator. NULL is ignored.
void r3v_cpu_tesselator_drop(R3vCpuTesselator *tesselator);

/// Number of chunks in the delta, 0 for NULL.
size_t r3v_geometry_delta_chunk_count(const R3vGeometryDelta *delta);

/// Chunk number `chunk` of the delta.
R3vStatus r3v_geometry_delta_chunk(const R3vGeometryDelta *delta,
                                   size_t chunk,
                                   R3vTriangleChunk *out);

/// Frees a delta that was not passed to `r3v_wgpu_renderer_update_geometry`.
/// NULL is ignored.
void r3v_geometry_delta_drop(R3vGeometryDelta *delta);

}  // extern "C"

}  // namespace renderer
}  // namespace r3v
