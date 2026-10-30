use crate::{geometry::RealTriangle, stl::StlMesh, urdf::UrdfGeometry};

pub const ROUND_SEGMENTS: usize = 32;
pub const SPHERE_RINGS: usize = 16;

pub fn tessellate(geometry: &UrdfGeometry) -> Option<StlMesh> {
    let triangles = match *geometry {
        UrdfGeometry::Mesh(_) => return None,
        UrdfGeometry::Box { size } => cuboid([size.x, size.y, size.z]),
        UrdfGeometry::Sphere { radius } => capsule(radius, 0.0),
        UrdfGeometry::Cylinder { radius, length } => cylinder(radius, length),
        UrdfGeometry::Capsule { radius, length } => capsule(radius, length),
    };
    Some(StlMesh { triangles })
}

pub fn cuboid(size: [f32; 3]) -> Vec<RealTriangle> {
    let half = size.map(|edge| edge * 0.5);
    let mut triangles = Vec::with_capacity(12);
    for axis in 0..3 {
        let (u_axis, v_axis) = ((axis + 1) % 3, (axis + 2) % 3);
        for sign in [1.0, -1.0] {
            let mut center = [0.0; 3];
            center[axis] = sign * half[axis];
            let (u_axis, v_axis) = if sign > 0.0 {
                (u_axis, v_axis)
            } else {
                (v_axis, u_axis)
            };
            let corner = |u: f32, v: f32| {
                let mut corner = center;
                corner[u_axis] = u * half[u_axis];
                corner[v_axis] = v * half[v_axis];
                corner
            };
            let [a, b, c, d] = [
                corner(-1.0, -1.0),
                corner(1.0, -1.0),
                corner(1.0, 1.0),
                corner(-1.0, 1.0),
            ];
            triangles.push(RealTriangle {
                v0: a,
                v1: b,
                v2: c,
            });
            triangles.push(RealTriangle {
                v0: a,
                v1: c,
                v2: d,
            });
        }
    }
    triangles
}

pub fn cylinder(radius: f32, length: f32) -> Vec<RealTriangle> {
    let top = length * 0.5;
    lathe(&[(0.0, top), (radius, top), (radius, -top), (0.0, -top)])
}

pub fn capsule(radius: f32, length: f32) -> Vec<RealTriangle> {
    let half_length = length * 0.5;
    let mut profile = Vec::with_capacity(SPHERE_RINGS + 2);
    for ring in 0..=SPHERE_RINGS {
        let polar = std::f32::consts::PI * ring as f32 / SPHERE_RINGS as f32;
        let (mut sin, cos) = polar.sin_cos();
        if ring == 0 || ring == SPHERE_RINGS {
            sin = 0.0;
        }
        let point = |offset: f32| (radius * sin, radius * cos + offset);
        if 2 * ring < SPHERE_RINGS {
            profile.push(point(half_length));
        } else if 2 * ring > SPHERE_RINGS {
            profile.push(point(-half_length));
        } else {
            profile.push(point(half_length));
            if length != 0.0 {
                profile.push(point(-half_length));
            }
        }
    }
    lathe(&profile)
}

fn lathe(profile: &[(f32, f32)]) -> Vec<RealTriangle> {
    let around = |(radius, z): (f32, f32), segment: usize| {
        let angle =
            std::f32::consts::TAU * (segment % ROUND_SEGMENTS) as f32 / ROUND_SEGMENTS as f32;
        let (sin, cos) = angle.sin_cos();
        if radius == 0.0 {
            return [0.0, 0.0, z];
        }
        [radius * cos, radius * sin, z]
    };
    let mut triangles = Vec::with_capacity(2 * ROUND_SEGMENTS * profile.len());
    for pair in profile.windows(2) {
        let (upper, lower) = (pair[0], pair[1]);
        for segment in 0..ROUND_SEGMENTS {
            let [a0, a1] = [segment, segment + 1].map(|segment| around(upper, segment));
            let [b0, b1] = [segment, segment + 1].map(|segment| around(lower, segment));
            if lower.0 != 0.0 {
                triangles.push(RealTriangle {
                    v0: b0,
                    v1: b1,
                    v2: a1,
                });
            }
            if upper.0 != 0.0 {
                triangles.push(RealTriangle {
                    v0: b0,
                    v1: a1,
                    v2: a0,
                });
            }
        }
    }
    triangles
}
