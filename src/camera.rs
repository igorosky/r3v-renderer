use crate::Camera;
use cgmath::prelude::*;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    position: [f32; 3],
    _pad0: f32,
    forward: [f32; 3],
    _pad1: f32,
}

impl From<&CameraState> for CameraUniform {
    fn from(state: &CameraState) -> Self {
        Self {
            view_proj: state.view_proj.into(),
            position: state.position.into(),
            _pad0: 0.0,
            forward: state.forward.into(),
            _pad1: 0.0,
        }
    }
}

impl Default for CameraUniform {
    fn default() -> Self {
        Self {
            view_proj: cgmath::Matrix4::identity().into(),
            position: [0.0; 3],
            _pad0: 0.0,
            forward: [0.0, 0.0, -1.0],
            _pad1: 0.0,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct CameraState {
    pub view_proj: cgmath::Matrix4<f32>,
    pub position: cgmath::Point3<f32>,
    pub forward: cgmath::Vector3<f32>,
}

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: cgmath::Matrix4<f32> = cgmath::Matrix4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 1.0,
);

const SAFE_PITCH: f32 = std::f32::consts::FRAC_PI_2 - 0.01;
const MIN_DISTANCE: f32 = 0.1;

struct CameraBasis {
    eye: cgmath::Point3<f32>,
    forward: cgmath::Vector3<f32>,
    right: cgmath::Vector3<f32>,
    up: cgmath::Vector3<f32>,
}

pub struct OrbitCamera {
    pub target: cgmath::Point3<f32>,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub near: f32,
    pub far: f32,
    pub new_uniform_available: bool,
}

impl OrbitCamera {
    pub fn new(target: cgmath::Point3<f32>, distance: f32, yaw_deg: f32, pitch_deg: f32) -> Self {
        Self {
            target,
            distance,
            yaw: yaw_deg.to_radians(),
            pitch: pitch_deg.to_radians(),
            near: 0.1,
            far: 100.0,
            new_uniform_available: true,
        }
    }

    fn calculate_basis(&self) -> CameraBasis {
        let eye_x = self.target.x + self.distance * self.yaw.sin() * self.pitch.cos();
        let eye_y = self.target.y + self.distance * self.pitch.sin();
        let eye_z = self.target.z + self.distance * self.yaw.cos() * self.pitch.cos();
        let eye = cgmath::Point3::new(eye_x, eye_y, eye_z);
        let forward = (self.target - eye).normalize();
        let world_up = cgmath::Vector3::unit_y();
        let right = forward.cross(world_up).normalize();
        let up = right.cross(forward).normalize();
        CameraBasis {
            eye,
            forward,
            right,
            up,
        }
    }

    pub fn forward(&self) -> cgmath::Vector3<f32> {
        self.calculate_basis().forward
    }

    pub fn with_clip_planes(mut self, near: f32, far: f32) -> Self {
        self.near = near;
        self.far = far;
        self
    }
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self::new(cgmath::Point3::new(0.0, 0.0, 0.0), 5.0, 0.0, 0.0)
    }
}

impl Camera for OrbitCamera {
    fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.yaw += delta_yaw;
        self.pitch += delta_pitch;
        self.pitch = self.pitch.clamp(-SAFE_PITCH, SAFE_PITCH);
        self.new_uniform_available = true;
    }

    fn zoom(&mut self, delta_distance: f32) {
        self.distance += delta_distance;
        self.distance = self.distance.max(MIN_DISTANCE);
        self.new_uniform_available = true;
    }

    fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let basis = self.calculate_basis();
        self.target += basis.right * delta_x;
        self.target += basis.up * delta_y;
        self.new_uniform_available = true;
    }

    fn get_state(&self, aspect_ratio: f32) -> CameraState {
        let basis = self.calculate_basis();
        let view = cgmath::Matrix4::look_at_rh(basis.eye, self.target, basis.up);
        let proj = OPENGL_TO_WGPU_MATRIX
            * cgmath::perspective(
                cgmath::Rad(std::f32::consts::FRAC_PI_4),
                aspect_ratio,
                self.near,
                self.far,
            );
        CameraState {
            view_proj: proj * view,
            position: basis.eye,
            forward: basis.forward,
        }
    }

    fn look_at(
        &mut self,
        target: crate::geometry::Point<f32>,
        distance: f32,
        yaw_deg: f32,
        pitch_deg: f32,
    ) {
        self.target = cgmath::Point3::new(target.x, target.y, target.z);
        self.distance = distance.max(MIN_DISTANCE);
        self.yaw = yaw_deg.to_radians();
        self.pitch = pitch_deg.to_radians().clamp(-SAFE_PITCH, SAFE_PITCH);
        self.new_uniform_available = true;
    }
}
