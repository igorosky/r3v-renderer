use crate::Camera;
use cgmath::prelude::*;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    view_proj: [[f32; 4]; 4],
}

impl CameraUniform {
    pub fn from_matrix(matrix: cgmath::Matrix4<f32>) -> Self {
        Self {
            view_proj: matrix.into(),
        }
    }
}

impl Default for CameraUniform {
    fn default() -> Self {
        Self {
            view_proj: cgmath::Matrix4::identity().into(),
        }
    }
}

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: cgmath::Matrix4<f32> = cgmath::Matrix4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.5,
    0.0, 0.0, 0.0, 1.0,
);

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

    fn calculate_vectors(&self) -> (cgmath::Point3<f32>, cgmath::Point3<f32>, cgmath::Vector3<f32>, cgmath::Vector3<f32>) {
        let eye_x = self.target.x + self.distance * self.yaw.sin() * self.pitch.cos();
        let eye_y = self.target.y + self.distance * self.pitch.sin();
        let eye_z = self.target.z + self.distance * self.yaw.cos() * self.pitch.cos();
        let eye = cgmath::Point3::new(eye_x, eye_y, eye_z);
        let forward = (self.target - eye).normalize();
        let world_up = cgmath::Vector3::unit_y();
        let right = forward.cross(world_up).normalize();
        let up = right.cross(forward).normalize();
        (eye, self.target, right, up)
    }

    pub fn with_clip_planes(mut self, near: f32, far: f32) -> Self {
        self.near = near;
        self.far = far;
        self
    }

    fn view_matrix(&self) -> cgmath::Matrix4<f32> {
        let (eye, target, _, up) = self.calculate_vectors();
        cgmath::Matrix4::look_at_rh(eye, target, up)
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

        const SAFE_PITCH: f32 = std::f32::consts::FRAC_PI_2 - 0.01;
        self.pitch = self.pitch.clamp(-SAFE_PITCH, SAFE_PITCH);
        self.new_uniform_available = true;
    }

    fn zoom(&mut self, delta_distance: f32) {
        self.distance += delta_distance;
        self.distance = self.distance.max(0.1); 
        self.new_uniform_available = true;
    }

    fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let (_, _, right, up) = self.calculate_vectors();
        self.target += right * delta_x;
        self.target += up * delta_y;
        self.new_uniform_available = true;
    }
    
    fn get_uniform(&self, aspect_ratio: f32) -> CameraUniform {
        let view = self.view_matrix();
        let proj = OPENGL_TO_WGPU_MATRIX * cgmath::perspective(cgmath::Rad(std::f32::consts::FRAC_PI_4), aspect_ratio, self.near, self.far);
        let view_proj = proj * view;
        CameraUniform::from_matrix(view_proj)
    }
    
    fn look_at(&mut self, target: crate::geometry::Point<f32>, distance: f32, yaw_deg: f32, pitch_deg: f32) {
        self.target = cgmath::Point3::new(target.x, target.y, target.z);
        self.distance = distance;
        self.yaw = yaw_deg.to_radians();
        self.pitch = pitch_deg.to_radians();
        self.new_uniform_available = true;   
    }
}
