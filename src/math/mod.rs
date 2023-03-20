mod double;
mod single;

#[cfg(feature = "f32")]
pub use single::*;

#[cfg(feature = "f64")]
pub use double::*;

use bevy::math::*;

pub trait AsVec3F32 {
    fn as_vec3_f32(&self) -> Vec3;
}

impl AsVec3F32 for bevy::math::DVec3 {
    fn as_vec3_f32(&self) -> Vec3 {
        self.as_vec3()
    }
}

impl AsVec3F32 for bevy::math::Vec3 {
    fn as_vec3_f32(&self) -> Vec3 {
        *self
    }
}

pub trait AsQuatF32Ext {
    fn as_quat_f32(&self) -> Quat;
}

impl AsQuatF32Ext for bevy::math::Quat {
    fn as_quat_f32(&self) -> Quat {
        *self
    }
}

impl AsQuatF32Ext for bevy::math::DQuat {
    fn as_quat_f32(&self) -> Quat {
        self.as_f32()
    }
}
