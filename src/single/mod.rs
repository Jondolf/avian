#[cfg(feature = "2d")]
pub(self) extern crate parry2d as parry;

#[cfg(feature = "3d")]
pub(self) extern crate parry3d as parry;

#[path = "../common/mod.rs"]
mod common;

use bevy::time::Time;
pub use common::*;

#[cfg(feature = "2d")]
pub type Vector = bevy::math::Vec2;

#[cfg(feature = "3d")]
pub type Vector = bevy::math::Vec3;

#[cfg(feature = "3d")]
pub type Matrix3 = bevy::math::Mat3;

pub type Scalar = f32;

pub const PI: Scalar = std::f32::consts::PI;

pub type Quaternion = bevy::math::Quat;

pub type Vector3 = bevy::math::Vec3;

impl DeltaSecondsScalarExt for Time {
    fn delta_seconds_scalar(&self) -> Scalar {
        self.delta_seconds()
    }
}
