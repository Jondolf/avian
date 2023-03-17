#[cfg(feature = "2d")]
pub extern crate parry2d_f64 as parry;

#[cfg(feature = "3d")]
pub extern crate parry3d_f64 as parry;

#[path = "../common/mod.rs"]
mod common;

pub use common::*;

#[cfg(feature = "2d")]
pub type Vector = bevy::math::DVec2;

#[cfg(feature = "3d")]
pub type Vector = bevy::math::DVec3;

#[cfg(feature = "3d")]
pub type Matrix3 = bevy::math::DMat3;

pub type Scalar = f64;

pub(self) const PI: Scalar = std::f64::consts::PI;

pub type Quaternion = bevy::math::DQuat;

pub type Vector3 = bevy::math::DVec3;

use bevy::prelude::*;

#[cfg(feature = "2d")]
impl From<Vec2> for components::Pos {
    fn from(value: Vec2) -> Self {
        // unfortunately, glam doesn't implement `From<Vec2> for DVec2`
        // see: https://github.com/bitshifter/glam-rs/issues/220
        value.as_dvec2().into()
    }
}

#[cfg(feature = "3d")]
impl From<Vec3> for components::Pos {
    fn from(value: Vec3) -> Self {
        // unfortunately, glam doesn't implement `From<Vec2> for DVec2`
        // see: https://github.com/bitshifter/glam-rs/issues/220
        value.as_dvec3().into()
    }
}

impl DeltaSecondsScalarExt for Time {
    fn delta_seconds_scalar(&self) -> Scalar {
        self.delta_seconds_f64()
    }
}
