#![allow(unused)] // Clippy doesn't like when the `f64` feature is disabled

use bevy::math::*;

#[cfg(feature = "2d")]
pub type Vector = DVec2;

#[cfg(feature = "3d")]
pub type Vector = DVec3;

#[cfg(feature = "3d")]
pub type Matrix3 = DMat3;

pub type Scalar = f64;

pub const PI: Scalar = std::f64::consts::PI;

pub type Quaternion = DQuat;

pub type Vector3 = DVec3;
