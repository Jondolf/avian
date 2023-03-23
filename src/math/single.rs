#![allow(unused)] // Clippy doesn't like when the `f32` feature is disabled

use bevy::math::*;

#[cfg(feature = "2d")]
pub type Vector = Vec2;

#[cfg(feature = "3d")]
pub type Vector = Vec3;

#[cfg(feature = "3d")]
pub type Matrix3 = Mat3;

pub type Scalar = f32;

pub const PI: Scalar = std::f32::consts::PI;

pub type Quaternion = Quat;

pub type Vector3 = Vec3;
