//! Math types and traits used in the crate. Most of the math types are feature-dependent, so they will
//! be different for `2d`/`3d` and `f32`/`f64`.

#[cfg(feature = "f32")]
mod single;
#[cfg(feature = "f32")]
pub use single::*;

#[cfg(feature = "f64")]
mod double;
#[cfg(feature = "f64")]
pub use double::*;

use bevy::math::*;

/// Adjust the precision of the math construct to the precision chosen for compilation.
pub trait AdjustPrecision {
    /// A math construct type with the desired precision.
    type Adjusted;
    /// Adjusts the precision of [`self`] to [`Self::Adjusted`](#associatedtype.Adjusted).
    fn adjust_precision(&self) -> Self::Adjusted;
}

/// Adjust the precision down to `f32` regardless of compilation.
pub trait AsF32 {
    /// The `f32` version of a math construct.
    type F32;
    /// Returns the `f32` version of this type.
    fn as_f32(&self) -> Self::F32;
}

impl AsF32 for bevy::math::DVec3 {
    type F32 = Vec3;
    fn as_f32(&self) -> Self::F32 {
        self.as_vec3()
    }
}

impl AsF32 for bevy::math::Vec3 {
    type F32 = Self;
    fn as_f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for bevy::math::DVec2 {
    type F32 = Vec2;
    fn as_f32(&self) -> Self::F32 {
        self.as_vec2()
    }
}

impl AsF32 for bevy::math::Vec4 {
    type F32 = Self;
    fn as_f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for bevy::math::DQuat {
    type F32 = bevy::math::Quat;
    fn as_f32(&self) -> Self::F32 {
        bevy::math::DQuat::as_f32(*self)
    }
}

impl AsF32 for bevy::math::Quat {
    type F32 = Self;
    fn as_f32(&self) -> Self::F32 {
        *self
    }
}
