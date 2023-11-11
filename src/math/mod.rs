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

use glam::*;

/// Adjust the precision of the math construct to the precision chosen for compilation.
pub trait AdjustPrecision {
    /// A math construct type with the desired precision.
    type Adjusted;
    /// Adjusts the precision of [`self`] to [`Self::Adjusted`](#associatedtype.Adjusted).
    fn adjust_precision(&self) -> Self::Adjusted;
}

pub trait FromMath<T> {
    fn from_math(value: T) -> Self;
    fn into_math<K: FromMath<Self>>(&self) -> K
    where
        Self: Sized,
    {
        K::from_math(*self)
    }
}

impl FromMath<Vec2> for Vec2 {
    fn from_math(value: Vec2) -> Self {
        value
    }
}

impl FromMath<Vec3> for Vec3 {
    fn from_math(value: Vec3) -> Self {
        value
    }
}

impl FromMath<DVec2> for DVec2 {
    fn from_math(value: DVec2) -> Self {
        value
    }
}

impl FromMath<DVec3> for DVec3 {
    fn from_math(value: DVec3) -> Self {
        value
    }
}

impl FromMath<Vec2> for Vec3 {
    fn from_math(value: Vec2) -> Self {
        value.extend(0.0)
    }
}

impl FromMath<Vec3> for Vec2 {
    fn from_math(value: Vec3) -> Self {
        value.truncate()
    }
}

impl FromMath<DVec2> for DVec3 {
    fn from_math(value: DVec2) -> Self {
        value.extend(0.0)
    }
}

impl FromMath<DVec3> for DVec2 {
    fn from_math(value: DVec3) -> Self {
        value.truncate()
    }
}

impl FromMath<Vec2> for DVec3 {
    fn from_math(value: Vec2) -> Self {
        value.extend(0.0).as_dvec3()
    }
}

impl FromMath<Vec3> for DVec2 {
    fn from_math(value: Vec3) -> Self {
        value.truncate().as_dvec2()
    }
}

impl FromMath<DVec2> for Vec3 {
    fn from_math(value: DVec2) -> Self {
        value.extend(0.0).as_vec3()
    }
}

impl FromMath<DVec3> for Vec2 {
    fn from_math(value: DVec3) -> Self {
        value.truncate().as_vec2()
    }
}

impl FromMath<Quat> for Quat {
    fn from_math(value: Quat) -> Self {
        value
    }
}

impl FromMath<DQuat> for DQuat {
    fn from_math(value: DQuat) -> Self {
        value
    }
}

impl FromMath<Quat> for DQuat {
    fn from_math(value: Quat) -> Self {
        value.as_f64()
    }
}

impl FromMath<DQuat> for Quat {
    fn from_math(value: DQuat) -> Self {
        value.as_f32()
    }
}

/// Adjust the precision down to `f32` regardless of compilation.
pub trait AsF32 {
    /// The `f32` version of a math construct.
    type F32;
    /// Returns the `f32` version of this type.
    fn as_f32(&self) -> Self::F32;
}

impl AsF32 for DVec3 {
    type F32 = Vec3;
    fn as_f32(&self) -> Self::F32 {
        self.as_vec3()
    }
}

impl AsF32 for Vec3 {
    type F32 = Self;
    fn as_f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DVec2 {
    type F32 = Vec2;
    fn as_f32(&self) -> Self::F32 {
        self.as_vec2()
    }
}

impl AsF32 for Vec2 {
    type F32 = Self;
    fn as_f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for Vec4 {
    type F32 = Self;
    fn as_f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DQuat {
    type F32 = Quat;
    fn as_f32(&self) -> Self::F32 {
        DQuat::as_f32(*self)
    }
}

impl AsF32 for Quat {
    type F32 = Self;
    fn as_f32(&self) -> Self::F32 {
        *self
    }
}
