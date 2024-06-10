use super::AdjustPrecision;
use bevy_math::*;

/// The floating point number type used by Bevy XPBD.
pub type Scalar = f64;
/// The PI/2 constant.
pub const FRAC_PI_2: Scalar = std::f64::consts::FRAC_PI_2;
/// The PI constant.
pub const PI: Scalar = std::f64::consts::PI;
/// The TAU constant.
pub const TAU: Scalar = std::f64::consts::TAU;

/// The vector type used by Bevy XPBD.
#[cfg(feature = "2d")]
pub type Vector = DVec2;
/// The vector type used by Bevy XPBD.
#[cfg(feature = "3d")]
pub type Vector = DVec3;
/// The vector type used by Bevy XPBD. This is always a 2D vector regardless of the chosen dimension.
pub type Vector2 = DVec2;
/// The vector type used by Bevy XPBD. This is always a 3D vector regardless of the chosen dimension.
pub type Vector3 = DVec3;

/// The 3x3 matrix type used by Bevy XPBD.
pub type Matrix3 = DMat3;
/// The quaternion type used by Bevy XPBD.
pub type Quaternion = DQuat;

impl AdjustPrecision for f32 {
    type Adjusted = Scalar;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self as Scalar
    }
}

impl AdjustPrecision for f64 {
    type Adjusted = Scalar;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Vec3 {
    type Adjusted = Vector3;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dvec3()
    }
}

impl AdjustPrecision for DVec3 {
    type Adjusted = Vector3;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Vec2 {
    type Adjusted = Vector2;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dvec2()
    }
}

impl AdjustPrecision for DVec2 {
    type Adjusted = Vector2;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Quat {
    type Adjusted = Quaternion;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dquat()
    }
}

impl AdjustPrecision for DQuat {
    type Adjusted = Quaternion;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}

impl AdjustPrecision for Mat3 {
    type Adjusted = Matrix3;
    fn adjust_precision(&self) -> Self::Adjusted {
        self.as_dmat3()
    }
}

impl AdjustPrecision for DMat3 {
    type Adjusted = Matrix3;
    fn adjust_precision(&self) -> Self::Adjusted {
        *self
    }
}
