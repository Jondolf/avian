//! Math types and traits used by the crate.
//!
//! Most of the math types are feature-dependent, so they will be different for `2d`/`3d` and `f32`/`f64`.

pub mod matrix;
pub use matrix::*;

#[cfg(feature = "f32")]
mod single;
#[cfg(feature = "f32")]
pub use single::*;

#[cfg(feature = "f64")]
mod double;
#[cfg(feature = "f64")]
pub use double::*;

use bevy_math::{prelude::*, *};

/// The active dimension.
#[cfg(feature = "2d")]
pub const DIM: usize = 2;
/// The active dimension.
#[cfg(feature = "3d")]
pub const DIM: usize = 3;

/// The `f32` vector type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) use bevy_math::Vec2 as VectorF32;

/// The `f32` vector type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) use bevy_math::Vec3 as VectorF32;

/// The ray type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) type Ray = Ray2d;
/// The ray type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type Ray = Ray3d;

// Note: This is called `Dir` instead of `Direction` because Bevy has a conflicting `Direction` type.
/// The direction type chosen based on the dimension.
#[cfg(feature = "2d")]
pub(crate) type Dir = Dir2;
/// The direction type chosen based on the dimension.
#[cfg(feature = "3d")]
pub(crate) type Dir = Dir3;

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
    fn f32(&self) -> Self::F32;
}

impl AsF32 for DVec3 {
    type F32 = Vec3;
    fn f32(&self) -> Self::F32 {
        self.as_vec3()
    }
}

impl AsF32 for Vec3 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DVec2 {
    type F32 = Vec2;
    fn f32(&self) -> Self::F32 {
        self.as_vec2()
    }
}

impl AsF32 for Vec2 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for Vec4 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DQuat {
    type F32 = Quat;
    fn f32(&self) -> Self::F32 {
        self.as_quat()
    }
}

impl AsF32 for Quat {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DMat2 {
    type F32 = Mat2;
    fn f32(&self) -> Self::F32 {
        self.as_mat2()
    }
}

impl AsF32 for Mat2 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

impl AsF32 for DMat3 {
    type F32 = Mat3;
    fn f32(&self) -> Self::F32 {
        self.as_mat3()
    }
}

impl AsF32 for Mat3 {
    type F32 = Self;
    fn f32(&self) -> Self::F32 {
        *self
    }
}

#[cfg(feature = "2d")]
pub(crate) fn cross(a: Vector, b: Vector) -> Scalar {
    a.perp_dot(b)
}

#[cfg(feature = "3d")]
pub(crate) fn cross(a: Vector, b: Vector) -> Vector {
    a.cross(b)
}

/// An extension trait for computing reciprocals without division by zero.
pub trait RecipOrZero {
    /// Computes the reciprocal of `self` if `self` is not zero,
    /// and returns zero otherwise to avoid division by zero.
    fn recip_or_zero(self) -> Self;
}

impl RecipOrZero for f32 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        if self != 0.0 && self.is_finite() {
            self.recip()
        } else {
            0.0
        }
    }
}

impl RecipOrZero for f64 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        if self != 0.0 && self.is_finite() {
            self.recip()
        } else {
            0.0
        }
    }
}

impl RecipOrZero for Vec2 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        Self::new(self.x.recip_or_zero(), self.y.recip_or_zero())
    }
}

impl RecipOrZero for Vec3 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        Self::new(
            self.x.recip_or_zero(),
            self.y.recip_or_zero(),
            self.z.recip_or_zero(),
        )
    }
}

impl RecipOrZero for DVec2 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        Self::new(self.x.recip_or_zero(), self.y.recip_or_zero())
    }
}

impl RecipOrZero for DVec3 {
    #[inline]
    fn recip_or_zero(self) -> Self {
        Self::new(
            self.x.recip_or_zero(),
            self.y.recip_or_zero(),
            self.z.recip_or_zero(),
        )
    }
}

/// An extension trait for matrix types.
pub trait MatExt {
    /// Computes the inverse of `self` if `self` is not zero,
    /// and returns zero otherwise to avoid division by zero.
    fn inverse_or_zero(self) -> Self;
}

impl MatExt for Mat2 {
    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }
}

impl MatExt for DMat2 {
    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }
}

impl MatExt for Mat3 {
    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }
}

impl MatExt for DMat3 {
    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }
}

impl MatExt for SymmetricMatrix3 {
    #[inline]
    fn inverse_or_zero(self) -> Self {
        if self.determinant() == 0.0 {
            Self::ZERO
        } else {
            self.inverse()
        }
    }
}

#[expect(clippy::unnecessary_cast)]
#[cfg(all(feature = "2d", any(feature = "parry-f32", feature = "parry-f64")))]
pub(crate) fn na_iso_to_iso(isometry: &parry::math::Isometry<Scalar>) -> Isometry2d {
    Isometry2d::new(
        Vector::from(isometry.translation).f32(),
        Rot2::from_sin_cos(isometry.rotation.im as f32, isometry.rotation.re as f32),
    )
}

#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use crate::prelude::*;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use parry::math::Isometry;

#[cfg(all(
    feature = "2d",
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub(crate) fn make_isometry(
    position: impl Into<Position>,
    rotation: impl Into<Rotation>,
) -> Isometry<Scalar> {
    let position: Position = position.into();
    let rotation: Rotation = rotation.into();
    Isometry::<Scalar>::new(position.0.into(), rotation.into())
}

#[cfg(all(
    feature = "3d",
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub(crate) fn make_isometry(
    position: impl Into<Position>,
    rotation: impl Into<Rotation>,
) -> Isometry<Scalar> {
    let position: Position = position.into();
    let rotation: Rotation = rotation.into();
    Isometry::<Scalar>::new(position.0.into(), rotation.to_scaled_axis().into())
}

/// Computes the skew-symmetric matrix corresponding to the given vector.
///
/// ```text
///                          [   0  -v.z  v.y ]
/// skew_symmetric_mat3(v) = [  v.z   0  -v.x ]
///                          [ -v.y  v.x   0  ]
/// ```
#[inline]
#[must_use]
#[cfg(feature = "3d")]
pub fn skew_symmetric_mat3(v: Vector3) -> Matrix3 {
    Matrix3::from_cols_array(&[0.0, v.z, -v.y, -v.z, 0.0, v.x, v.y, -v.x, 0.0])
}
