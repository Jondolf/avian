use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use bevy::reflect::Reflect;

use crate::{Matrix2, Matrix2x3, Scalar, SymmetricMatrix3, Vector2, Vector3};

/// The top right triangle of a symmetric 5x5 column-major matrix.
#[derive(Reflect, Clone, Copy, Debug, PartialEq)]
pub struct SymmetricMatrix5 {
    /// The upper left 3x3 block of the matrix.
    pub a: SymmetricMatrix3,
    /// The lower left 2x3 block of the matrix.
    pub b: Matrix2x3,
    /// The lower right 2x2 block of the matrix.
    pub d: Matrix2,
}

impl SymmetricMatrix5 {
    /// A symmetric 5x5 matrix with all elements set to `0.0`.
    pub const ZERO: Self = Self {
        a: SymmetricMatrix3::ZERO,
        b: Matrix2x3::ZERO,
        d: Matrix2::ZERO,
    };

    /// A symmetric 5x5 identity matrix, where all diagonal elements are `1.0`, and all off-diagonal elements are `0.0`.
    pub const IDENTITY: Self = Self {
        a: SymmetricMatrix3::IDENTITY,
        b: Matrix2x3::ZERO,
        d: Matrix2::IDENTITY,
    };

    /// All `NaN`s.
    pub const NAN: Self = Self {
        a: SymmetricMatrix3::NAN,
        b: Matrix2x3::NAN,
        d: Matrix2::NAN,
    };

    /// Returns `true` if, and only if, all elements are finite.
    /// If any element is either `NaN` or positive or negative infinity, this will return `false`.
    #[inline]
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.a.is_finite() && self.b.is_finite() && self.d.is_finite()
    }

    /// Returns `true` if any elements are `NaN`.
    #[inline]
    #[must_use]
    pub fn is_nan(&self) -> bool {
        self.a.is_nan() || self.b.is_nan() || self.d.is_nan()
    }

    /// Returns the inverse of `self`.
    ///
    /// If the matrix is not invertible the returned matrix will be invalid.
    #[inline]
    #[must_use]
    pub fn inverse(&self) -> Self {
        let inv_d = self.d.inverse();
        let bt_inv_d = self.b.transposed_mul_mat2(inv_d);
        let bt_inv_d_b = self.b.transposed_mul(bt_inv_d);
        let a = (self.a - bt_inv_d_b).inverse();

        let neg_bt = a.mul_by_transposed_mat2x3(bt_inv_d);
        let b = -neg_bt;

        let d = inv_d + bt_inv_d.mul_by_transposed(neg_bt);

        Self { a, b, d }
    }

    /// Computes `v * self`, where `v` is a 1x5 vector split into subvectors `v1` and `v2`.
    #[inline]
    pub fn transform_pair(&self, v1: &mut Vector3, v2: &mut Vector2) {
        let (a, b, d) = (self.a, self.b, self.d);
        v1.x = v1.x * a.m00 + v1.y * a.m01 + v1.z * a.m02 + v2.x * b.x_axis.x + v2.y * b.x_axis.y;
        v1.y = v1.x * a.m01 + v1.y * a.m11 + v1.z * a.m12 + v2.x * b.y_axis.x + v2.y * b.y_axis.y;
        v1.z = v1.x * a.m02 + v1.y * a.m12 + v1.z * a.m22 + v2.x * b.z_axis.x + v2.y * b.z_axis.y;

        v2.x = v1.dot(b.row(0)) + v2.x * d.x_axis.x + v2.y * d.y_axis.x;
        v2.y = v1.dot(b.row(1)) + v2.x * d.x_axis.y + v2.y * d.y_axis.y;
    }
}

impl Default for SymmetricMatrix5 {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Add<SymmetricMatrix5> for SymmetricMatrix5 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            a: self.a + rhs.a,
            b: self.b + rhs.b,
            d: self.d + rhs.d,
        }
    }
}

impl AddAssign<SymmetricMatrix5> for SymmetricMatrix5 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Sub<SymmetricMatrix5> for SymmetricMatrix5 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            a: self.a - rhs.a,
            b: self.b - rhs.b,
            d: self.d - rhs.d,
        }
    }
}

impl SubAssign<SymmetricMatrix5> for SymmetricMatrix5 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl Neg for SymmetricMatrix5 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self::Output {
        Self {
            a: -self.a,
            b: -self.b,
            d: -self.d,
        }
    }
}

impl Mul<SymmetricMatrix5> for Scalar {
    type Output = SymmetricMatrix5;
    #[inline]
    fn mul(self, rhs: SymmetricMatrix5) -> Self::Output {
        rhs * self
    }
}

impl Mul<Scalar> for SymmetricMatrix5 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Scalar) -> Self::Output {
        Self {
            a: self.a * rhs,
            b: self.b * rhs,
            d: self.d * rhs,
        }
    }
}

impl MulAssign<Scalar> for SymmetricMatrix5 {
    #[inline]
    fn mul_assign(&mut self, rhs: Scalar) {
        *self = *self * rhs;
    }
}

impl Div<SymmetricMatrix5> for Scalar {
    type Output = SymmetricMatrix5;
    #[inline]
    fn div(self, rhs: SymmetricMatrix5) -> Self::Output {
        rhs / self
    }
}

impl Div<Scalar> for SymmetricMatrix5 {
    type Output = Self;
    #[inline]
    fn div(self, rhs: Scalar) -> Self::Output {
        Self {
            a: self.a / rhs,
            b: self.b / rhs,
            d: self.d * rhs.recip(),
        }
    }
}

impl DivAssign<Scalar> for SymmetricMatrix5 {
    #[inline]
    fn div_assign(&mut self, rhs: Scalar) {
        *self = *self / rhs;
    }
}
