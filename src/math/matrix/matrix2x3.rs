use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use bevy::reflect::Reflect;

use crate::{Matrix2, Scalar, SymmetricMatrix3, Vector2, Vector3};

/// The top right triangle of a symmetric 2x3 column-major matrix.
#[derive(Reflect, Clone, Copy, Debug, Default, PartialEq)]
pub struct Matrix2x3 {
    /// The first column of the matrix.
    pub x_axis: Vector2,
    /// The second column of the matrix.
    pub y_axis: Vector2,
    /// The third column of the matrix.
    pub z_axis: Vector2,
}

impl Matrix2x3 {
    /// A symmetric 2x3 matrix with all elements set to `0.0`.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /// All `NaN`s.
    pub const NAN: Self = Self::new(
        Scalar::NAN,
        Scalar::NAN,
        Scalar::NAN,
        Scalar::NAN,
        Scalar::NAN,
        Scalar::NAN,
    );

    /// Creates a new 2x3 matrix.
    ///
    /// The elements are in column-major order `mCR`, where `C` is the column index
    /// and `R` is the row index.
    #[inline(always)]
    #[must_use]
    pub const fn new(
        m00: Scalar,
        m01: Scalar,
        m10: Scalar,
        m11: Scalar,
        m20: Scalar,
        m21: Scalar,
    ) -> Self {
        Self {
            x_axis: Vector2::new(m00, m01),
            y_axis: Vector2::new(m10, m11),
            z_axis: Vector2::new(m20, m21),
        }
    }

    /// Creates a new 2x3 matrix from the given columns.
    #[inline(always)]
    #[must_use]
    pub const fn from_cols(x_axis: Vector2, y_axis: Vector2, z_axis: Vector2) -> Self {
        Self {
            x_axis,
            y_axis,
            z_axis,
        }
    }

    /// Creates a new 2x3 matrix from the given rows.
    #[inline(always)]
    #[must_use]
    pub const fn from_rows(row1: Vector3, row2: Vector3) -> Self {
        Self {
            x_axis: Vector2::new(row1.x, row2.x),
            y_axis: Vector2::new(row1.y, row2.y),
            z_axis: Vector2::new(row1.z, row2.z),
        }
    }

    /// Returns the matrix column for the given `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is greater than 2.
    #[inline]
    #[must_use]
    pub const fn col(&self, index: usize) -> Vector2 {
        match index {
            0 => self.x_axis,
            1 => self.y_axis,
            2 => self.z_axis,
            _ => panic!("index out of bounds"),
        }
    }

    /// Returns a mutable reference to the matrix column for the given `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is greater than 2.
    #[inline]
    #[must_use]
    pub fn col_mut(&mut self, index: usize) -> &mut Vector2 {
        match index {
            0 => &mut self.x_axis,
            1 => &mut self.y_axis,
            2 => &mut self.z_axis,
            _ => panic!("index out of bounds"),
        }
    }

    /// Returns the matrix row for the given `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is greater than 1.
    #[inline]
    #[must_use]
    pub fn row(&self, index: usize) -> Vector3 {
        match index {
            0 => Vector3::new(self.x_axis.x, self.y_axis.x, self.z_axis.x),
            1 => Vector3::new(self.x_axis.y, self.y_axis.y, self.z_axis.y),
            _ => panic!("index out of bounds"),
        }
    }

    /// Returns `true` if, and only if, all elements are finite.
    /// If any element is either `NaN` or positive or negative infinity, this will return `false`.
    #[inline]
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.x_axis.is_finite()
    }

    /// Returns `true` if any elements are `NaN`.
    #[inline]
    #[must_use]
    pub fn is_nan(&self) -> bool {
        self.x_axis.is_nan() || self.y_axis.is_nan()
    }

    /// Computes `self * other.transpose()`.
    #[inline]
    #[must_use]
    pub fn mul_by_transposed(&self, other: Self) -> Matrix2 {
        let xx = self.x_axis.x * other.x_axis.x
            + self.y_axis.x * other.y_axis.x
            + self.z_axis.x * other.z_axis.x;
        let xy = self.x_axis.y * other.x_axis.x
            + self.y_axis.y * other.y_axis.x
            + self.z_axis.y * other.z_axis.x;
        let yy = self.x_axis.y * other.x_axis.y
            + self.y_axis.y * other.y_axis.y
            + self.z_axis.y * other.z_axis.y;

        Matrix2::from_cols_array_2d(&[[xx, xy], [xy, yy]])
    }

    /// Computes `self.transpose() * other`.
    #[inline]
    #[must_use]
    pub fn transposed_mul(&self, other: Self) -> SymmetricMatrix3 {
        let xx = self.x_axis.x * other.x_axis.x + self.x_axis.y * other.x_axis.y;

        let yx = self.y_axis.x * other.x_axis.x + self.y_axis.y * other.x_axis.y;
        let yy = self.y_axis.x * other.y_axis.x + self.y_axis.y * other.y_axis.y;

        let zx = self.z_axis.x * other.x_axis.x + self.z_axis.y * other.x_axis.y;
        let zy = self.z_axis.x * other.y_axis.x + self.z_axis.y * other.y_axis.y;
        let zz = self.z_axis.x * other.z_axis.x + self.z_axis.y * other.z_axis.y;

        SymmetricMatrix3::new(xx, yx, zx, yy, zy, zz)
    }

    /// Computes `(self.transpose() * other).transpose()`.
    #[inline]
    #[must_use]
    pub fn transposed_mul_mat2(&self, other: Matrix2) -> Matrix2x3 {
        Matrix2x3::new(
            self.x_axis.x * other.x_axis.x + self.x_axis.y * other.y_axis.x,
            self.x_axis.x * other.x_axis.y + self.x_axis.y * other.y_axis.y,
            self.y_axis.x * other.x_axis.x + self.y_axis.y * other.y_axis.x,
            self.y_axis.x * other.x_axis.y + self.y_axis.y * other.y_axis.y,
            self.z_axis.x * other.x_axis.x + self.z_axis.y * other.y_axis.x,
            self.z_axis.x * other.x_axis.y + self.z_axis.y * other.y_axis.y,
        )
    }
}

impl Add<Matrix2x3> for Matrix2x3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x_axis: self.x_axis + rhs.x_axis,
            y_axis: self.y_axis + rhs.y_axis,
            z_axis: self.z_axis + rhs.z_axis,
        }
    }
}

impl AddAssign<Matrix2x3> for Matrix2x3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Sub<Matrix2x3> for Matrix2x3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x_axis: self.x_axis - rhs.x_axis,
            y_axis: self.y_axis - rhs.y_axis,
            z_axis: self.z_axis - rhs.z_axis,
        }
    }
}

impl SubAssign<Matrix2x3> for Matrix2x3 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl Neg for Matrix2x3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self::Output {
        Self {
            x_axis: -self.x_axis,
            y_axis: -self.y_axis,
            z_axis: -self.z_axis,
        }
    }
}

impl Mul<Vector2> for Matrix2x3 {
    type Output = Vector3;
    #[inline]
    fn mul(self, rhs: Vector2) -> Self::Output {
        Vector3::new(
            rhs.dot(self.x_axis),
            rhs.dot(self.y_axis),
            rhs.dot(self.z_axis),
        )
    }
}

impl Mul<Matrix2x3> for Vector2 {
    type Output = Vector3;
    #[inline]
    fn mul(self, rhs: Matrix2x3) -> Self::Output {
        Vector3::new(
            self.dot(rhs.x_axis),
            self.dot(rhs.y_axis),
            self.dot(rhs.z_axis),
        )
    }
}

impl Mul<Matrix2x3> for Vector3 {
    type Output = Vector2;
    #[inline]
    fn mul(self, rhs: Matrix2x3) -> Self::Output {
        Vector2::new(self.dot(rhs.row(0)), self.dot(rhs.row(1)))
    }
}

impl Mul<Matrix2x3> for Scalar {
    type Output = Matrix2x3;
    #[inline]
    fn mul(self, rhs: Matrix2x3) -> Self::Output {
        rhs * self
    }
}

impl Mul<Scalar> for Matrix2x3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Scalar) -> Self::Output {
        Self {
            x_axis: self.x_axis * rhs,
            y_axis: self.y_axis * rhs,
            z_axis: self.z_axis * rhs,
        }
    }
}

impl MulAssign<Scalar> for Matrix2x3 {
    #[inline]
    fn mul_assign(&mut self, rhs: Scalar) {
        *self = *self * rhs;
    }
}

impl Div<Matrix2x3> for Scalar {
    type Output = Matrix2x3;
    #[inline]
    fn div(self, rhs: Matrix2x3) -> Self::Output {
        rhs / self
    }
}

impl Div<Scalar> for Matrix2x3 {
    type Output = Self;
    #[inline]
    fn div(self, rhs: Scalar) -> Self::Output {
        Self {
            x_axis: self.x_axis / rhs,
            y_axis: self.y_axis / rhs,
            z_axis: self.z_axis / rhs,
        }
    }
}

impl DivAssign<Scalar> for Matrix2x3 {
    #[inline]
    fn div_assign(&mut self, rhs: Scalar) {
        *self = *self / rhs;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mul_by_transposed() {
        let mat1 = Matrix2x3::new(2.0, 3.0, 1.0, 9.0, 4.0, 10.0);
        let mat2 = Matrix2x3::new(13.0, 5.0, 7.0, 9.0, 19.0, 61.0);
        assert_eq!(
            mat1.mul_by_transposed(mat2),
            Matrix2::from_cols_array(&[109.0, 292.0, 263.0, 706.0])
        );
    }
}
