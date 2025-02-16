use std::{
    iter::Sum,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign},
};

#[cfg(test)]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use bevy::reflect::Reflect;
use bevy_math::{Vec3, Vec3A};

use crate::{Matrix2x3, Matrix3, Scalar, Vector3};

// TODO: It could be nice to extract this into a `glam_matrix_extensions` crate or similar,
//       and have both `f32` and `f64` versions of matrix types.

/// The bottom left triangle (including the diagonal) of a symmetric 3x3 column-major matrix.
///
/// This is useful for storing a symmetric 3x3 matrix in a more compact form and performing some
/// matrix operations more efficiently.
///
/// Some defining properties of symmetric matrices include:
///
/// - The matrix is equal to its transpose.
/// - The matrix has real eigenvalues.
/// - The eigenvectors corresponding to the eigenvalues are orthogonal.
/// - The matrix is always diagonalizable.
///
/// The sum and difference of two symmetric matrices is always symmetric.
/// However, the product of two symmetric matrices is *only* symmetric
/// if the matrices are commutable, meaning that `AB = BA`.
#[derive(Reflect, Clone, Copy, Debug, PartialEq)]
pub struct SymmetricMatrix3 {
    /// The first element of the first column.
    pub m00: Scalar,
    /// The second element of the first column.
    pub m01: Scalar,
    /// The third element of the first column.
    pub m02: Scalar,
    /// The second element of the second column.
    pub m11: Scalar,
    /// The third element of the second column.
    pub m12: Scalar,
    /// The third element of the third column.
    pub m22: Scalar,
}

impl SymmetricMatrix3 {
    /// A symmetric 3x3 matrix with all elements set to `0.0`.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /// A symmetric 3x3 identity matrix, where all diagonal elements are `1.0`, and all off-diagonal elements are `0.0`.
    pub const IDENTITY: Self = Self::new(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    /// All `NaN`s.
    pub const NAN: Self = Self::new(
        Scalar::NAN,
        Scalar::NAN,
        Scalar::NAN,
        Scalar::NAN,
        Scalar::NAN,
        Scalar::NAN,
    );

    /// Creates a new symmetric 3x3 matrix from its bottom left triangle, including diagonal elements.
    ///
    /// The elements are in column-major order `mCR`, where `C` is the column index
    /// and `R` is the row index.
    #[inline(always)]
    #[must_use]
    pub const fn new(
        m00: Scalar,
        m01: Scalar,
        m02: Scalar,
        m11: Scalar,
        m12: Scalar,
        m22: Scalar,
    ) -> Self {
        Self {
            m00,
            m01,
            m02,
            m11,
            m12,
            m22,
        }
    }

    /// Creates a symmetric 3x3 matrix from three column vectors.
    ///
    /// Only the lower left triangle of the matrix is used. No check is performed to ensure
    /// that the given columns truly produce a symmetric matrix.
    #[inline(always)]
    #[must_use]
    pub const fn from_cols_unchecked(x_axis: Vec3, y_axis: Vec3, z_axis: Vec3) -> Self {
        Self {
            m00: x_axis.x,
            m01: x_axis.y,
            m02: x_axis.z,
            m11: y_axis.y,
            m12: y_axis.z,
            m22: z_axis.z,
        }
    }

    /// Creates a symmetric 3x3 matrix from a `[f32; 9]` array stored in column major order.
    ///
    /// Only the lower left triangle of the matrix is used. No check is performed to ensure
    /// that the given columns truly produce a symmetric matrix.
    #[inline]
    #[must_use]
    pub const fn from_cols_array_unchecked(m: &[f32; 9]) -> Self {
        Self::new(m[0], m[1], m[2], m[4], m[5], m[8])
    }

    /// Creates a `[f32; 9]` array storing data in column major order.
    #[inline]
    #[must_use]
    pub const fn to_cols_array(&self) -> [f32; 9] {
        [
            self.m00, self.m01, self.m02, self.m01, self.m11, self.m12, self.m02, self.m12,
            self.m22,
        ]
    }

    /// Creates a symmetric 3x3 matrix from a `[[f32; 3]; 3]` 3D array stored in column major order.
    ///
    /// Only the lower left triangle of the matrix is used. No check is performed to ensure
    /// that the given columns truly produce a symmetric matrix.
    #[inline]
    #[must_use]
    pub const fn from_cols_array_2d_unchecked(m: &[[f32; 3]; 3]) -> Self {
        Self::from_cols_unchecked(
            Vec3::from_array(m[0]),
            Vec3::from_array(m[1]),
            Vec3::from_array(m[2]),
        )
    }

    /// Creates a `[[f32; 3]; 3]` 3D array storing data in column major order.
    #[inline]
    #[must_use]
    pub const fn to_cols_array_2d(&self) -> [[f32; 3]; 3] {
        [
            [self.m00, self.m01, self.m02],
            [self.m01, self.m11, self.m12],
            [self.m02, self.m12, self.m22],
        ]
    }

    /// Creates a 3x3 matrix from the first 9 values in `slice`.
    ///
    /// Only the lower left triangle of the matrix is used. No check is performed to ensure
    /// that the given columns truly produce a symmetric matrix.
    ///
    /// # Panics
    ///
    /// Panics if `slice` is less than 9 elements long.
    #[inline]
    #[must_use]
    pub const fn from_cols_slice(slice: &[f32]) -> Self {
        Self::new(slice[0], slice[1], slice[2], slice[4], slice[5], slice[8])
    }

    /// Creates a symmetric 3x3 matrix with its diagonal set to `diagonal` and all other entries set to `0.0`.
    #[inline]
    #[must_use]
    #[doc(alias = "scale")]
    pub const fn from_diagonal(diagonal: Vector3) -> Self {
        Self::new(diagonal.x, 0.0, diagonal.y, 0.0, 0.0, diagonal.z)
    }

    /// Creates a symmetric 3x3 matrix from a 3x3 matrix.
    ///
    /// Only the lower left triangle of the matrix is used. No check is performed to ensure
    /// that the given matrix is truly symmetric matrix.
    #[inline]
    #[must_use]
    pub const fn from_matrix3_unchecked(mat: Matrix3) -> Self {
        Self::new(
            mat.x_axis.x,
            mat.x_axis.y,
            mat.x_axis.z,
            mat.y_axis.y,
            mat.y_axis.z,
            mat.z_axis.z,
        )
    }

    /// Creates a [`Matrix3`] from the symmetric 3x3 matrix in `self`.
    #[inline]
    #[must_use]
    pub const fn to_matrix3(&self) -> Matrix3 {
        Matrix3::from_cols_array(&self.to_cols_array())
    }

    /// Returns the matrix column for the given `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is greater than 2.
    #[inline]
    #[must_use]
    pub const fn col(&self, index: usize) -> Vector3 {
        match index {
            0 => Vector3::new(self.m00, self.m01, self.m02),
            1 => Vector3::new(self.m01, self.m11, self.m12),
            2 => Vector3::new(self.m02, self.m12, self.m22),
            _ => panic!("index out of bounds"),
        }
    }

    /// Returns the matrix row for the given `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is greater than 2.
    #[inline]
    #[must_use]
    pub const fn row(&self, index: usize) -> Vector3 {
        match index {
            0 => Vector3::new(self.m00, self.m01, self.m02),
            1 => Vector3::new(self.m01, self.m11, self.m12),
            2 => Vector3::new(self.m02, self.m12, self.m22),
            _ => panic!("index out of bounds"),
        }
    }

    /// Returns `true` if, and only if, all elements are finite.
    /// If any element is either `NaN` or positive or negative infinity, this will return `false`.
    #[inline]
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.m00.is_finite()
            && self.m01.is_finite()
            && self.m11.is_finite()
            && self.m02.is_finite()
            && self.m12.is_finite()
            && self.m22.is_finite()
    }

    /// Returns `true` if any elements are `NaN`.
    #[inline]
    #[must_use]
    pub fn is_nan(&self) -> bool {
        self.m00.is_nan()
            || self.m01.is_nan()
            || self.m11.is_nan()
            || self.m02.is_nan()
            || self.m12.is_nan()
            || self.m22.is_nan()
    }

    /// Returns the determinant of `self`.
    #[inline]
    #[must_use]
    pub fn determinant(&self) -> Scalar {
        self.to_matrix3().determinant()
    }

    /// Returns the inverse of `self`.
    ///
    /// If the matrix is not invertible the returned matrix will be invalid.
    #[inline]
    #[must_use]
    pub fn inverse(&self) -> Self {
        let m00 = self.m11 * self.m22 - self.m12 * self.m12;
        let m10 = self.m12 * self.m02 - self.m22 * self.m01;
        let m20 = self.m01 * self.m12 - self.m02 * self.m11;

        let inverse_determinant = 1.0 / (m00 * self.m00 + m10 * self.m01 + m20 * self.m02);

        let m11 = self.m22 * self.m00 - self.m02 * self.m02;
        let m21 = self.m02 * self.m01 - self.m00 * self.m12;
        let m22 = self.m00 * self.m11 - self.m01 * self.m01;

        Self {
            m00: m00 * inverse_determinant,
            m01: m10 * inverse_determinant,
            m02: m20 * inverse_determinant,
            m11: m11 * inverse_determinant,
            m12: m21 * inverse_determinant,
            m22: m22 * inverse_determinant,
        }
    }

    /// Takes the absolute value of each element in `self`.
    #[inline]
    #[must_use]
    pub fn abs(&self) -> Self {
        Self::new(
            self.m00.abs(),
            self.m01.abs(),
            self.m02.abs(),
            self.m11.abs(),
            self.m12.abs(),
            self.m22.abs(),
        )
    }

    /// Computes `(self * other.transpose()).transpose()`.
    #[inline]
    #[must_use]
    pub fn mul_by_transposed_mat2x3(&self, other: Matrix2x3) -> Matrix2x3 {
        Matrix2x3::new(
            self.m00 * other.x_axis.x + self.m01 * other.y_axis.x + self.m02 * other.z_axis.x,
            self.m00 * other.x_axis.y + self.m01 * other.y_axis.y + self.m02 * other.z_axis.y,
            self.m01 * other.x_axis.x + self.m11 * other.y_axis.x + self.m12 * other.z_axis.x,
            self.m01 * other.x_axis.y + self.m11 * other.y_axis.y + self.m12 * other.z_axis.y,
            self.m02 * other.x_axis.x + self.m12 * other.y_axis.x + self.m22 * other.z_axis.x,
            self.m02 * other.x_axis.y + self.m12 * other.y_axis.y + self.m22 * other.z_axis.y,
        )
    }

    /// Computes `skew_symmetric(vec) * self * skew_symmetric(vec).transpose()` for a symmetric matrix `self`.
    #[inline]
    #[must_use]
    pub fn skew(&self, vec: Vector3) -> Self {
        // 27 multiplications and 14 additions

        let xzy = vec.x * self.m12;
        let yzx = vec.y * self.m02;
        let zyx = vec.z * self.m01;

        let ixy = vec.y * self.m12 - vec.z * self.m11;
        let ixz = vec.y * self.m22 - vec.z * self.m12;
        let iyx = vec.z * self.m00 - vec.x * self.m02;
        let iyy = zyx - xzy;

        let iyz = vec.z * self.m02 - vec.x * self.m22;
        let izx = vec.x * self.m01 - vec.y * self.m00;
        let izy = vec.x * self.m11 - vec.y * self.m01;
        let izz = xzy - yzx;

        Self::new(
            vec.y * ixz - vec.z * ixy,
            vec.y * iyz - vec.z * iyy,
            vec.y * izz - vec.z * izy,
            vec.z * iyx - vec.x * iyz,
            vec.z * izx - vec.x * izz,
            vec.x * izy - vec.y * izx,
        )
    }

    /// Transforms a 3D vector.
    #[inline]
    #[must_use]
    pub fn mul_vector3(&self, rhs: Vector3) -> Vector3 {
        let mut res = self.col(0).mul(rhs.x);
        res = res.add(self.col(1).mul(rhs.y));
        res = res.add(self.col(2).mul(rhs.z));
        res
    }

    /// Transforms a [`Vec3A`].
    #[inline]
    #[must_use]
    pub fn mul_vec3a(&self, rhs: Vec3A) -> Vec3A {
        self.mul_vector3(rhs.into()).into()
    }

    /// Multiplies two 3x3 matrices.
    #[inline]
    #[must_use]
    pub fn mul_matrix3(&self, rhs: &Matrix3) -> Matrix3 {
        Matrix3::from_cols(
            self.mul(rhs.x_axis),
            self.mul(rhs.y_axis),
            self.mul(rhs.z_axis),
        )
    }

    /// Adds two 3x3 matrices.
    #[inline]
    #[must_use]
    pub fn add_matrix3(&self, rhs: &Matrix3) -> Matrix3 {
        Matrix3::from_cols(
            self.col(0).add(rhs.x_axis),
            self.col(1).add(rhs.y_axis),
            self.col(2).add(rhs.z_axis),
        )
    }

    /// Subtracts two 3x3 matrices.
    #[inline]
    #[must_use]
    pub fn sub_matrix3(&self, rhs: &Matrix3) -> Matrix3 {
        Matrix3::from_cols(
            self.col(0).sub(rhs.x_axis),
            self.col(1).sub(rhs.y_axis),
            self.col(2).sub(rhs.z_axis),
        )
    }

    /// Multiplies two 3x3 matrices.
    #[inline]
    #[must_use]
    pub fn mul_symmetric_matrix3(&self, rhs: &Self) -> Matrix3 {
        Matrix3::from_cols(
            self.mul_vector3(rhs.col(0)),
            self.mul_vector3(rhs.col(1)),
            self.mul_vector3(rhs.col(2)),
        )
    }

    /// Adds two 3x3 matrices.
    #[inline]
    #[must_use]
    pub fn add_symmetric_matrix3(&self, rhs: &Self) -> Self {
        Self::new(
            self.m00 + rhs.m00,
            self.m01 + rhs.m01,
            self.m02 + rhs.m02,
            self.m11 + rhs.m11,
            self.m12 + rhs.m12,
            self.m22 + rhs.m22,
        )
    }

    /// Subtracts two 3x3 matrices.
    #[inline]
    #[must_use]
    pub fn sub_symmetric_matrix3(&self, rhs: &Self) -> Self {
        Self::new(
            self.m00 - rhs.m00,
            self.m01 - rhs.m01,
            self.m02 - rhs.m02,
            self.m11 - rhs.m11,
            self.m12 - rhs.m12,
            self.m22 - rhs.m22,
        )
    }

    /// Multiplies a 3x3 matrix by a scalar.
    #[inline]
    #[must_use]
    pub fn mul_scalar(&self, rhs: f32) -> Self {
        Self::new(
            self.m00 * rhs,
            self.m01 * rhs,
            self.m02 * rhs,
            self.m11 * rhs,
            self.m12 * rhs,
            self.m22 * rhs,
        )
    }

    /// Divides a 3x3 matrix by a scalar.
    #[inline]
    #[must_use]
    pub fn div_scalar(&self, rhs: f32) -> Self {
        Self::new(
            self.m00 / rhs,
            self.m01 / rhs,
            self.m02 / rhs,
            self.m11 / rhs,
            self.m12 / rhs,
            self.m22 / rhs,
        )
    }
}

impl Default for SymmetricMatrix3 {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Add<SymmetricMatrix3> for SymmetricMatrix3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        self.add_symmetric_matrix3(&rhs)
    }
}

impl AddAssign<SymmetricMatrix3> for SymmetricMatrix3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Add<Matrix3> for SymmetricMatrix3 {
    type Output = Matrix3;
    #[inline]
    fn add(self, rhs: Matrix3) -> Self::Output {
        self.add_matrix3(&rhs)
    }
}

impl Add<SymmetricMatrix3> for Matrix3 {
    type Output = Matrix3;
    #[inline]
    fn add(self, rhs: SymmetricMatrix3) -> Self::Output {
        rhs.add_matrix3(&self)
    }
}

impl AddAssign<SymmetricMatrix3> for Matrix3 {
    #[inline]
    fn add_assign(&mut self, rhs: SymmetricMatrix3) {
        *self = self.add(rhs);
    }
}

impl Sub<SymmetricMatrix3> for SymmetricMatrix3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        self.sub_symmetric_matrix3(&rhs)
    }
}

impl SubAssign<SymmetricMatrix3> for SymmetricMatrix3 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl Sub<Matrix3> for SymmetricMatrix3 {
    type Output = Matrix3;
    #[inline]
    fn sub(self, rhs: Matrix3) -> Self::Output {
        self.sub_matrix3(&rhs)
    }
}

impl Sub<SymmetricMatrix3> for Matrix3 {
    type Output = Matrix3;
    #[inline]
    fn sub(self, rhs: SymmetricMatrix3) -> Self::Output {
        rhs.sub_matrix3(&self)
    }
}

impl SubAssign<SymmetricMatrix3> for Matrix3 {
    #[inline]
    fn sub_assign(&mut self, rhs: SymmetricMatrix3) {
        *self = self.sub(rhs);
    }
}

impl Neg for SymmetricMatrix3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self::Output {
        Self::new(
            -self.m00, -self.m01, -self.m02, -self.m11, -self.m12, -self.m22,
        )
    }
}

impl Mul<SymmetricMatrix3> for SymmetricMatrix3 {
    type Output = Matrix3;
    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        self.mul_symmetric_matrix3(&rhs)
    }
}

impl Mul<SymmetricMatrix3> for Matrix3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: SymmetricMatrix3) -> Self::Output {
        Self::from_cols_array_2d(&[
            [
                self.x_axis.x * rhs.m00 + self.y_axis.x * rhs.m01 + self.z_axis.x * rhs.m02,
                self.x_axis.y * rhs.m00 + self.y_axis.y * rhs.m01 + self.z_axis.y * rhs.m02,
                self.x_axis.z * rhs.m00 + self.y_axis.z * rhs.m01 + self.z_axis.z * rhs.m02,
            ],
            [
                self.x_axis.x * rhs.m01 + self.y_axis.x * rhs.m11 + self.z_axis.x * rhs.m12,
                self.x_axis.y * rhs.m01 + self.y_axis.y * rhs.m11 + self.z_axis.y * rhs.m12,
                self.x_axis.z * rhs.m01 + self.y_axis.z * rhs.m11 + self.z_axis.z * rhs.m12,
            ],
            [
                self.x_axis.x * rhs.m02 + self.y_axis.x * rhs.m12 + self.z_axis.x * rhs.m22,
                self.x_axis.y * rhs.m02 + self.y_axis.y * rhs.m12 + self.z_axis.y * rhs.m22,
                self.x_axis.z * rhs.m02 + self.y_axis.z * rhs.m12 + self.z_axis.z * rhs.m22,
            ],
        ])
    }
}

impl MulAssign<SymmetricMatrix3> for Matrix3 {
    #[inline]
    fn mul_assign(&mut self, rhs: SymmetricMatrix3) {
        *self = self.mul(rhs);
    }
}

impl Mul<Matrix3> for SymmetricMatrix3 {
    type Output = Matrix3;
    #[inline]
    fn mul(self, rhs: Matrix3) -> Self::Output {
        self.mul_matrix3(&rhs)
    }
}

impl Mul<SymmetricMatrix3> for Matrix2x3 {
    type Output = Matrix2x3;
    #[inline]
    fn mul(self, rhs: SymmetricMatrix3) -> Self::Output {
        Matrix2x3::new(
            self.x_axis.x * rhs.m00 + self.y_axis.x * rhs.m01 + self.z_axis.x * rhs.m02,
            self.x_axis.y * rhs.m00 + self.y_axis.y * rhs.m01 + self.z_axis.y * rhs.m02,
            self.x_axis.x * rhs.m01 + self.y_axis.x * rhs.m11 + self.z_axis.x * rhs.m12,
            self.x_axis.y * rhs.m01 + self.y_axis.y * rhs.m11 + self.z_axis.y * rhs.m12,
            self.x_axis.x * rhs.m02 + self.y_axis.x * rhs.m12 + self.z_axis.x * rhs.m22,
            self.x_axis.y * rhs.m02 + self.y_axis.y * rhs.m12 + self.z_axis.y * rhs.m22,
        )
    }
}

impl Mul<Vector3> for SymmetricMatrix3 {
    type Output = Vector3;
    #[inline]
    fn mul(self, rhs: Vector3) -> Self::Output {
        self.mul_vector3(rhs)
    }
}

impl Mul<SymmetricMatrix3> for Scalar {
    type Output = SymmetricMatrix3;
    #[inline]
    fn mul(self, rhs: SymmetricMatrix3) -> Self::Output {
        rhs.mul_scalar(self)
    }
}

impl Mul<Scalar> for SymmetricMatrix3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Scalar) -> Self::Output {
        self.mul_scalar(rhs)
    }
}

impl MulAssign<Scalar> for SymmetricMatrix3 {
    #[inline]
    fn mul_assign(&mut self, rhs: Scalar) {
        *self = self.mul_scalar(rhs);
    }
}

impl Div<SymmetricMatrix3> for Scalar {
    type Output = SymmetricMatrix3;
    #[inline]
    fn div(self, rhs: SymmetricMatrix3) -> Self::Output {
        rhs.div_scalar(self)
    }
}

impl Div<Scalar> for SymmetricMatrix3 {
    type Output = Self;
    #[inline]
    fn div(self, rhs: Scalar) -> Self::Output {
        self.div_scalar(rhs)
    }
}

impl DivAssign<Scalar> for SymmetricMatrix3 {
    #[inline]
    fn div_assign(&mut self, rhs: Scalar) {
        *self = self.div_scalar(rhs);
    }
}

impl Mul<Vec3A> for SymmetricMatrix3 {
    type Output = Vec3A;
    #[inline]
    fn mul(self, rhs: Vec3A) -> Self::Output {
        self.mul_vec3a(rhs)
    }
}

impl From<Matrix3> for SymmetricMatrix3 {
    #[inline]
    fn from(mat: Matrix3) -> Self {
        Self::from_matrix3_unchecked(mat)
    }
}

impl From<SymmetricMatrix3> for Matrix3 {
    #[inline]
    fn from(mat: SymmetricMatrix3) -> Self {
        Self::from_cols(mat.col(0), mat.col(1), mat.col(2))
    }
}

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
impl From<parry::na::Matrix3<Scalar>> for SymmetricMatrix3 {
    #[inline]
    fn from(mat: parry::na::Matrix3<Scalar>) -> Self {
        Self::new(mat.m11, mat.m21, mat.m31, mat.m22, mat.m32, mat.m33)
    }
}

impl Sum<SymmetricMatrix3> for SymmetricMatrix3 {
    fn sum<I: Iterator<Item = SymmetricMatrix3>>(iter: I) -> Self {
        iter.fold(Self::ZERO, Self::add)
    }
}

impl<'a> Sum<&'a SymmetricMatrix3> for SymmetricMatrix3 {
    fn sum<I: Iterator<Item = &'a SymmetricMatrix3>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, &b| a.add(b))
    }
}

#[cfg(not(target_arch = "spirv"))]
impl core::fmt::Display for SymmetricMatrix3 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.to_matrix3().fmt(f)
    }
}

#[cfg(test)]
impl AbsDiffEq for SymmetricMatrix3 {
    type Epsilon = Scalar;
    fn default_epsilon() -> Self::Epsilon {
        Scalar::EPSILON
    }
    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        AbsDiffEq::abs_diff_eq(&self.m00, &other.m00, epsilon)
            && AbsDiffEq::abs_diff_eq(&self.m01, &other.m01, epsilon)
            && AbsDiffEq::abs_diff_eq(&self.m11, &other.m11, epsilon)
            && AbsDiffEq::abs_diff_eq(&self.m02, &other.m02, epsilon)
            && AbsDiffEq::abs_diff_eq(&self.m12, &other.m12, epsilon)
            && AbsDiffEq::abs_diff_eq(&self.m22, &other.m22, epsilon)
    }
}

#[cfg(test)]
impl RelativeEq for SymmetricMatrix3 {
    fn default_max_relative() -> Self::Epsilon {
        Scalar::EPSILON
    }
    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        RelativeEq::relative_eq(&self.m00, &other.m00, epsilon, max_relative)
            && RelativeEq::relative_eq(&self.m01, &other.m01, epsilon, max_relative)
            && RelativeEq::relative_eq(&self.m11, &other.m11, epsilon, max_relative)
            && RelativeEq::relative_eq(&self.m02, &other.m02, epsilon, max_relative)
            && RelativeEq::relative_eq(&self.m12, &other.m12, epsilon, max_relative)
            && RelativeEq::relative_eq(&self.m22, &other.m22, epsilon, max_relative)
    }
}

#[cfg(test)]
impl UlpsEq for SymmetricMatrix3 {
    fn default_max_ulps() -> u32 {
        4
    }
    fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
        UlpsEq::ulps_eq(&self.m00, &other.m00, epsilon, max_ulps)
            && UlpsEq::ulps_eq(&self.m01, &other.m01, epsilon, max_ulps)
            && UlpsEq::ulps_eq(&self.m11, &other.m11, epsilon, max_ulps)
            && UlpsEq::ulps_eq(&self.m02, &other.m02, epsilon, max_ulps)
            && UlpsEq::ulps_eq(&self.m12, &other.m12, epsilon, max_ulps)
            && UlpsEq::ulps_eq(&self.m22, &other.m22, epsilon, max_ulps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mat2x3_mul_mat3x3() {
        let mat_3x3 = SymmetricMatrix3::new(13.0, 7.0, 19.0, 5.0, 61.0, 3.0);
        let mat2x3 = Matrix2x3::new(2.0, 3.0, 1.0, 9.0, 4.0, 10.0);
        assert_eq!(
            mat2x3 * mat_3x3,
            Matrix2x3::new(109.0, 292.0, 263.0, 676.0, 111.0, 636.0)
        );
    }

    #[test]
    fn inverse() {
        let mat = SymmetricMatrix3::new(13.0, 7.0, 19.0, 5.0, 61.0, 3.0);
        let inverse = mat.inverse();
        assert_eq!(
            inverse,
            SymmetricMatrix3::new(0.1093, -0.0336, -0.0098, 0.0095, 0.0195, -0.0005)
        );
    }
}
