//! Eigen decomposition of a symmetric 3x3 matrix.

// The eigensolver is a Rust adaptation, with modifications, of the pseudocode and approach described in
// "A Robust Eigensolver for 3 x 3 Symmetric Matrices" by David Eberly, Geometric Tools, Redmond WA 98052.
// https://www.geometrictools.com/Documentation/RobustEigenSymmetric3x3.pdf
use crate::math::{Matrix, Vector};
use bevy::math::Vec3Swizzles;

/// The eigen decomposition of a symmetric 3x3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "bevy_reflect", derive(bevy_reflect::Reflect))]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct SymmetricEigen3 {
    /// The eigenvalues of the symmetric 3x3 matrix.
    ///
    /// These should be in ascending order `eigen1 <= eigen2 <= eigen3`.
    pub eigenvalues: Vector,
    /// The three eigenvectors of the symmetric 3x3 matrix.
    /// Each eigenvector should be unit length and orthogonal to the other eigenvectors.
    ///
    /// The eigenvectors are ordered to correspond to the eigenvalues. For example,
    /// `eigenvectors.x_axis` corresponds to `eigenvalues.x`.
    pub eigenvectors: Matrix,
}

impl SymmetricEigen3 {
    /// Computes the eigen decomposition of the given symmetric 3x3 matrix.
    ///
    /// The eigenvalues are returned in ascending order `eigen1 <= eigen2 <= eigen3`.
    /// This can be reversed with the [`reverse`](Self::reverse) method.
    pub fn new(mat: Matrix) -> Self {
        let (mut eigenvalues, is_diagonal) = Self::eigenvalues(mat);

        if is_diagonal {
            // The matrix is already diagonal. Sort the eigenvalues in ascending order,
            // ordering the eigenvectors accordingly, and return early.
            let mut eigenvectors = Matrix::IDENTITY;
            if eigenvalues[0] > eigenvalues[1] {
                std::mem::swap(&mut eigenvalues.x, &mut eigenvalues.y);
                std::mem::swap(&mut eigenvectors.x_axis, &mut eigenvectors.y_axis);
            }
            if eigenvalues[1] > eigenvalues[2] {
                std::mem::swap(&mut eigenvalues.y, &mut eigenvalues.z);
                std::mem::swap(&mut eigenvectors.y_axis, &mut eigenvectors.z_axis);
            }
            return Self {
                eigenvalues,
                eigenvectors,
            };
        }

        // Compute the eigenvectors corresponding to the eigenvalues.
        let eigenvector1 = Self::eigenvector1(mat, eigenvalues.x);
        let eigenvector2 = Self::eigenvector2(mat, eigenvector1, eigenvalues.y);
        let eigenvector3 = Self::eigenvector3(eigenvector1, eigenvector2);

        Self {
            eigenvalues,
            eigenvectors: Matrix::from_cols(eigenvector1, eigenvector2, eigenvector3),
        }
    }

    /// Reverses the order of the eigenvalues and their corresponding eigenvectors.
    pub fn reverse(&self) -> Self {
        Self {
            eigenvalues: self.eigenvalues.zyx(),
            eigenvectors: Matrix::from_cols(
                self.eigenvectors.z_axis,
                self.eigenvectors.y_axis,
                self.eigenvectors.x_axis,
            ),
        }
    }

    /// Computes the eigenvalues of a symmetric 3x3 matrix, also returning whether the input matrix is diagonal.
    ///
    /// If the matrix is already diagonal, the eigenvalues are returned as is without reordering.
    /// Otherwise, the eigenvalues are computed and returned in ascending order
    /// such that `eigen1 <= eigen2 <= eigen3`.
    ///
    /// Reference: <https://en.wikipedia.org/wiki/Eigenvalue_algorithm#3%C3%973_matrices>
    pub fn eigenvalues(mat: Matrix) -> (Vector, bool) {
        let p1 = mat.y_axis.x.powi(2) + mat.z_axis.x.powi(2) + mat.z_axis.y.powi(2);

        if p1 == 0.0 {
            // The matrix is diagonal.
            return (Vector::new(mat.x_axis.x, mat.y_axis.y, mat.z_axis.z), true);
        }

        let q = (mat.x_axis.x + mat.y_axis.y + mat.z_axis.z) / 3.0;
        let p2 = (mat.x_axis.x - q).powi(2)
            + (mat.y_axis.y - q).powi(2)
            + (mat.z_axis.z - q).powi(2)
            + 2.0 * p1;
        let p = (p2 / 6.0).sqrt();
        let mat_b = 1.0 / p * (mat - q * Matrix::IDENTITY);
        let r = mat_b.determinant() / 2.0;

        // r should be in the [-1, 1] range for a symmetric matrix,
        // but computation error can leave it slightly outside this range.
        let phi = if r <= -1.0 {
            std::f32::consts::FRAC_PI_3
        } else if r >= 1.0 {
            0.0
        } else {
            r.acos() / 3.0
        };

        // The eigenvalues satisfy eigen3 <= eigen2 <= eigen1
        let eigen1 = q + 2.0 * p * phi.cos();
        let eigen3 = q + 2.0 * p * (phi + 2.0 * std::f32::consts::FRAC_PI_3).cos();
        let eigen2 = 3.0 * q - eigen1 - eigen3; // trace(mat) = eigen1 + eigen2 + eigen3
        (Vector::new(eigen3, eigen2, eigen1), false)
    }

    // TODO: Fall back to QL when the eigenvalue precision is poor.
    /// Computes the unit-length eigenvector corresponding to the `eigenvalue1` of `mat` that was
    /// computed from the root of a cubic polynomial with a multiplicity of 1.
    ///
    /// If the other two eigenvalues are well separated, this method can be used for computing
    /// all three eigenvectors. However, to avoid numerical issues when eigenvalues are close to
    /// each other, it's recommended to use the `eigenvector2` method for the second eigenvector.
    ///
    /// The third eigenvector can be computed as the cross product of the first two.
    pub fn eigenvector1(mat: Matrix, eigenvalue1: f32) -> Vector {
        let cols = mat - Matrix::from_diagonal(Vector::splat(eigenvalue1));
        let c0xc1 = cols.x_axis.cross(cols.y_axis);
        let c0xc2 = cols.x_axis.cross(cols.z_axis);
        let c1xc2 = cols.y_axis.cross(cols.z_axis);
        let d0 = c0xc1.length_squared();
        let d1 = c0xc2.length_squared();
        let d2 = c1xc2.length_squared();

        let mut d_max = d0;
        let mut i_max = 0;

        if d1 > d_max {
            d_max = d1;
            i_max = 1;
        }
        if d2 > d_max {
            i_max = 2;
        }
        if i_max == 0 {
            c0xc1 / d0.sqrt()
        } else if i_max == 1 {
            c0xc2 / d1.sqrt()
        } else {
            c1xc2 / d2.sqrt()
        }
    }

    /// Computes the unit-length eigenvector corresponding to the `eigenvalue2` of `mat` that was
    /// computed from the root of a cubic polynomial with a potential multiplicity of 2.
    ///
    /// The third eigenvector can be computed as the cross product of the first two.
    pub fn eigenvector2(mat: Matrix, eigenvector1: Vector, eigenvalue2: f32) -> Vector {
        // Compute right-handed orthonormal set { U, V, W }, where W is eigenvector1.
        let (u, v) = eigenvector1.any_orthonormal_pair();

        // The unit-length eigenvector is E = x0 * U + x1 * V. We need to compute x0 and x1.
        //
        // Define the symmetrix 2x2 matrix M = J^T * (mat - eigenvalue2 * I), where J = [U V]
        // and I is a 3x3 identity matrix. This means that E = J * X, where X is a column vector
        // with rows x0 and x1. The 3x3 linear system (mat - eigenvalue2 * I) * E = 0 reduces to
        // the 2x2 linear system M * X = 0.
        //
        // When eigenvalue2 != eigenvalue3, M has rank 1 and is not the zero matrix.
        // Otherwise, it has rank 0, and it is the zero matrix.

        let au = mat * u;
        let av = mat * v;

        let mut m00 = u.dot(au) - eigenvalue2;
        let mut m01 = u.dot(av);
        let mut m11 = v.dot(av) - eigenvalue2;
        let (abs_m00, abs_m01, abs_m11) = (m00.abs(), m01.abs(), m11.abs());

        if abs_m00 >= abs_m11 {
            let max_abs_component = abs_m00.max(abs_m01);
            if max_abs_component > 0.0 {
                if abs_m00 >= abs_m01 {
                    // m00 is the largest component of the row.
                    // Factor it out for normalization and discard to avoid underflow or overflow.
                    m01 /= m00;
                    m00 = 1.0 / (1.0 + m01 * m01).sqrt();
                    m01 *= m00;
                } else {
                    // m01 is the largest component of the row.
                    // Factor it out for normalization and discard to avoid underflow or overflow.
                    m00 /= m01;
                    m01 = 1.0 / (1.0 + m00 * m00).sqrt();
                    m00 *= m01;
                }
                return m01 * u - m00 * v;
            }
        } else {
            let max_abs_component = abs_m11.max(abs_m01);
            if max_abs_component > 0.0 {
                if abs_m11 >= abs_m01 {
                    // m11 is the largest component of the row.
                    // Factor it out for normalization and discard to avoid underflow or overflow.
                    m01 /= m11;
                    m11 = 1.0 / (1.0 + m01 * m01).sqrt();
                    m01 *= m11;
                } else {
                    // m01 is the largest component of the row.
                    // Factor it out for normalization and discard to avoid underflow or overflow.
                    m11 /= m01;
                    m01 = 1.0 / (1.0 + m11 * m11).sqrt();
                    m11 *= m01;
                }
                return m11 * u - m01 * v;
            }
        }

        // M is the zero matrix, any unit-length solution suffices.
        u
    }

    /// Computes the third eigenvector as the cross product of the first two.
    /// If the given eigenvectors are valid, the returned vector should be unit length.
    pub fn eigenvector3(eigenvector1: Vector, eigenvector2: Vector) -> Vector {
        eigenvector1.cross(eigenvector2)
    }
}

#[cfg(test)]
mod test {
    use super::SymmetricEigen3;
    use crate::math::{Matrix, Vector};
    use approx::assert_relative_eq;
    use rand::{Rng, SeedableRng};

    #[test]
    fn eigen_3x3() {
        let mat = Matrix::from_cols_array_2d(&[[2.0, 7.0, 8.0], [7.0, 6.0, 3.0], [8.0, 3.0, 0.0]]);
        let eigen = SymmetricEigen3::new(mat);

        assert_relative_eq!(
            eigen.eigenvalues,
            Vector::new(-7.605, 0.577, 15.028),
            epsilon = 0.001
        );
        assert_relative_eq!(
            Matrix::from_cols(
                eigen.eigenvectors.x_axis.abs(),
                eigen.eigenvectors.y_axis.abs(),
                eigen.eigenvectors.z_axis.abs()
            ),
            Matrix::from_cols(
                Vector::new(-1.075, 0.333, 1.0).normalize().abs(),
                Vector::new(0.542, -1.253, 1.0).normalize().abs(),
                Vector::new(1.359, 1.386, 1.0).normalize().abs()
            ),
            epsilon = 0.001
        );
    }

    #[test]
    fn eigen_3x3_diagonal() {
        let mat = Matrix::from_cols_array_2d(&[[2.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 3.0]]);
        let eigen = SymmetricEigen3::new(mat);

        assert_eq!(eigen.eigenvalues, Vector::new(2.0, 3.0, 5.0));
        assert_eq!(
            Matrix::from_cols(
                eigen.eigenvectors.x_axis.normalize().abs(),
                eigen.eigenvectors.y_axis.normalize().abs(),
                eigen.eigenvectors.z_axis.normalize().abs()
            ),
            Matrix::from_cols_array_2d(&[[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
        );
    }

    #[test]
    fn eigen_3x3_reconstruction() {
        let mut rng = rand_chacha::ChaCha8Rng::from_seed(Default::default());

        // Generate random symmetric matrices and verify that the eigen decomposition is correct.
        for _ in 0..10_000 {
            let eigenvalues = Vector::new(
                rng.gen_range(0.1..100.0),
                rng.gen_range(0.1..100.0),
                rng.gen_range(0.1..100.0),
            );
            let eigenvectors = Matrix::from_cols(
                Vector::new(
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                )
                .normalize(),
                Vector::new(
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                )
                .normalize(),
                Vector::new(
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                )
                .normalize(),
            );

            // Construct the symmetric matrix from the eigenvalues and eigenvectors.
            let mat1 = eigenvectors * Matrix::from_diagonal(eigenvalues) * eigenvectors.transpose();

            // Compute the eigen decomposition of the constructed matrix.
            let eigen = SymmetricEigen3::new(mat1);

            // Reconstruct the matrix from the computed eigenvalues and eigenvectors.
            let mat2 = eigen.eigenvectors
                * Matrix::from_diagonal(eigen.eigenvalues)
                * eigen.eigenvectors.transpose();

            // The reconstructed matrix should be close to the original matrix.
            // Note: The precision depends on how large the eigenvalues are.
            //       Larger eigenvalues can lead to larger absolute error.
            assert_relative_eq!(mat1, mat2, epsilon = 1e-2);
        }
    }
}
