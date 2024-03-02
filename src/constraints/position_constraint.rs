use crate::prelude::*;

/// A positional constraint applies a positional correction
/// with a given direction and magnitude at the local contact points `r1` and  `r2`.
pub trait PositionConstraint: XpbdConstraint<2> {
    /// Applies a positional correction to two bodies.
    ///
    /// Returns the positional impulse that is applied proportional to the inverse masses of the bodies.
    #[allow(clippy::too_many_arguments)]
    fn apply_positional_correction(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_lagrange: Scalar,
        direction: Vector,
        r1: Vector,
        r2: Vector,
    ) -> Vector {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return Vector::ZERO;
        }

        // Compute positional impulse
        let p = delta_lagrange * direction;
        let rot1 = *body1.rotation;
        let rot2 = *body2.rotation;

        let inv_mass1 = body1.effective_inv_mass();
        let inv_mass2 = body2.effective_inv_mass();
        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        // Apply positional and rotational updates
        if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
            body1.accumulated_translation.0 += p * inv_mass1;
            *body1.rotation += Self::get_delta_rot(rot1, inv_inertia1, r1, p);

            #[cfg(feature = "3d")]
            {
                // In 3D, subtracting quaternions like above can result in unnormalized rotations,
                // which causes stability issues (see #235) and panics when trying to rotate unit vectors.
                // TODO: It would be nice to avoid normalization if possible.
                //       Maybe the math above can be done in a way that keeps rotations normalized?
                body1.rotation.0 = body1.rotation.0.normalize();
            }
        }
        if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
            body2.accumulated_translation.0 -= p * inv_mass2;
            *body2.rotation -= Self::get_delta_rot(rot2, inv_inertia2, r2, p);

            #[cfg(feature = "3d")]
            {
                // See comments for `body1` above.
                body2.rotation.0 = body2.rotation.0.normalize();
            }
        }

        p
    }

    /// Computes the generalized inverse mass of a body when applying a positional correction
    /// at point `r` along the vector `n`.
    #[cfg(feature = "2d")]
    fn compute_generalized_inverse_mass(
        &self,
        body: &RigidBodyQueryItem,
        r: Vector,
        n: Vector,
    ) -> Scalar {
        if body.rb.is_dynamic() {
            body.inverse_mass.0 + body.inverse_inertia.0 * r.perp_dot(n).powi(2)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    /// Computes the generalized inverse mass of a body when applying a positional correction
    /// at point `r` along the vector `n`.
    #[cfg(feature = "3d")]
    fn compute_generalized_inverse_mass(
        &self,
        body: &RigidBodyQueryItem,
        r: Vector,
        n: Vector,
    ) -> Scalar {
        if body.rb.is_dynamic() {
            let inverse_inertia = body.effective_world_inv_inertia();

            let r_cross_n = r.cross(n); // Compute the cross product only once

            // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
            // a^T * b = a • b
            body.inverse_mass.0 + r_cross_n.dot(inverse_inertia * r_cross_n)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    /// Computes the update in rotation when applying a positional correction `p` at point `r`.
    #[cfg(feature = "2d")]
    fn get_delta_rot(_rot: Rotation, inverse_inertia: Scalar, r: Vector, p: Vector) -> Rotation {
        // Equation 8/9 but in 2D
        Rotation::from_radians(inverse_inertia * r.perp_dot(p))
    }

    /// Computes the update in rotation when applying a positional correction `p` at point `r`.
    #[cfg(feature = "3d")]
    fn get_delta_rot(rot: Rotation, inverse_inertia: Matrix3, r: Vector, p: Vector) -> Rotation {
        // Equation 8/9
        Rotation(Quaternion::from_vec4(0.5 * (inverse_inertia * r.cross(p)).extend(0.0)) * rot.0)
    }

    /// Computes the force acting along the constraint using the equation f = lambda * n / h^2
    fn compute_force(&self, lagrange: Scalar, direction: Vector, dt: Scalar) -> Vector {
        lagrange * direction / dt.powi(2)
    }
}
