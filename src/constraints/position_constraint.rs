use crate::prelude::*;

/// Positional constraints apply a position correction with a given direction and magnitude at the local contact points `r1` and  `r2`.
///
/// The constraint functions are based on equations 2-9 in the paper [Detailed Rigid Body Simulation with Extended Position Based Dynamics](https://matthias-research.github.io/pages/publications/PBDBodies.pdf).
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

        let inv_inertia1 = body1.world_inv_inertia().0;
        let inv_inertia2 = body2.world_inv_inertia().0;

        // Apply positional and rotational updates
        if body1.rb.is_dynamic() {
            body1.position.0 += p * body1.inverse_mass.0;
            *body1.rotation += Self::get_delta_rot(rot1, inv_inertia1, r1, p);
        }
        if body2.rb.is_dynamic() {
            body2.position.0 -= p * body2.inverse_mass.0;
            *body2.rotation -= Self::get_delta_rot(rot2, inv_inertia2, r2, p);
        }

        p
    }

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

    #[cfg(feature = "3d")]
    fn compute_generalized_inverse_mass(
        &self,
        body: &RigidBodyQueryItem,
        r: Vector,
        n: Vector,
    ) -> Scalar {
        if body.rb.is_dynamic() {
            let inverse_inertia = body.world_inv_inertia().0;

            let r_cross_n = r.cross(n); // Compute the cross product only once

            // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
            // a^T * b = a • b
            body.inverse_mass.0 + r_cross_n.dot(inverse_inertia * r_cross_n)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    #[cfg(feature = "2d")]
    fn get_delta_rot(_rot: Rotation, inverse_inertia: Scalar, r: Vector, p: Vector) -> Rotation {
        // Equation 8/9 but in 2D
        Rotation::from_radians(inverse_inertia * r.perp_dot(p))
    }

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
