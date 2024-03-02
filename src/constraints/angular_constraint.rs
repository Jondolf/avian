use crate::prelude::*;

/// An angular constraint applies an angular correction around a given axis.
pub trait AngularConstraint: XpbdConstraint<2> {
    /// Applies angular constraints for interactions between two bodies.
    ///
    /// Here in 2D, `axis` is a unit vector with the Z coordinate set to 1 or -1. It controls if the body should rotate counterclockwise or clockwise.
    ///
    /// Returns the angular impulse that is applied proportional to the inverse masses of the bodies.
    #[cfg(feature = "2d")]
    fn apply_angular_correction(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_lagrange: Scalar,
        axis: Vector3,
    ) -> Scalar {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return 0.0;
        }

        // Compute angular impulse
        // `axis.z` is 1 or -1 and it controls if the body should rotate counterclockwise or clockwise
        let p = -delta_lagrange * axis.z;

        let rot1 = *body1.rotation;
        let rot2 = *body2.rotation;

        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        // Apply rotational updates
        if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
            *body1.rotation += Self::get_delta_rot(rot1, inv_inertia1, p);
        }
        if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
            *body2.rotation -= Self::get_delta_rot(rot2, inv_inertia2, p);
        }

        p
    }

    /// Applies angular constraints for interactions between two bodies.
    ///
    /// Returns the angular impulse that is applied proportional to the inverse masses of the bodies.
    #[cfg(feature = "3d")]
    fn apply_angular_correction(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_lagrange: Scalar,
        axis: Vector,
    ) -> Vector {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return Vector::ZERO;
        }

        // Compute angular impulse
        let p = -delta_lagrange * axis;

        let rot1 = *body1.rotation;
        let rot2 = *body2.rotation;

        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        // Apply rotational updates
        if body1.rb.is_dynamic() {
            *body1.rotation += Self::get_delta_rot(rot1, inv_inertia1, p);

            // In 3D, subtracting quaternions like above can result in unnormalized rotations,
            // which causes stability issues (see #235) and panics when trying to rotate unit vectors.
            // TODO: It would be nice to avoid normalization if possible.
            //       Maybe the math above can be done in a way that keeps rotations normalized?
            body1.rotation.0 = body1.rotation.0.normalize();
        }
        if body2.rb.is_dynamic() {
            *body2.rotation -= Self::get_delta_rot(rot2, inv_inertia2, p);

            // See comments for `body1` above.
            body2.rotation.0 = body2.rotation.0.normalize();
        }

        p
    }

    /// Computes the generalized inverse mass of a body when applying an angular correction
    /// around `axis`.
    ///
    /// In 2D, `axis` should only have the z axis set to either -1 or 1 to indicate counterclockwise or
    /// clockwise rotation.
    #[cfg(feature = "2d")]
    fn compute_generalized_inverse_mass(&self, body: &RigidBodyQueryItem, axis: Vector3) -> Scalar {
        if body.rb.is_dynamic() {
            axis.dot(body.inverse_inertia.0 * axis)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    /// Computes the generalized inverse mass of a body when applying an angular correction
    /// around `axis`.
    #[cfg(feature = "3d")]
    fn compute_generalized_inverse_mass(&self, body: &RigidBodyQueryItem, axis: Vector) -> Scalar {
        if body.rb.is_dynamic() {
            axis.dot(body.effective_world_inv_inertia() * axis)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    /// Computes the update in rotation when applying an angular correction `p`.
    #[cfg(feature = "2d")]
    fn get_delta_rot(_rot: Rotation, inverse_inertia: Scalar, p: Scalar) -> Rotation {
        // Equation 8/9 but in 2D
        Rotation::from_radians(inverse_inertia * p)
    }

    /// Computes the update in rotation when applying an angular correction `p`.
    #[cfg(feature = "3d")]
    fn get_delta_rot(rot: Rotation, inverse_inertia: Matrix3, p: Vector) -> Rotation {
        // Equation 8/9
        Rotation(Quaternion::from_vec4(0.5 * (inverse_inertia * p).extend(0.0)) * rot.0)
    }

    /// Computes the torque acting along the constraint using the equation tau = lambda * n / h^2
    fn compute_torque(&self, lagrange: Scalar, axis: Vector3, dt: Scalar) -> Torque {
        // Eq (17)
        #[cfg(feature = "2d")]
        {
            lagrange * axis.z / dt.powi(2)
        }
        #[cfg(feature = "3d")]
        {
            lagrange * axis / dt.powi(2)
        }
    }
}
