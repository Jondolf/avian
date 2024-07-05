use super::XpbdConstraint;
use crate::prelude::*;

/// An angular constraint applies an angular correction around a given axis.
pub trait AngularConstraint: XpbdConstraint<2> {
    /// Applies an angular correction to two bodies.
    ///
    /// Returns the angular impulse that is applied proportional
    /// to the inverse moments of inertia of the bodies.
    #[cfg(feature = "2d")]
    fn apply_angular_lagrange_update(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_lagrange: Scalar,
    ) -> Scalar {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return 0.0;
        }

        self.apply_angular_impulse(body1, body2, -delta_lagrange)
    }

    /// Applies an angular impulse to two bodies.
    ///
    /// Returns the impulse that is applied proportional
    /// to the inverse moments of inertia of the bodies.
    #[cfg(feature = "2d")]
    fn apply_angular_impulse(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        impulse: Scalar,
    ) -> Scalar {
        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        // Apply rotational updates
        if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
            let delta_angle = Self::get_delta_rot(*body1.rotation, inv_inertia1, impulse);
            *body1.rotation = body1.rotation.add_angle(delta_angle);
        }
        if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
            let delta_angle = Self::get_delta_rot(*body2.rotation, inv_inertia2, -impulse);
            *body2.rotation = body2.rotation.add_angle(delta_angle);
        }

        impulse
    }

    /// Applies an angular correction to two bodies.
    ///
    /// Returns the angular impulse that is applied proportional
    /// to the inverse moments of inertia of the bodies.
    #[cfg(feature = "3d")]
    fn apply_angular_lagrange_update(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_lagrange: Scalar,
        axis: Vector,
    ) -> Vector {
        if delta_lagrange.abs() <= Scalar::EPSILON {
            return Vector::ZERO;
        }

        let impulse = -delta_lagrange * axis;

        self.apply_angular_impulse(body1, body2, impulse)
    }

    /// Applies an angular impulse to two bodies.
    ///
    /// Returns the impulse that is applied proportional
    /// to the inverse moments of inertia of the bodies.
    #[cfg(feature = "3d")]
    fn apply_angular_impulse(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        impulse: Vector,
    ) -> Vector {
        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        // Apply rotational updates
        if body1.rb.is_dynamic() {
            // In 3D, adding quaternions can result in unnormalized rotations,
            // which causes stability issues (see #235) and panics when trying to rotate unit vectors.
            // TODO: It would be nice to avoid normalization if possible.
            //       Maybe the math above can be done in a way that keeps rotations normalized?
            let delta_quat = Self::get_delta_rot(*body1.rotation, inv_inertia1, impulse);
            body1.rotation.0 = (body1.rotation.0 + delta_quat).normalize();
        }
        if body2.rb.is_dynamic() {
            // See comments for `body1` above.
            let delta_quat = Self::get_delta_rot(*body2.rotation, inv_inertia2, -impulse);
            body2.rotation.0 = (body2.rotation.0 + delta_quat).normalize();
        }

        impulse
    }

    /// Applies an angular correction that aligns the orientation of the bodies.
    ///
    /// Returns the torque exerted by the alignment.
    #[cfg(feature = "2d")]
    fn align_orientation(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        angle: Scalar,
        lagrange: &mut Scalar,
        compliance: Scalar,
        dt: Scalar,
    ) -> Torque {
        if angle.abs() <= Scalar::EPSILON {
            return Torque::ZERO;
        }

        let w1 = body1.effective_world_inv_inertia();
        let w2 = body2.effective_world_inv_inertia();
        let w = [w1, w2];

        // Compute Lagrange multiplier update
        let delta_lagrange = self.compute_lagrange_update(*lagrange, angle, &w, compliance, dt);
        *lagrange += delta_lagrange;

        // Apply angular correction to aling the bodies
        self.apply_angular_lagrange_update(body1, body2, delta_lagrange);

        // Return constraint torque
        self.compute_torque(delta_lagrange, dt)
    }

    /// Applies an angular correction that aligns the orientation of the bodies.
    ///
    /// Returns the torque exerted by the alignment.
    #[cfg(feature = "3d")]
    fn align_orientation(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        rotation_difference: Vector,
        lagrange: &mut Scalar,
        compliance: Scalar,
        dt: Scalar,
    ) -> Torque {
        let angle = rotation_difference.length();

        if angle <= Scalar::EPSILON {
            return Torque::ZERO;
        }

        let axis = rotation_difference / angle;

        // Compute generalized inverse masses
        let w1 = AngularConstraint::compute_generalized_inverse_mass(self, body1, axis);
        let w2 = AngularConstraint::compute_generalized_inverse_mass(self, body2, axis);
        let w = [w1, w2];

        // Compute Lagrange multiplier update
        let delta_lagrange = self.compute_lagrange_update(*lagrange, angle, &w, compliance, dt);
        *lagrange += delta_lagrange;

        // Apply angular correction to aling the bodies
        self.apply_angular_lagrange_update(body1, body2, delta_lagrange, axis);

        // Return constraint torque
        self.compute_torque(delta_lagrange, axis, dt)
    }

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

        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        // Apply rotational updates
        if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
            let delta_angle = Self::get_delta_rot(*body1.rotation, inv_inertia1, p);
            *body1.rotation = body1.rotation.add_angle(delta_angle);
        }
        if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
            let delta_angle = Self::get_delta_rot(*body2.rotation, inv_inertia2, -p);
            *body2.rotation = body2.rotation.add_angle(delta_angle);
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

        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        // Apply rotational updates
        if body1.rb.is_dynamic() {
            // In 3D, adding quaternions can result in unnormalized rotations,
            // which causes stability issues (see #235) and panics when trying to rotate unit vectors.
            // TODO: It would be nice to avoid normalization if possible.
            //       Maybe the math above can be done in a way that keeps rotations normalized?
            let delta_quat = Self::get_delta_rot(*body1.rotation, inv_inertia1, p);
            body1.rotation.0 = (body1.rotation.0 + delta_quat).normalize();
        }
        if body2.rb.is_dynamic() {
            // See comments for `body1` above.
            let delta_quat = Self::get_delta_rot(*body2.rotation, inv_inertia2, -p);
            body2.rotation.0 = (body2.rotation.0 + delta_quat).normalize();
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
    fn get_delta_rot(_rot: Rotation, inverse_inertia: Scalar, p: Scalar) -> Scalar {
        // Equation 8/9 but in 2D
        inverse_inertia * p
    }

    /// Computes the update in rotation when applying an angular correction `p`.
    #[cfg(feature = "3d")]
    fn get_delta_rot(rot: Rotation, inverse_inertia: Matrix3, p: Vector) -> Quaternion {
        // Equation 8/9
        Quaternion::from_vec4(0.5 * (inverse_inertia * p).extend(0.0)) * rot.0
    }

    /// Computes the torque acting along the constraint using the equation `tau = lambda * n / h^2`,
    /// where `n` is the Z axis in 2D.
    #[cfg(feature = "2d")]
    fn compute_torque(&self, lagrange: Scalar, dt: Scalar) -> Torque {
        // Eq (17)
        lagrange / dt.powi(2)
    }

    /// Computes the torque acting along the constraint using the equation `tau = lambda * n / h^2`
    #[cfg(feature = "3d")]
    fn compute_torque(&self, lagrange: Scalar, axis: Vector, dt: Scalar) -> Torque {
        // Eq (17)
        lagrange * axis / dt.powi(2)
    }
}
