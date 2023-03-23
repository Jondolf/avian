pub mod joints;
pub mod penetration;

use crate::prelude::*;

pub trait Constraint {
    fn clear_lagrange_multipliers(&mut self);
}

/// Positional constraints apply a position correction with a given direction and magnitude at the local contact points `r1` and  `r2`.
///
/// The constraint functions are based on equations 2-9 in the paper [Detailed Rigid Body Simulation with Extended Position Based Dynamics](https://matthias-research.github.io/pages/publications/PBDBodies.pdf).
pub trait PositionConstraint: Constraint {
    /// Computes the change of a given Lagrange multiplier when a positional correction is applied to two bodies.
    #[allow(clippy::too_many_arguments)]
    fn get_delta_pos_lagrange(
        body1: &RigidBodyQueryItem,
        body2: &RigidBodyQueryItem,
        inv_inertia1: InvInertia,
        inv_inertia2: InvInertia,
        lagrange: Scalar,
        dir: Vector,
        magnitude: Scalar,
        r1: Vector,
        r2: Vector,
        compliance: Scalar,
        sub_dt: Scalar,
    ) -> Scalar {
        // If both bodies are not dynamic and compliance is 0, we return 0.0 early to avoid division by zero later.
        if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() && compliance <= Scalar::EPSILON {
            return 0.0;
        }

        // Compute generalized inverse masses (equations 2-3)
        let w1 =
            Self::get_generalized_inverse_mass(body1.rb, body1.inv_mass.0, inv_inertia1.0, r1, dir);
        let w2 =
            Self::get_generalized_inverse_mass(body2.rb, body2.inv_mass.0, inv_inertia2.0, r2, dir);

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-magnitude - tilde_compliance * lagrange) / (w1 + w2 + tilde_compliance)
    }

    /// Applies position constraints for interactions between two bodies.
    ///
    /// Returns the positional impulse that is applied proportional to the inverse masses of the bodies.
    #[allow(clippy::too_many_arguments)]
    fn apply_pos_constraint(
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        inv_inertia1: InvInertia,
        inv_inertia2: InvInertia,
        delta_lagrange: Scalar,
        dir: Vector,
        r1: Vector,
        r2: Vector,
    ) -> Vector {
        // Positional impulse
        let p = delta_lagrange * dir;

        let rot1 = *body1.rot;
        let rot2 = *body2.rot;

        // Update positions and rotations of the bodies (equations 6-9)
        if body1.rb.is_dynamic() {
            body1.pos.0 += p / body1.mass.0;
            *body1.rot += Self::get_delta_rot(rot1, inv_inertia1.0, r1, p);
        }
        if body2.rb.is_dynamic() {
            body2.pos.0 -= p / body2.mass.0;
            *body2.rot -= Self::get_delta_rot(rot2, inv_inertia2.0, r2, p);
        }

        p
    }

    #[cfg(feature = "2d")]
    fn get_generalized_inverse_mass(
        rb: &RigidBody,
        inv_mass: Scalar,
        inv_inertia: Scalar,
        r: Vector,
        n: Vector,
    ) -> Scalar {
        if rb.is_dynamic() {
            inv_mass + inv_inertia * r.perp_dot(n).powi(2)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    #[cfg(feature = "3d")]
    fn get_generalized_inverse_mass(
        rb: &RigidBody,
        inv_mass: Scalar,
        inv_inertia: Matrix3,
        r: Vector,
        n: Vector,
    ) -> Scalar {
        if rb.is_dynamic() {
            let r_cross_n = r.cross(n); // Compute the cross product only once

            // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
            // a^T * b = a • b
            inv_mass + r_cross_n.dot(inv_inertia * r_cross_n)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    #[cfg(feature = "2d")]
    fn get_delta_rot(_rot: Rot, inv_inertia: Scalar, r: Vector, p: Vector) -> Rot {
        // Equation 8/9 but in 2D
        Rot::from_radians(inv_inertia * r.perp_dot(p))
    }

    #[cfg(feature = "3d")]
    fn get_delta_rot(rot: Rot, inv_inertia: Matrix3, r: Vector, p: Vector) -> Rot {
        // Equation 8/9
        Rot(Quaternion::from_vec4(0.5 * (inv_inertia * r.cross(p)).extend(0.0)) * rot.0)
    }
}

pub trait AngularConstraint: Constraint {
    #[allow(clippy::too_many_arguments)]
    fn get_delta_ang_lagrange(
        rb1: &RigidBody,
        rb2: &RigidBody,
        inv_inertia1: InvInertia,
        inv_inertia2: InvInertia,
        lagrange: Scalar,
        axis: Vector3,
        angle: Scalar,
        compliance: Scalar,
        sub_dt: Scalar,
    ) -> Scalar {
        // If both bodies are not dynamic and compliance is 0, we return 0.0 early to avoid division by zero later.
        if !rb1.is_dynamic() && !rb2.is_dynamic() && compliance <= Scalar::EPSILON {
            return 0.0;
        }

        // Compute generalized inverse masses (equations 2-3)
        let w1 = Self::get_generalized_inverse_mass(rb1, inv_inertia1.0, axis);
        let w2 = Self::get_generalized_inverse_mass(rb2, inv_inertia2.0, axis);

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-angle - tilde_compliance * lagrange) / (w1 + w2 + tilde_compliance)
    }

    /// Applies angular constraints for interactions between two bodies.
    ///
    /// Here in 2D, `axis` is a unit vector with the Z coordinate set to 1 or -1. It controls if the body should rotate counterclockwise or clockwise.
    ///
    /// Returns the angular impulse that is applied proportional to the inverse masses of the bodies.
    #[cfg(feature = "2d")]
    fn apply_ang_constraint(
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        inv_inertia1: InvInertia,
        inv_inertia2: InvInertia,
        delta_lagrange: Scalar,
        axis: Vector3,
    ) -> Scalar {
        // `axis.z` is 1 or -1 and it controls if the body should rotate counterclockwise or clockwise
        let p = -delta_lagrange * axis.z;

        let rot1 = *body1.rot;
        let rot2 = *body2.rot;

        if body1.rb.is_dynamic() {
            *body1.rot += Self::get_delta_rot(rot1, inv_inertia1.0, p);
        }

        if body2.rb.is_dynamic() {
            *body2.rot -= Self::get_delta_rot(rot2, inv_inertia2.0, p);
        }

        p
    }

    /// Applies angular constraints for interactions between two bodies.
    ///
    /// Returns the angular impulse that is applied proportional to the inverse masses of the bodies.
    #[cfg(feature = "3d")]
    fn apply_ang_constraint(
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        inv_inertia1: InvInertia,
        inv_inertia2: InvInertia,
        delta_lagrange: Scalar,
        axis: Vector,
    ) -> Vector {
        let p = -delta_lagrange * axis;

        let rot1 = *body1.rot;
        let rot2 = *body2.rot;

        if body1.rb.is_dynamic() {
            *body1.rot += Self::get_delta_rot(rot1, inv_inertia1.0, p);
        }

        if body2.rb.is_dynamic() {
            *body2.rot -= Self::get_delta_rot(rot2, inv_inertia2.0, p);
        }

        p
    }

    #[cfg(feature = "2d")]
    fn get_generalized_inverse_mass(rb: &RigidBody, inv_inertia: Scalar, axis: Vector3) -> Scalar {
        if rb.is_dynamic() {
            axis.dot(inv_inertia * axis)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    #[cfg(feature = "3d")]
    fn get_generalized_inverse_mass(rb: &RigidBody, inv_inertia: Matrix3, axis: Vector) -> Scalar {
        if rb.is_dynamic() {
            axis.dot(inv_inertia * axis)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    #[cfg(feature = "2d")]
    fn get_delta_rot(_rot: Rot, inv_inertia: Scalar, p: Scalar) -> Rot {
        // Equation 8/9 but in 2D
        Rot::from_radians(inv_inertia * p)
    }

    #[cfg(feature = "3d")]
    fn get_delta_rot(rot: Rot, inv_inertia: Matrix3, p: Vector) -> Rot {
        // Equation 8/9
        Rot(Quaternion::from_vec4(0.5 * (inv_inertia * p).extend(0.0)) * rot.0)
    }
}
