pub mod joints;
pub mod penetration;

use crate::{components::*, Vector};
use bevy::prelude::*;

pub trait Constraint {
    fn clear_lagrange_multipliers(&mut self);
}

/// Positional constraints apply a position correction with a given direction and magnitude at the local contact points `r_a` and  `r_b`.
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
        lagrange: f32,
        dir: Vector,
        magnitude: f32,
        r_a: Vector,
        r_b: Vector,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        // If both bodies are not dynamic and compliance is 0, we return 0.0 early to avoid division by zero later.
        if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() && compliance <= f32::EPSILON {
            return 0.0;
        }

        // Compute generalized inverse masses (equations 2-3)
        let w_a = Self::get_generalized_inverse_mass(
            body1.rb,
            body1.inv_mass.0,
            inv_inertia1.0,
            r_a,
            dir,
        );
        let w_b = Self::get_generalized_inverse_mass(
            body2.rb,
            body2.inv_mass.0,
            inv_inertia2.0,
            r_b,
            dir,
        );

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-magnitude - tilde_compliance * lagrange) / (w_a + w_b + tilde_compliance)
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
        delta_lagrange: f32,
        dir: Vector,
        r_a: Vector,
        r_b: Vector,
    ) -> Vector {
        // Positional impulse
        let p = delta_lagrange * dir;

        let rot_a = *body1.rot;
        let rot_b = *body2.rot;

        // Update positions and rotations of the bodies (equations 6-9)
        if body1.rb.is_dynamic() {
            body1.pos.0 += p / body1.mass.0;
            *body1.rot += Self::get_delta_rot(rot_a, inv_inertia1.0, r_a, p);
        }
        if body2.rb.is_dynamic() {
            body2.pos.0 -= p / body2.mass.0;
            *body2.rot -= Self::get_delta_rot(rot_b, inv_inertia2.0, r_b, p);
        }

        p
    }

    #[cfg(feature = "2d")]
    fn get_generalized_inverse_mass(
        rb: &RigidBody,
        inv_mass: f32,
        inv_inertia: f32,
        r: Vector,
        n: Vector,
    ) -> f32 {
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
        inv_mass: f32,
        inv_inertia: Mat3,
        r: Vector,
        n: Vector,
    ) -> f32 {
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
    fn get_delta_rot(_rot: Rot, inv_inertia: f32, r: Vector, p: Vector) -> Rot {
        // Equation 8/9 but in 2D
        Rot::from_radians(inv_inertia * r.perp_dot(p))
    }

    #[cfg(feature = "3d")]
    fn get_delta_rot(rot: Rot, inv_inertia: Mat3, r: Vector, p: Vector) -> Rot {
        // Equation 8/9
        Rot(Quat::from_vec4(0.5 * (inv_inertia * r.cross(p)).extend(0.0)) * rot.0)
    }
}

pub trait AngularConstraint: Constraint {
    #[allow(clippy::too_many_arguments)]
    fn get_delta_ang_lagrange(
        rb1: &RigidBody,
        rb2: &RigidBody,
        inv_inertia1: InvInertia,
        inv_inertia2: InvInertia,
        lagrange: f32,
        axis: Vec3,
        angle: f32,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        // If both bodies are not dynamic and compliance is 0, we return 0.0 early to avoid division by zero later.
        if !rb1.is_dynamic() && !rb2.is_dynamic() && compliance <= f32::EPSILON {
            return 0.0;
        }

        // Compute generalized inverse masses (equations 2-3)
        let w_a = Self::get_generalized_inverse_mass(rb1, inv_inertia1.0, axis);
        let w_b = Self::get_generalized_inverse_mass(rb2, inv_inertia2.0, axis);

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-angle - tilde_compliance * lagrange) / (w_a + w_b + tilde_compliance)
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
        delta_lagrange: f32,
        axis: Vec3,
    ) -> f32 {
        // `axis.z` is 1 or -1 and it controls if the body should rotate counterclockwise or clockwise
        let p = -delta_lagrange * axis.z;

        let rot_a = *body1.rot;
        let rot_b = *body2.rot;

        if body1.rb.is_dynamic() {
            *body1.rot += Self::get_delta_rot(rot_a, inv_inertia1.0, p);
        }

        if body2.rb.is_dynamic() {
            *body2.rot -= Self::get_delta_rot(rot_b, inv_inertia2.0, p);
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
        delta_lagrange: f32,
        axis: Vec3,
    ) -> Vec3 {
        let p = -delta_lagrange * axis;

        let rot_a = *body1.rot;
        let rot_b = *body2.rot;

        if body1.rb.is_dynamic() {
            *body1.rot += Self::get_delta_rot(rot_a, inv_inertia1.0, p);
        }

        if body2.rb.is_dynamic() {
            *body2.rot -= Self::get_delta_rot(rot_b, inv_inertia2.0, p);
        }

        p
    }

    #[cfg(feature = "2d")]
    fn get_generalized_inverse_mass(rb: &RigidBody, inv_inertia: f32, axis: Vec3) -> f32 {
        if rb.is_dynamic() {
            axis.dot(inv_inertia * axis)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    #[cfg(feature = "3d")]
    fn get_generalized_inverse_mass(rb: &RigidBody, inv_inertia: Mat3, axis: Vec3) -> f32 {
        if rb.is_dynamic() {
            axis.dot(inv_inertia * axis)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }

    #[cfg(feature = "2d")]
    fn get_delta_rot(_rot: Rot, inv_inertia: f32, p: f32) -> Rot {
        // Equation 8/9 but in 2D
        Rot::from_radians(inv_inertia * p)
    }

    #[cfg(feature = "3d")]
    fn get_delta_rot(rot: Rot, inv_inertia: Mat3, p: Vec3) -> Rot {
        // Equation 8/9
        Rot(Quat::from_vec4(0.5 * (inv_inertia * p).extend(0.0)) * rot.0)
    }
}
