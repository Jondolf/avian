pub mod joints;
pub mod penetration;

use crate::{components::*, Vector};

#[cfg(feature = "3d")]
use std::f32::consts::PI;

#[cfg(feature = "3d")]
use bevy::math::{Mat3, Quat};

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
        body1: &ConstraintBodyQueryItem,
        body2: &ConstraintBodyQueryItem,
        lagrange: f32,
        dir: Vector,
        magnitude: f32,
        r_a: Vector,
        r_b: Vector,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        // Compute generalized inverse masses (equations 2-3)
        let w_a = Self::get_generalized_inverse_mass(
            body1.mass_props.inv_mass,
            body1.mass_props.inv_inertia(&body1.rot),
            r_a,
            dir,
        );
        let w_b = Self::get_generalized_inverse_mass(
            body2.mass_props.inv_mass,
            body2.mass_props.inv_inertia(&body2.rot),
            r_b,
            dir,
        );

        let w = w_a + w_b;

        if w <= f32::EPSILON {
            return 0.0;
        }

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-magnitude - tilde_compliance * lagrange) / (w + tilde_compliance)
    }

    /// Applies position constraints for interactions between two bodies.
    ///
    /// Returns the positional impulse that is applied proportional to the inverse masses of the bodies.
    fn apply_pos_constraint(
        body1: &mut ConstraintBodyQueryItem,
        body2: &mut ConstraintBodyQueryItem,
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
        if *body1.rb != RigidBody::Static {
            body1.pos.0 += p / body1.mass_props.mass;
            *body1.rot += Self::get_delta_rot(rot_a, body1.mass_props.inv_inertia(&rot_a), r_a, p);
        }
        if *body2.rb != RigidBody::Static {
            body2.pos.0 -= p / body2.mass_props.mass;
            *body2.rot -= Self::get_delta_rot(rot_b, body2.mass_props.inv_inertia(&rot_b), r_b, p);
        }

        p
    }

    #[allow(clippy::too_many_arguments)]
    fn limit_distance(
        &mut self,
        min: f32,
        max: f32,
        r_a: Vector,
        r_b: Vector,
        pos_a: &Pos,
        pos_b: &Pos,
    ) -> Vector {
        let pos_offset = (pos_b.0 + r_b) - (pos_a.0 + r_a);
        let distance = pos_offset.length();

        if distance <= f32::EPSILON {
            return Vector::ZERO;
        }

        // Equation 25
        if distance < min {
            // Separation distance lower limit
            -pos_offset / distance * (distance - min)
        } else if distance > max {
            // Separation distance upper limit
            -pos_offset / distance * (distance - max)
        } else {
            Vector::ZERO
        }
    }

    #[cfg(feature = "2d")]
    fn get_generalized_inverse_mass(inv_mass: f32, inv_inertia: f32, r: Vector, n: Vector) -> f32 {
        inv_mass + inv_inertia * r.perp_dot(n).powi(2)
    }

    #[cfg(feature = "3d")]
    fn get_generalized_inverse_mass(inv_mass: f32, inv_inertia: Mat3, r: Vector, n: Vector) -> f32 {
        let r_cross_n = r.cross(n); // Compute the cross product only once

        // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
        // a^T * b = a • b
        inv_mass + r_cross_n.dot(inv_inertia * r_cross_n)
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
        body1: &ConstraintBodyQueryItem,
        body2: &ConstraintBodyQueryItem,
        lagrange: f32,
        axis: Vector,
        angle: f32,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        // Compute generalized inverse masses (equations 2-3)
        let w_a =
            Self::get_generalized_inverse_mass(body1.mass_props.inv_inertia(&body1.rot), axis);
        let w_b =
            Self::get_generalized_inverse_mass(body2.mass_props.inv_inertia(&body2.rot), axis);

        let w = w_a + w_b;

        if w <= f32::EPSILON {
            return 0.0;
        }

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-angle - tilde_compliance * lagrange) / (w + tilde_compliance)
    }

    /// Applies angular constraints for interactions between two bodies.
    ///
    /// Returns the angular impulse that is applied proportional to the inverse masses of the bodies.
    fn apply_ang_constraint(
        body1: &mut ConstraintBodyQueryItem,
        body2: &mut ConstraintBodyQueryItem,
        delta_lagrange: f32,
        axis: Vector,
    ) -> Vector {
        let p = delta_lagrange * axis;

        let rot_a = *body1.rot;
        let rot_b = *body2.rot;

        if *body1.rb != RigidBody::Static {
            *body1.rot += Self::get_delta_rot(rot_a, body1.mass_props.inv_inertia(&rot_a), p);
        }

        if *body2.rb != RigidBody::Static {
            *body2.rot -= Self::get_delta_rot(rot_b, body2.mass_props.inv_inertia(&rot_b), p);
        }

        p
    }

    #[cfg(feature = "3d")]
    fn limit_angle(
        n: Vector,
        n1: Vector,
        n2: Vector,
        alpha: f32,
        beta: f32,
        max_correction: f32,
    ) -> Option<Vector> {
        let mut phi = n1.cross(n2).dot(n).asin();

        if n1.dot(n2) < 0.0 {
            phi = PI - phi;
        }

        if phi > PI {
            phi -= 2.0 * PI;
        }

        if phi < -PI {
            phi += 2.0 * PI;
        }

        if phi < alpha || phi > beta {
            phi = phi.clamp(alpha, beta);

            let rot = Quat::from_axis_angle(n, phi);
            let mut omega = rot.mul_vec3(n1).cross(n2);

            phi = omega.length();

            if phi > max_correction {
                omega *= max_correction / phi;
            }

            return Some(omega);
        }

        None
    }

    #[cfg(feature = "2d")]
    fn get_generalized_inverse_mass(inv_inertia: f32, _axis: Vector) -> f32 {
        inv_inertia
    }

    #[cfg(feature = "3d")]
    fn get_generalized_inverse_mass(inv_inertia: Mat3, axis: Vector) -> f32 {
        axis.dot(inv_inertia * axis)
    }

    #[cfg(feature = "2d")]
    fn get_delta_rot(_rot: Rot, inv_inertia: f32, _p: Vector) -> Rot {
        // Equation 8/9 but in 2D
        Rot::from_radians(inv_inertia)
    }

    #[cfg(feature = "3d")]
    fn get_delta_rot(rot: Rot, inv_inertia: Mat3, p: Vector) -> Rot {
        // Equation 8/9
        Rot(Quat::from_vec4(0.5 * (inv_inertia * p).extend(0.0)) * rot.0)
    }
}
