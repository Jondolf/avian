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
    /// Applies position constraints for interactions between two dynamic bodies.
    ///
    /// Returns the change of the Lagrange multiplier.
    #[allow(clippy::too_many_arguments)]
    fn _constrain_pos(
        delta_x: Vector,
        r_a: Vector,
        r_b: Vector,
        rb_a: &RigidBody,
        rb_b: &RigidBody,
        pos_a: &mut Pos,
        pos_b: &mut Pos,
        rot_a: &mut Rot,
        rot_b: &mut Rot,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        lagrange: &mut f32,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        let magnitude = delta_x.length();

        // Avoid division by zero when normalizing the vector later.
        // We compare against epsilon to avoid potential floating point precision problems.
        if magnitude <= f32::EPSILON {
            return 0.0;
        }

        let dir = delta_x / magnitude;

        let delta_lagrange = Self::get_delta_lagrange(
            *lagrange,
            dir,
            magnitude,
            r_a,
            r_b,
            mass_props_a,
            mass_props_b,
            compliance,
            sub_dt,
        );

        *lagrange += delta_lagrange;

        // Positional impulse
        let p = delta_lagrange * dir;

        // Update positions and rotations of the bodies (equations 6-9)
        if *rb_a != RigidBody::Static {
            pos_a.0 += p / mass_props_a.mass;
            *rot_a += Self::get_delta_rot(*rot_a, mass_props_a.inv_inertia, r_a, p);
        }
        if *rb_b != RigidBody::Static {
            pos_b.0 -= p / mass_props_b.mass;
            *rot_b -= Self::get_delta_rot(*rot_b, mass_props_b.inv_inertia, r_b, p);
        }

        delta_lagrange
    }

    #[allow(clippy::too_many_arguments)]
    fn limit_distance(
        &mut self,
        min: f32,
        max: f32,
        r_a: Vector,
        r_b: Vector,
        pos_a: &mut Pos,
        pos_b: &mut Pos,
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

    #[allow(clippy::too_many_arguments)]
    fn get_delta_lagrange(
        lagrange: f32,
        dir: Vector,
        magnitude: f32,
        r_a: Vector,
        r_b: Vector,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        // Compute generalized inverse masses (equations 2-3)
        let w_a = Self::get_generalized_inverse_mass(mass_props_a, r_a, dir);
        let w_b = Self::get_generalized_inverse_mass(mass_props_b, r_b, dir);

        let w = w_a + w_b;

        if w <= f32::EPSILON {
            return 0.0;
        }

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-magnitude - tilde_compliance * lagrange) / (w + tilde_compliance)
    }

    #[cfg(feature = "2d")]
    fn get_generalized_inverse_mass(mass_props: &MassProperties, r: Vector, n: Vector) -> f32 {
        mass_props.inv_mass + mass_props.inv_inertia * r.perp_dot(n).powi(2)
    }

    #[cfg(feature = "3d")]
    fn get_generalized_inverse_mass(mass_props: &MassProperties, r: Vector, n: Vector) -> f32 {
        let r_cross_n = r.cross(n); // Compute the cross product only once

        // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
        // a^T * b = a • b
        mass_props.inv_mass + r_cross_n.dot(mass_props.inv_inertia * r_cross_n)
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
    /// Applies angular constraints for two dynamic bodies.
    ///
    /// Returns the change of the Lagrange multiplier.
    #[allow(clippy::too_many_arguments)]
    fn _constrain_angle(
        delta_q: Vector,
        rb_a: &RigidBody,
        rb_b: &RigidBody,
        rot_a: &mut Rot,
        rot_b: &mut Rot,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        lagrange: &mut f32,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        let angle = delta_q.length();

        // Avoid division by zero when normalizing the vector later.
        // We compare against epsilon to avoid potential floating point precision problems.
        if angle <= f32::EPSILON {
            return 0.0;
        }

        let axis = delta_q / angle;

        let delta_lagrange = Self::get_delta_lagrange(
            *lagrange,
            axis,
            angle,
            mass_props_a,
            mass_props_b,
            compliance,
            sub_dt,
        );

        *lagrange += delta_lagrange;

        let p = delta_lagrange * axis;

        if *rb_a != RigidBody::Static {
            *rot_a += Self::get_delta_rot(*rot_a, mass_props_a.inv_inertia, p);
        }

        if *rb_b != RigidBody::Static {
            *rot_b -= Self::get_delta_rot(*rot_b, mass_props_b.inv_inertia, p);
        }

        delta_lagrange
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

    #[allow(clippy::too_many_arguments)]
    fn get_delta_lagrange(
        lagrange: f32,
        axis: Vector,
        angle: f32,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        // Compute generalized inverse masses (equations 2-3)
        let w_a = Self::get_generalized_inverse_mass(mass_props_a.inv_inertia, axis);
        let w_b = Self::get_generalized_inverse_mass(mass_props_b.inv_inertia, axis);

        let w = w_a + w_b;

        if w <= f32::EPSILON {
            return 0.0;
        }

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-angle - tilde_compliance * lagrange) / (w + tilde_compliance)
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
