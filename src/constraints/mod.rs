pub mod penetration;

use crate::{components::*, Vector};

#[cfg(feature = "3d")]
use bevy::math::{Mat3, Quat};

/// Positional constraints for dynamic-dynamic and dynamic-static interactions.
///
/// The constraints apply a position correction with a given direction and magnitude at the local contact points `r_a` and  `r_b`.
///
/// The constraint functions are based on equations 2-9 in the paper [Detailed Rigid Body Simulation with Extended Position Based Dynamics](https://matthias-research.github.io/pages/publications/PBDBodies.pdf).
pub trait PositionConstraint {
    /// Applies position constraints for interactions between two dynamic bodies.
    ///
    /// Returns the change of the Lagrange multiplier.
    #[allow(clippy::too_many_arguments)]
    fn _constrain_pos(
        rb_a: RigidBody,
        rb_b: RigidBody,
        pos_a: &mut Pos,
        pos_b: &mut Pos,
        rot_a: &mut Rot,
        rot_b: &mut Rot,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        dir: Vector,
        magnitude: f32,
        r_a: Vector,
        r_b: Vector,
        lagrange: &mut f32,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        let delta_lagrange = Self::get_delta_lagrange(
            lagrange,
            magnitude * dir,
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
        if rb_a != RigidBody::Static {
            pos_a.0 += p / mass_props_a.mass;
            *rot_a = Self::get_new_rot(*rot_a, mass_props_a.inv_inertia, r_a, p);
        }
        if rb_b != RigidBody::Static {
            pos_b.0 -= p / mass_props_b.mass;
            *rot_b = Self::get_new_rot(*rot_b, mass_props_b.inv_inertia, r_b, -p);
        }

        delta_lagrange
    }

    #[allow(clippy::too_many_arguments)]
    fn get_delta_lagrange(
        lagrange: &mut f32,
        delta_x: Vector,
        r_a: Vector,
        r_b: Vector,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        let c = delta_x.length();

        // Avoid division by zero when normalizing the vector later.
        // We compare against epsilon to avoid potential floating point precision problems.
        if c <= f32::EPSILON {
            return 0.0;
        }

        let n = delta_x / c;

        // Compute generalized inverse masses (equations 2-3)
        let w_a = Self::get_generalized_inverse_mass(mass_props_a, r_a, n);
        let w_b = Self::get_generalized_inverse_mass(mass_props_b, r_b, n);

        // Compute Lagrange multiplier updates (equations 4-5)
        let tilde_compliance = compliance / sub_dt.powi(2);

        (-c - tilde_compliance * *lagrange) / (w_a + w_b + tilde_compliance)
    }

    #[cfg(feature = "2d")]
    fn get_generalized_inverse_mass(mass_props: &MassProperties, r: Vector, dir: Vector) -> f32 {
        mass_props.inv_mass + mass_props.inv_inertia * r.perp_dot(dir).powi(2)
    }

    #[cfg(feature = "3d")]
    fn get_generalized_inverse_mass(mass_props: &MassProperties, r: Vector, dir: Vector) -> f32 {
        mass_props.inv_mass + r.cross(dir).dot(mass_props.inv_inertia * r.cross(dir))
    }

    #[cfg(feature = "2d")]
    fn get_new_rot(rot: Rot, inv_inertia: f32, r: Vector, p: Vector) -> Rot {
        // Equation 8/9 but in 2D
        rot + Rot::from_radians(inv_inertia * r.perp_dot(p))
    }

    #[cfg(feature = "3d")]
    fn get_new_rot(rot: Rot, inv_inertia: Mat3, r: Vector, p: Vector) -> Rot {
        // Equation 8/9
        rot + Rot(Quat::from_vec4(0.5 * (inv_inertia * r.cross(p)).extend(0.0)) * rot.0)
    }
}
