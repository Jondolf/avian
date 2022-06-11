pub mod penetration;

use crate::components::*;

use bevy::prelude::*;

pub trait PositionConstraint {
    /// Applies position constraints for interactions between two dynamic bodies.
    ///
    /// Returns the change of the Lagrange multiplier.
    #[allow(clippy::too_many_arguments)]
    fn _constrain_pos_dynamic(
        pos_a: &mut Pos,
        pos_b: &mut Pos,
        rot_a: &mut Rot,
        rot_b: &mut Rot,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        dir: Vec2,
        magnitude: f32,
        r_a: Vec2,
        r_b: Vec2,
        lagrange: &mut f32,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        // Compute generalized inverse masses (equations 2-3)
        let w_a = mass_props_a.inv_mass + mass_props_a.inv_inertia * r_a.perp_dot(dir).powi(2);
        let w_b = mass_props_b.inv_mass + mass_props_b.inv_inertia * r_b.perp_dot(dir).powi(2);

        // Compute Lagrange multiplier updates (equations 4-5)
        let a = compliance / sub_dt.powi(2);
        let delta_lagrange = (-magnitude - a * *lagrange) / (w_a + w_b + a);
        *lagrange += delta_lagrange;

        // Positional impulse
        let p = delta_lagrange * dir;

        // Update positions and rotations of the bodies (equations 6-9)
        pos_a.0 += p / mass_props_a.mass;
        pos_b.0 -= p / mass_props_b.mass;
        *rot_a += Rot::from_radians(mass_props_a.inv_inertia * r_a.perp_dot(p));
        *rot_b += Rot::from_radians(mass_props_b.inv_inertia * r_b.perp_dot(-p));

        delta_lagrange
    }

    /// Applies position constraints for interactions between a dynamic body "a" and a static body "b".
    ///
    /// Returns the change of the Lagrange multiplier.
    #[allow(clippy::too_many_arguments)]
    fn _constrain_pos_static(
        pos_a: &mut Pos,
        rot_a: &mut Rot,
        mass_props_a: &MassProperties,
        dir: Vec2,
        magnitude: f32,
        r_a: Vec2,
        lagrange: &mut f32,
        compliance: f32,
        sub_dt: f32,
    ) -> f32 {
        // Compute generalized inverse mass (equation 2)
        let w_a = mass_props_a.inv_mass + mass_props_a.inv_inertia * r_a.perp_dot(dir).powi(2);

        // Compute Lagrange multiplier updates (equations 4-5)
        let a = compliance / sub_dt.powi(2);
        let delta_lagrange = (-magnitude - a * *lagrange) / (w_a + a);
        *lagrange += delta_lagrange;

        // Positional impulse
        let p = delta_lagrange * dir;

        // Update position and rotation of the dynamic body (equations 6 and 8)
        pos_a.0 += p / mass_props_a.mass;
        *rot_a += Rot::from_radians(mass_props_a.inv_inertia * r_a.perp_dot(p));

        delta_lagrange
    }
}
