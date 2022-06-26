use super::{Constraint, PositionConstraint};
use crate::{components::*, resources::Contact, Vector};

use bevy::prelude::*;

/// A constraint between two bodies that prevents overlap with a given compliance.
///
/// A compliance of 0.0 resembles a constraint with infinite stiffness, so the bodies should not have any overlap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PenetrationConstraint {
    /// Entity "a" in the constraint.
    pub entity_a: Entity,
    /// Entity "b" in the constraint.
    pub entity_b: Entity,
    pub contact_data: Contact,
    /// Lagrange multiplier for the normal force
    pub normal_lagrange: f32,
    /// Lagrange multiplier for the tangential force
    pub tangential_lagrange: f32,
    /// The constraint's compliance, the inverse of stiffness, has the unit meters / Newton
    pub compliance: f32,
    /// Normal force acting along this constraint
    pub normal_force: Vector,
}

impl PenetrationConstraint {
    pub fn new(entity_a: Entity, entity_b: Entity, contact_data: Contact) -> Self {
        Self {
            entity_a,
            entity_b,
            contact_data,
            normal_lagrange: 0.0,
            tangential_lagrange: 0.0,
            compliance: 0.0,
            normal_force: Vector::ZERO,
        }
    }

    /// Solves overlap between two bodies according to their masses
    #[allow(clippy::too_many_arguments)]
    pub fn constrain(
        &mut self,
        body1: &mut ConstraintBodyQueryItem,
        body2: &mut ConstraintBodyQueryItem,
        friction_a: &Friction,
        friction_b: &Friction,
        sub_dt: f32,
    ) {
        let p_a = body1.pos.0 + body1.rot.rotate(self.contact_data.local_r_a);
        let p_b = body2.pos.0 + body2.rot.rotate(self.contact_data.local_r_b);

        let d = (p_a - p_b).dot(self.contact_data.normal);

        if d > 0.0 {
            let delta_lagrange_n = Self::get_delta_pos_lagrange(
                body1,
                body2,
                self.normal_lagrange,
                self.contact_data.normal,
                d,
                self.contact_data.world_r_a,
                self.contact_data.world_r_b,
                self.compliance,
                sub_dt,
            );

            self.normal_lagrange += delta_lagrange_n;

            Self::apply_pos_constraint(
                body1,
                body2,
                delta_lagrange_n,
                self.contact_data.normal,
                self.contact_data.world_r_a,
                self.contact_data.world_r_b,
            );

            let p_a = body1.pos.0 + body1.rot.rotate(self.contact_data.local_r_a);
            let p_b = body2.pos.0 + body2.rot.rotate(self.contact_data.local_r_b);

            let delta_lagrange_t = Self::get_delta_pos_lagrange(
                body1,
                body2,
                self.tangential_lagrange,
                self.contact_data.normal,
                d,
                self.contact_data.world_r_a,
                self.contact_data.world_r_b,
                self.compliance,
                sub_dt,
            );

            let static_friction_coefficient =
                (friction_a.static_coefficient + friction_b.static_coefficient) * 0.5;

            let lagrange_n = self.normal_lagrange;
            let lagrange_t = self.tangential_lagrange + delta_lagrange_t;

            if lagrange_t > static_friction_coefficient * lagrange_n {
                let prev_p_a =
                    body1.prev_pos.0 + body1.prev_rot.rotate(self.contact_data.local_r_a);
                let prev_p_b =
                    body2.prev_pos.0 + body2.prev_rot.rotate(self.contact_data.local_r_b);
                let delta_p = (p_a - prev_p_a) - (p_b - prev_p_b);
                let delta_p_t =
                    delta_p - delta_p.dot(self.contact_data.normal) * self.contact_data.normal;

                self.tangential_lagrange += delta_lagrange_t;

                Self::apply_pos_constraint(
                    body1,
                    body2,
                    delta_lagrange_t,
                    delta_p_t.normalize_or_zero(),
                    self.contact_data.world_r_a,
                    self.contact_data.world_r_b,
                );
            }

            self.update_normal_force(self.contact_data.normal, sub_dt);
        }
    }

    fn update_normal_force(&mut self, normal: Vector, sub_dt: f32) {
        // Equation 10
        self.normal_force = self.normal_lagrange * normal / sub_dt.powi(2);
    }
}

impl Constraint for PenetrationConstraint {
    fn clear_lagrange_multipliers(&mut self) {
        self.normal_lagrange = 0.0;
        self.tangential_lagrange = 0.0;
    }
}

impl PositionConstraint for PenetrationConstraint {}
