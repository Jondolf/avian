use super::PositionConstraint;
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
    /// Lagrange multiplier for the normal force
    pub normal_lagrange: f32,
    /// Lagrange multiplier for the tangential force
    pub tangential_lagrange: f32,
    /// The constraint's compliance, the inverse of stiffness
    pub compliance: f32,
    /// Normal force acting along this constraint
    pub normal_force: Vector,
}

impl PenetrationConstraint {
    pub fn new_with_compliance(entity_a: Entity, entity_b: Entity, compliance: f32) -> Self {
        Self {
            entity_a,
            entity_b,
            normal_lagrange: 0.0,
            tangential_lagrange: 0.0,
            compliance,
            normal_force: Vector::ZERO,
        }
    }

    /// Solves overlap between two bodies according to their masses
    #[allow(clippy::too_many_arguments)]
    pub fn constrain(
        &mut self,
        rb_a: RigidBody,
        rb_b: RigidBody,
        pos_a: &mut Pos,
        pos_b: &mut Pos,
        rot_a: &mut Rot,
        rot_b: &mut Rot,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        contact: Contact,
        sub_dt: f32,
    ) {
        // Initialize Lagrange multipliers for the normal and tangential forces with zero
        self.normal_lagrange = 0.0;
        self.tangential_lagrange = 0.0;

        // Apply position correction with direction contact.normal and magnitude contact.penetration
        Self::_constrain_pos(
            rb_a,
            rb_b,
            pos_a,
            pos_b,
            rot_a,
            rot_b,
            &mass_props_a.with_rotation(*rot_a),
            &mass_props_b.with_rotation(*rot_b),
            contact.normal,
            contact.penetration,
            contact.r_a,
            contact.r_b,
            &mut self.normal_lagrange,
            self.compliance,
            sub_dt,
        );

        self.update_normal_force(contact.normal, sub_dt);
    }

    fn update_normal_force(&mut self, normal: Vector, sub_dt: f32) {
        // Equation 10
        self.normal_force = self.normal_lagrange * normal / sub_dt.powi(2);
    }
}

impl PositionConstraint for PenetrationConstraint {}
