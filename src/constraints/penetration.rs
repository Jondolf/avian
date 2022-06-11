use super::PositionConstraint;
use crate::{components::*, resources::Contact};

use bevy::prelude::*;

/// A constraint between two bodies that prevents overlap with a given compliance.
///
/// A compliance of 0.0 resembles a constraint with infinite stiffness, so the bodies should not have any overlap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PenetrationConstraint {
    /// Entity "a" in the constraint. This should always be a dynamic body.
    pub entity_a: Entity,
    /// Entity "b" in the constraint. This can be either a dynamic or a static body.
    pub entity_b: Entity,
    pub normal_lagrange: f32,
    /// The inverse of the constraint's stiffness
    pub compliance: f32,
    /// Normal force exerted by this constraint at dynamic body "a"
    pub normal_force: Vec2,
}

impl PenetrationConstraint {
    pub fn new_with_compliance(entity_a: Entity, entity_b: Entity, compliance: f32) -> Self {
        Self {
            entity_a,
            entity_b,
            normal_lagrange: 0.0,
            compliance,
            normal_force: Vec2::ZERO,
        }
    }

    /// Solves overlap between two dynamic bodies according to their masses.
    #[allow(clippy::too_many_arguments)]
    pub fn constrain_dynamic(
        &mut self,
        pos_a: &mut Pos,
        pos_b: &mut Pos,
        rot_a: &mut Rot,
        rot_b: &mut Rot,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        contact: Contact,
        sub_dt: f32,
    ) {
        Self::_constrain_pos_dynamic(
            pos_a,
            pos_b,
            rot_a,
            rot_b,
            mass_props_a,
            mass_props_b,
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

    /// Solves overlap between a dynamic body and a static body.
    #[allow(clippy::too_many_arguments)]
    pub fn constrain_static(
        &mut self,
        pos_a: &mut Pos,
        rot_a: &mut Rot,
        mass_props_a: &MassProperties,
        contact: Contact,
        sub_dt: f32,
    ) {
        Self::_constrain_pos_static(
            pos_a,
            rot_a,
            mass_props_a,
            contact.normal,
            contact.penetration,
            contact.r_a,
            &mut self.normal_lagrange,
            self.compliance,
            sub_dt,
        );

        self.update_normal_force(contact.normal, sub_dt);
    }

    fn update_normal_force(&mut self, normal: Vec2, sub_dt: f32) {
        // Equation 10
        self.normal_force = self.normal_lagrange * normal / sub_dt.powi(2);
    }
}

impl PositionConstraint for PenetrationConstraint {}
