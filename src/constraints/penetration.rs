use super::PositionConstraint;
use crate::{components::*, resources::Contact};

use bevy::prelude::*;

/// A constraint between two bodies that prevents overlap with a given compliance.
///
/// A compliance of 0.0 resembles a constraint with infinite stiffness, so the bodies should not have any overlap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PenetrationConstraint {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub lagrange: f32,
    pub compliance: f32,
}

impl PenetrationConstraint {
    pub fn new_with_compliance(entity_a: Entity, entity_b: Entity, compliance: f32) -> Self {
        Self {
            entity_a,
            entity_b,
            lagrange: 0.0,
            compliance,
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
        prev_pos_a: &PrevPos,
        prev_pos_b: &PrevPos,
        friction_a: &Friction,
        friction_b: &Friction,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        contact: Contact,
        sub_dt: f32,
    ) {
        let delta_lagrange = Self::_constrain_pos_dynamic(
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
            &mut self.lagrange,
            self.compliance,
            sub_dt,
        );

        // Compute static friction
        let friction = Self::get_static_friction(
            pos_a.0 - prev_pos_a.0,
            pos_b.0 - prev_pos_b.0,
            friction_a,
            friction_b,
            contact.normal,
            delta_lagrange,
        );
        // Apply static friction
        pos_a.0 -= friction;
        pos_b.0 += friction;
    }

    /// Solves overlap between a dynamic body and a static body.
    #[allow(clippy::too_many_arguments)]
    pub fn constrain_static(
        &mut self,
        pos_a: &mut Pos,
        rot_a: &mut Rot,
        prev_pos_a: &PrevPos,
        friction_a: &Friction,
        friction_b: &Friction,
        mass_props_a: &MassProperties,
        contact: Contact,
        sub_dt: f32,
    ) {
        let delta_lagrange = Self::_constrain_pos_static(
            pos_a,
            rot_a,
            mass_props_a,
            contact.normal,
            contact.penetration,
            contact.r_a,
            &mut self.lagrange,
            self.compliance,
            sub_dt,
        );

        // Compute and apply static friction
        pos_a.0 -= Self::get_static_friction(
            pos_a.0 - prev_pos_a.0,
            Vec2::ZERO,
            friction_a,
            friction_b,
            contact.normal,
            delta_lagrange,
        );
    }

    fn get_static_friction(
        delta_pos_a: Vec2,
        delta_pos_b: Vec2,
        friction_a: &Friction,
        friction_b: &Friction,
        contact_normal: Vec2,
        normal_force: f32,
    ) -> Vec2 {
        let static_friction_coefficient =
            (friction_a.static_coefficient + friction_b.static_coefficient) * 0.5;
        let relative_mov = delta_pos_a - delta_pos_b;
        let tangential_mov = relative_mov - (relative_mov.dot(contact_normal)) * contact_normal;
        if tangential_mov.length() < normal_force * static_friction_coefficient {
            tangential_mov
        } else {
            Vec2::ZERO
        }
    }
}

impl PositionConstraint for PenetrationConstraint {}
