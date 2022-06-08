use crate::DELTA_TIME;

use bevy::prelude::*;

/// Amount of substeps used in XPBD simulation
pub struct XPBDSubsteps(pub u32);

impl Default for XPBDSubsteps {
    fn default() -> Self {
        Self(8)
    }
}

/// Substep delta time
pub(crate) struct SubDeltaTime(pub f32);

impl Default for SubDeltaTime {
    fn default() -> Self {
        Self(DELTA_TIME / XPBDSubsteps::default().0 as f32)
    }
}

#[derive(Debug)]
pub struct Gravity(pub Vec2);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec2::new(0.0, -9.81))
    }
}

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
}

/// Stores penetration constraints for potentially colliding dynamic-dynamic entity pairs.
#[derive(Debug, Default)]
pub(crate) struct DynamicPenetrationConstraints(pub Vec<PenetrationConstraint>);

/// Stores penetration constraints for potentially colliding dynamic-static entity pairs.
#[derive(Debug, Default)]
pub(crate) struct StaticPenetrationConstraints(pub Vec<PenetrationConstraint>);

/// Stores dynamic-dynamic contact data.
#[derive(Default, Debug)]
pub struct DynamicContacts(pub Vec<Contact>);

/// Stores dynamic-static contact data.
#[derive(Default, Debug)]
pub struct StaticContacts(pub Vec<Contact>);

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    /// Local contact point a
    pub r_a: Vec2,
    /// Local contact point b
    pub r_b: Vec2,
    /// Contact normal from contact point a to b
    pub normal: Vec2,
    /// Penetration depth
    pub penetration: f32,
}
