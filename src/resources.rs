use crate::{constraints::penetration::PenetrationConstraint, Vector, DELTA_TIME};

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
pub struct Gravity(pub Vector);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vector::Y * -9.81)
    }
}

/// Stores penetration constraints for potentially colliding entity pairs.
#[derive(Debug, Default)]
pub(crate) struct PenetrationConstraints(pub Vec<PenetrationConstraint>);

/// Stores contact data and Lagrange multipliers for the normal forces.
#[derive(Default, Debug)]
pub struct Contacts(pub Vec<(Contact, f32)>);

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    /// Local contact point a
    pub r_a: Vector,
    /// Local contact point b
    pub r_b: Vector,
    /// Contact normal from contact point a to b
    pub normal: Vector,
    /// Penetration depth
    pub penetration: f32,
}
