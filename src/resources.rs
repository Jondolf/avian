use crate::{constraints::penetration::PenetrationConstraint, Vector, DELTA_TIME};

use bevy::prelude::*;

/// Number of substeps used in XPBD simulation
pub struct NumSubsteps(pub u32);

impl Default for NumSubsteps {
    fn default() -> Self {
        Self(8)
    }
}

/// Number of iterations used in XPBD position solving
pub struct NumPosIters(pub u32);

impl Default for NumPosIters {
    fn default() -> Self {
        Self(4)
    }
}

/// Substep delta time
pub(crate) struct SubDeltaTime(pub f32);

impl Default for SubDeltaTime {
    fn default() -> Self {
        Self(DELTA_TIME / NumSubsteps::default().0 as f32)
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

#[derive(Default, Debug)]
pub struct BroadCollisionPairs(pub Vec<(Entity, Entity)>);

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    /// Local contact point a in local coordinates
    pub local_r_a: Vector,
    /// Local contact point b in local coordinates
    pub local_r_b: Vector,
    /// Local contact point a in world coordinates
    pub world_r_a: Vector,
    /// Local contact point b in world coordinates
    pub world_r_b: Vector,
    /// Contact normal from contact point a to b
    pub normal: Vector,
    /// Penetration depth
    pub penetration: f32,
}
