//! Collision events, contact data and helpers.

use crate::prelude::*;

/// A [collision event](Collider#collision-events) that is sent for each contact pair during the narrow phase.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct Collision(pub Contact);

/// A [collision event](Collider#collision-events) that is sent when two entities start colliding.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct CollisionStarted(pub Entity, pub Entity);

/// A [collision event](Collider#collision-events) that is sent when two entities stop colliding.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct CollisionEnded(pub Entity, pub Entity);

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    /// First entity in the contact.
    pub entity1: Entity,
    /// Second entity in the contact.
    pub entity2: Entity,
    /// Contact point on the first entity in global coordinates.
    pub point1: Vector,
    /// Contact point on the second entity in global coordinates.
    pub point2: Vector,
    /// Contact normal from contact point 1 to 2.
    pub normal: Vector,
    /// Penetration depth.
    pub penetration: Scalar,
}

/// Computes one pair of contact points between two shapes.
#[allow(clippy::too_many_arguments)]
pub(crate) fn compute_contact(
    entity1: Entity,
    entity2: Entity,
    position1: Vector,
    position2: Vector,
    rotation1: &Rotation,
    rotation2: &Rotation,
    collider1: &Collider,
    collider2: &Collider,
) -> Option<Contact> {
    if let Ok(Some(contact)) = parry::query::contact(
        &utils::make_isometry(position1, rotation1),
        collider1.get_shape().0.as_ref(),
        &utils::make_isometry(position2, rotation2),
        collider2.get_shape().0.as_ref(),
        0.0,
    ) {
        return Some(Contact {
            entity1,
            entity2,
            point1: contact.point1.into(),
            point2: contact.point2.into(),
            normal: contact.normal1.into(),
            penetration: -contact.dist,
        });
    }
    None
}
