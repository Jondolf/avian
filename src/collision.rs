//! Collision events, contact data and helpers.

use parry::query::PersistentQueryDispatcher;

use crate::prelude::*;

/// A [collision event](Collider#collision-events) that is sent for each contact pair during the narrow phase.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct Collision(pub Contacts);

/// A [collision event](Collider#collision-events) that is sent when two entities start colliding.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct CollisionStarted(pub Entity, pub Entity);

/// A [collision event](Collider#collision-events) that is sent when two entities stop colliding.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct CollisionEnded(pub Entity, pub Entity);

/// All contacts between two colliders.
///
/// The contacts are stored in contact manifolds.
/// Each manifold contains one or more contact points, and each contact
/// in a given manifold shares the same contact normal.
#[derive(Clone, Debug, PartialEq)]
pub struct Contacts {
    /// First entity in the contact.
    pub entity1: Entity,
    /// Second entity in the contact.
    pub entity2: Entity,
    /// A list of contact manifolds between two colliders.
    /// Each manifold contains one or more contact points, but each contact
    /// in a given manifold shares the same contact normal.
    pub manifolds: Vec<ContactManifold>,
}

/// A contact manifold between two colliders, containing a set of contact points.
/// Each contact in a manifold shares the same contact normal.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifold {
    /// First entity in the contact.
    pub entity1: Entity,
    /// Second entity in the contact.
    pub entity2: Entity,
    /// The contacts in this manifold.
    pub contacts: Vec<ContactData>,
    /// A world-space contact normal shared by all contacts in this manifold.
    pub normal: Vector,
}

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactData {
    /// First entity in the contact.
    pub entity1: Entity,
    /// Second entity in the contact.
    pub entity2: Entity,
    /// Contact point on the first entity in local coordinates.
    pub local_point1: Vector,
    /// Contact point on the second entity in local coordinates.
    pub local_point2: Vector,
    /// Contact point on the first entity in global coordinates.
    pub point1: Vector,
    /// Contact point on the second entity in global coordinates.
    pub point2: Vector,
    /// Contact normal from contact point 1 to 2 in world coordinates.
    pub normal: Vector,
    /// Penetration depth.
    pub penetration: Scalar,
}

/// Computes one pair of contact points between two shapes.
#[allow(clippy::too_many_arguments)]
pub(crate) fn compute_contacts(
    entity1: Entity,
    entity2: Entity,
    position1: Vector,
    position2: Vector,
    rotation1: &Rotation,
    rotation2: &Rotation,
    collider1: &Collider,
    collider2: &Collider,
) -> Contacts {
    let isometry1 = utils::make_isometry(position1, rotation1);
    let isometry2 = utils::make_isometry(position2, rotation2);
    let isometry12 = isometry1.inv_mul(&isometry2);

    // Todo: Reuse manifolds from previous frame to improve performance
    let mut manifolds: Vec<parry::query::ContactManifold<(), ()>> = vec![];
    let _ = parry::query::DefaultQueryDispatcher.contact_manifolds(
        &isometry12,
        collider1.get_shape().0.as_ref(),
        collider2.get_shape().0.as_ref(),
        0.0,
        &mut manifolds,
        &mut None,
    );
    Contacts {
        entity1,
        entity2,
        manifolds: manifolds
            .clone()
            .into_iter()
            .map(|manifold| ContactManifold {
                entity1,
                entity2,
                normal: rotation1.rotate(manifold.local_n1.into()),
                contacts: manifold
                    .contacts()
                    .iter()
                    .map(|contact| ContactData {
                        entity1,
                        entity2,
                        local_point1: contact.local_p1.into(),
                        local_point2: contact.local_p2.into(),
                        point1: position1 + rotation1.rotate(contact.local_p1.into()),
                        point2: position2 + rotation2.rotate(contact.local_p2.into()),
                        normal: rotation1.rotate(manifolds[0].local_n1.into()),
                        penetration: -contact.dist,
                    })
                    .filter(|c| c.penetration > 0.0)
                    .collect(),
            })
            .collect(),
    }
}
