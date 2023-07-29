//! Computes contacts between entities and sends collision events.
//!
//! See [`NarrowPhasePlugin`].

use crate::prelude::*;
use bevy::prelude::*;
use parry::query::{PersistentQueryDispatcher, QueryDispatcher};

/// Computes contacts between entities and sends collision events.
///
/// Collisions are only checked between entities contained in [`BroadCollisionPairs`],
/// which is handled by the [`BroadPhasePlugin`].
///
/// The following collision events are sent each frame:
///
/// - [`Collision`]
/// - [`CollisionStarted`]
/// - [`CollisionEnded`]
pub struct NarrowPhasePlugin;

impl Plugin for NarrowPhasePlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<Collision>()
            .add_event::<CollisionStarted>()
            .add_event::<CollisionEnded>();

        let substep_schedule = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");

        substep_schedule.add_systems((collect_collisions).chain().in_set(SubstepSet::NarrowPhase));
    }
}

/// A [collision event](Collider#collision-events) that is sent for each contact pair during the narrow phase.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct Collision(pub Contacts);

/// A [collision event](Collider#collision-events) that is sent when two entities start colliding.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct CollisionStarted(pub Entity, pub Entity);

/// A [collision event](Collider#collision-events) that is sent when two entities stop colliding.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct CollisionEnded(pub Entity, pub Entity);

#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
fn collect_collisions(
    mut bodies: Query<(
        Option<&RigidBody>,
        &Position,
        Option<&AccumulatedTranslation>,
        &Rotation,
        &Collider,
        Option<&CollisionLayers>,
        Option<&mut CollidingEntities>,
        Option<&Sleeping>,
    )>,
    broad_collision_pairs: Res<BroadCollisionPairs>,
    mut collision_ev_writer: EventWriter<Collision>,
    mut collision_started_ev_writer: EventWriter<CollisionStarted>,
    mut collision_ended_ev_writer: EventWriter<CollisionEnded>,
) {
    let mut collision_events = Vec::with_capacity(broad_collision_pairs.0.len());
    let mut started_collisions = Vec::new();
    let mut ended_collisions = Vec::new();

    for (entity1, entity2) in broad_collision_pairs.0.iter() {
        if let Ok([bundle1, bundle2]) = bodies.get_many_mut([*entity1, *entity2]) {
            let (
                rb1,
                position1,
                accumulated_translation1,
                rotation1,
                collider1,
                layers1,
                mut colliding_entities1,
                sleeping1,
            ) = bundle1;
            let (
                rb2,
                position2,
                accumulated_translation2,
                rotation2,
                collider2,
                layers2,
                mut colliding_entities2,
                sleeping2,
            ) = bundle2;

            let layers1 = layers1.map_or(CollisionLayers::default(), |l| *l);
            let layers2 = layers2.map_or(CollisionLayers::default(), |l| *l);

            // Skip collision if collision layers are incompatible
            if !layers1.interacts_with(layers2) {
                continue;
            }

            let inactive1 = rb1.map_or(false, |rb| rb.is_static()) || sleeping1.is_some();
            let inactive2 = rb2.map_or(false, |rb| rb.is_static()) || sleeping2.is_some();

            // No collision if the bodies are static or sleeping
            if inactive1 && inactive2 {
                continue;
            }

            let contacts = compute_contacts(
                *entity1,
                *entity2,
                position1.0 + accumulated_translation1.map_or(Vector::default(), |t| t.0),
                position2.0 + accumulated_translation2.map_or(Vector::default(), |t| t.0),
                rotation1,
                rotation2,
                collider1,
                collider2,
            );

            if !contacts.manifolds.is_empty() {
                let mut collision_started_1 = false;
                let mut collision_started_2 = false;

                // Add entity to set of colliding entities
                if let Some(ref mut entities) = colliding_entities1 {
                    collision_started_1 = entities.insert(*entity2);
                }
                if let Some(ref mut entities) = colliding_entities2 {
                    collision_started_2 = entities.insert(*entity1);
                }

                if collision_started_1 || collision_started_2 {
                    started_collisions.push(CollisionStarted(*entity1, *entity2));
                }

                collision_events.push(Collision(contacts));
            } else {
                let mut collision_ended_1 = false;
                let mut collision_ended_2 = false;

                // Remove entity from set of colliding entities
                if let Some(mut entities) = colliding_entities1 {
                    collision_ended_1 = entities.remove(entity2);
                }
                if let Some(mut entities) = colliding_entities2 {
                    collision_ended_2 = entities.remove(entity1);
                }

                if collision_ended_1 || collision_ended_2 {
                    ended_collisions.push(CollisionEnded(*entity1, *entity2));
                }
            }
        }
    }

    collision_ev_writer.send_batch(collision_events);
    collision_started_ev_writer.send_batch(started_collisions);
    collision_ended_ev_writer.send_batch(ended_collisions);
}

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
    /// A contact normal shared by all contacts in this manifold,
    /// expressed in the local space of the first entity.
    pub normal: Vector,
}

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactData {
    /// Contact point on the first entity in local coordinates.
    pub point1: Vector,
    /// Contact point on the second entity in local coordinates.
    pub point2: Vector,
    /// A contact normal expressed in the local space of the first entity.
    pub normal: Vector,
    /// Penetration depth.
    pub penetration: Scalar,
    /// True if both colliders are convex. Currently, contacts between
    /// convex and non-convex colliders have to be handled differently.
    pub(crate) convex: bool,
}

impl ContactData {
    /// Returns the global contact point on the first entity,
    /// transforming the local point by the given entity position and rotation.
    pub fn global_point1(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation.rotate(self.point1)
    }

    /// Returns the global contact point on the second entity,
    /// transforming the local point by the given entity position and rotation.
    pub fn global_point2(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation.rotate(self.point2)
    }

    /// Returns the world-space contact normal pointing towards the exterior of the first entity.
    pub fn global_normal(&self, rotation: &Rotation) -> Vector {
        rotation.rotate(self.normal)
    }
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
    let convex = collider1.is_convex() && collider2.is_convex();
    #[cfg(feature = "2d")]
    let prediction_distance = 2.0;
    #[cfg(feature = "3d")]
    let prediction_distance = 0.002;

    if convex {
        // Todo: Reuse manifolds from previous frame to improve performance
        let mut manifolds: Vec<parry::query::ContactManifold<(), ()>> = vec![];
        let _ = parry::query::DefaultQueryDispatcher.contact_manifolds(
            &isometry12,
            collider1.get_shape().0.as_ref(),
            collider2.get_shape().0.as_ref(),
            prediction_distance,
            &mut manifolds,
            &mut None,
        );
        Contacts {
            entity1,
            entity2,
            manifolds: manifolds
                .iter()
                .map(|manifold| ContactManifold {
                    entity1,
                    entity2,
                    normal: manifold.local_n1.into(),
                    contacts: manifold
                        .contacts()
                        .iter()
                        .map(|contact| ContactData {
                            point1: contact.local_p1.into(),
                            point2: contact.local_p2.into(),
                            normal: manifold.local_n1.into(),
                            penetration: -contact.dist,
                            convex,
                        })
                        .collect(),
                })
                .collect(),
        }
    } else {
        // For some reason, convex colliders sink into non-convex colliders
        // if we use contact manifolds, so we have to compute a single contact point instead.
        // Todo: Find out why this is and use contact manifolds for both types of colliders.
        let contact = parry::query::DefaultQueryDispatcher
            .contact(
                &isometry12,
                collider1.get_shape().0.as_ref(),
                collider2.get_shape().0.as_ref(),
                prediction_distance,
            )
            .unwrap()
            .map(|contact| ContactData {
                point1: contact.point1.into(),
                point2: contact.point2.into(),
                normal: contact.normal1.into(),
                penetration: -contact.dist,
                convex,
            });
        Contacts {
            entity1,
            entity2,
            manifolds: contact.map_or(vec![], |contact| {
                vec![ContactManifold {
                    entity1,
                    entity2,
                    contacts: vec![contact],
                    normal: contact.normal,
                }]
            }),
        }
    }
}
