//! Computes contacts between entities and sends collision events.
//!
//! See [`NarrowPhasePlugin`].

mod contact_data;
pub mod contact_query;

pub use contact_data::*;
pub use contact_query::*;

use crate::prelude::*;
use bevy::prelude::*;
#[cfg(feature = "parallel")]
use bevy::tasks::{ComputeTaskPool, ParallelSlice};

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
            .add_event::<CollisionEnded>()
            .init_resource::<NarrowPhaseConfig>()
            .init_resource::<Collisions>()
            .init_resource::<PreviousCollisions>()
            .register_type::<NarrowPhaseConfig>();

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            send_collision_events
                .after(PhysicsStepSet::Substeps)
                .before(PhysicsStepSet::Sleeping),
        );

        let substep_schedule = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");

        substep_schedule.add_systems(
            (reset_substep_collision_states, collect_collisions)
                .chain()
                .in_set(SubstepSet::NarrowPhase),
        );

        // Remove collisions against removed colliders from `Collisions`
        app.add_systems(
            Last,
            |mut removals: RemovedComponents<Collider>, mut collisions: ResMut<Collisions>| {
                for removed in removals.iter() {
                    collisions.remove_collisions_with_entity(removed);
                }
            },
        );
    }
}

/// A resource for configuring the [narrow phase](NarrowPhasePlugin).
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[reflect(Resource)]
pub struct NarrowPhaseConfig {
    /// The maximum separation distance allowed for a collision to be accepted.
    ///
    /// This can be used for things like **speculative contacts** where the contacts should
    /// include pairs of entities that *might* be in contact after constraint solving or
    /// other positional changes.
    pub prediction_distance: Scalar,
}

impl Default for NarrowPhaseConfig {
    fn default() -> Self {
        Self {
            #[cfg(feature = "2d")]
            prediction_distance: 5.0,
            #[cfg(feature = "3d")]
            prediction_distance: 0.005,
        }
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
    bodies: Query<(
        Option<&RigidBody>,
        &Position,
        Option<&AccumulatedTranslation>,
        &Rotation,
        &Collider,
        Option<&CollisionLayers>,
        Option<&Sleeping>,
    )>,
    broad_collision_pairs: Res<BroadCollisionPairs>,
    mut collisions: ResMut<Collisions>,
    narrow_phase_config: Res<NarrowPhaseConfig>,
) {
    #[cfg(feature = "parallel")]
    {
        let pool = ComputeTaskPool::get();
        // Todo: Verify if `par_splat_map` is deterministic. If not, sort the collisions.
        let new_collisions = broad_collision_pairs
            .0
            .par_splat_map(pool, None, |chunks| {
                let mut collisions: Vec<Contacts> = vec![];
                for (entity1, entity2) in chunks {
                    if let Ok([bundle1, bundle2]) = bodies.get_many([*entity1, *entity2]) {
                        let (
                            rb1,
                            position1,
                            accumulated_translation1,
                            rotation1,
                            collider1,
                            layers1,
                            sleeping1,
                        ) = bundle1;
                        let (
                            rb2,
                            position2,
                            accumulated_translation2,
                            rotation2,
                            collider2,
                            layers2,
                            sleeping2,
                        ) = bundle2;

                        if check_collision_validity(
                            rb1, rb2, layers1, layers2, sleeping1, sleeping2,
                        ) {
                            let position1 = position1.0
                                + accumulated_translation1.copied().unwrap_or_default().0;
                            let position2 = position2.0
                                + accumulated_translation2.copied().unwrap_or_default().0;
                            let contacts = Contacts {
                                entity1: *entity1,
                                entity2: *entity2,
                                during_current_frame: true,
                                during_current_substep: true,
                                manifolds: contact_query::contact_manifolds(
                                    collider1,
                                    position1,
                                    *rotation1,
                                    collider2,
                                    position2,
                                    *rotation2,
                                    narrow_phase_config.prediction_distance,
                                ),
                            };

                            if !contacts.manifolds.is_empty() {
                                collisions.push(contacts);
                            }
                        }
                    }
                }
                collisions
            })
            .into_iter()
            .flatten();
        collisions.extend(new_collisions);
    }
    #[cfg(not(feature = "parallel"))]
    {
        for (entity1, entity2) in broad_collision_pairs.0.iter() {
            if let Ok([bundle1, bundle2]) = bodies.get_many([*entity1, *entity2]) {
                let (
                    rb1,
                    position1,
                    accumulated_translation1,
                    rotation1,
                    collider1,
                    layers1,
                    sleeping1,
                ) = bundle1;
                let (
                    rb2,
                    position2,
                    accumulated_translation2,
                    rotation2,
                    collider2,
                    layers2,
                    sleeping2,
                ) = bundle2;

                if check_collision_validity(rb1, rb2, layers1, layers2, sleeping1, sleeping2) {
                    let position1 =
                        position1.0 + accumulated_translation1.copied().unwrap_or_default().0;
                    let position2 =
                        position2.0 + accumulated_translation2.copied().unwrap_or_default().0;
                    let contacts = Contacts {
                        entity1: *entity1,
                        entity2: *entity2,
                        during_current_frame: true,
                        during_current_substep: true,
                        manifolds: contact_query::contact_manifolds(
                            collider1,
                            position1,
                            *rotation1,
                            collider2,
                            position2,
                            *rotation2,
                            narrow_phase_config.prediction_distance,
                        ),
                    };

                    if !contacts.manifolds.is_empty() {
                        collisions.insert_collision_pair(contacts);
                    }
                }
            }
        }
    }
}

fn check_collision_validity(
    rb1: Option<&RigidBody>,
    rb2: Option<&RigidBody>,
    layers1: Option<&CollisionLayers>,
    layers2: Option<&CollisionLayers>,
    sleeping1: Option<&Sleeping>,
    sleeping2: Option<&Sleeping>,
) -> bool {
    let layers1 = layers1.map_or(CollisionLayers::default(), |l| *l);
    let layers2 = layers2.map_or(CollisionLayers::default(), |l| *l);

    // Skip collision if collision layers are incompatible
    if !layers1.interacts_with(layers2) {
        return false;
    }

    let inactive1 = rb1.map_or(false, |rb| rb.is_static()) || sleeping1.is_some();
    let inactive2 = rb2.map_or(false, |rb| rb.is_static()) || sleeping2.is_some();

    // No collision if the bodies are static or sleeping
    if inactive1 && inactive2 {
        return false;
    }

    true
}

fn reset_substep_collision_states(mut collisions: ResMut<Collisions>) {
    for contacts in collisions.get_internal_mut().values_mut() {
        contacts.during_current_substep = false;
    }
}

// Todo: This system feels overly complex and slow.
// It only runs once per frame and not once per substep, but it would be nice to do this
// without so much iteration over hash maps and querying.
/// Sends collision events and updates [`CollidingEntities`].
fn send_collision_events(
    mut colliders: Query<&mut CollidingEntities>,
    mut collisions: ResMut<Collisions>,
    mut previous_collisions: ResMut<PreviousCollisions>,
    mut collision_ev_writer: EventWriter<Collision>,
    mut collision_started_ev_writer: EventWriter<CollisionStarted>,
    mut collision_ended_ev_writer: EventWriter<CollisionEnded>,
) {
    for ((entity1, entity2), contacts) in collisions.get_internal_mut().iter_mut() {
        let penetrating = contacts.manifolds.iter().any(|manifold| {
            manifold
                .contacts
                .iter()
                .any(|contact| contact.penetration > Scalar::EPSILON)
        });

        // Bodies were penetrating during this physics frame
        if contacts.during_current_frame && penetrating {
            // Send collision event.
            collision_ev_writer.send(Collision(contacts.clone()));

            // Get or insert the previous contacts.
            // If the bodies weren't penetrating previously, they started colliding this frame.
            if let Some(previous_contacts) = previous_collisions.get_mut(*entity1, *entity2) {
                if !previous_contacts.during_current_frame {
                    collision_started_ev_writer.send(CollisionStarted(*entity1, *entity2));
                    previous_contacts.during_current_frame = true;

                    // Update colliding entities
                    if let Ok(mut entities) = colliders.get_mut(*entity1) {
                        entities.insert(*entity2);
                    }
                    if let Ok(mut entities) = colliders.get_mut(*entity2) {
                        entities.insert(*entity1);
                    }
                }
            } else {
                previous_collisions.insert_collision_pair(contacts.clone());
                collision_started_ev_writer.send(CollisionStarted(*entity1, *entity2));

                // Update colliding entities
                if let Ok(mut entities) = colliders.get_mut(*entity1) {
                    entities.insert(*entity2);
                }
                if let Ok(mut entities) = colliders.get_mut(*entity2) {
                    entities.insert(*entity1);
                }
            }

            // Reset collision state to not penetrating.
            contacts.during_current_frame = false;
        } else if let Some(previous_contacts) = previous_collisions.get_mut(*entity1, *entity2) {
            // If the bodies weren't penetrating last frame either, we can return early.
            if !previous_contacts.manifolds.iter().any(|manifold| {
                manifold
                    .contacts
                    .iter()
                    .any(|contact| contact.penetration > Scalar::EPSILON)
            }) {
                previous_contacts.during_current_frame = false;
                continue;
            }

            // If the bodies aren't penetrating currently but they were penetrating last frame,
            // the collision has ended.
            if previous_contacts.during_current_frame {
                collision_ended_ev_writer.send(CollisionEnded(*entity1, *entity2));

                // Update colliding entities
                if let Ok(mut entities) = colliders.get_mut(*entity1) {
                    entities.remove(entity2);
                }
                if let Ok(mut entities) = colliders.get_mut(*entity2) {
                    entities.remove(entity1);
                }
            }

            // Reset previous collision state to match current collision state.
            previous_contacts.during_current_frame = false;
        }
    }

    // Clear collisions at the end of each frame to avoid unnecessary iteration and memory usage
    collisions.get_internal_mut().clear();
}
