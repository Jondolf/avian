//! Computes contacts between entities and sends collision events.
//!
//! See [`NarrowPhasePlugin`].

mod contact_data;
pub mod contact_query;

use bevy::utils::HashSet;
pub use contact_data::*;
pub use contact_query::*;

use crate::prelude::*;
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
            .register_type::<NarrowPhaseConfig>();

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            (
                // Reset collision states before the narrow phase
                (|mut collisions: ResMut<Collisions>| {
                    collisions.iter_mut().for_each(|contacts| {
                        contacts.during_previous_frame = contacts.during_current_frame;
                        contacts.during_current_frame = false;
                        contacts.during_current_substep = false;
                    })
                })
                .after(PhysicsStepSet::BroadPhase)
                .before(PhysicsStepSet::Substeps),
                // Send collision events
                send_collision_events
                    .after(PhysicsStepSet::Sleeping)
                    .before(PhysicsStepSet::SpatialQuery),
            )
                .chain(),
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
        &Position,
        Option<&AccumulatedTranslation>,
        &Rotation,
        &Collider,
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
                let mut new_collisions: Vec<Contacts> = vec![];
                for (entity1, entity2) in chunks {
                    if let Ok([bundle1, bundle2]) = bodies.get_many([*entity1, *entity2]) {
                        let (position1, accumulated_translation1, rotation1, collider1) = bundle1;
                        let (position2, accumulated_translation2, rotation2, collider2) = bundle2;

                        let position1 =
                            position1.0 + accumulated_translation1.copied().unwrap_or_default().0;
                        let position2 =
                            position2.0 + accumulated_translation2.copied().unwrap_or_default().0;

                        let during_previous_frame = collisions
                            .get_internal()
                            .get(&(*entity1, *entity2))
                            .map_or(false, |c| c.during_previous_frame);

                        let contacts = Contacts {
                            entity1: *entity1,
                            entity2: *entity2,
                            during_current_frame: true,
                            during_current_substep: true,
                            during_previous_frame,
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
                            new_collisions.push(contacts);
                        }
                    }
                }
                new_collisions
            })
            .into_iter()
            .flatten();
        collisions.extend(new_collisions);
    }
    #[cfg(not(feature = "parallel"))]
    {
        for (entity1, entity2) in broad_collision_pairs.0.iter() {
            if let Ok([bundle1, bundle2]) = bodies.get_many([*entity1, *entity2]) {
                let (position1, accumulated_translation1, rotation1, collider1) = bundle1;
                let (position2, accumulated_translation2, rotation2, collider2) = bundle2;

                let position1 =
                    position1.0 + accumulated_translation1.copied().unwrap_or_default().0;
                let position2 =
                    position2.0 + accumulated_translation2.copied().unwrap_or_default().0;

                let during_previous_frame = collisions
                    .get_internal()
                    .get(&(*entity1, *entity2))
                    .map_or(false, |c| c.during_previous_frame);

                let contacts = Contacts {
                    entity1: *entity1,
                    entity2: *entity2,
                    during_current_frame: true,
                    during_current_substep: true,
                    during_previous_frame,
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

fn reset_substep_collision_states(mut collisions: ResMut<Collisions>) {
    for contacts in collisions.get_internal_mut().values_mut() {
        contacts.during_current_substep = false;
    }
}

/// Sends collision events and updates [`CollidingEntities`].
fn send_collision_events(
    sleeping: Query<(Ref<Position>, Ref<Rotation>)>,
    mut colliders: Query<&mut CollidingEntities>,
    mut collisions: ResMut<Collisions>,
    mut collision_ev_writer: EventWriter<Collision>,
    mut collision_started_ev_writer: EventWriter<CollisionStarted>,
    mut collision_ended_ev_writer: EventWriter<CollisionEnded>,
) {
    let mut ended_collisions = HashSet::<(Entity, Entity)>::new();

    for ((entity1, entity2), contacts) in collisions.get_internal_mut().iter_mut() {
        // Collision ended
        if !contacts.during_current_frame {
            // Keep the collision active if the bodies were colliding during the previous frame
            // but neither body has moved, for example when they are sleeping.
            if let Ok([(pos1, rot1), (pos2, rot2)]) = sleeping.get_many([*entity1, *entity2]) {
                if !pos1.is_changed()
                    && !rot1.is_changed()
                    && !pos2.is_changed()
                    && !rot2.is_changed()
                {
                    contacts.during_current_frame = true;
                    continue;
                }
            }

            ended_collisions.insert((*entity1, *entity2));

            if let Ok(mut colliding_entities1) = colliders.get_mut(*entity1) {
                colliding_entities1.remove(entity2);
            }
            if let Ok(mut colliding_entities2) = colliders.get_mut(*entity2) {
                colliding_entities2.remove(entity1);
            }

            continue;
        }

        collision_ev_writer.send(Collision(contacts.clone()));

        // Collision started
        if contacts.during_current_frame && !contacts.during_previous_frame {
            collision_started_ev_writer.send(CollisionStarted(*entity1, *entity2));
            contacts.during_previous_frame = true;

            if let Ok(mut colliding_entities1) = colliders.get_mut(*entity1) {
                colliding_entities1.insert(*entity2);
            } else {
                ended_collisions.insert((*entity1, *entity2));
            }
            if let Ok(mut colliding_entities2) = colliders.get_mut(*entity2) {
                colliding_entities2.insert(*entity1);
            } else {
                ended_collisions.insert((*entity1, *entity2));
            }
        }
    }

    // Send CollisionEnded events
    collision_ended_ev_writer.send_batch(
        ended_collisions
            .iter()
            .map(|(entity1, entity2)| CollisionEnded(*entity1, *entity2)),
    );

    // Clear collisions at the end of each frame to avoid unnecessary iteration and memory usage
    collisions.retain(|contacts| !ended_collisions.contains(&(contacts.entity1, contacts.entity2)));
}
