//! Computes contacts between entities.
//!
//! See [`NarrowPhasePlugin`].

use crate::prelude::*;
use bevy::ecs::query::Has;
#[cfg(feature = "parallel")]
use bevy::tasks::{ComputeTaskPool, ParallelSlice};

/// Computes contacts between entities.
///
/// Collisions are only checked between entities contained in [`BroadCollisionPairs`],
/// which is handled by the [`BroadPhasePlugin`].
///
/// The results of the narrow phase are added into [`Collisions`].
pub struct NarrowPhasePlugin;

impl Plugin for NarrowPhasePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<NarrowPhaseConfig>()
            .init_resource::<Collisions>()
            .register_type::<NarrowPhaseConfig>();

        // Manage collision states like `during_current_frame` and remove old contacts
        // TODO: It would be nice not to have collision state logic in the narrow phase
        app.get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first")
            .add_systems(
                (
                    // Reset collision states before the substepping loop
                    reset_collision_states
                        .after(PhysicsStepSet::BroadPhase)
                        .before(PhysicsStepSet::Substeps),
                    // Remove ended collisions after contact reporting
                    ((|mut collisions: ResMut<Collisions>| {
                        collisions.retain(|contacts| contacts.during_current_frame)
                    }),)
                        .chain()
                        .after(PhysicsStepSet::ReportContacts)
                        .before(PhysicsStepSet::Sleeping),
                )
                    .chain(),
            );

        // Reset substep collision states and collect contacts into `Collisions`
        app.get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first")
            .add_systems(
                (reset_substep_collision_states, collect_collisions)
                    .chain()
                    .in_set(SubstepSet::NarrowPhase),
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
            prediction_distance: 1.0,
            #[cfg(feature = "3d")]
            prediction_distance: 0.01,
        }
    }
}

/// Computes contacts based on [`BroadCollisionPairs`] and adds them to [`Collisions`].
#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
pub fn collect_collisions(
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
        // TODO: Verify if `par_splat_map` is deterministic. If not, sort the collisions.
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

                        let previous_contact = collisions.get_internal().get(&(*entity1, *entity2));

                        let contacts = Contacts {
                            entity1: *entity1,
                            entity2: *entity2,
                            during_current_frame: previous_contact
                                .map_or(false, |c| c.during_current_frame),
                            during_current_substep: true,
                            during_previous_frame: previous_contact
                                .map_or(false, |c| c.during_previous_frame),
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

                let previous_contact = collisions.get_internal().get(&(*entity1, *entity2));

                let contacts = Contacts {
                    entity1: *entity1,
                    entity2: *entity2,
                    during_current_frame: previous_contact
                        .map_or(false, |c| c.during_current_frame),
                    during_current_substep: true,
                    during_previous_frame: previous_contact
                        .map_or(false, |c| c.during_previous_frame),
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

// TODO: The collision state handling feels a bit confusing and error-prone.
//       Ideally, the narrow phase wouldn't need to handle it at all, or it would at least be simpler.
/// Resets collision states like `during_current_frame` and `during_previous_frame`.
pub fn reset_collision_states(
    mut collisions: ResMut<Collisions>,
    query: Query<(Option<&RigidBody>, Has<Sleeping>)>,
) {
    for contacts in collisions.get_internal_mut().values_mut() {
        if let Ok([(rb1, sleeping1), (rb2, sleeping2)]) =
            query.get_many([contacts.entity1, contacts.entity2])
        {
            let active1 = !rb1.map_or(false, |rb| rb.is_static()) && !sleeping1;
            let active2 = !rb2.map_or(false, |rb| rb.is_static()) && !sleeping2;

            // Reset collision states if either of the bodies is active (not static or sleeping)
            // Otherwise, the bodies are still in contact.
            if active1 || active2 {
                contacts.during_previous_frame = true;
                contacts.during_current_frame = false;
                contacts.during_current_substep = false;
            } else {
                contacts.during_previous_frame = true;
                contacts.during_current_frame = true;
            }
        } else {
            contacts.during_current_frame = false;
        }
    }
}

/// Reset `during_current_substep` for each collision in [`Collisions`].
pub fn reset_substep_collision_states(mut collisions: ResMut<Collisions>) {
    for contacts in collisions.get_internal_mut().values_mut() {
        contacts.during_current_substep = false;
    }
}
