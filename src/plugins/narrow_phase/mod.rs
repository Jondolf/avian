//! Computes contacts between entities and sends collision events.
//!
//! See [`NarrowPhasePlugin`].

pub mod contact_query;

pub use contact_query::*;

use crate::prelude::*;
use bevy::prelude::*;
#[cfg(feature = "parallel")]
use bevy::tasks::{ComputeTaskPool, ParallelSlice};
use indexmap::IndexMap;

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
    prediction_distance: Scalar,
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

// Collisions are currently stored in an `IndexMap` that uses fxhash.
// It should have faster iteration than `HashMap` while mostly retaining other performance characteristics.
// In a simple benchmark, the difference seemed pretty negligible though.
//
// `IndexMap` preserves insertion order, which affects the order in which collisions are detected.
// This can be good or bad depending on the situation, but it can make spawned stacks appear more
// consistent and uniform, for example in the `move_marbles` example.
// ==========================================
/// All collision pairs.
#[derive(Resource, Debug, Default)]
pub struct Collisions(pub(crate) IndexMap<(Entity, Entity), Contacts, fxhash::FxBuildHasher>);

impl Collisions {
    /// Returns a reference to the [contacts](Contacts) stored for the given entity pair if they are colliding,
    /// else returns `None`.
    ///
    /// The order of the entities does not matter.
    pub fn get(&self, entity1: Entity, entity2: Entity) -> Option<&Contacts> {
        self.0
            .get(&(entity1, entity2))
            .filter(|contacts| contacts.during_current_frame)
            .or_else(|| {
                self.0
                    .get(&(entity2, entity1))
                    .filter(|contacts| contacts.during_current_frame)
            })
    }

    /// Returns a mutable reference to the [contacts](Contacts) stored for the given entity pair if they are colliding,
    /// else returns `None`.
    ///
    /// The order of the entities does not matter.
    pub fn get_mut(&mut self, entity1: Entity, entity2: Entity) -> Option<&mut Contacts> {
        // For lifetime reasons, the mutable borrows can't be in the same scope,
        // so we check if the key exists first (there's probably a better way though)
        if self.0.contains_key(&(entity1, entity2)) {
            self.0
                .get_mut(&(entity1, entity2))
                .filter(|contacts| contacts.during_current_frame)
        } else {
            self.0
                .get_mut(&(entity2, entity1))
                .filter(|contacts| contacts.during_current_frame)
        }
    }

    /// Returns an iterator over the current collisions that have happened during the current physics frame.
    pub fn iter(&self) -> impl Iterator<Item = &Contacts> {
        self.0
            .values()
            .filter(|collision| collision.during_current_frame)
    }

    /// Returns a mutable iterator over the collisions that have happened during the current physics frame.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Contacts> {
        self.0
            .values_mut()
            .filter(|collision| collision.during_current_frame)
    }

    /// Returns an iterator over all collisions with a given entity.
    pub fn collisions_with_entity(&self, entity: Entity) -> impl Iterator<Item = &Contacts> {
        self.0
            .iter()
            .filter_map(move |((entity1, entity2), contacts)| {
                if contacts.during_current_frame && (*entity1 == entity || *entity2 == entity) {
                    Some(contacts)
                } else {
                    None
                }
            })
    }

    /// Returns an iterator over all collisions with a given entity.
    pub fn collisions_with_entity_mut(
        &mut self,
        entity: Entity,
    ) -> impl Iterator<Item = &mut Contacts> {
        self.0
            .iter_mut()
            .filter_map(move |((entity1, entity2), contacts)| {
                if contacts.during_current_frame && (*entity1 == entity || *entity2 == entity) {
                    Some(contacts)
                } else {
                    None
                }
            })
    }

    /// Retains only the collisions for which the specified predicate returns `true`.
    /// Collisions for which the predicate returns `false` are removed from the `HashMap`.
    pub fn retain<F>(&mut self, keep: F)
    where
        F: FnMut(&(Entity, Entity), &mut Contacts) -> bool,
    {
        self.0.retain(keep);
    }

    /// Removes collisions against the given entity from the `HashMap`.
    fn remove_collisions_with_entity(&mut self, entity: Entity) {
        self.retain(|(entity1, entity2), _| *entity1 != entity && *entity2 != entity);
    }
}

/// Stores the collision pairs from the previous frame.
/// This is used for detecting when collisions have started or ended.
#[derive(Resource, Clone, Debug, Default, PartialEq)]
struct PreviousCollisions(IndexMap<(Entity, Entity), Contacts, fxhash::FxBuildHasher>);

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
                let mut collisions: Vec<((Entity, Entity), Contacts)> = vec![];
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
                                collisions.push(((*entity1, *entity2), contacts));
                            }
                        }
                    }
                }
                collisions
            })
            .into_iter()
            .flatten();
        collisions.0.extend(new_collisions);
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
                        collisions.0.insert((*entity1, *entity2), contacts);
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
    for contacts in collisions.0.values_mut() {
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
    for ((entity1, entity2), contacts) in collisions.0.iter_mut() {
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
            if let Some(previous_contacts) = previous_collisions.0.get_mut(&(*entity1, *entity2)) {
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
                previous_collisions
                    .0
                    .insert((*entity1, *entity2), contacts.clone());
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
        } else if let Some(previous_contacts) = previous_collisions.0.get_mut(&(*entity1, *entity2))
        {
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
    collisions.0.clear();
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
    /// True if the bodies have been in contact during this frame.
    pub during_current_frame: bool,
    /// True if the bodies have been in contact during this substep.
    pub during_current_substep: bool,
}

/// A contact manifold between two colliders, containing a set of contact points.
/// Each contact in a manifold shares the same contact normal.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifold {
    /// The contacts in this manifold.
    pub contacts: Vec<ContactData>,
    /// A contact normal shared by all contacts in this manifold,
    /// expressed in the local space of the first entity.
    pub normal1: Vector,
    /// A contact normal shared by all contacts in this manifold,
    /// expressed in the local space of the second entity.
    pub normal2: Vector,
}

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactData {
    /// Contact point on the first entity in local coordinates.
    pub point1: Vector,
    /// Contact point on the second entity in local coordinates.
    pub point2: Vector,
    /// A contact normal expressed in the local space of the first entity.
    pub normal1: Vector,
    /// A contact normal expressed in the local space of the second entity.
    pub normal2: Vector,
    /// Penetration depth.
    pub penetration: Scalar,
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
    pub fn global_normal1(&self, rotation: &Rotation) -> Vector {
        rotation.rotate(self.normal1)
    }

    /// Returns the world-space contact normal pointing towards the exterior of the second entity.
    pub fn global_normal2(&self, rotation: &Rotation) -> Vector {
        rotation.rotate(self.normal2)
    }
}
