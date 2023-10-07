//! Computes contacts between entities and sends collision events.
//!
//! See [`NarrowPhasePlugin`].

use crate::prelude::*;
#[cfg(feature = "parallel")]
use bevy::tasks::{ComputeTaskPool, ParallelSlice};
use bevy::{ecs::query::Has, prelude::*, utils::HashSet};
use indexmap::IndexMap;
use parry::query::PersistentQueryDispatcher;

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
            (
                // Reset collision states before the narrow phase
                (|mut collisions: ResMut<Collisions>| {
                    collisions
                        .iter_mut()
                        .for_each(|c| c.during_current_frame = false)
                })
                .after(PhysicsStepSet::BroadPhase)
                .before(PhysicsStepSet::Substeps),
                // Wake up sleeping bodies when the other body moves or despawns
                (apply_deferred, wake_up_on_collision_ended)
                    .chain()
                    .after(PhysicsStepSet::Substeps)
                    .before(PhysicsStepSet::Sleeping),
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
#[derive(Resource, Clone, Debug, Default, PartialEq)]
pub struct Collisions(IndexMap<(Entity, Entity), Contacts, fxhash::FxBuildHasher>);

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

    /// Returns a reference to the internal `IndexMap`.
    pub fn get_internal(&self) -> &IndexMap<(Entity, Entity), Contacts, fxhash::FxBuildHasher> {
        &self.0
    }

    /// Returns a mutable reference to the internal `IndexMap`.
    pub fn get_internal_mut(
        &mut self,
    ) -> &mut IndexMap<(Entity, Entity), Contacts, fxhash::FxBuildHasher> {
        &mut self.0
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

    /// Inserts contact data for a collision between two entities.
    ///
    /// If a collision entry with the same entities already exists, it will be overwritten,
    /// and the old value will be returned. Otherwise, `None` is returned.
    ///
    /// **Note**: Manually inserting collisions can be error prone and should generally be avoided.
    /// If you simply want to modify existing collisions, consider using methods like [`get_mut`](#method.get_mut)
    /// or [`iter_mut`](#method.iter_mut).
    pub fn insert_collision_pair(&mut self, contacts: Contacts) -> Option<Contacts> {
        // Todo: We might want to order the data by Entity ID so that entity1, point1 etc. are for the "smaller"
        // entity ID. This requires changes elsewhere as well though.
        self.0
            .insert((contacts.entity1, contacts.entity2), contacts)
    }

    /// Extends [`Collisions`] with all collision pairs in the given iterable.
    ///
    /// This is mostly equivalent to calling [`insert_collision_pair`](#method.insert_collision_pair)
    /// for each of the collision pairs.
    pub fn extend<I: IntoIterator<Item = Contacts>>(&mut self, collisions: I) {
        // (Note: this is a copy of `std`/`hashbrown`/`indexmap`'s reservation logic.)
        // Keys may be already present or show multiple times in the iterator.
        // Reserve the entire hint lower bound if the map is empty.
        // Otherwise reserve half the hint (rounded up), so the map
        // will only resize twice in the worst case.
        let iter = collisions.into_iter();
        let reserve = if self.get_internal().is_empty() {
            iter.size_hint().0
        } else {
            (iter.size_hint().0 + 1) / 2
        };
        self.get_internal_mut().reserve(reserve);
        iter.for_each(move |contacts| {
            self.insert_collision_pair(contacts);
        });
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
#[derive(Resource, Clone, Debug, Default, Deref, DerefMut, PartialEq)]
struct PreviousCollisions(Collisions);

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
                            let contacts = compute_contacts(
                                *entity1,
                                *entity2,
                                position1.0
                                    + accumulated_translation1.map_or(Vector::default(), |t| t.0),
                                position2.0
                                    + accumulated_translation2.map_or(Vector::default(), |t| t.0),
                                rotation1,
                                rotation2,
                                collider1,
                                collider2,
                                narrow_phase_config.prediction_distance,
                            );

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
                    let contacts = compute_contacts(
                        *entity1,
                        *entity2,
                        position1.0 + accumulated_translation1.map_or(Vector::default(), |t| t.0),
                        position2.0 + accumulated_translation2.map_or(Vector::default(), |t| t.0),
                        rotation1,
                        rotation2,
                        collider1,
                        collider2,
                        narrow_phase_config.prediction_distance,
                    );

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
    mut colliders: Query<(&mut CollidingEntities, Has<Sleeping>)>,
    mut collisions: ResMut<Collisions>,
    mut previous_collisions: ResMut<PreviousCollisions>,
    mut collision_ev_writer: EventWriter<Collision>,
    mut collision_started_ev_writer: EventWriter<CollisionStarted>,
    mut collision_ended_ev_writer: EventWriter<CollisionEnded>,
) {
    let mut ended_collisions = HashSet::<(Entity, Entity)>::new();
    for ((entity1, entity2), contacts) in collisions.get_internal_mut().iter_mut() {
        // Bodies were penetrating during this physics frame
        if contacts.during_current_frame {
            // Send collision event.
            collision_ev_writer.send(Collision(contacts.clone()));

            if let Ok([(mut entities1, sleeping1), (mut entities2, sleeping2)]) =
                colliders.get_many_mut([*entity1, *entity2])
            {
                // Get or insert the previous contacts.
                // If the bodies weren't penetrating previously, they started colliding this frame.
                if let Some(previous_contacts) = previous_collisions
                    .get_internal_mut()
                    .get_mut(&(*entity1, *entity2))
                {
                    if !previous_contacts.during_current_frame {
                        previous_contacts.during_current_frame = true;
                        collision_started_ev_writer.send(CollisionStarted(*entity1, *entity2));
                        entities1.insert(*entity2);
                        entities2.insert(*entity1);
                    }
                } else {
                    previous_collisions.insert_collision_pair(contacts.clone());
                    collision_started_ev_writer.send(CollisionStarted(*entity1, *entity2));
                    entities1.insert(*entity2);
                    entities2.insert(*entity1);
                }

                if sleeping1 || sleeping2 {
                    continue;
                }
            } else {
                // One of the colliders despawned, collision ended
                ended_collisions.insert((*entity1, *entity2));
                collision_ended_ev_writer.send(CollisionEnded(*entity1, *entity2));
            }
        } else if let Some(previous_contacts) = previous_collisions.get_mut(*entity1, *entity2) {
            if let Ok([(mut entities1, sleeping1), (mut entities2, sleeping2)]) =
                colliders.get_many_mut([*entity1, *entity2])
            {
                if sleeping1 || sleeping2 {
                    contacts.during_current_frame = true;
                    continue;
                }
                entities1.remove(entity2);
                entities2.remove(entity1);
                ended_collisions.insert((*entity1, *entity2));
            }

            collision_ended_ev_writer.send(CollisionEnded(*entity1, *entity2));

            // Reset previous collision state to match current collision state.
            previous_contacts.during_current_frame = false;
        }
    }

    // Only retain previous collisions with entities from current collisions
    previous_collisions.retain(|key, _| collisions.0.contains_key(key));

    // Clear collisions at the end of each frame to avoid unnecessary iteration and memory usage
    collisions.retain(|key, _| !ended_collisions.contains(key));
}

fn wake_up_on_collision_ended(
    mut commands: Commands,
    previous_collisions: Res<PreviousCollisions>,
    mut colliding: Query<&CollidingEntities, (Changed<Position>, Without<Sleeping>)>,
    mut removed_colliders: RemovedComponents<Collider>,
    mut sleeping: Query<(Entity, &CollidingEntities, &mut TimeSleeping), With<Sleeping>>,
) {
    // Wake up bodies when a body they're colliding with moves
    for colliding_entities1 in colliding.iter_mut() {
        let mut query = sleeping.iter_many_mut(colliding_entities1.iter());
        if let Some((entity2, _, mut time_sleeping)) = query.fetch_next() {
            commands.entity(entity2).remove::<Sleeping>();
            time_sleeping.0 = 0.0;
        }
    }

    // Wake up bodies when a body they're colliding with is despawned
    for entity in removed_colliders.iter() {
        let mut query =
            sleeping.iter_many_mut(previous_collisions.iter().filter_map(move |contacts| {
                if contacts.entity1 == entity {
                    Some(contacts.entity2)
                } else if contacts.entity2 == entity {
                    Some(contacts.entity1)
                } else {
                    None
                }
            }));

        if let Some((entity2, _, mut time_sleeping)) = query.fetch_next() {
            commands.entity(entity2).remove::<Sleeping>();
            time_sleeping.0 = 0.0;
        }
    }
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
    /// First entity in the contact.
    pub entity1: Entity,
    /// Second entity in the contact.
    pub entity2: Entity,
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
    prediction_distance: Scalar,
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
        prediction_distance,
        &mut manifolds,
        &mut None,
    );
    Contacts {
        entity1,
        entity2,
        manifolds: manifolds
            .iter()
            .filter_map(|manifold| {
                let subpos1 = manifold.subshape_pos1.unwrap_or_default();
                let subpos2 = manifold.subshape_pos2.unwrap_or_default();
                let normal1: Vector = subpos1
                    .rotation
                    .transform_vector(&manifold.local_n1)
                    .normalize()
                    .into();
                let normal2: Vector = subpos2
                    .rotation
                    .transform_vector(&manifold.local_n2)
                    .normalize()
                    .into();

                // Make sure normals are valid
                if !normal1.is_normalized() || !normal2.is_normalized() {
                    return None;
                }

                Some(ContactManifold {
                    entity1,
                    entity2,
                    normal1,
                    normal2,
                    contacts: manifold
                        .contacts()
                        .iter()
                        .map(|contact| ContactData {
                            point1: subpos1.transform_point(&contact.local_p1).into(),
                            point2: subpos2.transform_point(&contact.local_p2).into(),
                            normal1,
                            normal2,
                            penetration: -contact.dist,
                        })
                        .collect(),
                })
            })
            .collect(),
        during_current_frame: true,
        during_current_substep: true,
    }
}
