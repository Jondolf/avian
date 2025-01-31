//! Collects pairs of potentially colliding entities into [`BroadCollisionPairs`] using
//! [AABB](ColliderAabb) intersection checks.
//!
//! See [`BroadPhasePlugin`].

use std::marker::PhantomData;

use crate::prelude::*;
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        system::{lifetimeless::Read, StaticSystemParam, SystemParamItem},
    },
    prelude::*,
};

/// Collects pairs of potentially colliding entities into [`BroadCollisionPairs`] using
/// [AABB](ColliderAabb) intersection checks. This speeds up narrow phase collision detection,
/// as the number of precise collision checks required is greatly reduced.
///
/// Currently, the broad phase uses the [sweep and prune](https://en.wikipedia.org/wiki/Sweep_and_prune) algorithm.
///
/// The broad phase systems run in [`PhysicsStepSet::BroadPhase`].
///
/// [`CollisionHooks`] can be provided with generics to apply custom filtering for collision pairs.
pub struct BroadPhasePlugin<H: CollisionHooks = ()>(PhantomData<H>);

impl<H: CollisionHooks> Default for BroadPhasePlugin<H> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<H: CollisionHooks + 'static> Plugin for BroadPhasePlugin<H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(&self, app: &mut App) {
        app.configure_sets(
            PhysicsSchedule,
            (
                BroadPhaseSet::First,
                BroadPhaseSet::UpdateStructures,
                BroadPhaseSet::CollectCollisions,
                BroadPhaseSet::Last,
            )
                .chain()
                .in_set(PhysicsStepSet::BroadPhase),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            (update_aabb_intervals, add_new_aabb_intervals)
                .chain()
                .in_set(BroadPhaseSet::UpdateStructures),
        );

        physics_schedule
            .add_systems(collect_collision_pairs::<H>.in_set(BroadPhaseSet::CollectCollisions));
    }
}

/// System sets for systems running in [`PhysicsStepSet::BroadPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BroadPhaseSet {
    /// Runs at the start of the broad phase. Empty by default.
    First,
    /// Updates acceleration structures and other data needed for broad phase collision detection.
    UpdateStructures,
    /// Detects potential intersections between entities and adds them to the [`BroadCollisionPairs`] resource.
    CollectCollisions,
    /// Runs at the end of the broad phase. Empty by default.
    Last,
}

/// A list of entity pairs for potential collisions collected during the broad phase.
#[derive(Component, Reflect, Debug, Default, Deref, DerefMut)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug)]
pub struct BroadCollisionPairs(pub Vec<(Entity, Entity)>);

/// Contains the entities whose AABBs intersect the AABB of this entity.
/// Updated automatically during broad phase collision detection.
///
/// Note that this component is only added to bodies with [`SweptCcd`] by default,
/// but can be added to any entity.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct AabbIntersections(pub Vec<Entity>);

/// Entities with [`ColliderAabb`]s sorted along an axis by their extents.
#[derive(Component, Default)]
pub struct AabbIntervals(
    Vec<(
        Entity,
        ColliderParent,
        ColliderAabb,
        CollisionLayers,
        AabbIntervalFlags,
    )>,
);

bitflags::bitflags! {
    /// Flags for AABB intervals in the broad phase.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct AabbIntervalFlags: u8 {
        /// Set if [`AabbIntersections`] should be stored for this entity.
        const STORE_INTERSECTIONS = 1 << 0;
        /// Set if the body is sleeping or static.
        const IS_INACTIVE = 1 << 1;
        /// Set if [`CollisionHooks::filter_pairs`] should be called for this entity.
        const CUSTOM_FILTER = 1 << 2;
    }
}

impl MapEntities for AabbIntervals {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        for interval in self.0.iter_mut() {
            interval.0 = entity_mapper.map_entity(interval.0);
        }
    }
}

/// Updates [`AabbIntervals`] to keep them in sync with the [`ColliderAabb`]s.
#[allow(clippy::type_complexity)]
fn update_aabb_intervals(
    mut interval_query: Query<&mut AabbIntervals>,
    aabbs: Query<
        (
            &ColliderAabb,
            Option<&ColliderParent>,
            Option<&CollisionLayers>,
            Option<&ActiveCollisionHooks>,
            Has<AabbIntersections>,
            Has<Sleeping>,
        ),
        Without<ColliderDisabled>,
    >,
    rbs: Query<&RigidBody>,
) {
    for mut intervals in &mut interval_query {
        intervals
            .0
            .retain_mut(|(collider_entity, collider_parent, aabb, layers, flags)| {
                if let Ok((
                    new_aabb,
                    new_parent,
                    new_layers,
                    hooks,
                    new_store_intersections,
                    is_sleeping,
                )) = aabbs.get(*collider_entity)
                {
                    if !new_aabb.min.is_finite() || !new_aabb.max.is_finite() {
                        return false;
                    }

                    *aabb = *new_aabb;
                    *collider_parent = new_parent.map_or(ColliderParent(*collider_entity), |p| *p);
                    *layers = new_layers.map_or(CollisionLayers::default(), |layers| *layers);

                    let is_static = new_parent
                        .is_some_and(|p| rbs.get(p.get()).is_ok_and(RigidBody::is_static));

                    flags.set(AabbIntervalFlags::IS_INACTIVE, is_static || is_sleeping);
                    flags.set(
                        AabbIntervalFlags::STORE_INTERSECTIONS,
                        new_store_intersections,
                    );
                    flags.set(
                        AabbIntervalFlags::CUSTOM_FILTER,
                        hooks.is_some_and(|h| h.contains(ActiveCollisionHooks::FILTER_PAIRS)),
                    );

                    true
                } else {
                    false
                }
            });
    }
}

type AabbIntervalQueryData = (
    Entity,
    Option<Read<ColliderParent>>,
    Read<ColliderAabb>,
    Option<Read<RigidBody>>,
    Option<Read<CollisionLayers>>,
    Option<Read<ActiveCollisionHooks>>,
    Has<AabbIntersections>,
);

/// Adds new [`ColliderAabb`]s to [`AabbIntervals`].
#[allow(clippy::type_complexity)]
fn add_new_aabb_intervals(
    mut physics_world_query: ParamSet<(
        Query<(Entity, &mut AabbIntervals)>,
        Query<(Entity, &mut AabbIntervals), With<MainPhysicsWorld>>,
    )>,
    mut added_aabbs: QueryByIndex<
        PhysicsWorldId,
        AabbIntervalQueryData,
        (Added<ColliderAabb>, Without<ColliderDisabled>),
    >,
    mut aabbs: QueryByIndex<PhysicsWorldId, AabbIntervalQueryData>,
    mut re_enabled_colliders: RemovedComponents<ColliderDisabled>,
) {
    for (physics_world_id, intervals) in &mut physics_world_query.p0() {
        let id = PhysicsWorldId::Id(physics_world_id);
        add_new_aabb_intervals_single(
            added_aabbs.at(&id),
            aabbs.at(&id),
            intervals,
            &mut re_enabled_colliders,
        );
    }

    if let Ok((_, intervals)) = physics_world_query.p1().get_single_mut() {
        add_new_aabb_intervals_single(
            added_aabbs.at(&PhysicsWorldId::Main),
            aabbs.at(&PhysicsWorldId::Main),
            intervals,
            &mut re_enabled_colliders,
        );
    }
}

/// Adds new [`ColliderAabb`]s to [`AabbIntervals`].
#[allow(clippy::type_complexity)]
fn add_new_aabb_intervals_single(
    added_aabbs: Query<
        AabbIntervalQueryData,
        (
            (Added<ColliderAabb>, Without<ColliderDisabled>),
            With<PhysicsWorldId>,
        ),
    >,
    aabbs: Query<AabbIntervalQueryData, ((), With<PhysicsWorldId>)>,
    mut intervals: Mut<AabbIntervals>,
    re_enabled_colliders: &mut RemovedComponents<ColliderDisabled>,
) {
    let re_enabled_aabbs = aabbs.iter_many(re_enabled_colliders.read());
    let aabbs = added_aabbs.iter().chain(re_enabled_aabbs).map(
        |(entity, parent, aabb, rb, layers, hooks, store_intersections)| {
            let mut flags = AabbIntervalFlags::empty();
            flags.set(AabbIntervalFlags::STORE_INTERSECTIONS, store_intersections);
            flags.set(
                AabbIntervalFlags::IS_INACTIVE,
                rb.is_some_and(|rb| rb.is_static()),
            );
            flags.set(
                AabbIntervalFlags::CUSTOM_FILTER,
                hooks.is_some_and(|h| h.contains(ActiveCollisionHooks::FILTER_PAIRS)),
            );
            (
                entity,
                parent.map_or(ColliderParent(entity), |p| *p),
                *aabb,
                layers.map_or(CollisionLayers::default(), |layers| *layers),
                flags,
            )
        },
    );
    intervals.0.extend(aabbs);
}

/// Collects bodies that are potentially colliding.
fn collect_collision_pairs<H: CollisionHooks>(
    mut physics_world_query: Query<(&mut BroadCollisionPairs, &mut AabbIntervals)>,
    mut aabb_intersection_query: Query<&mut AabbIntersections>,
    hooks: StaticSystemParam<H>,
    mut commands: Commands,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    for mut intersections in &mut aabb_intersection_query {
        intersections.clear();
    }

    for (mut broad_collision_pairs, mut intervals) in physics_world_query.iter_mut() {
        sweep_and_prune::<H>(
            &mut intervals,
            &mut broad_collision_pairs.0,
            &mut aabb_intersection_query,
            &hooks,
            &mut commands,
        );
    }
}

/// Sorts the entities by their minimum extents along an axis and collects the entity pairs that have intersecting AABBs.
///
/// Sweep and prune exploits temporal coherence, as bodies are unlikely to move significantly between two simulation steps. Insertion sort is used, as it is good at sorting nearly sorted lists efficiently.
fn sweep_and_prune<H: CollisionHooks>(
    intervals: &mut AabbIntervals,
    broad_collision_pairs: &mut Vec<(Entity, Entity)>,
    aabb_intersection_query: &mut Query<&mut AabbIntersections>,
    hooks: &H::Item<'_, '_>,
    commands: &mut Commands,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    // Sort bodies along the x-axis using insertion sort, a sorting algorithm great for sorting nearly sorted lists.
    insertion_sort(&mut intervals.0, |a, b| a.2.min.x > b.2.min.x);

    // Clear broad phase collisions from previous iteration.
    broad_collision_pairs.clear();

    // Find potential collisions by checking for AABB intersections along all axes.
    for (i, (entity1, parent1, aabb1, layers1, flags1)) in intervals.0.iter().enumerate() {
        for (entity2, parent2, aabb2, layers2, flags2) in intervals.0.iter().skip(i + 1) {
            // x doesn't intersect; check this first so we can discard as soon as possible.
            if aabb2.min.x > aabb1.max.x {
                break;
            }

            // No collisions between bodies that haven't moved or colliders with incompatible layers
            // or colliders with the same parent.
            if flags1
                .intersection(*flags2)
                .contains(AabbIntervalFlags::IS_INACTIVE)
                || !layers1.interacts_with(*layers2)
                || parent1 == parent2
            {
                continue;
            }

            // y doesn't intersect.
            if aabb1.min.y > aabb2.max.y || aabb1.max.y < aabb2.min.y {
                continue;
            }

            #[cfg(feature = "3d")]
            // z doesn't intersect.
            if aabb1.min.z > aabb2.max.z || aabb1.max.z < aabb2.min.z {
                continue;
            }

            // Apply user-defined filter.
            if flags1
                .union(*flags2)
                .contains(AabbIntervalFlags::CUSTOM_FILTER)
            {
                let should_collide = hooks.filter_pairs(*entity1, *entity2, commands);
                if !should_collide {
                    continue;
                }
            }

            // Create a collision pair.
            broad_collision_pairs.push((*entity1, *entity2));

            // TODO: Handle this more efficiently.
            if flags1.contains(AabbIntervalFlags::STORE_INTERSECTIONS) {
                if let Ok(mut intersections) = aabb_intersection_query.get_mut(*entity1) {
                    intersections.push(*entity2);
                }
            }
            if flags2.contains(AabbIntervalFlags::STORE_INTERSECTIONS) {
                if let Ok(mut intersections) = aabb_intersection_query.get_mut(*entity2) {
                    intersections.push(*entity1);
                }
            }
        }
    }
}

/// Sorts a list iteratively using comparisons. In an ascending sort order, when a smaller value is encountered, it is moved lower in the list until it is larger than the item before it.
///
/// This is relatively slow for large lists, but very efficient in cases where the list is already mostly sorted.
fn insertion_sort<T>(items: &mut [T], comparison: fn(&T, &T) -> bool) {
    for i in 1..items.len() {
        let mut j = i;
        while j > 0 && comparison(&items[j - 1], &items[j]) {
            items.swap(j - 1, j);
            j -= 1;
        }
    }
}
