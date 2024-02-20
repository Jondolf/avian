//! Collects pairs of potentially colliding entities into [`BroadCollisionPairs`] using
//! [AABB](ColliderAabb) intersection checks.
//!
//! See [`BroadPhasePlugin`].

use crate::prelude::*;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};

/// Collects pairs of potentially colliding entities into [`BroadCollisionPairs`] using
/// [AABB](ColliderAabb) intersection checks. This speeds up narrow phase collision detection,
/// as the number of precise collision checks required is greatly reduced.
///
/// Currently, the broad phase uses the [sweep and prune](https://en.wikipedia.org/wiki/Sweep_and_prune) algorithm.
///
/// The broad phase systems run in [`PhysicsStepSet::BroadPhase`].
pub struct BroadPhasePlugin;

impl Plugin for BroadPhasePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AabbIntervals>();

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
            .add_systems(collect_collision_pairs.in_set(BroadPhaseSet::CollectCollisions));
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
#[derive(Reflect, Resource, Default, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Resource)]
pub struct BroadCollisionPairs(pub Vec<(Entity, Entity)>);

/// True if the rigid body hasn't moved.
type IsBodyInactive = bool;

/// Entities with [`ColliderAabb`]s sorted along an axis by their extents.
#[derive(Resource, Default)]
struct AabbIntervals(
    Vec<(
        Entity,
        ColliderParent,
        ColliderAabb,
        CollisionLayers,
        IsBodyInactive,
    )>,
);

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
    aabbs: Query<(
        &ColliderAabb,
        Option<&ColliderParent>,
        Option<&CollisionLayers>,
        Ref<Position>,
        Ref<Rotation>,
    )>,
    rbs: Query<&RigidBody>,
    mut intervals: ResMut<AabbIntervals>,
) {
    intervals.0.retain_mut(
        |(collider_entity, collider_parent, aabb, layers, is_inactive)| {
            if let Ok((new_aabb, new_parent, new_layers, position, rotation)) =
                aabbs.get(*collider_entity)
            {
                *aabb = *new_aabb;
                *collider_parent = new_parent.map_or(ColliderParent(*collider_entity), |p| *p);
                *layers = new_layers.map_or(CollisionLayers::default(), |layers| *layers);

                let is_static =
                    new_parent.is_some_and(|p| rbs.get(p.get()).is_ok_and(RigidBody::is_static));
                *is_inactive = is_static || (!position.is_changed() && !rotation.is_changed());

                true
            } else {
                false
            }
        },
    );
}

/// Adds new [`ColliderAabb`]s to [`AabbIntervals`].
#[allow(clippy::type_complexity)]
fn add_new_aabb_intervals(
    aabbs: Query<
        (
            Entity,
            Option<&ColliderParent>,
            &ColliderAabb,
            Option<&RigidBody>,
            Option<&CollisionLayers>,
        ),
        Added<ColliderAabb>,
    >,
    mut intervals: ResMut<AabbIntervals>,
) {
    let aabbs = aabbs.iter().map(|(ent, parent, aabb, rb, layers)| {
        (
            ent,
            parent.map_or(ColliderParent(ent), |p| *p),
            *aabb,
            // Default to treating collider as immovable/static for filtering unnecessary collision checks
            layers.map_or(CollisionLayers::default(), |layers| *layers),
            rb.map_or(false, |rb| rb.is_static()),
        )
    });
    intervals.0.extend(aabbs);
}

/// Collects bodies that are potentially colliding.
fn collect_collision_pairs(
    intervals: ResMut<AabbIntervals>,
    mut broad_collision_pairs: ResMut<BroadCollisionPairs>,
) {
    sweep_and_prune(intervals, &mut broad_collision_pairs.0);
}

/// Sorts the entities by their minimum extents along an axis and collects the entity pairs that have intersecting AABBs.
///
/// Sweep and prune exploits temporal coherence, as bodies are unlikely to move significantly between two simulation steps. Insertion sort is used, as it is good at sorting nearly sorted lists efficiently.
fn sweep_and_prune(
    mut intervals: ResMut<AabbIntervals>,
    broad_collision_pairs: &mut Vec<(Entity, Entity)>,
) {
    // Sort bodies along the x-axis using insertion sort, a sorting algorithm great for sorting nearly sorted lists.
    insertion_sort(&mut intervals.0, |a, b| a.2.min.x > b.2.min.x);

    // Clear broad phase collisions from previous iteration.
    broad_collision_pairs.clear();

    // Find potential collisions by checking for AABB intersections along all axes.
    for (i, (ent1, parent1, aabb1, layers1, inactive1)) in intervals.0.iter().enumerate() {
        for (ent2, parent2, aabb2, layers2, inactive2) in intervals.0.iter().skip(i + 1) {
            // x doesn't intersect; check this first so we can discard as soon as possible
            if aabb2.min.x > aabb1.max.x {
                break;
            }

            // No collisions between bodies that haven't moved or colliders with incompatible layers or colliders with the same parent
            if (*inactive1 && *inactive2) || !layers1.interacts_with(*layers2) || parent1 == parent2
            {
                continue;
            }

            // y doesn't intersect
            if aabb1.min.y > aabb2.max.y || aabb1.max.y < aabb2.min.y {
                continue;
            }

            #[cfg(feature = "3d")]
            // z doesn't intersect
            if aabb1.min.z > aabb2.max.z || aabb1.max.z < aabb2.min.z {
                continue;
            }

            broad_collision_pairs.push((*ent1, *ent2));
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
