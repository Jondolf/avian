//! Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
//! the number of potential contacts for the [narrow phase](crate::narrow_phase).
//!
//! See [`BroadPhasePlugin`].

use std::marker::PhantomData;

use crate::{data_structures::pair_key::PairKey, prelude::*};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        system::{lifetimeless::Read, StaticSystemParam, SystemParamItem},
    },
    prelude::*,
    utils::HashSet,
};

/// Finds pairs of entities with overlapping [`ColliderAabb`]s to reduce
/// the number of potential contacts for the [narrow phase](crate::narrow_phase).
///
/// Intersection pairs found by the broad phase are added to the [`BroadPhasePairSet`]
/// and [`BroadPhaseAddedPairs`] resources.
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
        app.register_type::<(BroadPhasePairSet, BroadPhaseAddedPairs)>();

        app.init_resource::<BroadPhasePairSet>()
            .init_resource::<BroadPhaseAddedPairs>()
            .init_resource::<AabbIntervals>();

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

    fn finish(&self, app: &mut App) {
        // Register timer and counter diagnostics for collision detection.
        app.register_physics_diagnostics::<CollisionDiagnostics>();
    }
}

/// System sets for systems running in [`PhysicsStepSet::BroadPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BroadPhaseSet {
    /// Runs at the start of the broad phase. Empty by default.
    First,
    /// Updates acceleration structures and other data needed for broad phase collision detection.
    UpdateStructures,
    /// Finds pairs of entities with overlapping [`ColliderAabb`]s, and adds them
    /// to the [`BroadPhasePairSet`] and [`BroadPhaseAddedPairs`] resources.
    CollectCollisions,
    /// Runs at the end of the broad phase. Empty by default.
    Last,
}

/// A set of all collider pairs with intersecting AABBs found by the broad phase.
///
/// This is used for efficient lookup of broad phase pairs.
#[derive(Reflect, Resource, Debug, Default, Deref, DerefMut)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Resource)]
pub struct BroadPhasePairSet(pub HashSet<PairKey>);

/// A list of entity pairs with intersecting AABBs found by the broad phase
/// during the current time step.
///
/// Only contains pairs that are not already in the [`BroadPhasePairSet`] resource.
#[derive(Reflect, Resource, Debug, Default, Deref, DerefMut)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Resource)]
pub struct BroadPhaseAddedPairs(pub Vec<(Entity, Entity)>);

/// Entities with [`ColliderAabb`]s sorted along an axis by their extents.
#[derive(Resource, Default)]
struct AabbIntervals(
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
        /// Set if the body is sleeping or static.
        const IS_INACTIVE = 1 << 0;
        /// Set if [`CollisionHooks::filter_pairs`] should be called for this entity.
        const CUSTOM_FILTER = 1 << 1;
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
    aabbs: Query<
        (
            &ColliderAabb,
            Option<&ColliderParent>,
            Option<&CollisionLayers>,
            Option<&ActiveCollisionHooks>,
            Has<Sleeping>,
        ),
        Without<ColliderDisabled>,
    >,
    rbs: Query<&RigidBody>,
    mut intervals: ResMut<AabbIntervals>,
) {
    intervals
        .0
        .retain_mut(|(collider_entity, collider_parent, aabb, layers, flags)| {
            if let Ok((new_aabb, new_parent, new_layers, hooks, is_sleeping)) =
                aabbs.get(*collider_entity)
            {
                if !new_aabb.min.is_finite() || !new_aabb.max.is_finite() {
                    return false;
                }

                *aabb = *new_aabb;
                *collider_parent = new_parent.map_or(ColliderParent(*collider_entity), |p| *p);
                *layers = new_layers.map_or(CollisionLayers::default(), |layers| *layers);

                let is_static =
                    new_parent.is_some_and(|p| rbs.get(p.get()).is_ok_and(RigidBody::is_static));

                flags.set(AabbIntervalFlags::IS_INACTIVE, is_static || is_sleeping);
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

type AabbIntervalQueryData = (
    Entity,
    Option<Read<ColliderParent>>,
    Read<ColliderAabb>,
    Option<Read<RigidBody>>,
    Option<Read<CollisionLayers>>,
    Option<Read<ActiveCollisionHooks>>,
);

/// Adds new [`ColliderAabb`]s to [`AabbIntervals`].
#[allow(clippy::type_complexity)]
fn add_new_aabb_intervals(
    added_aabbs: Query<AabbIntervalQueryData, (Added<ColliderAabb>, Without<ColliderDisabled>)>,
    aabbs: Query<AabbIntervalQueryData>,
    mut intervals: ResMut<AabbIntervals>,
    mut re_enabled_colliders: RemovedComponents<ColliderDisabled>,
) {
    let re_enabled_aabbs = aabbs.iter_many(re_enabled_colliders.read());
    let aabbs = added_aabbs.iter().chain(re_enabled_aabbs).map(
        |(entity, parent, aabb, rb, layers, hooks)| {
            let mut flags = AabbIntervalFlags::empty();
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

/// Finds pairs of entities with overlapping [`ColliderAabb`]s, and adds them
/// to the [`BroadPhasePairSet`] and [`BroadPhaseAddedPairs`] resources.
fn collect_collision_pairs<H: CollisionHooks>(
    intervals: ResMut<AabbIntervals>,
    mut broad_phase_pairs: ResMut<BroadPhasePairSet>,
    mut added_broad_phase_pairs: ResMut<BroadPhaseAddedPairs>,
    hooks: StaticSystemParam<H>,
    mut commands: Commands,
    mut diagnostics: ResMut<CollisionDiagnostics>,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    let start = bevy::utils::Instant::now();

    sweep_and_prune::<H>(
        intervals,
        &mut broad_phase_pairs.0,
        &mut added_broad_phase_pairs.0,
        &mut hooks.into_inner(),
        &mut commands,
    );

    diagnostics.broad_phase = start.elapsed();
}

/// Sorts the entities by their minimum extents along an axis and collects the entity pairs that have intersecting AABBs.
///
/// Sweep and prune exploits temporal coherence, as bodies are unlikely to move significantly between two simulation steps. Insertion sort is used, as it is good at sorting nearly sorted lists efficiently.
fn sweep_and_prune<H: CollisionHooks>(
    mut intervals: ResMut<AabbIntervals>,
    broad_phase_pairs: &mut HashSet<PairKey>,
    added_broad_phase_pairs: &mut Vec<(Entity, Entity)>,
    hooks: &mut H::Item<'_, '_>,
    commands: &mut Commands,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    // Sort bodies along the x-axis using insertion sort, a sorting algorithm great for sorting nearly sorted lists.
    insertion_sort(&mut intervals.0, |a, b| a.2.min.x > b.2.min.x);

    // Clear the list of added broad phase pairs.
    added_broad_phase_pairs.clear();

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

            // Create a key for this pair of entities.
            // The key is used for fast lookup in the broad phase pair set.
            let pair_key = PairKey::new(entity1.index(), entity2.index());

            // Avoid duplicate pairs.
            if broad_phase_pairs.contains(&pair_key) {
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

            // Create a new collision pair.
            broad_phase_pairs.insert(pair_key);
            added_broad_phase_pairs.push((*entity1, *entity2));
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
