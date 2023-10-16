//! Collects pairs of potentially colliding entities into [`BroadCollisionPairs`] using
//! [AABB](ColliderAabb) intersection checks.
//!
//! See [`BroadPhasePlugin`].

use crate::prelude::*;
use bevy::prelude::*;

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

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            (
                update_aabb,
                update_aabb_intervals,
                add_new_aabb_intervals,
                collect_collision_pairs,
            )
                .chain()
                .in_set(PhysicsStepSet::BroadPhase),
        );
    }
}

type AABBChanged = Or<(
    Changed<Position>,
    Changed<Rotation>,
    Changed<LinearVelocity>,
    Changed<AngularVelocity>,
)>;

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
#[allow(clippy::type_complexity)]
fn update_aabb(
    mut bodies: Query<
        (
            &Collider,
            &mut ColliderAabb,
            &Position,
            &Rotation,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        AABBChanged,
    >,
    dt: Res<DeltaTime>,
) {
    // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
    let safety_margin_factor = 2.0 * dt.0;

    for (collider, mut aabb, pos, rot, lin_vel, ang_vel) in &mut bodies {
        let lin_vel = lin_vel.map_or(Vector::ZERO, |v| v.0);

        #[cfg(feature = "2d")]
        let ang_vel_magnitude = ang_vel.map_or(0.0, |v| v.0.abs());
        #[cfg(feature = "3d")]
        let ang_vel_magnitude = ang_vel.map_or(0.0, |v| v.0.length());

        // Compute AABB half extents and center
        let computed_aabb = collider
            .get_shape()
            .compute_aabb(&utils::make_isometry(*pos, *rot));
        let half_extents = Vector::from(computed_aabb.half_extents());
        let center = Vector::from(computed_aabb.center());

        // Todo: Somehow consider the shape of the object for the safety margin
        // caused by angular velocity. For example, balls shouldn't get any safety margin.
        let ang_vel_safety_margin = safety_margin_factor * ang_vel_magnitude;

        // Compute AABB mins and maxs, extending them by a safety margin that depends on the velocity
        // of the body. Linear velocity only extends the AABB in the movement direction.
        let mut mins = center - half_extents - ang_vel_safety_margin;
        mins += safety_margin_factor * lin_vel.min(Vector::ZERO);
        let mut maxs = center + half_extents + ang_vel_safety_margin;
        maxs += safety_margin_factor * lin_vel.max(Vector::ZERO);

        aabb.mins.coords = mins.into();
        aabb.maxs.coords = maxs.into();
    }
}

/// True if the rigid body hasn't moved.
type IsBodyInactive = bool;

/// Entities with [`ColliderAabb`]s sorted along an axis by their extents.
#[derive(Resource, Default)]
struct AabbIntervals(Vec<(Entity, ColliderAabb, CollisionLayers, IsBodyInactive)>);

/// Updates [`AabbIntervals`] to keep them in sync with the [`ColliderAabb`]s.
fn update_aabb_intervals(
    aabbs: Query<(&ColliderAabb, Ref<Position>, Ref<Rotation>)>,
    mut intervals: ResMut<AabbIntervals>,
) {
    intervals.0.retain_mut(|(entity, aabb, _, is_inactive)| {
        if let Ok((new_aabb, position, rotation)) = aabbs.get(*entity) {
            *aabb = *new_aabb;
            *is_inactive = !position.is_changed() && !rotation.is_changed();
            true
        } else {
            false
        }
    });
}

/// Adds new [`ColliderAabb`]s to [`AabbIntervals`].
#[allow(clippy::type_complexity)]
fn add_new_aabb_intervals(
    aabbs: Query<
        (
            Entity,
            &ColliderAabb,
            Option<&RigidBody>,
            Option<&CollisionLayers>,
        ),
        Added<ColliderAabb>,
    >,
    mut intervals: ResMut<AabbIntervals>,
) {
    let aabbs = aabbs.iter().map(|(ent, aabb, rb, layers)| {
        (
            ent,
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
    insertion_sort(&mut intervals.0, |a, b| a.1.mins.x > b.1.mins.x);

    // Clear broad phase collisions from previous iteration.
    broad_collision_pairs.clear();

    // Find potential collisions by checking for AABB intersections along all axes.
    for (i, (ent1, aabb1, layers1, inactive1)) in intervals.0.iter().enumerate() {
        for (ent2, aabb2, layers2, inactive2) in intervals.0.iter().skip(i + 1) {
            // No collisions between bodies that haven't moved or colliders with incompatible layers
            if (*inactive1 && *inactive2) || !layers1.interacts_with(*layers2) {
                continue;
            }

            // x doesn't intersect
            if aabb2.mins.x > aabb1.maxs.x {
                break;
            }

            // y doesn't intersect
            if aabb1.mins.y > aabb2.maxs.y || aabb1.maxs.y < aabb2.mins.y {
                continue;
            }

            #[cfg(feature = "3d")]
            // z doesn't intersect
            if aabb1.mins.z > aabb2.maxs.z || aabb1.maxs.z < aabb2.mins.z {
                continue;
            }

            broad_collision_pairs.push((*ent1, *ent2));
        }
    }
}

/// Sorts a list iteratively using comparisons. In an ascending sort order, when a smaller value is encountered, it is moved lower in the list until it is larger than the item before it.
///
/// This is relatively slow for large lists, but very efficient in cases where the list is already mostly sorted.
fn insertion_sort<T>(items: &mut Vec<T>, comparison: fn(&T, &T) -> bool) {
    for i in 1..items.len() {
        let mut j = i;
        while j > 0 && comparison(&items[j - 1], &items[j]) {
            items.swap(j - 1, j);
            j -= 1;
        }
    }
}
