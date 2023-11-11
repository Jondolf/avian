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
        app.init_resource::<AabbIntervals2d>()
            .init_resource::<AabbIntervals3d>();

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            (
                update_aabb_2d,
                update_aabb_3d,
                update_aabb_intervals_2d,
                update_aabb_intervals_3d,
                add_new_aabb_intervals_2d,
                add_new_aabb_intervals_3d,
                collect_collision_pairs_2d,
                collect_collision_pairs_3d,
            )
                .chain()
                .in_set(PhysicsStepSet::BroadPhase),
        );
    }
}

/// A list of entity pairs for potential collisions collected during the broad phase.
#[derive(Reflect, Resource, Default, Debug)]
#[reflect(Resource)]
pub struct BroadCollisionPairs2d(pub Vec<(Entity, Entity)>);

/// A list of entity pairs for potential collisions collected during the broad phase.
#[derive(Reflect, Resource, Default, Debug)]
#[reflect(Resource)]
pub struct BroadCollisionPairs3d(pub Vec<(Entity, Entity)>);

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
#[allow(clippy::type_complexity)]
fn update_aabb_2d(
    mut colliders: Query<
        (
            &Collider2d,
            &mut ColliderAabb2d,
            &Position2d,
            &Rotation2d,
            Option<&ColliderParent>,
            Option<&LinearVelocity2d>,
            Option<&AngularVelocity2d>,
        ),
        Or<(
            Changed<Position2d>,
            Changed<Rotation2d>,
            Changed<LinearVelocity2d>,
            Changed<AngularVelocity2d>,
            Changed<Collider2d>,
        )>,
    >,
    parent_velocity: Query<
        (
            &Position2d,
            Option<&LinearVelocity2d>,
            Option<&AngularVelocity2d>,
        ),
        With<Children>,
    >,
    dt: Res<Time>,
    narrow_phase_config: Option<Res<NarrowPhaseConfig2d>>,
) {
    // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
    let safety_margin_factor = 2.0 * dt.delta_seconds_adjusted();

    for (collider, mut aabb, pos, rot, collider_parent, lin_vel, ang_vel) in &mut colliders {
        let (lin_vel, ang_vel) = if let (Some(lin_vel), Some(ang_vel)) = (lin_vel, ang_vel) {
            (*lin_vel, *ang_vel)
        } else if let Some(Ok((parent_pos, Some(lin_vel), Some(ang_vel)))) =
            collider_parent.map(|p| parent_velocity.get(p.get()))
        {
            // If the rigid body is rotating, off-center colliders will orbit around it,
            // which affects their linear velocities. We need to compute the linear velocity
            // at the offset position.
            // TODO: This assumes that the colliders would continue moving in the same direction,
            //       but because they are orbiting, the direction will change. We should take
            //       into account the uniform circular motion.
            let offset = pos.0 - parent_pos.0;
            let vel_at_offset = lin_vel.at_point(offset, ang_vel.0);
            (LinearVelocity2d(vel_at_offset), *ang_vel)
        } else {
            (LinearVelocity2d::ZERO, AngularVelocity2d::ZERO)
        };

        // Compute current isometry and predicted isometry for next feame
        let start_iso = utils::make_isometry_2d(*pos, *rot);
        let end_iso = utils::make_isometry_2d(
            pos.0 + lin_vel.0 * safety_margin_factor,
            *rot + Rotation2d::from_radians(safety_margin_factor * ang_vel.0),
        );

        // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
        aabb.0 = collider
            .shape_scaled()
            .compute_swept_aabb(&start_iso, &end_iso);

        // Add narrow phase prediction distance to AABBs to avoid missed collisions
        let prediction_distance = if let Some(ref config) = narrow_phase_config {
            config.prediction_distance
        } else {
            1.0
        };
        aabb.maxs.x += prediction_distance;
        aabb.mins.x -= prediction_distance;
        aabb.maxs.y += prediction_distance;
        aabb.mins.y -= prediction_distance;
    }
}

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
#[allow(clippy::type_complexity)]
fn update_aabb_3d(
    mut colliders: Query<
        (
            &Collider3d,
            &mut ColliderAabb3d,
            &Position3d,
            &Rotation3d,
            Option<&ColliderParent>,
            Option<&LinearVelocity3d>,
            Option<&AngularVelocity3d>,
        ),
        Or<(
            Changed<Position3d>,
            Changed<Rotation3d>,
            Changed<LinearVelocity3d>,
            Changed<AngularVelocity3d>,
            Changed<Collider3d>,
        )>,
    >,
    parent_velocity: Query<
        (
            &Position3d,
            Option<&LinearVelocity3d>,
            Option<&AngularVelocity3d>,
        ),
        With<Children>,
    >,
    dt: Res<Time>,
    narrow_phase_config: Option<Res<NarrowPhaseConfig3d>>,
) {
    // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
    let safety_margin_factor = 2.0 * dt.delta_seconds_adjusted();

    for (collider, mut aabb, pos, rot, collider_parent, lin_vel, ang_vel) in &mut colliders {
        let (lin_vel, ang_vel) = if let (Some(lin_vel), Some(ang_vel)) = (lin_vel, ang_vel) {
            (*lin_vel, *ang_vel)
        } else if let Some(Ok((parent_pos, Some(lin_vel), Some(ang_vel)))) =
            collider_parent.map(|p| parent_velocity.get(p.get()))
        {
            // If the rigid body is rotating, off-center colliders will orbit around it,
            // which affects their linear velocities. We need to compute the linear velocity
            // at the offset position.
            // TODO: This assumes that the colliders would continue moving in the same direction,
            //       but because they are orbiting, the direction will change. We should take
            //       into account the uniform circular motion.
            let offset = pos.0 - parent_pos.0;
            let vel_at_offset = lin_vel.at_point(offset, ang_vel.0);
            (LinearVelocity3d(vel_at_offset), *ang_vel)
        } else {
            (LinearVelocity3d::ZERO, AngularVelocity3d::ZERO)
        };

        // Compute current isometry and predicted isometry for next feame
        let start_iso = utils::make_isometry_3d(*pos, *rot);
        let end_iso = {
            let q = Quaternion::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
            let (x, y, z, w) = (
                rot.x + safety_margin_factor * 0.5 * q.x,
                rot.y + safety_margin_factor * 0.5 * q.y,
                rot.z + safety_margin_factor * 0.5 * q.z,
                rot.w + safety_margin_factor * 0.5 * q.w,
            );
            utils::make_isometry_3d(
                pos.0 + lin_vel.0 * safety_margin_factor,
                Quaternion::from_xyzw(x, y, z, w).normalize(),
            )
        };

        // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
        aabb.0 = collider
            .shape_scaled()
            .compute_swept_aabb(&start_iso, &end_iso);

        // Add narrow phase prediction distance to AABBs to avoid missed collisions
        let prediction_distance = if let Some(ref config) = narrow_phase_config {
            config.prediction_distance
        } else {
            0.005
        };
        aabb.maxs.x += prediction_distance;
        aabb.mins.x -= prediction_distance;
        aabb.maxs.y += prediction_distance;
        aabb.mins.y -= prediction_distance;
        aabb.maxs.z += prediction_distance;
        aabb.mins.z -= prediction_distance;
    }
}

/// True if the rigid body hasn't moved.
type IsBodyInactive = bool;

/// Entities with [`ColliderAabb`]s sorted along an axis by their extents.
#[derive(Resource, Default)]
struct AabbIntervals2d(Vec<(Entity, ColliderAabb2d, CollisionLayers, IsBodyInactive)>);

/// Updates [`AabbIntervals`] to keep them in sync with the [`ColliderAabb`]s.
#[allow(clippy::type_complexity)]
fn update_aabb_intervals_2d(
    aabbs: Query<(
        &ColliderAabb2d,
        Option<&CollisionLayers>,
        Ref<Position2d>,
        Ref<Rotation2d>,
    )>,
    mut intervals: ResMut<AabbIntervals2d>,
) {
    intervals
        .0
        .retain_mut(|(entity, aabb, layers, is_inactive)| {
            if let Ok((new_aabb, new_layers, position, rotation)) = aabbs.get(*entity) {
                *aabb = *new_aabb;
                *layers = new_layers.map_or(CollisionLayers::default(), |layers| *layers);
                *is_inactive = !position.is_changed() && !rotation.is_changed();
                true
            } else {
                false
            }
        });
}

/// Adds new [`ColliderAabb`]s to [`AabbIntervals`].
#[allow(clippy::type_complexity)]
fn add_new_aabb_intervals_2d(
    aabbs: Query<
        (
            Entity,
            &ColliderAabb2d,
            Option<&RigidBody>,
            Option<&CollisionLayers>,
        ),
        Added<ColliderAabb2d>,
    >,
    mut intervals: ResMut<AabbIntervals2d>,
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
fn collect_collision_pairs_2d(
    intervals: ResMut<AabbIntervals2d>,
    mut broad_collision_pairs: ResMut<BroadCollisionPairs2d>,
) {
    sweep_and_prune_2d(intervals, &mut broad_collision_pairs.0);
}

/// Sorts the entities by their minimum extents along an axis and collects the entity pairs that have intersecting AABBs.
///
/// Sweep and prune exploits temporal coherence, as bodies are unlikely to move significantly between two simulation steps. Insertion sort is used, as it is good at sorting nearly sorted lists efficiently.
fn sweep_and_prune_2d(
    mut intervals: ResMut<AabbIntervals2d>,
    broad_collision_pairs: &mut Vec<(Entity, Entity)>,
) {
    // Sort bodies along the x-axis using insertion sort, a sorting algorithm great for sorting nearly sorted lists.
    insertion_sort(&mut intervals.0, |a, b| a.1.mins.x > b.1.mins.x);

    // Clear broad phase collisions from previous iteration.
    broad_collision_pairs.clear();

    // Find potential collisions by checking for AABB intersections along all axes.
    for (i, (ent1, aabb1, layers1, inactive1)) in intervals.0.iter().enumerate() {
        for (ent2, aabb2, layers2, inactive2) in intervals.0.iter().skip(i + 1) {
            // x doesn't intersect; check this first so we can discard as soon as possible
            if aabb2.mins.x > aabb1.maxs.x {
                break;
            }

            // No collisions between bodies that haven't moved or colliders with incompatible layers
            if (*inactive1 && *inactive2) || !layers1.interacts_with(*layers2) {
                continue;
            }

            // y doesn't intersect
            if aabb1.mins.y > aabb2.maxs.y || aabb1.maxs.y < aabb2.mins.y {
                continue;
            }

            broad_collision_pairs.push((*ent1, *ent2));
        }
    }
}

/// Entities with [`ColliderAabb`]s sorted along an axis by their extents.
#[derive(Resource, Default)]
struct AabbIntervals3d(Vec<(Entity, ColliderAabb3d, CollisionLayers, IsBodyInactive)>);

/// Updates [`AabbIntervals`] to keep them in sync with the [`ColliderAabb`]s.
#[allow(clippy::type_complexity)]
fn update_aabb_intervals_3d(
    aabbs: Query<(
        &ColliderAabb3d,
        Option<&CollisionLayers>,
        Ref<Position3d>,
        Ref<Rotation3d>,
    )>,
    mut intervals: ResMut<AabbIntervals3d>,
) {
    intervals
        .0
        .retain_mut(|(entity, aabb, layers, is_inactive)| {
            if let Ok((new_aabb, new_layers, position, rotation)) = aabbs.get(*entity) {
                *aabb = *new_aabb;
                *layers = new_layers.map_or(CollisionLayers::default(), |layers| *layers);
                *is_inactive = !position.is_changed() && !rotation.is_changed();
                true
            } else {
                false
            }
        });
}

/// Adds new [`ColliderAabb`]s to [`AabbIntervals`].
#[allow(clippy::type_complexity)]
fn add_new_aabb_intervals_3d(
    aabbs: Query<
        (
            Entity,
            &ColliderAabb3d,
            Option<&RigidBody>,
            Option<&CollisionLayers>,
        ),
        Added<ColliderAabb3d>,
    >,
    mut intervals: ResMut<AabbIntervals3d>,
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
fn collect_collision_pairs_3d(
    intervals: ResMut<AabbIntervals3d>,
    mut broad_collision_pairs: ResMut<BroadCollisionPairs3d>,
) {
    sweep_and_prune_3d(intervals, &mut broad_collision_pairs.0);
}

/// Sorts the entities by their minimum extents along an axis and collects the entity pairs that have intersecting AABBs.
///
/// Sweep and prune exploits temporal coherence, as bodies are unlikely to move significantly between two simulation steps. Insertion sort is used, as it is good at sorting nearly sorted lists efficiently.
fn sweep_and_prune_3d(
    mut intervals: ResMut<AabbIntervals3d>,
    broad_collision_pairs: &mut Vec<(Entity, Entity)>,
) {
    // Sort bodies along the x-axis using insertion sort, a sorting algorithm great for sorting nearly sorted lists.
    insertion_sort(&mut intervals.0, |a, b| a.1.mins.x > b.1.mins.x);

    // Clear broad phase collisions from previous iteration.
    broad_collision_pairs.clear();

    // Find potential collisions by checking for AABB intersections along all axes.
    for (i, (ent1, aabb1, layers1, inactive1)) in intervals.0.iter().enumerate() {
        for (ent2, aabb2, layers2, inactive2) in intervals.0.iter().skip(i + 1) {
            // x doesn't intersect; check this first so we can discard as soon as possible
            if aabb2.mins.x > aabb1.maxs.x {
                break;
            }

            // No collisions between bodies that haven't moved or colliders with incompatible layers
            if (*inactive1 && *inactive2) || !layers1.interacts_with(*layers2) {
                continue;
            }

            // y doesn't intersect
            if aabb1.mins.y > aabb2.maxs.y || aabb1.maxs.y < aabb2.mins.y {
                continue;
            }

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
