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

/// A list of entity pairs for potential collisions collected during the broad phase.
#[derive(Reflect, Resource, Default, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Resource)]
pub struct BroadCollisionPairs(pub Vec<(Entity, Entity)>);

type AABBChanged = Or<(
    Changed<Position>,
    Changed<Rotation>,
    Changed<LinearVelocity>,
    Changed<AngularVelocity>,
    Changed<Collider>,
)>;

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
#[allow(clippy::type_complexity)]
fn update_aabb(
    mut colliders: Query<
        (
            &Collider,
            &mut ColliderAabb,
            &Position,
            &Rotation,
            Option<&ColliderParent>,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        AABBChanged,
    >,
    parent_velocity: Query<
        (&Position, Option<&LinearVelocity>, Option<&AngularVelocity>),
        With<Children>,
    >,
    dt: Res<Time>,
    narrow_phase_config: Option<Res<NarrowPhaseConfig>>,
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
            #[cfg(feature = "2d")]
            let vel_at_offset =
                lin_vel.0 + Vector::new(-ang_vel.0 * offset.y, ang_vel.0 * offset.x) * 1.0;
            #[cfg(feature = "3d")]
            let vel_at_offset = lin_vel.0 + ang_vel.cross(offset);
            (LinearVelocity(vel_at_offset), *ang_vel)
        } else {
            (LinearVelocity::ZERO, AngularVelocity::ZERO)
        };

        // Compute current isometry and predicted isometry for next feame
        let start_iso = utils::make_isometry(*pos, *rot);
        let end_iso = {
            #[cfg(feature = "2d")]
            {
                utils::make_isometry(
                    pos.0 + lin_vel.0 * safety_margin_factor,
                    *rot + Rotation::from_radians(safety_margin_factor * ang_vel.0),
                )
            }
            #[cfg(feature = "3d")]
            {
                let q = Quaternion::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
                let (x, y, z, w) = (
                    rot.x + safety_margin_factor * 0.5 * q.x,
                    rot.y + safety_margin_factor * 0.5 * q.y,
                    rot.z + safety_margin_factor * 0.5 * q.z,
                    rot.w + safety_margin_factor * 0.5 * q.w,
                );
                utils::make_isometry(
                    pos.0 + lin_vel.0 * safety_margin_factor,
                    Quaternion::from_xyzw(x, y, z, w).normalize(),
                )
            }
        };

        // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
        aabb.0 = collider
            .shape_scaled()
            .compute_swept_aabb(&start_iso, &end_iso);

        // Add narrow phase prediction distance to AABBs to avoid missed collisions
        let prediction_distance = if let Some(ref config) = narrow_phase_config {
            config.prediction_distance
        } else {
            #[cfg(feature = "2d")]
            {
                1.0
            }
            #[cfg(feature = "3d")]
            {
                0.005
            }
        };
        aabb.maxs.x += prediction_distance;
        aabb.mins.x -= prediction_distance;
        aabb.maxs.y += prediction_distance;
        aabb.mins.y -= prediction_distance;
        #[cfg(feature = "3d")]
        {
            aabb.maxs.z += prediction_distance;
            aabb.mins.z -= prediction_distance;
        }
    }
}

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
        &ColliderParent,
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
                *collider_parent = *new_parent;
                *layers = new_layers.map_or(CollisionLayers::default(), |layers| *layers);

                let is_static = rbs.get(new_parent.get()).is_ok_and(RigidBody::is_static);
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
            &ColliderParent,
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
            *parent,
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
    insertion_sort(&mut intervals.0, |a, b| a.2.mins.x > b.2.mins.x);

    // Clear broad phase collisions from previous iteration.
    broad_collision_pairs.clear();

    // Find potential collisions by checking for AABB intersections along all axes.
    for (i, (ent1, parent1, aabb1, layers1, inactive1)) in intervals.0.iter().enumerate() {
        for (ent2, parent2, aabb2, layers2, inactive2) in intervals.0.iter().skip(i + 1) {
            // x doesn't intersect; check this first so we can discard as soon as possible
            if aabb2.mins.x > aabb1.maxs.x {
                break;
            }

            // No collisions between bodies that haven't moved or colliders with incompatible layers or colliders with the same parent
            if (*inactive1 && *inactive2) || !layers1.interacts_with(*layers2) || parent1 == parent2
            {
                continue;
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
