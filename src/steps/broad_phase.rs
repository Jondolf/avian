use crate::prelude::*;
use bevy::{
    prelude::*,
    utils::{StableHashMap, StableHashSet},
};

pub struct BroadPhasePlugin;

impl Plugin for BroadPhasePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<BroadCollisions>()
            .init_resource::<AabbIntervals>()
            .add_system_set_to_stage(
                FixedUpdateStage,
                SystemSet::new()
                    .label(PhysicsStep::BroadPhase)
                    .with_run_criteria(first_substep)
                    .with_system(add_new_aabb_intervals.before(collect_collision_pairs))
                    .with_system(remove_old_aabb_intervals.before(update_aabb_intervals))
                    .with_system(update_aabb_intervals.before(collect_collision_pairs))
                    .with_system(collect_collision_pairs),
            );
    }
}

#[derive(Resource, Default, Debug)]
pub struct BroadCollisions(pub StableHashMap<Entity, Vec<Entity>>);

#[derive(Resource, Default)]
struct AabbIntervals(Vec<(Entity, ColliderAabb)>);

fn update_aabb_intervals(aabbs: Query<&ColliderAabb>, mut intervals: ResMut<AabbIntervals>) {
    for (ent, aabb) in intervals.0.iter_mut() {
        *aabb = *aabbs.get(*ent).unwrap();
    }
}

fn add_new_aabb_intervals(
    aabbs: Query<(Entity, &ColliderAabb), Added<ColliderAabb>>,
    mut intervals: ResMut<AabbIntervals>,
) {
    let aabbs = aabbs.iter().map(|(ent, aabb)| (ent, *aabb));
    intervals.0.extend(aabbs);
}

fn remove_old_aabb_intervals(
    removals: RemovedComponents<ColliderAabb>,
    mut intervals: ResMut<AabbIntervals>,
) {
    let removed = removals.iter().collect::<StableHashSet<Entity>>();
    intervals.0.retain(|(ent, _)| !removed.contains(ent));
}

/// Collects bodies that are potentially colliding.
fn collect_collision_pairs(
    intervals: ResMut<AabbIntervals>,
    mut broad_collisions: ResMut<BroadCollisions>,
) {
    sweep_and_prune(intervals, &mut broad_collisions.0);
}

fn sweep_and_prune(
    mut intervals: ResMut<AabbIntervals>,
    broad_collisions: &mut StableHashMap<Entity, Vec<Entity>>,
) {
    // Sort bodies along the x-axis using insertion sort, a sorting algorithm great for sorting nearly sorted lists.
    insertion_sort(&mut intervals.0, |a, b| a.1.mins.x > b.1.mins.x);

    // Clear broad phase collisions from previous iteration.
    broad_collisions.clear();

    // Find potential collisions by checking for AABB intersections along all axes.
    for (i, (ent_a, aabb_a)) in intervals.0.iter().enumerate() {
        let colliding_with_a = broad_collisions.insert_unique_unchecked(*ent_a, vec![]).1;
        for (ent_b, aabb_b) in intervals.0.iter().skip(i + 1) {
            // x doesn't intersect
            if aabb_b.mins.x > aabb_a.maxs.x {
                break;
            }

            // y doesn't intersect
            if aabb_a.mins.y > aabb_b.maxs.y || aabb_a.maxs.y < aabb_b.mins.y {
                continue;
            }

            #[cfg(feature = "3d")]
            // z doesn't intersect
            if aabb_a.mins.z > aabb_b.maxs.z || aabb_a.maxs.z < aabb_b.mins.z {
                continue;
            }

            colliding_with_a.push(*ent_b);
        }
    }
}

fn insertion_sort<T>(items: &mut Vec<T>, comparison: fn(&T, &T) -> bool) {
    for i in 1..items.len() {
        let mut j = i;
        while j > 0 && comparison(&items[(j - 1)], &items[j]) {
            items.swap(j - 1, j);
            j -= 1;
        }
    }
}
