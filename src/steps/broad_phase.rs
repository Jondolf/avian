use crate::prelude::*;
use bevy::prelude::*;
use parry::bounding_volume::BoundingVolume;

pub struct BroadPhasePlugin;

impl Plugin for BroadPhasePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<BroadCollisionPairs>()
            .add_system_set_to_stage(
                FixedUpdateStage,
                SystemSet::new()
                    .label(PhysicsStep::BroadPhase)
                    .with_run_criteria(first_substep)
                    .with_system(collect_collision_pairs),
            );
    }
}

#[derive(Default, Debug)]
pub struct BroadCollisionPairs(pub Vec<(Entity, Entity)>);

/// Collects bodies that are potentially colliding.
fn collect_collision_pairs(
    bodies: Query<(Entity, &ColliderAabb, &RigidBody)>,
    mut broad_collision_pairs: ResMut<BroadCollisionPairs>,
) {
    brute_force_collision_pairs(bodies, &mut broad_collision_pairs.0);
}

/// Collects bodies that are potentially colliding using a brute force algorithm (better one coming soon).
fn brute_force_collision_pairs(
    bodies: Query<(Entity, &ColliderAabb, &RigidBody)>,
    broad_collision_pairs: &mut Vec<(Entity, Entity)>,
) {
    broad_collision_pairs.clear();

    for [(ent_a, aabb_a, rb_a), (ent_b, aabb_b, rb_b)] in bodies.iter_combinations() {
        // At least one of the bodies is dynamic and their AABBs intersect
        if (rb_a.is_dynamic() || rb_b.is_dynamic()) && aabb_a.intersects(&aabb_b.0) {
            broad_collision_pairs.push((ent_a, ent_b));
        }
    }
}
