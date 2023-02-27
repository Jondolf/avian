use crate::{broad_phase::BroadCollisionPairs, prelude::*};
use bevy::{prelude::*, utils::HashMap};
use parry::math::Isometry;

pub struct NarrowPhasePlugin;

impl Plugin for NarrowPhasePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Collisions>().add_system_to_stage(
            FixedUpdateStage,
            collect_collisions
                .label(PhysicsStep::NarrowPhase)
                .after(PhysicsStep::Integrate),
        );
    }
}

#[derive(Default, Debug)]
pub struct Collisions(pub HashMap<(Entity, Entity), Collision>);

/// Data related to a collision between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Collision {
    pub entity_a: Entity,
    pub entity_b: Entity,
    /// Local contact point a in local coordinates
    pub local_r_a: Vector,
    /// Local contact point b in local coordinates
    pub local_r_b: Vector,
    /// Local contact point a in world coordinates
    pub world_r_a: Vector,
    /// Local contact point b in world coordinates
    pub world_r_b: Vector,
    /// Contact normal from contact point a to b
    pub normal: Vector,
    /// Penetration depth
    pub penetration: f32,
}

/// Iterates through broad phase collision pairs, checks which ones are actually colliding, and stores the collision data in the [`Collisions`] resource.
fn collect_collisions(
    bodies: Query<(RigidBodyQuery, &ColliderShape)>,
    broad_collision_pairs: Res<BroadCollisionPairs>,
    mut collisions: ResMut<Collisions>,
) {
    collisions.0.clear();

    for (ent_a, ent_b) in broad_collision_pairs.0.iter().cloned() {
        // Get components for entity a and b
        if let Ok([(body1, collider_shape_a), (body2, collider_shape_b)]) =
            bodies.get_many([ent_a, ent_b])
        {
            // No need to solve collisions if neither of the bodies is dynamic
            if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                continue;
            }

            // Detect if the entities are actually colliding and get collision data
            if let Some(collision) = get_collision(
                ent_a,
                ent_b,
                body1.pos.0,
                body2.pos.0,
                body1.local_com.0,
                body2.local_com.0,
                body1.rot,
                body2.rot,
                &collider_shape_a.0,
                &collider_shape_b.0,
            ) {
                collisions.0.insert((ent_a, ent_b), collision);
            }
        }
    }
}

/// Computes one pair of collision points between two shapes.
#[allow(clippy::too_many_arguments)]
fn get_collision(
    ent_a: Entity,
    ent_b: Entity,
    pos_a: Vector,
    pos_b: Vector,
    local_com_a: Vector,
    local_com_b: Vector,
    rot_a: &Rot,
    rot_b: &Rot,
    shape_a: &Shape,
    shape_b: &Shape,
) -> Option<Collision> {
    if let Some(collision) = parry::query::contact(
        &make_isometry(pos_a, rot_a),
        shape_a.0.as_ref(),
        &make_isometry(pos_b, rot_b),
        shape_b.0.as_ref(),
        0.0,
    )
    .unwrap()
    {
        let world_r_a = Vector::from(collision.point1) - pos_a + local_com_a;
        let world_r_b = Vector::from(collision.point2) - pos_b + local_com_b;

        return Some(Collision {
            entity_a: ent_a,
            entity_b: ent_b,
            local_r_a: rot_a.inv().rotate(world_r_a),
            local_r_b: rot_b.inv().rotate(world_r_b),
            world_r_a,
            world_r_b,
            normal: Vector::from(collision.normal1),
            penetration: -collision.dist,
        });
    }
    None
}

#[cfg(feature = "2d")]
fn make_isometry(pos: Vector, rot: &Rot) -> Isometry<f32> {
    Isometry::<f32>::new(pos.into(), (*rot).into())
}

#[cfg(feature = "3d")]
fn make_isometry(pos: Vector, rot: &Rot) -> Isometry<f32> {
    Isometry::<f32>::new(pos.into(), rot.to_scaled_axis().into())
}
