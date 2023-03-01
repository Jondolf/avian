use crate::{broad_phase::BroadCollisions, prelude::*};
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

#[derive(Resource, Default, Debug)]
pub struct Collisions(pub HashMap<(Entity, Entity), Collision>);

/// Data related to a collision between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Collision {
    pub entity1: Entity,
    pub entity2: Entity,
    /// Local contact point a in local coordinates
    pub local_r1: Vector,
    /// Local contact point b in local coordinates
    pub local_r2: Vector,
    /// Local contact point a in world coordinates
    pub world_r1: Vector,
    /// Local contact point b in world coordinates
    pub world_r2: Vector,
    /// Contact normal from contact point a to b
    pub normal: Vector,
    /// Penetration depth
    pub penetration: f32,
}

/// Iterates through broad phase collision pairs, checks which ones are actually colliding, and stores the collision data in the [`Collisions`] resource.
fn collect_collisions(
    bodies: Query<(RigidBodyQuery, &ColliderShape)>,
    broad_collisions: Res<BroadCollisions>,
    mut collisions: ResMut<Collisions>,
) {
    collisions.0.clear();

    for (ent1, colliding_with) in broad_collisions.0.iter() {
        let (body1, collider_shape1) = bodies.get(*ent1).unwrap();
        for (i, (body2, collider_shape2)) in bodies.iter_many(colliding_with.iter()).enumerate() {
            if let Some(collision) = get_collision(
                *ent1,
                colliding_with[i],
                body1.pos.0,
                body2.pos.0,
                body1.local_com.0,
                body2.local_com.0,
                body1.rot,
                body2.rot,
                &collider_shape1.0,
                &collider_shape2.0,
            ) {
                collisions.0.insert((*ent1, colliding_with[i]), collision);
            }
        }
    }
}

/// Computes one pair of collision points between two shapes.
#[allow(clippy::too_many_arguments)]
fn get_collision(
    ent1: Entity,
    ent2: Entity,
    pos1: Vector,
    pos2: Vector,
    local_com1: Vector,
    local_com2: Vector,
    rot1: &Rot,
    rot2: &Rot,
    shape1: &Shape,
    shape2: &Shape,
) -> Option<Collision> {
    if let Ok(Some(collision)) = parry::query::contact(
        &make_isometry(pos1, rot1),
        shape1.0.as_ref(),
        &make_isometry(pos2, rot2),
        shape2.0.as_ref(),
        0.0,
    ) {
        let world_r1 = Vector::from(collision.point1) - pos1 + local_com1;
        let world_r2 = Vector::from(collision.point2) - pos2 + local_com2;

        return Some(Collision {
            entity1: ent1,
            entity2: ent2,
            local_r1: rot1.inv().rotate(world_r1),
            local_r2: rot2.inv().rotate(world_r2),
            world_r1,
            world_r2,
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
