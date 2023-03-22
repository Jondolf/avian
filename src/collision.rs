use crate::prelude::*;

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
    pub penetration: Scalar,
}

/// Computes one pair of collision points between two shapes.
#[allow(clippy::too_many_arguments)]
pub(crate) fn get_collision(
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
        &utils::make_isometry(pos1, rot1),
        shape1.0.as_ref(),
        &utils::make_isometry(pos2, rot2),
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
            normal: collision.normal1.into(),
            penetration: -collision.dist,
        });
    }
    None
}
