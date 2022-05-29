use crate::{components::*, SUB_DT};

use bevy::prelude::*;

#[derive(Bundle, Default)]
pub struct DynamicBodyBundle {
    pub pos: Pos,
    pub prev_pos: PrevPos,

    pub lin_vel: LinVel,
    pub pre_solve_lin_vel: PreSolveLinVel,

    pub mass: Mass,
    pub restitution: Restitution,
}

#[derive(Bundle, Default)]
pub struct StaticBodyBundle {
    pub pos: Pos,
    pub restitution: Restitution,
}

#[derive(Bundle)]
pub struct ColliderBundle {
    pub collider: Collider,
}

impl ColliderBundle {
    pub fn with_shape(shape: ColliderShape) -> Self {
        Self {
            collider: Collider { shape },
        }
    }
}

impl DynamicBodyBundle {
    pub fn new_with_pos_and_vel(pos: Vec2, vel: Vec2) -> Self {
        Self {
            pos: Pos(pos),
            prev_pos: PrevPos(pos - vel * SUB_DT),
            lin_vel: LinVel(vel),
            ..default()
        }
    }
}
