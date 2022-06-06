use crate::{components::*, SUB_DT};

use bevy::prelude::*;

#[derive(Bundle, Default)]
pub struct DynamicBodyBundle {
    pub pos: Pos,
    pub prev_pos: PrevPos,

    pub rot: Rot,
    pub prev_rot: PrevRot,

    pub lin_vel: LinVel,
    pub pre_solve_lin_vel: PreSolveLinVel,

    pub ang_vel: AngVel,
    pub pre_solve_ang_vel: PreSolveAngVel,

    pub restitution: Restitution,
    pub friction: Friction,

    pub density: Density,
    pub mass_props: MassProperties,
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
    pub fn with_rot(self, rot_degrees: f32) -> Self {
        Self {
            rot: Rot::from_degrees(rot_degrees),
            ..self
        }
    }
}

#[derive(Bundle, Default)]
pub struct StaticBodyBundle {
    pub pos: Pos,
    pub rot: Rot,

    pub restitution: Restitution,
    pub friction: Friction,
}

impl StaticBodyBundle {
    pub fn new_with_pos_and_rot(pos: Vec2, rot_degrees: f32) -> Self {
        Self {
            pos: Pos(pos),
            rot: Rot::from_degrees(rot_degrees),
            ..default()
        }
    }
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
