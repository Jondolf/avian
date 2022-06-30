use crate::{components::*, Vector};

use bevy::prelude::*;

#[derive(Bundle, Default)]
pub struct RigidBodyBundle {
    pub rigid_body: RigidBody,

    pub pos: Pos,
    pub prev_pos: PrevPos,

    pub rot: Rot,
    pub prev_rot: PrevRot,

    pub lin_vel: LinVel,
    pub pre_solve_lin_vel: PreSolveLinVel,

    pub ang_vel: AngVel,
    pub pre_solve_ang_vel: PreSolveAngVel,

    pub external_force: ExternalForce,
    pub external_torque: ExternalTorque,

    pub restitution: Restitution,
    pub friction: Friction,

    pub mass_props: MassProperties,
    pub explicit_mass_props: ExplicitMassProperties,
}

impl RigidBodyBundle {
    pub fn new_dynamic() -> Self {
        Self {
            rigid_body: RigidBody::Dynamic,
            ..default()
        }
    }
    pub fn new_static() -> Self {
        Self {
            rigid_body: RigidBody::Static,
            ..default()
        }
    }
    pub fn with_pos(self, pos: Vector) -> Self {
        Self {
            pos: Pos(pos),
            ..self
        }
    }
    #[cfg(feature = "2d")]
    pub fn with_rot(self, rot: Rot) -> Self {
        Self { rot, ..self }
    }
    #[cfg(feature = "3d")]
    pub fn with_rot(self, quat: Quat) -> Self {
        Self {
            rot: Rot(quat),
            ..self
        }
    }
    pub fn with_explicit_mass_properties(self, mass_props: MassProperties) -> Self {
        Self {
            explicit_mass_props: ExplicitMassProperties(mass_props),
            ..self
        }
    }
}
