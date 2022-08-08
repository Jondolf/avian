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

    pub mass: Mass,
    pub inv_mass: InvMass,
    pub inertia: Inertia,
    pub inv_inertia: InvInertia,
    pub local_center_of_mass: LocalCom,
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

    pub fn new_kinematic() -> Self {
        Self {
            rigid_body: RigidBody::Kinematic,
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

    /// Computes the mass properties that a [`Collider`] would have with a given density, and adds those to the body.
    pub fn with_mass_props_from_shape(self, shape: &Shape, density: f32) -> Self {
        let ColliderMassProperties {
            mass,
            inv_mass,
            inertia,
            inv_inertia,
            local_center_of_mass,
            ..
        } = ColliderMassProperties::from_shape_and_density(shape, density);

        Self {
            mass,
            inv_mass,
            inertia,
            inv_inertia,
            local_center_of_mass,
            ..self
        }
    }
}

#[derive(Bundle, Default)]
pub struct ColliderBundle {
    collider_shape: ColliderShape,
    collider_aabb: ColliderAabb,
    pub mass_props: ColliderMassProperties,
    pub(crate) prev_mass_props: PrevColliderMassProperties,
}

impl ColliderBundle {
    /// Creates a new [`ColliderBundle`] from a given [`ColliderShape`] and density.
    pub fn new(shape: &Shape, density: f32) -> Self {
        let aabb = ColliderAabb::from_shape(shape);
        let mass_props = ColliderMassProperties::from_shape_and_density(shape, density);

        Self {
            collider_shape: ColliderShape(shape.to_owned()),
            collider_aabb: aabb,
            mass_props,
            prev_mass_props: PrevColliderMassProperties(ColliderMassProperties::ZERO),
        }
    }

    pub fn update_mass_props(&mut self) {
        self.mass_props = ColliderMassProperties::from_shape_and_density(
            &self.collider_shape.0,
            self.mass_props.density,
        );
    }
}
