use crate::prelude::*;
use bevy::ecs::query::WorldQuery;
use std::ops::{AddAssign, SubAssign};

/// A [`WorldQuery`] to make querying and modifying rigid bodies more convenient.
#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct RigidBodyQuery {
    pub entity: Entity,
    pub rb: &'static mut RigidBody,
    pub pos: &'static mut Position,
    pub rot: &'static mut Rotation,
    pub prev_pos: &'static mut PreviousPosition,
    pub prev_rot: &'static mut PreviousRotation,
    pub lin_vel: &'static mut LinearVelocity,
    pub pre_solve_lin_vel: &'static mut PreSolveLinearVelocity,
    pub ang_vel: &'static mut AngularVelocity,
    pub pre_solve_ang_vel: &'static mut PreSolveAngularVelocity,
    pub mass: &'static mut Mass,
    pub inverse_mass: &'static mut InverseMass,
    pub inertia: &'static mut Inertia,
    pub inverse_inertia: &'static mut InverseInertia,
    pub center_of_mass: &'static mut CenterOfMass,
    pub friction: &'static mut Friction,
    pub restitution: &'static mut Restitution,
}

impl<'w> RigidBodyQueryItem<'w> {
    /// Computes the world-space inverse inertia (does nothing in 2D).
    #[cfg(feature = "2d")]
    pub(crate) fn world_inv_inertia(&self) -> InverseInertia {
        *self.inverse_inertia
    }

    // Computes the world-space inverse inertia tensor.
    #[cfg(feature = "3d")]
    pub fn world_inv_inertia(&self) -> InverseInertia {
        self.inverse_inertia.rotated(&self.rot)
    }
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub(crate) struct MassPropsQuery {
    pub mass: &'static mut Mass,
    pub inverse_mass: &'static mut InverseMass,
    pub inertia: &'static mut Inertia,
    pub inverse_inertia: &'static mut InverseInertia,
    pub center_of_mass: &'static mut CenterOfMass,
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub(crate) struct ColliderQuery {
    pub collider: &'static mut Collider,
    pub aabb: &'static mut ColliderAabb,
    pub mass_properties: &'static mut ColliderMassProperties,
    pub previous_mass_properties: &'static mut PreviousColliderMassProperties,
}

impl<'w> AddAssign<ColliderMassProperties> for MassPropsQueryItem<'w> {
    fn add_assign(&mut self, rhs: ColliderMassProperties) {
        self.mass.0 += rhs.mass.0;
        self.inverse_mass.0 = 1.0 / self.mass.0;
        self.inertia.0 += rhs.inertia.0;
        self.inverse_inertia.0 = self.inertia.inverse().0;
        self.center_of_mass.0 += rhs.center_of_mass.0;
    }
}

impl<'w> SubAssign<ColliderMassProperties> for MassPropsQueryItem<'w> {
    fn sub_assign(&mut self, rhs: ColliderMassProperties) {
        self.mass.0 -= rhs.mass.0;
        self.inverse_mass.0 = 1.0 / self.mass.0;
        self.inertia.0 -= rhs.inertia.0;
        self.inverse_inertia.0 = self.inertia.inverse().0;
        self.center_of_mass.0 -= rhs.center_of_mass.0;
    }
}
