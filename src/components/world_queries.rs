#![allow(missing_docs)]

use crate::prelude::*;
use bevy::ecs::query::WorldQuery;
use std::ops::{AddAssign, SubAssign};

/// A [`WorldQuery`] to make querying and modifying rigid bodies more convenient.
#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct RigidBodyQuery {
    pub entity: Entity,
    pub rb: &'static mut RigidBody,
    pub position: &'static mut Position,
    pub rotation: &'static mut Rotation,
    pub previous_position: &'static mut PreviousPosition,
    pub previous_rotation: &'static mut PreviousRotation,
    pub linear_velocity: &'static mut LinearVelocity,
    pub(crate) pre_solve_linear_velocity: &'static mut PreSolveLinearVelocity,
    pub angular_velocity: &'static mut AngularVelocity,
    pub(crate) pre_solve_angular_velocity: &'static mut PreSolveAngularVelocity,
    pub mass: &'static mut Mass,
    pub inverse_mass: &'static mut InverseMass,
    pub inertia: &'static mut Inertia,
    pub inverse_inertia: &'static mut InverseInertia,
    pub center_of_mass: &'static mut CenterOfMass,
    pub friction: &'static mut Friction,
    pub restitution: &'static mut Restitution,
    pub locked_axes: Option<&'static LockedAxes>,
}

impl<'w> RigidBodyQueryItem<'w> {
    /// Computes the effective inverse mass, taking into account any translation locking.
    pub fn effective_inv_mass(&self) -> Vector {
        let mut inv_mass = Vector::splat(self.inverse_mass.0);

        if let Some(locked_axes) = self.locked_axes {
            inv_mass = locked_axes.apply_to_vec(inv_mass);
        }

        inv_mass
    }

    /// Computes the effective world-space inverse inertia, taking into account any rotation locking.
    #[cfg(feature = "2d")]
    pub fn effective_world_inv_inertia(&self) -> Scalar {
        let mut inv_inertia = self.inverse_inertia.0;

        if let Some(locked_axes) = self.locked_axes {
            inv_inertia = locked_axes.apply_to_rotation(inv_inertia);
        }

        inv_inertia
    }

    /// Computes the effective world-space inverse inertia tensor, taking into account any rotation locking.
    #[cfg(feature = "3d")]
    pub fn effective_world_inv_inertia(&self) -> Matrix3 {
        let mut inv_inertia = self.inverse_inertia.rotated(&self.rotation).0;

        if let Some(locked_axes) = self.locked_axes {
            inv_inertia = locked_axes.apply_to_rotation(inv_inertia);
        }

        inv_inertia
    }
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub(crate) struct MassPropertiesQuery {
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

impl<'w> AddAssign<ColliderMassProperties> for MassPropertiesQueryItem<'w> {
    fn add_assign(&mut self, rhs: ColliderMassProperties) {
        self.mass.0 += rhs.mass.0;
        self.inverse_mass.0 = 1.0 / self.mass.0;
        self.inertia.0 += rhs.inertia.0;
        self.inverse_inertia.0 = self.inertia.inverse().0;
        self.center_of_mass.0 += rhs.center_of_mass.0;
    }
}

impl<'w> SubAssign<ColliderMassProperties> for MassPropertiesQueryItem<'w> {
    fn sub_assign(&mut self, rhs: ColliderMassProperties) {
        self.mass.0 -= rhs.mass.0;
        self.inverse_mass.0 = 1.0 / self.mass.0;
        self.inertia.0 -= rhs.inertia.0;
        self.inverse_inertia.0 = self.inertia.inverse().0;
        self.center_of_mass.0 -= rhs.center_of_mass.0;
    }
}
