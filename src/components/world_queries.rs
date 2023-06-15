use crate::prelude::*;
use bevy::ecs::query::WorldQuery;
use std::ops::{AddAssign, SubAssign};

/// A [`WorldQuery`] to make querying and modifying rigid bodies more convenient.
#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct RigidBodyQuery {
    pub rb: &'static mut RigidBody,
    pub pos: &'static mut Pos,
    pub rot: &'static mut Rot,
    pub prev_pos: &'static mut PrevPos,
    pub prev_rot: &'static mut PrevRot,
    pub lin_vel: &'static mut LinVel,
    pub pre_solve_lin_vel: &'static mut PreSolveLinVel,
    pub ang_vel: &'static mut AngVel,
    pub pre_solve_ang_vel: &'static mut PreSolveAngVel,
    pub mass: &'static mut Mass,
    pub inv_mass: &'static mut InvMass,
    pub inertia: &'static mut Inertia,
    pub inv_inertia: &'static mut InvInertia,
    pub local_com: &'static mut LocalCom,
    pub friction: &'static mut Friction,
    pub restitution: &'static mut Restitution,
}

impl<'w> RigidBodyQueryItem<'w> {
    /// Computes the world-space inverse inertia (does nothing in 2D).
    #[cfg(feature = "2d")]
    pub(crate) fn world_inv_inertia(&self) -> InvInertia {
        *self.inv_inertia
    }

    // Computes the world-space inverse inertia tensor.
    #[cfg(feature = "3d")]
    pub fn world_inv_inertia(&self) -> InvInertia {
        self.inv_inertia.rotated(&self.rot)
    }
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub(crate) struct MassPropsQuery {
    pub mass: &'static mut Mass,
    pub inv_mass: &'static mut InvMass,
    pub inertia: &'static mut Inertia,
    pub inv_inertia: &'static mut InvInertia,
    pub local_com: &'static mut LocalCom,
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub(crate) struct ColliderQuery {
    pub collider: &'static mut Collider,
    pub aabb: &'static mut ColliderAabb,
    pub mass_props: &'static mut ColliderMassProperties,
    pub prev_mass_props: &'static mut PrevColliderMassProperties,
}

impl<'w> AddAssign<ColliderMassProperties> for MassPropsQueryItem<'w> {
    fn add_assign(&mut self, rhs: ColliderMassProperties) {
        self.mass.0 += rhs.mass.0;
        self.inv_mass.0 = 1.0 / self.mass.0;
        self.inertia.0 += rhs.inertia.0;
        self.inv_inertia.0 = self.inertia.inverse().0;
        self.local_com.0 += rhs.local_center_of_mass.0;
    }
}

impl<'w> SubAssign<ColliderMassProperties> for MassPropsQueryItem<'w> {
    fn sub_assign(&mut self, rhs: ColliderMassProperties) {
        self.mass.0 -= rhs.mass.0;
        self.inv_mass.0 = 1.0 / self.mass.0;
        self.inertia.0 -= rhs.inertia.0;
        self.inv_inertia.0 = self.inertia.inverse().0;
        self.local_com.0 -= rhs.local_center_of_mass.0;
    }
}
