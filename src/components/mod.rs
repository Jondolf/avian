use bevy::prelude::*;
use parry2d::shape::SharedShape;

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct Pos(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct PrevPos(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct Rot(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct PrevRot(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct LinVel(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct PreSolveLinVel(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct AngVel(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct PreSolveAngVel(pub Vec2);

#[derive(Component, Debug)]
pub struct Mass(pub f32);

impl Default for Mass {
    fn default() -> Self {
        Self(1.0) // Default to 1 kg
    }
}

#[derive(Component, Debug, Default)]
pub struct Inertia(pub f32);

/// 0.0: perfectly inelastic\
/// 1.0: perfectly elastic\
/// 2.0: kinetic energy is doubled
#[derive(Component, Debug)]
pub struct Restitution(pub f32);

impl Default for Restitution {
    fn default() -> Self {
        Self(0.3)
    }
}

pub type ColliderShape = SharedShape;

#[derive(Component, Clone)]
pub struct Collider {
    pub shape: ColliderShape,
}
