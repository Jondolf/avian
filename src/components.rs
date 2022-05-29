use bevy::prelude::*;
use parry2d::shape::SharedShape;

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct Pos(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct PrevPos(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct Vel(pub Vec2);

#[derive(Component, Debug, Default, Deref, DerefMut)]
pub struct PreSolveVel(pub Vec2);

#[derive(Component, Debug)]
pub struct Mass(pub f32);

impl Default for Mass {
    fn default() -> Self {
        Self(1.0) // Default to 1 kg
    }
}

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

#[derive(Component)]
pub struct ParticleCollider(pub SharedShape);
