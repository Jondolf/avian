mod rotation;

pub use rotation::*;

use bevy::prelude::*;
use parry2d::shape::SharedShape;

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct Pos(pub Vec2);

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct PrevPos(pub Vec2);

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct LinVel(pub Vec2);

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct PreSolveLinVel(pub Vec2);

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct AngVel(pub f32);

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct PreSolveAngVel(pub f32);

#[derive(Clone, Copy, Component, Debug)]
pub struct Density(pub f32);

impl Default for Density {
    fn default() -> Self {
        Self(1.0)
    }
}

/// 0.0: perfectly inelastic\
/// 1.0: perfectly elastic\
/// 2.0: kinetic energy is doubled
#[derive(Clone, Copy, Component, Debug)]
pub struct Restitution(pub f32);

impl Default for Restitution {
    fn default() -> Self {
        Self(0.3)
    }
}

/// 0.0: no friction at all, the body slides infinitely\
/// 1.0: very high friction\
/// Values larger than 1.0 may work, but they can cause weird behaviour in some situations.
#[derive(Clone, Copy, Component, Debug)]
pub struct Friction {
    pub static_coefficient: f32,
    pub dynamic_coefficient: f32,
}

impl Friction {
    pub const ZERO: Self = Self {
        static_coefficient: 0.0,
        dynamic_coefficient: 0.0,
    };

    /// Creates a new Friction component with equal static and dynamic friction coefficients.
    fn new(friction_coefficient: f32) -> Self {
        Self {
            static_coefficient: friction_coefficient,
            dynamic_coefficient: friction_coefficient,
        }
    }
}

impl Default for Friction {
    fn default() -> Self {
        Self::new(0.7)
    }
}

pub type ColliderShape = SharedShape;

#[derive(Clone, Copy, Component, Default, PartialEq)]
pub struct MassProperties {
    pub mass: f32,
    pub inv_mass: f32,
    pub inertia: f32,
    pub inv_inertia: f32,
    pub local_center_of_mass: Vec2,
}

#[derive(Component, Clone)]
pub struct Collider {
    pub shape: ColliderShape,
}

impl Collider {
    pub fn mass_properties(&self, density: f32) -> MassProperties {
        let props = self.shape.mass_properties(density);
        let inertia = props.principal_inertia();

        MassProperties {
            mass: props.mass(),
            inv_mass: props.inv_mass,
            inertia,
            inv_inertia: 1.0 / inertia,
            local_center_of_mass: Vec2::new(props.local_com.x, props.local_com.y),
        }
    }
}
