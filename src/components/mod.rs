mod rotation;

pub use rotation::*;

use bevy::prelude::*;
use parry2d::{bounding_volume::AABB, shape::SharedShape};

use crate::utils::aabb_with_margin;

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

#[derive(Clone, Copy, Component, PartialEq)]
pub struct MassProperties {
    pub mass: f32,
    pub inv_mass: f32,
    pub inertia: f32,
    pub inv_inertia: f32,
    pub local_center_of_mass: Vec2,
}

impl MassProperties {
    pub const ZERO: Self = Self {
        mass: 0.0,
        inv_mass: 0.0,
        inertia: 0.0,
        inv_inertia: 0.0,
        local_center_of_mass: Vec2::ZERO,
    };
}

impl MassProperties {
    /// Computes mass properties for a given shape and density. This shape can be a [`ColliderShape`], which is just a type alias for [`SharedShape`].
    pub fn from_shape(shape: &SharedShape, density: f32) -> Self {
        let props = shape.mass_properties(density);
        let inertia = props.principal_inertia();

        Self {
            mass: props.mass(),
            inv_mass: props.inv_mass,
            inertia,
            inv_inertia: 1.0 / inertia,
            local_center_of_mass: Vec2::new(props.local_com.x, props.local_com.y),
        }
    }
}

impl Default for MassProperties {
    fn default() -> Self {
        Self::ZERO
    }
}

/// Explicitly configured mass properties attached to the body itself.
///
/// The final [`MassProperties`] of the body will be computed as the sum of its explicit mass properties and the mass properties of the attached [`Collider`].
///
/// Explicit mass properties are zero by default, as mass properties are generally computed from the collider.
///
/// You should usually only use explicit mass properties if a body doesn't have a collider or you want to have extra control over the body's mass properties.
#[derive(Clone, Copy, Component, Default, Deref, DerefMut, PartialEq)]
pub struct ExplicitMassProperties(pub MassProperties);

pub type ColliderShape = SharedShape;

#[derive(Component, Clone)]
pub struct Collider {
    pub shape: ColliderShape,
    pub aabb: AABB,
    pub density: f32,
    pub mass_properties: MassProperties,
}

impl Collider {
    /// Creates a new collider from a given [`ColliderShape`] with a default density of 1.0.
    pub fn from_shape(shape: ColliderShape) -> Self {
        let aabb = shape.compute_local_aabb();
        let density = 1.0;
        let mass_properties = MassProperties::from_shape(&shape, density);

        Self {
            shape,
            aabb,
            density,
            mass_properties,
        }
    }
    /// Chanhes the mass properties of the collider according to a given density.
    pub fn with_density(mut self, density: f32) -> Self {
        self.density = density;
        self.update_mass_props();
        self
    }

    pub fn update_mass_props(&mut self) {
        let props = self.shape.mass_properties(self.density);
        let inertia = props.principal_inertia();

        self.mass_properties = MassProperties {
            mass: props.mass(),
            inv_mass: props.inv_mass,
            inertia,
            inv_inertia: 1.0 / inertia,
            local_center_of_mass: Vec2::new(props.local_com.x, props.local_com.y),
        };
    }

    pub(crate) fn update_aabb_with_margin(
        &mut self,
        pos: &Vec2,
        rot: &Rot,
        shape: &SharedShape,
        margin: f32,
    ) {
        self.aabb = aabb_with_margin(pos, rot, shape, margin);
    }
}
