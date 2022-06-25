mod rotation;

pub use rotation::*;

use bevy::prelude::*;
use parry::{bounding_volume::AABB, shape::SharedShape};

use crate::{utils::aabb_with_margin, Vector};

#[cfg(feature = "3d")]
use crate::utils::get_rotated_inertia_tensor;

#[derive(Clone, Copy, Component, PartialEq, Eq)]
pub enum RigidBody {
    Dynamic,
    Static,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self::Dynamic
    }
}

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct Pos(pub Vector);

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct PrevPos(pub Vector);

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct LinVel(pub Vector);

#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct PreSolveLinVel(pub Vector);

#[cfg(feature = "2d")]
#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct AngVel(pub f32);

#[cfg(feature = "3d")]
#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct AngVel(pub Vec3);

#[cfg(feature = "2d")]
#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct PreSolveAngVel(pub f32);

#[cfg(feature = "3d")]
#[derive(Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
pub struct PreSolveAngVel(pub Vec3);

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
/// 1.0: high friction\
#[derive(Clone, Copy, Component, Debug)]
pub struct Friction {
    pub dynamic_coefficient: f32,
}

impl Friction {
    pub const ZERO: Self = Self {
        dynamic_coefficient: 0.0,
    };

    /// Creates a new Friction component with a given dynamic friction coefficient.
    fn new(friction_coefficient: f32) -> Self {
        Self {
            dynamic_coefficient: friction_coefficient,
        }
    }
}

impl Default for Friction {
    fn default() -> Self {
        Self::new(0.7)
    }
}

#[cfg(feature = "2d")]
type Inertia = f32;

#[cfg(feature = "3d")]
type Inertia = Mat3;

#[derive(Clone, Copy, Component, PartialEq)]
pub struct MassProperties {
    pub mass: f32,
    pub inv_mass: f32,
    pub inertia: Inertia,
    pub inv_inertia: Inertia,
    pub local_center_of_mass: Vector,
}

impl MassProperties {
    pub const ZERO: Self = Self {
        mass: 0.0,
        inv_mass: 0.0,

        #[cfg(feature = "2d")]
        inertia: 0.0,
        #[cfg(feature = "3d")]
        inertia: Mat3::ZERO,

        #[cfg(feature = "2d")]
        inv_inertia: 0.0,
        #[cfg(feature = "3d")]
        inv_inertia: Mat3::ZERO,

        local_center_of_mass: Vector::ZERO,
    };
}

impl MassProperties {
    /// Computes mass properties for a given shape and density. This shape can be a [`ColliderShape`], which is just a type alias for [`SharedShape`].
    pub fn from_shape(shape: &SharedShape, density: f32) -> Self {
        let props = shape.mass_properties(density);

        Self {
            mass: props.mass(),
            inv_mass: props.inv_mass,

            #[cfg(feature = "2d")]
            inertia: props.principal_inertia(),
            #[cfg(feature = "3d")]
            inertia: props.reconstruct_inertia_matrix().into(),

            #[cfg(feature = "2d")]
            inv_inertia: 1.0 / props.principal_inertia(),
            #[cfg(feature = "3d")]
            inv_inertia: props.reconstruct_inverse_inertia_matrix().into(),

            local_center_of_mass: props.local_com.into(),
        }
    }
    #[cfg(feature = "2d")]
    pub(crate) fn with_rotation(self, _rot: &Rot) -> Self {
        self
    }
    #[cfg(feature = "3d")]
    pub fn with_rotation(self, rot: &Rot) -> Self {
        Self {
            inertia: get_rotated_inertia_tensor(self.inertia, rot.0),
            inv_inertia: get_rotated_inertia_tensor(self.inv_inertia, rot.0),
            ..self
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

        self.mass_properties = MassProperties {
            mass: props.mass(),
            inv_mass: props.inv_mass,

            #[cfg(feature = "2d")]
            inertia: props.principal_inertia(),
            #[cfg(feature = "3d")]
            inertia: props.reconstruct_inertia_matrix().into(),

            #[cfg(feature = "2d")]
            inv_inertia: 1.0 / props.principal_inertia(),
            #[cfg(feature = "3d")]
            inv_inertia: props.reconstruct_inverse_inertia_matrix().into(),

            local_center_of_mass: props.local_com.into(),
        }
    }

    pub(crate) fn update_aabb_with_margin(
        &mut self,
        pos: &Vector,
        rot: &Rot,
        shape: &SharedShape,
        margin: f32,
    ) {
        self.aabb = aabb_with_margin(pos, rot, shape, margin);
    }
}
