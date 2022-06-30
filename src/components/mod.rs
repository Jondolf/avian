mod rotation;

pub use rotation::*;

use bevy::{ecs::query::WorldQuery, prelude::*};
use parry::{bounding_volume::AABB, shape::SharedShape};

use crate::{utils::aabb_with_margin, Vector};

#[cfg(feature = "3d")]
use crate::utils::get_rotated_inertia_tensor;

#[derive(WorldQuery)]
#[world_query(mutable)]
/// Groups components that bodies generally need to have in all constraints.\
/// This makes passing components to functions much more concise.
pub struct ConstraintBodyQuery<'w> {
    pub rb: &'w RigidBody,
    pub pos: &'w mut Pos,
    pub rot: &'w mut Rot,
    pub prev_pos: &'w PrevPos,
    pub prev_rot: &'w PrevRot,
    pub mass_props: &'w MassProperties,
}

#[derive(Reflect, Clone, Copy, Component, PartialEq, Eq)]
#[reflect(Component)]
pub enum RigidBody {
    /// Dynamic bodies are bodies that are affected by forces, velocity and collisions.
    ///
    /// You should generally move dynamic bodies by modifying the [`ExternalForce`], [`LinVel`] or [`AngVel`] components. Directly changing the [`Pos`] or [`Rot`] works as well, but it may cause unwanted behaviour if the body happens to teleport into the colliders of other bodies.
    Dynamic,

    /// Static bodies are not affected by any forces, collisions or velocity, and they act as if they have an infinite mass and moment of inertia. The only way to move a static body is to manually change its position.
    ///
    /// Collisions with static bodies will affect dynamic bodies, but not other static bodies or kinematic bodies.
    ///
    /// Static bodies are typically used for things like the ground, walls and any other objects that you don't want to move.
    Static,

    /// Kinematic bodies are bodies that are not affected by any external forces or collisions. They will realistically affect colliding dynamic bodies, but not other kinematic bodies.
    ///
    /// Unlike static bodies, the [`Pos`], [`LinVel`] and [`AngVel`] components will move kinematic bodies as expected. These components will never be altered by the physics engine, so you can kinematic bodies freely.
    Kinematic,
}

impl RigidBody {
    pub fn is_dynamic(&self) -> bool {
        *self == Self::Dynamic
    }

    pub fn is_static(&self) -> bool {
        *self == Self::Static
    }

    pub fn is_kinematic(&self) -> bool {
        *self == Self::Kinematic
    }
}

impl Default for RigidBody {
    fn default() -> Self {
        Self::Dynamic
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct Pos(pub Vector);

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct PrevPos(pub Vector);

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct LinVel(pub Vector);

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct PreSolveLinVel(pub Vector);

#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct AngVel(pub f32);

#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct AngVel(pub Vec3);

#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct PreSolveAngVel(pub f32);

#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct PreSolveAngVel(pub Vec3);

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct ExternalForce(pub Vector);

#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct ExternalTorque(pub f32);

#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct ExternalTorque(pub Vector);

/// 0.0: perfectly inelastic\
/// 1.0: perfectly elastic\
/// 2.0: kinetic energy is doubled
#[derive(Reflect, Clone, Copy, Component, Debug)]
#[reflect(Component)]
pub struct Restitution(pub f32);

impl Default for Restitution {
    fn default() -> Self {
        Self(0.3)
    }
}

/// 0.0: no friction at all, the body slides infinitely\
/// 1.0: high friction\
#[derive(Reflect, Clone, Copy, Component, Debug)]
#[reflect(Component)]
pub struct Friction {
    pub dynamic_coefficient: f32,
    pub static_coefficient: f32,
}

impl Friction {
    pub const ZERO: Self = Self {
        dynamic_coefficient: 0.0,
        static_coefficient: 0.0,
    };

    /// Creates a new Friction component with the same dynamic and static friction coefficients.
    fn new(friction_coefficient: f32) -> Self {
        Self {
            dynamic_coefficient: friction_coefficient,
            static_coefficient: friction_coefficient,
        }
    }
}

impl Default for Friction {
    fn default() -> Self {
        Self::new(0.7)
    }
}

#[cfg(feature = "2d")]
pub(crate) type Inertia = f32;

#[cfg(feature = "3d")]
pub(crate) type Inertia = Mat3;

#[derive(Reflect, Clone, Copy, Component, PartialEq)]
#[reflect(Component)]
pub struct MassProperties {
    pub mass: f32,
    pub inv_mass: f32,

    /// The moment of inertia of the body. In 2D this is scalar because bodies can only rotate around one axis.
    #[cfg(feature = "2d")]
    pub inertia: f32,

    /// The local moment of inertia of the body as a 3x3 tensor matrix. This is computed in local-space, so the object's orientation is not taken into account.
    ///
    /// For the world-space version, consider using [`MassProperties::world_inertia`].

    /// The inverse moment of inertia of the body. In 2D this is scalar because bodies can only rotate around one axis.
    #[cfg(feature = "3d")]
    pub inertia: Mat3,

    #[cfg(feature = "2d")]
    pub inv_inertia: f32,

    /// The local inverse moment of inertia of the body as a 3x3 tensor matrix. This is computed in local-space, so the object's orientation is not taken into account.
    ///
    /// For the world-space version, consider using [`MassProperties::world_inv_inertia`].
    #[cfg(feature = "3d")]
    pub inv_inertia: Mat3,

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
    #[allow(dead_code)]
    /// This is equivalent to just "inertia" in 2D.
    pub(crate) fn world_inertia(&self, _rot: &Rot) -> f32 {
        self.inertia
    }
    #[cfg(feature = "2d")]
    /// This is equivalent to just "inv_inertia" in 2D.
    pub(crate) fn world_inv_inertia(&self, _rot: &Rot) -> f32 {
        self.inv_inertia
    }
    #[cfg(feature = "3d")]
    /// Computes the world-space moment of inertia of the body.
    ///
    /// This is a rather expensive operation, so use it sparingly.
    pub fn world_inertia(self, rot: &Rot) -> Mat3 {
        get_rotated_inertia_tensor(self.inertia, rot.0)
    }
    #[cfg(feature = "3d")]
    /// Computes the world-space inverse moment of inertia of the body.
    ///
    /// This is a rather expensive operation, so use it sparingly.
    pub fn world_inv_inertia(self, rot: &Rot) -> Mat3 {
        get_rotated_inertia_tensor(self.inv_inertia, rot.0)
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
#[derive(Reflect, Clone, Copy, Component, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
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
