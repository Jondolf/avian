mod rotation;

pub use rotation::*;

use crate::Vector;
use bevy::{ecs::query::WorldQuery, prelude::*};
use parry::{bounding_volume::Aabb, shape::SharedShape};
use std::ops::{AddAssign, SubAssign};

#[cfg(feature = "3d")]
use crate::utils::get_rotated_inertia_tensor;

#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct RigidBodyQuery<'w> {
    pub rb: &'w RigidBody,
    pub pos: &'w mut Pos,
    pub rot: &'w mut Rot,
    pub prev_pos: &'w PrevPos,
    pub prev_rot: &'w PrevRot,
    pub lin_vel: &'w mut LinVel,
    pub pre_solve_lin_vel: &'w mut PreSolveLinVel,
    pub ang_vel: &'w mut AngVel,
    pub pre_solve_ang_vel: &'w mut PreSolveAngVel,
    pub mass: &'w Mass,
    pub inv_mass: &'w InvMass,
    pub inertia: &'w Inertia,
    pub inv_inertia: &'w InvInertia,
    pub local_com: &'w LocalCom,
    pub friction: &'w Friction,
    pub restitution: &'w Restitution,
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub(crate) struct MassPropsQueryMut<'w> {
    pub mass: &'w mut Mass,
    pub inv_mass: &'w mut InvMass,
    pub inertia: &'w mut Inertia,
    pub inv_inertia: &'w mut InvInertia,
    pub local_com: &'w mut LocalCom,
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub(crate) struct ColliderQuery<'w> {
    pub shape: &'w mut ColliderShape,
    pub aabb: &'w mut ColliderAabb,
    pub mass_props: &'w mut ColliderMassProperties,
    pub prev_mass_props: &'w mut PrevColliderMassProperties,
}

impl<'_w, 'w> AddAssign<ColliderMassProperties> for MassPropsQueryMutItem<'_w, 'w> {
    fn add_assign(&mut self, rhs: ColliderMassProperties) {
        self.mass.0 += rhs.mass.0;
        self.inv_mass.0 = 1.0 / self.mass.0;
        self.inertia.0 += rhs.inertia.0;
        self.inv_inertia.0 = self.inertia.inverse().0;
        self.local_com.0 += rhs.local_center_of_mass.0;
    }
}

impl<'_w, 'w> SubAssign<ColliderMassProperties> for MassPropsQueryMutItem<'_w, 'w> {
    fn sub_assign(&mut self, rhs: ColliderMassProperties) {
        self.mass.0 -= rhs.mass.0;
        self.inv_mass.0 = 1.0 / self.mass.0;
        self.inertia.0 -= rhs.inertia.0;
        self.inv_inertia.0 = self.inertia.inverse().0;
        self.local_com.0 -= rhs.local_center_of_mass.0;
    }
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

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Pos(pub Vector);

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct PrevPos(pub Vector);

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct LinVel(pub Vector);

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct PreSolveLinVel(pub Vector);

#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct AngVel(pub f32);

#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct AngVel(pub Vec3);

#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct PreSolveAngVel(pub f32);

#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct PreSolveAngVel(pub Vec3);

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct ExternalForce(pub Vector);

#[cfg(feature = "2d")]
pub(crate) type Torque = f32;

#[cfg(feature = "3d")]
pub(crate) type Torque = Vec3;

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct ExternalTorque(pub Torque);

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
        Self::new(0.3)
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Mass(pub f32);

impl Mass {
    pub const ZERO: Self = Self(0.0);
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InvMass(pub f32);

impl InvMass {
    pub const ZERO: Self = Self(0.0);
}

#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
/// The moment of inertia of the body. In 2D this is scalar because bodies can only rotate around one axis.
pub struct Inertia(pub f32);

#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
/// The local moment of inertia of the body as a 3x3 tensor matrix. This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account, use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
pub struct Inertia(pub Mat3);

impl Inertia {
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(0.0);
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Mat3::ZERO);

    #[cfg(feature = "2d")]
    /// In 2D this does nothing, but it is there for convenience so that you don't have to handle 2D and 3D separately.
    pub fn rotated(&self, _rot: &Rot) -> Self {
        *self
    }

    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rot) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    #[cfg(feature = "2d")]
    pub fn inverse(&self) -> InvInertia {
        InvInertia(1.0 / self.0)
    }

    #[cfg(feature = "3d")]
    pub fn inverse(&self) -> InvInertia {
        InvInertia(self.0.inverse())
    }
}

#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
/// The inverse moment of inertia of the body. In 2D this is scalar because bodies can only rotate around one axis.
pub struct InvInertia(pub f32);

#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
/// The local inverse moment of inertia of the body as a 3x3 tensor matrix. This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account, use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
pub struct InvInertia(pub Mat3);

impl InvInertia {
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(0.0);
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Mat3::ZERO);

    #[cfg(feature = "2d")]
    /// In 2D this does nothing, but it is there for convenience so that you don't have to handle 2D and 3D separately.
    pub fn rotated(&self, _rot: &Rot) -> Self {
        *self
    }

    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rot) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    #[cfg(feature = "2d")]
    pub fn inverse(&self) -> Inertia {
        Inertia(1.0 / self.0)
    }

    #[cfg(feature = "3d")]
    pub fn inverse(&self) -> Inertia {
        Inertia(self.0.inverse())
    }
}

impl From<Inertia> for InvInertia {
    fn from(inertia: Inertia) -> Self {
        inertia.inverse()
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
/// The local center of mass of the body.
pub struct LocalCom(pub Vector);

impl LocalCom {
    pub const ZERO: Self = Self(Vector::ZERO);
}

/// Mass properties derived from a given collider shape and density.
///
/// These will be added to the body's actual [`Mass`], [`InvMass`], [`Inertia`], [`InvInertia`] and [`LocalCom`] components.
///
/// You should generally not create or modify this directly. [`Collider`] components can generate this automatically using the collider's shape and density with the `from_shape_and_density` method.
#[derive(Reflect, Clone, Copy, Component, PartialEq)]
#[reflect(Component)]
pub struct ColliderMassProperties {
    pub mass: Mass,
    pub inv_mass: InvMass,
    pub inertia: Inertia,
    pub inv_inertia: InvInertia,
    pub local_center_of_mass: LocalCom,
    pub density: f32,
}

impl ColliderMassProperties {
    pub const ZERO: Self = Self {
        mass: Mass::ZERO,
        inv_mass: InvMass(f32::INFINITY),
        inertia: Inertia::ZERO,
        inv_inertia: InvInertia::ZERO,
        local_center_of_mass: LocalCom::ZERO,
        density: 0.0,
    };
}

impl ColliderMassProperties {
    /// Computes mass properties for a given shape and density.
    pub fn from_shape_and_density(shape: &SharedShape, density: f32) -> Self {
        let props = shape.mass_properties(density);

        Self {
            mass: Mass(props.mass()),
            inv_mass: InvMass(props.inv_mass),

            #[cfg(feature = "2d")]
            inertia: Inertia(props.principal_inertia()),
            #[cfg(feature = "3d")]
            inertia: Inertia(props.reconstruct_inertia_matrix().into()),

            #[cfg(feature = "2d")]
            inv_inertia: InvInertia(1.0 / props.principal_inertia()),
            #[cfg(feature = "3d")]
            inv_inertia: InvInertia(props.reconstruct_inverse_inertia_matrix().into()),

            local_center_of_mass: LocalCom(props.local_com.into()),

            density,
        }
    }
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        Self::ZERO
    }
}

#[derive(Clone, Copy, Component, Default, Deref, DerefMut, PartialEq)]
pub(crate) struct PrevColliderMassProperties(pub ColliderMassProperties);

/// A physics shape used for things like colliders.
pub type Shape = SharedShape;

#[derive(Clone, Component, Deref, DerefMut)]
pub(crate) struct ColliderShape(pub Shape);

impl Default for ColliderShape {
    fn default() -> Self {
        #[cfg(feature = "2d")]
        {
            Self(Shape::cuboid(0.5, 0.5))
        }
        #[cfg(feature = "3d")]
        {
            Self(Shape::cuboid(0.5, 0.5, 0.5))
        }
    }
}

#[derive(Clone, Copy, Component, Deref, DerefMut, PartialEq)]
pub(crate) struct ColliderAabb(pub Aabb);

impl ColliderAabb {
    /// Creates a new collider from a given [`ColliderShape`] with a default density of 1.0.
    pub fn from_shape(shape: &Shape) -> Self {
        Self(shape.compute_local_aabb())
    }
}

impl Default for ColliderAabb {
    fn default() -> Self {
        ColliderAabb(Aabb::new_invalid())
    }
}
