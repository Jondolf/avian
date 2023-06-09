//! Components used for rigid bodies, colliders and mass properties.

mod rotation;
mod world_queries;

pub use rotation::*;
pub use world_queries::*;

use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;
use parry::{bounding_volume::Aabb, shape::SharedShape};

#[cfg(feature = "3d")]
use crate::utils::get_rotated_inertia_tensor;

/// The rigid body type. A rigid body can be either dynamic, kinematic or static.
#[derive(Reflect, Default, Clone, Copy, Component, PartialEq, Eq)]
#[reflect(Component)]
pub enum RigidBody {
    /// Dynamic bodies are bodies that are affected by forces, velocity and collisions.
    ///
    /// You should generally move dynamic bodies by modifying the [`ExternalForce`], [`LinVel`] or [`AngVel`] components. Directly changing the [`Pos`] or [`Rot`] works as well, but it may cause unwanted behaviour if the body happens to teleport into the colliders of other bodies.
    #[default]
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
    /// Checks if the rigid body is dynamic.
    pub fn is_dynamic(&self) -> bool {
        *self == Self::Dynamic
    }

    /// Checks if the rigid body is static.
    pub fn is_static(&self) -> bool {
        *self == Self::Static
    }

    /// Checks if the rigid body is kinematic.
    pub fn is_kinematic(&self) -> bool {
        *self == Self::Kinematic
    }
}

/// Indicates that a body is not simulated by the physics engine until woken up again.
/// This is done to improve performance and to help prevent small jitter that is typically present in collisions.
///
/// Bodies are marked as sleeping when their linear and angular velocity is below the [`SleepingThreshold`] for a time
/// indicated by [`DeactivationTime`]. A sleeping body is woken up when an active body interacts with it through
/// collisions or other constraints, or when gravity changes, or when the body's
/// position, rotation, velocity, or external forces are modified.
///
/// Note that sleeping can cause unrealistic behaviour in some cases.
/// For example, removing the floor under sleeping bodies can leave them floating in the air.
/// Sleeping can be disabled for specific entities with the [`SleepingDisabled`] component,
/// or for all entities by setting the [`SleepingThreshold`] to a negative value.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq, From)]
#[reflect(Component)]
pub struct Sleeping;

/// How long the velocity of the body has been below the [`SleepingThreshold`],
/// i.e. how long the body has been able to sleep.
///
/// See [`Sleeping`] for further information.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[reflect(Component)]
pub struct TimeSleeping(pub Scalar);

/// Indicates that the body can not be deactivated by the physics engine. See [`Sleeping`] for information about sleeping.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq, From)]
#[reflect(Component)]
pub struct SleepingDisabled;

/// The position of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct Pos(pub Vector);

#[cfg(all(feature = "2d", feature = "f64"))]
impl From<Vec2> for Pos {
    fn from(value: Vec2) -> Self {
        value.as_dvec2().into()
    }
}

#[cfg(all(feature = "3d", feature = "f64"))]
impl From<Vec3> for Pos {
    fn from(value: Vec3) -> Self {
        value.as_dvec3().into()
    }
}

/// The previous position of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct PrevPos(pub Vector);

/// The linear velocity of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct LinVel(pub Vector);

impl LinVel {
    pub const ZERO: LinVel = LinVel(Vector::ZERO);
}

#[cfg(all(feature = "2d", feature = "f64"))]
impl From<Vec2> for LinVel {
    fn from(value: Vec2) -> Self {
        value.as_dvec2().into()
    }
}

#[cfg(all(feature = "3d", feature = "f64"))]
impl From<Vec3> for LinVel {
    fn from(value: Vec3) -> Self {
        value.as_dvec3().into()
    }
}

/// The linear velocity of a body before the velocity solve is performed.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct PreSolveLinVel(pub Vector);

/// The angular velocity of a body in radians. Positive values will result in counter-clockwise rotation.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct AngVel(pub Scalar);

/// The angular velocity of a body in radians.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct AngVel(pub Vector);

impl AngVel {
    #[cfg(feature = "2d")]
    pub const ZERO: AngVel = AngVel(0.0);
    #[cfg(feature = "3d")]
    pub const ZERO: AngVel = AngVel(Vector::ZERO);
}

#[cfg(all(feature = "3d", feature = "f64"))]
impl From<Vec3> for AngVel {
    fn from(value: Vec3) -> Self {
        value.as_dvec3().into()
    }
}

/// The angular velocity of a body in radians before the velocity solve is performed. Positive values will result in counter-clockwise rotation.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct PreSolveAngVel(pub Scalar);

/// The angular velocity of a body in radians before the velocity solve is performed.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct PreSolveAngVel(pub Vector);

/// An external force applied to a body during the integration step. It persists during the simulation, so it must be cleared manually.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalForce(pub Vector);

#[cfg(feature = "2d")]
pub(crate) type Torque = Scalar;

#[cfg(feature = "3d")]
pub(crate) type Torque = Vector;

/// An external torque applied to a body during the integration step. It persists during the simulation, so it must be cleared manually.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalTorque(pub Torque);

#[cfg(all(feature = "3d", feature = "f64"))]
impl From<Vec3> for ExternalTorque {
    fn from(value: Vec3) -> Self {
        value.as_dvec3().into()
    }
}

/// Determines how coefficients are combined. The default is `Average`.
///
/// When combine rules clash with each other, the following priority order is used: `Max > Multiply > Min > Average`.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum CoefficientCombine {
    // The discriminants allow priority ordering to work automatically via comparison methods
    /// Coefficients are combined by computing their average.
    #[default]
    Average = 1,
    /// Coefficients are combined by choosing the smaller coefficient.
    Min = 2,
    /// Coefficients are combined by computing their product.
    Multiply = 3,
    /// Coefficients are combined by choosing the larger coefficient.
    Max = 4,
}

/// Restitution controls how elastic or bouncy a body is.
///
/// 0.0: perfectly inelastic\
/// 1.0: perfectly elastic\
/// 2.0: kinetic energy is doubled
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, PartialOrd)]
#[reflect(Component)]
pub struct Restitution {
    /// Coefficient of restitution.
    pub coefficient: Scalar,
    /// The coefficient combine rule used when two bodies interact.
    pub combine_rule: CoefficientCombine,
}

impl Restitution {
    pub const ZERO: Self = Self {
        coefficient: 0.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// Creates a new `Restitution` component with the given restitution coefficient.
    pub fn new(coefficient: Scalar) -> Self {
        Self {
            coefficient,
            ..default()
        }
    }

    /// Sets the [`CoefficientCombine`] rule used.
    pub fn with_combine_rule(&self, combine_rule: CoefficientCombine) -> Self {
        Self {
            combine_rule,
            ..*self
        }
    }

    /// Combines the properties of two `Restitution` components.
    pub fn combine(&self, other: Self) -> Self {
        // Choose rule with higher priority
        let rule = self.combine_rule.max(other.combine_rule);

        Self {
            coefficient: match rule {
                CoefficientCombine::Average => (self.coefficient + other.coefficient) * 0.5,
                CoefficientCombine::Min => self.coefficient.min(other.coefficient),
                CoefficientCombine::Multiply => self.coefficient * other.coefficient,
                CoefficientCombine::Max => self.coefficient.max(other.coefficient),
            },
            combine_rule: rule,
        }
    }
}

impl Default for Restitution {
    fn default() -> Self {
        Self {
            coefficient: 0.3,
            combine_rule: CoefficientCombine::default(),
        }
    }
}

impl From<Scalar> for Restitution {
    fn from(coefficient: Scalar) -> Self {
        Self {
            coefficient,
            ..default()
        }
    }
}

/// Friction prevents relative tangential movement at contact points.
///
/// For surfaces that are at rest relative to each other, static friction is used. Once it is overcome, the bodies start sliding relative to each other, and dynamic friction is applied instead.
///
/// 0.0: no friction at all, the body slides indefinitely\
/// 1.0: high friction\
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, PartialOrd)]
#[reflect(Component)]
pub struct Friction {
    /// Coefficient of dynamic friction.
    pub dynamic_coefficient: Scalar,
    /// Coefficient of static friction.
    pub static_coefficient: Scalar,
    /// The coefficient combine rule used when two bodies interact.
    pub combine_rule: CoefficientCombine,
}

impl Friction {
    pub const ZERO: Self = Self {
        dynamic_coefficient: 0.0,
        static_coefficient: 0.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// Creates a new `Friction` component with the same dynamic and static friction coefficients.
    pub fn new(friction_coefficient: Scalar) -> Self {
        Self {
            dynamic_coefficient: friction_coefficient,
            static_coefficient: friction_coefficient,
            ..default()
        }
    }

    /// Sets the [`CoefficientCombine`] rule used.
    pub fn with_combine_rule(&self, combine_rule: CoefficientCombine) -> Self {
        Self {
            combine_rule,
            ..*self
        }
    }

    /// Combines the properties of two `Friction` components.
    pub fn combine(&self, other: Self) -> Self {
        // Choose rule with higher priority
        let rule = self.combine_rule.max(other.combine_rule);
        let (dynamic1, dynamic2) = (self.dynamic_coefficient, other.dynamic_coefficient);
        let (static1, static2) = (self.static_coefficient, other.static_coefficient);

        Self {
            dynamic_coefficient: match rule {
                CoefficientCombine::Average => (dynamic1 + dynamic2) * 0.5,
                CoefficientCombine::Min => dynamic1.min(dynamic2),
                CoefficientCombine::Multiply => dynamic1 * dynamic2,
                CoefficientCombine::Max => dynamic1.max(dynamic2),
            },
            static_coefficient: match rule {
                CoefficientCombine::Average => (static1 + static2) * 0.5,
                CoefficientCombine::Min => static1.min(static2),
                CoefficientCombine::Multiply => static1 * static2,
                CoefficientCombine::Max => static1.max(static2),
            },
            combine_rule: rule,
        }
    }
}

impl Default for Friction {
    fn default() -> Self {
        Self {
            dynamic_coefficient: 0.3,
            static_coefficient: 0.3,
            combine_rule: CoefficientCombine::default(),
        }
    }
}

impl From<Scalar> for Friction {
    fn from(coefficient: Scalar) -> Self {
        Self {
            dynamic_coefficient: coefficient,
            static_coefficient: coefficient,
            ..default()
        }
    }
}

/// The mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Mass(pub Scalar);

impl Mass {
    pub const ZERO: Self = Self(0.0);
}

/// The inverse mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InvMass(pub Scalar);

impl InvMass {
    pub const ZERO: Self = Self(0.0);
}

/// The moment of inertia of a body. This represents the torque needed for a desired angular acceleration.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Inertia(pub Scalar);

/// The local moment of inertia of the body as a 3x3 tensor matrix.
/// This represents the torque needed for a desired angular acceleration along different axes.
///
/// This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account, use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Inertia(pub Matrix3);

#[cfg(feature = "3d")]
impl Default for Inertia {
    fn default() -> Self {
        Self(Matrix3::ZERO)
    }
}

impl Inertia {
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(0.0);
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Matrix3::ZERO);

    /// In 2D this does nothing, but it is there for convenience so that you don't have to handle 2D and 3D separately.
    #[cfg(feature = "2d")]
    #[allow(dead_code)]
    pub(crate) fn rotated(&self, _rot: &Rot) -> Self {
        *self
    }

    /// Returns the inertia tensor's world-space version that takes the body's orientation into account.
    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rot) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    /// Returns the inverted moment of inertia.
    #[cfg(feature = "2d")]
    pub fn inverse(&self) -> InvInertia {
        InvInertia(1.0 / self.0)
    }

    /// Returns the inverted moment of inertia.
    #[cfg(feature = "3d")]
    pub fn inverse(&self) -> InvInertia {
        InvInertia(self.0.inverse())
    }
}

/// The inverse moment of inertia of the body. This represents the inverse of the torque needed for a desired angular acceleration.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InvInertia(pub Scalar);

/// The local inverse moment of inertia of the body as a 3x3 tensor matrix.
/// This represents the inverse of the torque needed for a desired angular acceleration along different axes.
///
/// This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account, use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InvInertia(pub Matrix3);

#[cfg(feature = "3d")]
impl Default for InvInertia {
    fn default() -> Self {
        InvInertia(Matrix3::ZERO)
    }
}

impl InvInertia {
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(0.0);
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Matrix3::ZERO);

    /// In 2D this does nothing, but it is there for convenience so that you don't have to handle 2D and 3D separately.
    #[cfg(feature = "2d")]
    pub fn rotated(&self, _rot: &Rot) -> Self {
        *self
    }

    /// Returns the inertia tensor's world-space version that takes the body's orientation into account.
    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rot) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    /// Returns the original moment of inertia.
    #[cfg(feature = "2d")]
    pub fn inverse(&self) -> Inertia {
        Inertia(1.0 / self.0)
    }

    /// Returns the original moment of inertia.
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

/// The local center of mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct LocalCom(pub Vector);

impl LocalCom {
    pub const ZERO: Self = Self(Vector::ZERO);
}

/// The mass properties derived from a given collider shape and density.
///
/// These will be added to the body's actual [`Mass`], [`InvMass`], [`Inertia`], [`InvInertia`] and [`LocalCom`] components.
///
/// You should generally not create or modify this directly. Instead, you can generate this automatically using a given collider shape and density with the associated `from_shape_and_density` method.
#[derive(Reflect, Clone, Copy, Component, PartialEq)]
#[reflect(Component)]
pub struct ColliderMassProperties {
    /// Mass given by collider.
    pub mass: Mass,
    /// Inverse mass given by collider.
    pub inv_mass: InvMass,
    /// Inertia given by collider.
    pub inertia: Inertia,
    /// Inverse inertia given by collider.
    pub inv_inertia: InvInertia,
    /// Local center of mass given by collider.
    pub local_center_of_mass: LocalCom,
    /// Density used for calculating other mass properties.
    pub density: Scalar,
}

impl ColliderMassProperties {
    pub const ZERO: Self = Self {
        mass: Mass::ZERO,
        inv_mass: InvMass(Scalar::INFINITY),
        inertia: Inertia::ZERO,
        inv_inertia: InvInertia::ZERO,
        local_center_of_mass: LocalCom::ZERO,
        density: 0.0,
    };
}

impl ColliderMassProperties {
    /// Computes mass properties for a given shape and density.
    pub fn from_shape_and_density(shape: &SharedShape, density: Scalar) -> Self {
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

/// The previous [`ColliderMassProperties`].
#[derive(Clone, Copy, Component, Default, Deref, DerefMut, PartialEq)]
pub(crate) struct PrevColliderMassProperties(pub ColliderMassProperties);

/// A physics shape used for things like colliders.
pub type Shape = SharedShape;

/// A component for the [`Shape`] used for a collider.
#[derive(Clone, Component, Deref, DerefMut)]
pub struct ColliderShape(pub Shape);

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

/// The Axis-Aligned Bounding Box of a collider.
#[derive(Clone, Copy, Component, Deref, DerefMut, PartialEq)]
pub struct ColliderAabb(pub Aabb);

impl ColliderAabb {
    /// Creates a new collider from a given [`Shape`] with a default density of 1.0.
    pub fn from_shape(shape: &Shape) -> Self {
        Self(shape.compute_local_aabb())
    }
}

impl Default for ColliderAabb {
    fn default() -> Self {
        ColliderAabb(Aabb::new_invalid())
    }
}

#[cfg(test)]
mod tests {
    use crate::prelude::*;

    #[test]
    fn coefficient_combine_works() {
        let r1 = Restitution::new(0.3).with_combine_rule(CoefficientCombine::Average);

        // (0.3 + 0.7) / 2.0 == 0.5
        assert_eq!(
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Average)),
            Restitution::new(0.5).with_combine_rule(CoefficientCombine::Average)
        );

        // 0.3.min(0.7) == 0.3
        assert_eq!(
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Min)),
            Restitution::new(0.3).with_combine_rule(CoefficientCombine::Min)
        );

        // 0.3 * 0.7 == 0.21
        assert_eq!(
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Multiply)),
            Restitution::new(0.21).with_combine_rule(CoefficientCombine::Multiply)
        );

        // 0.3.max(0.7) == 0.7
        assert_eq!(
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Max)),
            Restitution::new(0.7).with_combine_rule(CoefficientCombine::Max)
        );
    }
}
