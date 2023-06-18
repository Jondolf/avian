//! Components used for rigid bodies, colliders and mass properties.

mod collider;
mod layers;
mod mass_properties;
mod rotation;
mod world_queries;

pub use collider::*;
pub use layers::*;
pub use mass_properties::*;
pub use rotation::*;
pub use world_queries::*;

use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;

/// A non-deformable body used for the simulation of most physics objects.
///
/// A rigid body can be either dynamic, kinematic or static.
///
/// - **Dynamic bodies** are similar to real life objects and are affected by forces and contacts.
/// - **Kinematic bodies** can only be moved programmatically, which is useful for things like character controllers and moving platforms.
/// - **Static bodies** can not move, so they can be good for objects in the environment like the ground and walls.
///
/// ## Creation
///
/// Creating a rigid body is as simple as adding the [`RigidBody`] component:
///
/// ```ignore
/// commands.spawn(RigidBody::Dynamic);
/// ```
///
/// Bevy XPBD will automatically add any missing components, like the following:
///
/// - [`Position`]
/// - [`Rotation`]
/// - [`LinearVelocity`]
/// - [`AngularVelocity`]
/// - [`ExternalForce`]
/// - [`ExternalTorque`]
/// - [`Friction`]
/// - [`Restitution`]
/// - [`Mass`]
/// - [`Inertia`]
/// - [`CenterOfMass`]
///
/// You can change any of these during initialization and runtime in order to alter the behaviour of the body.
///
/// Note that by default, rigid bodies don't have any mass, so dynamic bodies will gain infinite velocity upon any interaction.
/// See the [section below](#adding-mass-properties] for how to add mass properties.
///
/// ## Adding mass properties
///
/// You should always give dynamic rigid bodies mass properties. The easiest way to do this is to [add a collider](collider), since colliders
/// by default have [their own mass properties](ColliderMassProperties) that are added to the body's own mass properties.
///
/// ```ignore
/// // The mass properties will be computed from a ball shape with a radius of 0.5 and a density of 1.
/// commands.spawn((RigidBody::Dynamic, Collider::ball(0.5)));
/// ```
///
/// If you don't want to add a collider, you can instead add a [`MassPropsBundle`] with the mass properties computed from a collider
/// shape using the [`MassPropsBundle::new_computed`](MassPropsBundle#method.new_computed) method.
///
/// ```ignore
/// // This is equivalent to the earlier approach, but no collider will be added.
/// commands.spawn((RigidBody::Dynamic, MassPropertiesBundle::new_computed(&Collider::ball(0.5), 1.0)));
/// ```
///
/// If you want, you can also define the mass properties explicitly by adding the components manually.
/// Note that the mass properties of colliders are added on top of the existing mass properties, so if you
/// want to define the body's mass properties explicitly, you might want to add
/// [`ColliderMassProperties::ZERO`](ColliderMassProperties#associatedconstant.ZERO) to the colliders.
#[derive(Reflect, Default, Clone, Copy, Component, PartialEq, Eq)]
#[reflect(Component)]
pub enum RigidBody {
    /// Dynamic bodies are bodies that are affected by forces, velocity and collisions.
    ///
    /// You should generally move dynamic bodies by modifying the [`ExternalForce`], [`LinearVelocity`] or [`AngularVelocity`] components. Directly changing the [`Position`] or [`Rotation`] works as well, but it may cause unwanted behaviour if the body happens to teleport into the colliders of other bodies.
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
    /// Unlike static bodies, the [`Position`], [`LinearVelocity`] and [`AngularVelocity`] components will move kinematic bodies as expected. These components will never be altered by the physics engine, so you can kinematic bodies freely.
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
pub struct Position(pub Vector);

#[cfg(all(feature = "2d", feature = "f64"))]
impl From<Vec2> for Position {
    fn from(value: Vec2) -> Self {
        value.as_dvec2().into()
    }
}

#[cfg(all(feature = "3d", feature = "f64"))]
impl From<Vec3> for Position {
    fn from(value: Vec3) -> Self {
        value.as_dvec3().into()
    }
}

/// The previous position of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct PreviousPosition(pub Vector);

/// The linear velocity of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct LinearVelocity(pub Vector);

impl LinearVelocity {
    pub const ZERO: LinearVelocity = LinearVelocity(Vector::ZERO);
}

#[cfg(all(feature = "2d", feature = "f64"))]
impl From<Vec2> for LinearVelocity {
    fn from(value: Vec2) -> Self {
        value.as_dvec2().into()
    }
}

#[cfg(all(feature = "3d", feature = "f64"))]
impl From<Vec3> for LinearVelocity {
    fn from(value: Vec3) -> Self {
        value.as_dvec3().into()
    }
}

/// The linear velocity of a body before the velocity solve is performed.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct PreSolveLinearVelocity(pub Vector);

/// The angular velocity of a body in radians. Positive values will result in counter-clockwise rotation.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[reflect(Component)]
pub struct AngularVelocity(pub Scalar);

/// The angular velocity of a body in radians.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct AngularVelocity(pub Vector);

impl AngularVelocity {
    #[cfg(feature = "2d")]
    pub const ZERO: AngularVelocity = AngularVelocity(0.0);
    #[cfg(feature = "3d")]
    pub const ZERO: AngularVelocity = AngularVelocity(Vector::ZERO);
}

#[cfg(all(feature = "3d", feature = "f64"))]
impl From<Vec3> for AngularVelocity {
    fn from(value: Vec3) -> Self {
        value.as_dvec3().into()
    }
}

/// The angular velocity of a body in radians before the velocity solve is performed. Positive values will result in counter-clockwise rotation.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[reflect(Component)]
pub struct PreSolveAngularVelocity(pub Scalar);

/// The angular velocity of a body in radians before the velocity solve is performed.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct PreSolveAngularVelocity(pub Vector);

/// An external force applied to a body during the integration step. It persists during the simulation, so it must be cleared manually.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalForce(pub Vector);

#[cfg(feature = "2d")]
pub(crate) type Torque = Scalar;

#[cfg(feature = "3d")]
pub(crate) type Torque = Vector;

pub(crate) trait FloatZero {
    const ZERO: Self;
}

impl FloatZero for Scalar {
    const ZERO: Self = 0.0;
}

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
