//! Components used for rigid bodies, colliders and mass properties.

mod collider;
mod layers;
mod locked_axes;
mod mass_properties;
mod rotation;
mod world_queries;

use std::ops::{Deref, DerefMut};

pub use collider::*;
pub use layers::*;
pub use locked_axes::*;
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
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     commands.spawn(RigidBody::Dynamic);
/// }
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
/// See the [section below](#adding-mass-properties) for how to add mass properties.
///
/// ## Adding mass properties
///
/// You should always give dynamic rigid bodies mass properties. The easiest way to do this is to [add a collider](Collider), since colliders
/// by default have [their own mass properties](ColliderMassProperties) that are added to the body's own mass properties.
///
/// ```ignore
/// // The mass properties will be computed from a ball shape with a radius of 0.5 and a density of 1.
/// commands.spawn((RigidBody::Dynamic, Collider::ball(0.5)));
/// ```
///
/// If you don't want to add a collider, you can instead add a [`MassPropertiesBundle`] with the mass properties computed from a collider
/// shape using the [`MassPropertiesBundle::new_computed`](MassPropertiesBundle#method.new_computed) method.
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
    /// You should generally move dynamic bodies by modifying the [`ExternalForce`], [`ExternalTorque`], [`LinearVelocity`] and [`AngularVelocity`] components.
    /// Directly changing the [`Position`] or [`Rotation`] works as well, but it may cause unwanted behaviour if the body happens to teleport into the colliders of other bodies.
    #[default]
    Dynamic,

    /// Static bodies are not affected by any forces, collisions or velocity, and they act as if they have an infinite mass and moment of inertia.
    /// The only way to move a static body is to manually change its position.
    ///
    /// Collisions with static bodies will affect dynamic bodies, but not other static bodies or kinematic bodies.
    ///
    /// Static bodies are typically used for things like the ground, walls and any other objects that you don't want to move.
    Static,

    /// Kinematic bodies are bodies that are not affected by any external forces or collisions. They will realistically affect colliding dynamic bodies, but not other kinematic bodies.
    ///
    /// Unlike static bodies, the [`Position`], [`LinearVelocity`] and [`AngularVelocity`] components will move kinematic bodies as expected.
    /// These components will never be altered by the physics engine, so you can move kinematic bodies freely.
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

/// The previous position of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct PreviousPosition(pub Vector);

/// The linear velocity of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub struct LinearVelocity(pub Vector);

impl LinearVelocity {
    /// Zero linear velocity.
    pub const ZERO: LinearVelocity = LinearVelocity(Vector::ZERO);
}

/// The linear velocity of a body before the velocity solve is performed.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub(crate) struct PreSolveLinearVelocity(pub Vector);

/// The angular velocity of a body in radians. Positive values will result in counterclockwise rotation.
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
    /// Zero angular velocity.
    #[cfg(feature = "2d")]
    pub const ZERO: AngularVelocity = AngularVelocity(0.0);
    /// Zero angular velocity.
    #[cfg(feature = "3d")]
    pub const ZERO: AngularVelocity = AngularVelocity(Vector::ZERO);
}

/// The angular velocity of a body in radians before the velocity solve is performed. Positive values will result in counter-clockwise rotation.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[reflect(Component)]
pub(crate) struct PreSolveAngularVelocity(pub Scalar);

/// The angular velocity of a body in radians before the velocity solve is performed.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub(crate) struct PreSolveAngularVelocity(pub Vector);

/// An external force applied to a dynamic [rigid body](RigidBody) during the integration step.
///
/// By default, the force persists across frames. You can clear the force manually using
/// [`clear`](#method.clear) or set `persistent` to false.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Apply a force every physics frame.
///     commands.spawn((RigidBody::Dynamic, ExternalForce::new(Vec3::Y)));
///
///     // Apply an initial force and automatically clear it every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple forces.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::default()
///             .apply_force(Vec3::Y)
///             .apply_force(Vec3::X),
///     ));
///
///     // Apply a force at a specific point relative to the given center of mass, also applying a torque.
///     // In this case, the torque would cause the body to rotate counterclockwise.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::default().apply_force_at_point(Vec3::Y, Vec3::X, Vec3::ZERO),
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalForce {
    /// The total external force that will be applied.
    force: Vector,
    /// True if the force persists across frames, and false if the force is automatically cleared every physics frame.
    ///
    /// If you clear the force manually, use the [`clear`](#method.clear) method. This will clear the force and
    /// the torque that is applied when the force is not applied at the center of mass.
    pub persistent: bool,
    /// The torque caused by forces applied at certain points using [`apply_force_at_point`](#method.apply_force_At_point).
    torque: Torque,
}

impl Deref for ExternalForce {
    type Target = Vector;

    fn deref(&self) -> &Self::Target {
        &self.force
    }
}

impl DerefMut for ExternalForce {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.force
    }
}

impl Default for ExternalForce {
    fn default() -> Self {
        Self {
            force: Vector::ZERO,
            persistent: true,
            torque: Torque::ZERO,
        }
    }
}

impl ExternalForce {
    /// Zero external force.
    pub const ZERO: Self = Self {
        force: Vector::ZERO,
        persistent: true,
        torque: Torque::ZERO,
    };

    /// Creates a new [`ExternalForce`] component with a given `force`.
    pub fn new(force: Vector) -> Self {
        Self { force, ..default() }
    }

    /// Sets the force. Note that the torque caused by any forces will not be reset.
    pub fn set_force(&mut self, force: Vector) -> &mut Self {
        **self = force;
        self
    }

    /// Adds the given `force` to the force that will be applied.
    pub fn apply_force(&mut self, force: Vector) -> &mut Self {
        **self += force;
        self
    }

    /// Applies the given `force` at a local `point`, which will also cause torque to be applied.
    pub fn apply_force_at_point(
        &mut self,
        force: Vector,
        point: Vector,
        center_of_mass: Vector,
    ) -> &mut Self {
        **self += force;
        #[cfg(feature = "2d")]
        {
            self.torque += (point - center_of_mass).perp_dot(force);
        }
        #[cfg(feature = "3d")]
        {
            self.torque += (point - center_of_mass).cross(force);
        }
        self
    }

    /// Returns the force.
    pub fn force(&self) -> Vector {
        self.force
    }

    /// Returns the torque caused by forces applied at certain points using
    /// [`apply_force_at_point`](#method.apply_force_At_point).
    pub fn torque(&self) -> Torque {
        self.torque
    }

    /// Determines if the force is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Sets the force and the potential torque caused by the force to zero.
    pub fn clear(&mut self) {
        self.force = Vector::ZERO;
        self.torque = Torque::ZERO;
    }
}

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

/// An external torque applied to a dynamic [rigid body](RigidBody) during the integration step.
///
/// By default, the torque persists across frames. You can clear the torque manually using
/// [`clear`](#method.clear) or set `persistent` to false.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Apply a torque every physics frame.
///     commands.spawn((RigidBody::Dynamic, ExternalTorque::new(Vec3::Y)));
///
///     // Apply an initial torque and automatically clear it every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalTorque::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple torques.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalTorque::default()
///             .apply_torque(Vec3::Y)
///             .apply_torque(Vec3::X),
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalTorque {
    /// The total external torque that will be applied.
    pub torque: Torque,
    /// True if the torque persists across frames, and false if the torque is automatically cleared every physics frame.
    pub persistent: bool,
}

impl Deref for ExternalTorque {
    type Target = Torque;

    fn deref(&self) -> &Self::Target {
        &self.torque
    }
}

impl DerefMut for ExternalTorque {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.torque
    }
}

impl Default for ExternalTorque {
    fn default() -> Self {
        Self {
            torque: Torque::ZERO,
            persistent: true,
        }
    }
}

impl ExternalTorque {
    /// Zero external torque.
    pub const ZERO: Self = Self {
        torque: Torque::ZERO,
        persistent: true,
    };

    /// Creates a new [`ExternalTorque`] component with a given `torque`.
    pub fn new(torque: Torque) -> Self {
        Self {
            torque,
            ..default()
        }
    }

    /// Sets the torque.
    pub fn set_torque(&mut self, torque: Torque) -> &mut Self {
        **self = torque;
        self
    }

    /// Adds the given `torque` to the torque that will be applied.
    pub fn apply_torque(&mut self, torque: Torque) -> &mut Self {
        **self += torque;
        self
    }

    /// Determines if the torque is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Returns the torque.
    pub fn torque(&self) -> Torque {
        self.torque
    }

    /// Sets the torque to zero.
    pub fn clear(&mut self) {
        self.torque = Torque::ZERO;
    }
}

/// Controls how [gravity](Gravity) affects a specific [rigid body](RigidBody).
///
/// A gravity scale of `0.0` will disable gravity, while `2.0` will double the gravity.
/// Using a negative value will flip the direction of the gravity.
#[derive(
    Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deref, DerefMut, From,
)]
#[reflect(Component)]
pub struct GravityScale(pub Scalar);

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

/// Controls how elastic or bouncy an entity is when colliding with other entities.
///
/// 0.0: Perfectly inelastic\
/// 1.0: Perfectly elastic\
/// 2.0: Kinetic energy is doubled
///
/// ## Example
///
/// Create a new [`Restitution`] component with a restitution coefficient of 0.4:
///
/// ```ignore
/// Restitution::new(0.4)
/// ```
///
/// Configure how two restitution coefficients are combined with [`CoefficientCombine`]:
///
/// ```ignore
/// Restitution::new(0.4).with_combine_rule(CoefficientCombine::Multiply)
/// ```
///
/// Combine the properties of two [`Restitution`] components:
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// let first = Restitution::new(0.8).with_combine_rule(CoefficientCombine::Average);
/// let second = Restitution::new(0.5).with_combine_rule(CoefficientCombine::Multiply);
///
/// // CoefficientCombine::Multiply has higher priority, so the coefficients are multiplied
/// assert_eq!(
///     first.combine(second),
///     Restitution::new(0.4).with_combine_rule(CoefficientCombine::Multiply)
/// );
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, PartialOrd)]
#[reflect(Component)]
pub struct Restitution {
    /// Coefficient of restitution.
    pub coefficient: Scalar,
    /// The coefficient combine rule used when two bodies interact.
    pub combine_rule: CoefficientCombine,
}

impl Restitution {
    /// Zero restitution and [`CoefficientCombine::Average`].
    pub const ZERO: Self = Self {
        coefficient: 0.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// Creates a new [`Restitution`] component with the given restitution coefficient.
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

    /// Combines the properties of two [`Restitution`] components.
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

/// Controls how strongly the material of an entity prevents relative tangential movement at contact points.
///
/// For surfaces that are at rest relative to each other, static friction is used.
/// Once the static friction is overcome, the bodies will start sliding relative to each other, and dynamic friction is applied instead.
///
/// 0.0: No friction at all, the body slides indefinitely\
/// 1.0: High friction\
///
/// ## Example
///
/// Create a new [`Friction`] component with dynamic and static friction coefficients of 0.4:
///
/// ```ignore
/// Friction::new(0.4)
/// ```
///
/// Set the other friction coefficient:
///
/// ```ignore
/// // 0.4 static and 0.6 dynamic
/// Friction::new(0.4).with_dynamic_coefficient(0.6)
/// // 0.4 dynamic and 0.6 static
/// Friction::new(0.4).with_static_coefficient(0.6)
/// ```
///
/// Configure how the friction coefficients of two [`Friction`] components are combined with [`CoefficientCombine`]:
///
/// ```ignore
/// Friction::new(0.4).with_combine_rule(CoefficientCombine::Multiply)
/// ```
///
/// Combine the properties of two [`Friction`] components:
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// let first = Friction::new(0.8).with_combine_rule(CoefficientCombine::Average);
/// let second = Friction::new(0.5).with_combine_rule(CoefficientCombine::Multiply);
///
/// // CoefficientCombine::Multiply has higher priority, so the coefficients are multiplied
/// assert_eq!(
///     first.combine(second),
///     Friction::new(0.4).with_combine_rule(CoefficientCombine::Multiply)
/// );
/// ```
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
    /// Zero dynamic and static friction and [`CoefficientCombine::Average`].
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

    /// Sets the coefficient of dynamic friction.
    pub fn with_dynamic_coefficient(&self, coefficient: Scalar) -> Self {
        Self {
            dynamic_coefficient: coefficient,
            ..*self
        }
    }

    /// Sets the coefficient of static friction.
    pub fn with_static_coefficient(&self, coefficient: Scalar) -> Self {
        Self {
            static_coefficient: coefficient,
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

/// Automatically slows down a dynamic [rigid body](RigidBody), decreasing it's [linear velocity](LinearVelocity)
/// each frame. This can be used to simulate air resistance.
///
/// The default linear damping coefficient is `0.0`, which corresponds to no damping.
#[derive(
    Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deref, DerefMut, From,
)]
#[reflect(Component)]
pub struct LinearDamping(pub Scalar);

/// Automatically slows down a dynamic [rigid body](RigidBody), decreasing it's [angular velocity](AngularVelocity)
/// each frame. This can be used to simulate air resistance.
///
/// The default angular damping coefficient is `0.0`, which corresponds to no damping.
#[derive(
    Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deref, DerefMut, From,
)]
#[reflect(Component)]
pub struct AngularDamping(pub Scalar);

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
