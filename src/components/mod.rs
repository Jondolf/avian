//! Commonly used components.

mod forces;
mod layers;
mod locked_axes;
mod mass_properties;
mod rotation;
mod world_queries;

pub use forces::*;
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
/// ## Rigid body types
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
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic rigid body and specify its position (optional)
///     commands.spawn((
///         RigidBody::Dynamic,
///         TransformBundle::from_transform(Transform::from_xyz(0.0, 3.0, 0.0)),
///     ));
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
/// - [`ExternalImpulse`]
/// - [`ExternalAngularImpulse`]
/// - [`Friction`]
/// - [`Restitution`]
/// - [`Mass`]
/// - [`Inertia`]
/// - [`CenterOfMass`]
///
/// You can change any of these during initialization and runtime in order to alter the behaviour of the body.
///
/// By default, rigid bodies will get a mass based on their collider and density.
/// See [mass properties](#mass-properties).
///
/// ## Movement
///
/// A rigid body can be moved in three ways: by modifying its position directly,
/// by changing its velocity, or by applying forces or impulses.
///
/// To change the position of a rigid body, you can simply modify its `Transform`:
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn move_bodies(mut query: Query<&mut Transform, With<RigidBody>>) {
///     for mut transform in query.iter_mut() {
///         transform.translation.x += 0.1;
///     }
/// }
/// ```
///
/// However, moving a dynamic body by changing its position directly is similar
/// to teleporting the body, which can result in unexpected behavior since the body can move
/// inside walls.
///
/// You can instead change the velocity of a dynamic or kinematic body with the [`LinearVelocity`]
/// and [`AngularVelocity`] components:
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn accelerate_bodies(mut query: Query<(&mut LinearVelocity, &mut AngularVelocity)>) {
///     for (mut linear_velocity, mut angular_velocity) in query.iter_mut() {
///         linear_velocity.x += 0.05;
#[cfg_attr(feature = "2d", doc = "        angular_velocity.0 += 0.05;")]
#[cfg_attr(feature = "3d", doc = "        angular_velocity.z += 0.05;")]
///     }
/// }
/// ```
///
/// For applying forces and impulses to dynamic bodies, see the [`ExternalForce`], [`ExternalTorque`],
/// [`ExternalImpulse`] and [`ExternalAngularImpulse`] components.
///
/// Bevy XPBD does not have a built-in character controller, so if you need one,
/// you will need to implement it yourself or use a third party option.
/// You can take a look at the [`basic_dynamic_character`] and [`basic_kinematic_character`]
/// examples for a simple implementation.
///
/// [`basic_dynamic_character`]: https://github.com/Jondolf/bevy_xpbd/blob/42fb8b21c756a7f4dd91071597dc251245ddaa8f/crates/bevy_xpbd_3d/examples/basic_dynamic_character.rs
/// [`basic_kinematic_character`]: https://github.com/Jondolf/bevy_xpbd/blob/42fb8b21c756a7f4dd91071597dc251245ddaa8f/crates/bevy_xpbd_3d/examples/basic_kinematic_character.rs
///
/// ## Mass properties
///
/// The mass properties of a rigid body consist of its [`Mass`], [`Inertia`]
/// and local [`CenterOfMass`]. They control how forces and torques impact a rigid body
/// and how it affects other bodies.
///
/// You should always give dynamic rigid bodies mass properties so that forces
/// are applied to them correctly. The easiest way to do that is to simply [add a collider](Collider):
///
/// ```
/// # use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((RigidBody::Dynamic, Collider::ball(0.5)));
/// # }
/// ```
///
/// This will automatically compute the [collider's mass properties](ColliderMassProperties)
/// and add them to the body's own mass properties.
///
/// By default, each collider has a density of `1.0`. This can be configured with
/// the [`ColliderDensity`] component:
///
/// ```
/// # use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::ball(0.5),
///     ColliderDensity(2.5),
/// ));
/// # }
/// ```
///
/// If you don't want to add a collider, you can instead add a [`MassPropertiesBundle`]
/// with the mass properties computed from a collider shape using the
/// [`MassPropertiesBundle::new_computed`](MassPropertiesBundle::new_computed) method.
///
/// ```
/// # use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
/// #
/// # fn setup(mut commands: Commands) {
/// // This is equivalent to the earlier approach, but no collider will be added
/// commands.spawn((
///     RigidBody::Dynamic,
///     MassPropertiesBundle::new_computed(&Collider::ball(0.5), 2.5),
/// ));
/// # }
/// ```
///
/// You can also specify the exact values of the mass properties by adding the components manually.
/// To avoid the collider mass properties from being added to the body's own mass properties,
/// you can simply set the collider's density to zero.
///
/// ```
/// # use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
/// #
/// # fn setup(mut commands: Commands) {
/// // Create a rigid body with a mass of 5.0 and a collider with no mass
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::ball(0.5),
///     ColliderDensity(0.0),
///     Mass(5.0),
///     // ...the rest of the mass properties
/// ));
/// # }
///
/// ```
///
/// ## See more
///
/// - [Colliders](Collider)
/// - [Gravity] and [gravity scale](GravityScale)
/// - [Linear](LinearDamping) and [angular](AngularDamping) velocity damping
/// - [Lock translational and rotational axes](LockedAxes)
/// - [Dominance]
/// - [Automatic deactivation with sleeping](Sleeping)
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub enum RigidBody {
    /// Dynamic bodies are bodies that are affected by forces, velocity and collisions.
    #[default]
    Dynamic,

    /// Static bodies are not affected by any forces, collisions or velocity, and they act as if they have an infinite mass and moment of inertia.
    /// The only way to move a static body is to manually change its position.
    ///
    /// Collisions with static bodies will affect dynamic bodies, but not other static bodies or kinematic bodies.
    ///
    /// Static bodies are typically used for things like the ground, walls and any other objects that you don't want to move.
    Static,

    /// Kinematic bodies are bodies that are not affected by any external forces or collisions.
    /// They will realistically affect colliding dynamic bodies, but not other kinematic bodies.
    ///
    /// Unlike static bodies, kinematic bodies can have velocity.
    /// The engine doesn't modify the values of a kinematic body's components,
    /// so you have full control of them.
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

/// Indicates that a [rigid body](RigidBody) is not simulated by the physics engine until woken up again.
/// This is done to improve performance and to help prevent small jitter that is typically present in collisions.
///
/// Bodies are marked as sleeping when their linear and angular velocity is below the [`SleepingThreshold`] for a time
/// indicated by [`DeactivationTime`]. A sleeping body is woken up when an active body interacts with it through
/// collisions or other constraints, or when gravity changes, or when the body's
/// position, rotation, velocity, or external forces are modified.
///
/// Sleeping can be disabled for specific entities with the [`SleepingDisabled`] component,
/// or for all entities by setting the [`SleepingThreshold`] to a negative value.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Sleeping;

/// How long the velocity of the body has been below the [`SleepingThreshold`],
/// i.e. how long the body has been able to sleep.
///
/// See [`Sleeping`] for further information.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct TimeSleeping(pub Scalar);

/// Indicates that the body can not be deactivated by the physics engine. See [`Sleeping`] for information about sleeping.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct SleepingDisabled;

/// The global position of a [rigid body](RigidBody) or a [collider](Collider).
///
/// ## Relation to `Transform` and `GlobalTransform`
///
/// [`Position`] is used for physics internally and kept in sync with `Transform`
/// by the [`SyncPlugin`]. It rarely needs to be used directly in your own code, as `Transform` can still
/// be used for almost everything. Using [`Position`] should only be required for managing positions
/// in systems running in the [`SubstepSchedule`]. However, if you prefer, you can also use [`Position`]
/// for everything.
///
/// The reasons why the engine uses a separate [`Position`] component can be found
/// [here](crate#why-are-there-separate-position-and-rotation-components).
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "         Position::from_xy(0.0, 20.0),")]
#[cfg_attr(feature = "3d", doc = "         Position::from_xyz(0.0, 2.0, 0.0),")]
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Position(pub Vector);

impl Position {
    /// Creates a [`Position`] component with the given global `position`.
    pub fn new(position: Vector) -> Self {
        Self(position)
    }

    /// Creates a [`Position`] component with the global position `(x, y)`.
    #[cfg(feature = "2d")]
    pub fn from_xy(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a [`Position`] component with the global position `(x, y, z)`.
    #[cfg(feature = "3d")]
    pub fn from_xyz(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

impl From<GlobalTransform> for Position {
    #[cfg(feature = "2d")]
    fn from(value: GlobalTransform) -> Self {
        Self::from_xy(
            value.translation().adjust_precision().x,
            value.translation().adjust_precision().y,
        )
    }

    #[cfg(feature = "3d")]
    fn from(value: GlobalTransform) -> Self {
        Self::from_xyz(
            value.translation().adjust_precision().x,
            value.translation().adjust_precision().y,
            value.translation().adjust_precision().z,
        )
    }
}

impl From<&GlobalTransform> for Position {
    #[cfg(feature = "2d")]
    fn from(value: &GlobalTransform) -> Self {
        Self::from_xy(
            value.translation().adjust_precision().x,
            value.translation().adjust_precision().y,
        )
    }

    #[cfg(feature = "3d")]
    fn from(value: &GlobalTransform) -> Self {
        Self::from_xyz(
            value.translation().adjust_precision().x,
            value.translation().adjust_precision().y,
            value.translation().adjust_precision().z,
        )
    }
}

/// The position of a [rigid body](RigidBody) at the start of a substep.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct PreviousPosition(pub Vector);

/// Translation accumulated during a sub-step.
///
/// When updating position during integration or constraint solving, the required translation
/// is added to [`AccumulatedTranslation`], instead of [`Position`]. This improves numerical stability
/// of the simulation, especially for bodies far away from world origin.
///
/// After each substep, actual [`Position`] is updated during [`SubstepSet::ApplyTranslation`].
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct AccumulatedTranslation(pub Vector);

/// The linear velocity of a [rigid body](RigidBody).
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn accelerate_bodies(mut query: Query<&mut LinearVelocity>) {
///     for mut linear_velocity in query.iter_mut() {
///         linear_velocity.x += 0.05;
///     }
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct LinearVelocity(pub Vector);

impl LinearVelocity {
    /// Zero linear velocity.
    pub const ZERO: LinearVelocity = LinearVelocity(Vector::ZERO);
}

/// The linear velocity of a [rigid body](RigidBody) before the velocity solve is performed.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub(crate) struct PreSolveLinearVelocity(pub Vector);

/// The angular velocity of a [rigid body](RigidBody) in radians per second.
/// Positive values will result in counterclockwise rotation.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// use bevy_xpbd_2d::prelude::*;
///
/// fn increase_angular_velocities(mut query: Query<&mut AngularVelocity>) {
///     for mut angular_velocity in query.iter_mut() {
///         angular_velocity.0 += 0.05;
///     }
/// }
/// ```
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct AngularVelocity(pub Scalar);

/// The angular velocity of a [rigid body](RigidBody) as a rotation axis
/// multiplied by the angular speed in radians per second.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// use bevy_xpbd_3d::prelude::*;
///
/// fn increase_angular_velocities(mut query: Query<&mut AngularVelocity>) {
///     for mut angular_velocity in query.iter_mut() {
///         angular_velocity.z += 0.05;
///     }
/// }
/// ```
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
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

/// The angular velocity of a [rigid body](RigidBody) in radians per second, before
/// the velocity solve is performed. Positive values will result in counterclockwise rotation.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[reflect(Component)]
pub(crate) struct PreSolveAngularVelocity(pub Scalar);

/// The angular velocity of a [rigid body](RigidBody) as a rotation axis
/// multiplied by the angular speed in radians per second, before the velocity solve is performed.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[reflect(Component)]
pub(crate) struct PreSolveAngularVelocity(pub Vector);

/// Controls how [gravity](Gravity) affects a specific [rigid body](RigidBody).
///
/// A gravity scale of `0.0` will disable gravity, while `2.0` will double the gravity.
/// Using a negative value will flip the direction of the gravity.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// // Spawn a body with 1.5 times the normal gravity
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         GravityScale(1.5),
///     ));
/// }
/// ```
#[derive(
    Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deref, DerefMut, From,
)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct GravityScale(pub Scalar);

/// Determines how coefficients are combined for [`Restitution`] and [`Friction`].
/// The default is `Average`.
///
/// When combine rules clash with each other, the following priority order is used:
/// `Max > Multiply > Min > Average`.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
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

/// A component for the [coefficient of restitution](https://en.wikipedia.org/wiki/Coefficient_of_restitution).
/// This controls how bouncy a [rigid body](RigidBody) is.
///
/// The coefficient is between 0 and 1, where 0 corresponds to a **perfectly inelastic collision**, and 1 corresponds
/// to a **perfectly elastic collision** that preserves all kinetic energy. The default coefficient is 0.3, and it currently
/// can not be configured at a global level.
///
/// When two bodies collide, their restitution coefficients are combined using the specified [`CoefficientCombine`] rule.
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
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
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
#[doc(alias = "Bounciness")]
#[doc(alias = "Elasticity")]
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Restitution {
    /// The [coefficient of restitution](https://en.wikipedia.org/wiki/Coefficient_of_restitution).
    ///
    /// This should be between 0 and 1, where 0 corresponds to a **perfectly inelastic collision**, and 1 corresponds
    /// to a **perfectly elastic collision** that preserves all kinetic energy. The default value is 0.3.
    pub coefficient: Scalar,
    /// The coefficient combine rule used when two bodies collide.
    pub combine_rule: CoefficientCombine,
}

impl Restitution {
    /// A restitution coefficient of 0.0 and a combine rule of [`CoefficientCombine::Average`].
    ///
    /// This is equivalent to [`Restitution::PERFECTLY_INELASTIC`](#associatedconstant.PERFECTLY_INELASTIC).
    pub const ZERO: Self = Self {
        coefficient: 0.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// A restitution coefficient of 0.0, which corresponds to a perfectly inelastic collision.
    ///
    /// Uses [`CoefficientCombine::Average`].
    pub const PERFECTLY_INELASTIC: Self = Self {
        coefficient: 0.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// A restitution coefficient of 1.0, which corresponds to a perfectly elastic collision.
    ///
    /// Uses [`CoefficientCombine::Average`].
    pub const PERFECTLY_ELASTIC: Self = Self {
        coefficient: 1.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// Creates a new [`Restitution`] component with the given restitution coefficient.
    pub fn new(coefficient: Scalar) -> Self {
        Self {
            coefficient: coefficient.clamp(0.0, 1.0),
            combine_rule: CoefficientCombine::Average,
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
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
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
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Friction {
    /// Coefficient of dynamic friction.
    pub dynamic_coefficient: Scalar,
    /// Coefficient of static friction.
    pub static_coefficient: Scalar,
    /// The coefficient combine rule used when two bodies collide.
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

/// Automatically slows down a dynamic [rigid body](RigidBody), decreasing its
/// [linear velocity](LinearVelocity) each frame. This can be used to simulate air resistance.
///
/// The default linear damping coefficient is `0.0`, which corresponds to no damping.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         LinearDamping(0.8),
///     ));
/// }
/// ```
#[derive(
    Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deref, DerefMut, From,
)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct LinearDamping(pub Scalar);

/// Automatically slows down a dynamic [rigid body](RigidBody), decreasing its
/// [angular velocity](AngularVelocity) each frame. This can be used to simulate air resistance.
///
/// The default angular damping coefficient is `0.0`, which corresponds to no damping.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         AngularDamping(1.6),
///     ));
/// }
/// ```
#[derive(
    Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deref, DerefMut, From,
)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct AngularDamping(pub Scalar);

/// **Dominance** allows [dynamic rigid bodies](RigidBody::Dynamic) to dominate
/// each other during physical interactions.
/// 
/// The body with a higher dominance acts as if it had infinite mass, and will be unaffected during
/// collisions and other interactions, while the other body will be affected normally.
/// 
/// The dominance must be between `-127` and `127`, and the default value is `0`.
/// Note that static and kinematic bodies will always have a higher dominance value
/// than dynamic bodies regardless of the value of this component.
/// 
/// ## Example
/// 
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// // Player dominates all dynamic bodies with a dominance lower than 5
/// fn spawn_player(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(1.0, 0.4),
///         Dominance(5),
///     ));
/// }
/// ```
#[rustfmt::skip]
#[derive(Component, Reflect, Debug, Clone, Copy, Default, Deref, DerefMut, From, PartialEq, PartialOrd, Eq, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Dominance(pub i8);

#[cfg(test)]
mod tests {
    use crate::prelude::*;
    use approx::assert_relative_eq;

    #[test]
    fn restitution_clamping_works() {
        assert_eq!(Restitution::new(-2.0).coefficient, 0.0);
        assert_eq!(Restitution::new(0.6).coefficient, 0.6);
        assert_eq!(Restitution::new(3.0).coefficient, 1.0);
    }

    #[test]
    fn coefficient_combine_works() {
        let r1 = Restitution::new(0.3).with_combine_rule(CoefficientCombine::Average);

        // (0.3 + 0.7) / 2.0 == 0.5
        let average_result =
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Average));
        let average_expected = Restitution::new(0.5).with_combine_rule(CoefficientCombine::Average);
        assert_relative_eq!(
            average_result.coefficient,
            average_expected.coefficient,
            epsilon = 0.0001
        );
        assert_eq!(average_result.combine_rule, average_expected.combine_rule);

        // 0.3.min(0.7) == 0.3
        assert_eq!(
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Min)),
            Restitution::new(0.3).with_combine_rule(CoefficientCombine::Min)
        );

        // 0.3 * 0.7 == 0.21
        let multiply_result =
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Multiply));
        let multiply_expected =
            Restitution::new(0.21).with_combine_rule(CoefficientCombine::Multiply);
        assert_relative_eq!(
            multiply_result.coefficient,
            multiply_expected.coefficient,
            epsilon = 0.0001
        );
        assert_eq!(multiply_result.combine_rule, multiply_expected.combine_rule);

        // 0.3.max(0.7) == 0.7
        assert_eq!(
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Max)),
            Restitution::new(0.7).with_combine_rule(CoefficientCombine::Max)
        );
    }
}
