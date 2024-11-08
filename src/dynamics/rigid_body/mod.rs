//! Common components and bundles for rigid bodies.

pub mod mass_properties;

// Components
mod forces;
mod locked_axes;
mod physics_material;
mod world_query;

pub use forces::{ExternalAngularImpulse, ExternalForce, ExternalImpulse, ExternalTorque};
pub use locked_axes::LockedAxes;
pub use physics_material::{
    CoefficientCombine, DefaultFriction, DefaultRestitution, Friction, Restitution,
};
pub use world_query::*;

#[cfg(feature = "2d")]
pub(crate) use forces::FloatZero;
pub(crate) use forces::Torque;

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
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic rigid body and specify its position (optional)
///     commands.spawn((
///         RigidBody::Dynamic,
///         Transform::from_xyz(0.0, 3.0, 0.0),
///     ));
/// }
/// ```
///
/// Avian will automatically add any missing components, like the following:
///
/// - [`Position`]
/// - [`Rotation`]
/// - [`LinearVelocity`]
/// - [`AngularVelocity`]
/// - [`ExternalForce`]
/// - [`ExternalTorque`]
/// - [`ExternalImpulse`]
/// - [`ExternalAngularImpulse`]
/// - [`Mass`]
/// - [`AngularInertia`]
/// - [`CenterOfMass`]
///
/// You can change any of these during initialization and runtime in order to alter the behaviour of the body.
///
/// By default, rigid bodies will get a mass based on the attached colliders and their densities.
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
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
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
/// Avian does not have a built-in character controller, so if you need one,
/// you will need to implement it yourself or use a third party option.
/// You can take a look at the [`basic_dynamic_character`] and [`basic_kinematic_character`]
/// examples for a simple implementation.
///
/// [`basic_dynamic_character`]: https://github.com/Jondolf/avian/blob/42fb8b21c756a7f4dd91071597dc251245ddaa8f/crates/avian3d/examples/basic_dynamic_character.rs
/// [`basic_kinematic_character`]: https://github.com/Jondolf/avian/blob/42fb8b21c756a7f4dd91071597dc251245ddaa8f/crates/avian3d/examples/basic_kinematic_character.rs
///
/// ## Mass properties
///
/// The mass properties of a rigid body consist of its [`Mass`], [`AngularInertia`]
/// and local [`CenterOfMass`]. They control how forces and torques impact a rigid body
/// and how it affects other bodies.
///
/// You should always give dynamic rigid bodies mass properties so that forces
/// are applied to them correctly. The easiest way to do that is to simply [add a collider](Collider):
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
#[cfg_attr(
    feature = "2d",
    doc = "commands.spawn((RigidBody::Dynamic, Collider::circle(0.5)));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "commands.spawn((RigidBody::Dynamic, Collider::sphere(0.5)));"
)]
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
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "    Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "    Collider::sphere(0.5),")]
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
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // This is equivalent to the earlier approach, but no collider will be added
/// commands.spawn((
///     RigidBody::Dynamic,
#[cfg_attr(
    feature = "2d",
    doc = "    MassPropertiesBundle::new_computed(&Collider::circle(0.5), 2.5),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    MassPropertiesBundle::new_computed(&Collider::sphere(0.5), 2.5),"
)]
/// ));
/// # }
/// ```
///
/// You can also specify the exact values of the mass properties by adding the components manually.
/// To avoid the collider mass properties from being added to the body's own mass properties,
/// you can simply set the collider's density to zero.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Create a rigid body with a mass of 5.0 and a collider with no mass
/// commands.spawn((
///     RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "    Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "    Collider::sphere(0.5),")]
///     ColliderDensity(0.0),
///     Mass::new(5.0),
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
/// - [Friction] and [restitution](Restitution)
/// - [Lock translational and rotational axes](LockedAxes)
/// - [Dominance]
/// - [Continuous Collision Detection](dynamics::ccd)
/// - [Automatic deactivation with sleeping](Sleeping)
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[require(
    LinearVelocity,
    AngularVelocity,
    // TODO: Make these force components optional.
    ExternalForce,
    ExternalTorque,
    ExternalImpulse,
    ExternalAngularImpulse,
    Mass,
    AngularInertia,
    CenterOfMass,
    // Currently required for solver internals.
    // Some of these might be removed in the future.
    AccumulatedTranslation,
    PreSolveAccumulatedTranslation,
    PreSolveLinearVelocity,
    PreSolveAngularVelocity,
    PreSolveRotation,
    PreviousRotation,
)]
#[cfg_attr(feature = "3d", require(GlobalAngularInertia))]
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
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Sleeping;

/// How long the velocity of the body has been below the [`SleepingThreshold`],
/// i.e. how long the body has been able to sleep.
///
/// See [`Sleeping`] for further information.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct TimeSleeping(pub Scalar);

/// Indicates that the body can not be deactivated by the physics engine. See [`Sleeping`] for information about sleeping.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct SleepingDisabled;

/// Translation accumulated during the physics frame.
///
/// When updating position during integration or constraint solving, the required translation
/// is added to [`AccumulatedTranslation`], instead of [`Position`]. This improves numerical stability
/// of the simulation, especially for bodies far away from world origin.
///
/// At the end of each physics frame, the actual [`Position`] is updated in [`SolverSet::ApplyTranslation`].
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AccumulatedTranslation(pub Vector);

/// The linear velocity of a [rigid body](RigidBody).
///
/// ## Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn accelerate_bodies(mut query: Query<&mut LinearVelocity>) {
///     for mut linear_velocity in query.iter_mut() {
///         linear_velocity.x += 0.05;
///     }
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
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
/// use avian2d::prelude::*;
/// use bevy::prelude::*;
///
/// fn increase_angular_velocities(mut query: Query<&mut AngularVelocity>) {
///     for mut angular_velocity in query.iter_mut() {
///         angular_velocity.0 += 0.05;
///     }
/// }
/// ```
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Deref, DerefMut, Component, Debug, Default, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AngularVelocity(pub Scalar);

/// The angular velocity of a [rigid body](RigidBody) as a rotation axis
/// multiplied by the angular speed in radians per second.
///
/// ## Example
///
/// ```
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
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
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Spawn a body with 1.5 times the normal gravity
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         GravityScale(1.5),
///     ));
/// }
/// ```
#[derive(Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Deref, DerefMut, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct GravityScale(pub Scalar);

impl Default for GravityScale {
    fn default() -> Self {
        Self(1.0)
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
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct LinearDamping(pub Scalar);

/// Automatically slows down a dynamic [rigid body](RigidBody), decreasing its
/// [angular velocity](AngularVelocity) each frame. This can be used to simulate air resistance.
///
/// The default angular damping coefficient is `0.0`, which corresponds to no damping.
///
/// ## Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
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
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Player dominates all dynamic bodies with a dominance lower than 5
/// fn spawn_player(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(0.4, 1.0),
///         Dominance(5),
///     ));
/// }
/// ```
#[rustfmt::skip]
#[derive(Component, Reflect, Debug, Clone, Copy, Default, Deref, DerefMut, From, PartialEq, PartialOrd, Eq, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Dominance(pub i8);
