//! External forces, torques, impulses, and acceleration for dynamic [rigid bodies](RigidBody).
//!
//! # Overview
//!
//! In addition to [`Gravity`], it is possible to apply your own forces, impulses, and acceleration
//! to dynamic rigid bodies to simulate various effects such as force fields, motors, and thrusters.
//! They have the following relationships:
//!
//! | Type             | Formula      | Relation to Velocity | Unit        |
//! | ---------------- | ------------ | -------------------- | ----------- |
//! | **Force**        | `F = m * a`  | `Δa = F / m`         | kg⋅m/s² (N) |
//! | **Impulse**      | `J = F * Δt` | `Δv = J / m`         | kg⋅m/s (N⋅s) |
//! | **Acceleration** | `a = F / m`  | `Δv = a * Δt`        | m/s²        |
//!
//! A force applies an acceleration to a body, which in turn modifies its velocity over time,
//! while an impulse applies an immediate change in velocity. Both forces and impulses consider [mass properties],
//! while acceleration by itself is independent of mass.
//!
//! The rotational equivalents are torques, angular impulses, and angular acceleration, which work similarly.
//!
//! [mass properties]: crate::dynamics::rigid_body::mass_properties
//!
//! ## Constant Forces
//!
//! Constant forces and torques that persist across time steps can be applied using the following components:
//!
//! - [`ConstantForce`]: Applies a constant force in world space.
//! - [`ConstantTorque`]: Applies a constant torque in world space.
//! - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
//! - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
//!
//! They also have local space equivalents:
//!
//! - [`ConstantLocalForce`]: Applies a constant force in local space.
//! - [`ConstantLocalTorque`]: Applies a constant torque in local space.
//! - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
//! - [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space.
//!
//! These components are useful for simulating continuously applied forces that are expected
//! to remain the same across time steps, such as per-body gravity or force fields.
//! They are gathered in the [`AccumulatedWorldForces`] and [`AccumulatedLocalForces`]
//! components, which are applied at each [substep](SubstepCount) of the physics simulation.
//!
//! You can use constant forces by adding the components to your entities:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! commands.spawn((
//!     RigidBody::Dynamic,
//!     Collider::capsule(0.5, 1.0),
//!     // Apply a constant force of 10 N in the positive Y direction.
#![cfg_attr(feature = "2d", doc = "    ConstantForce::new(0.0, 10.0),")]
#![cfg_attr(feature = "3d", doc = "    ConstantForce::new(0.0, 10.0, 0.0),")]
//! ));
//! # }
//! ```
//!
//! The forces are only constant in the sense that they persist across time steps.
//! They can still be modified in systems like normal.
//!
//! ## One-Time Forces and Impulses
//!
//! It is common to apply many individual forces and impulses to dynamic rigid bodies,
//! and to clear them afterwards. This can be done using the [`ForceHelper`] system parameter.
//!
//! To use the [`ForceHelper`], you can add it to your system, and use the [`entity`](ForceHelper::entity)
//! method to get access to [`EntityForces`] for applying forces, impulses, and acceleration to that entity.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
#![cfg_attr(feature = "serialize", doc = "# use bevy::prelude::*;")]
//! #
//! # #[cfg(feature = "f32")]
//! fn apply_forces(query: Query<Entity, With<RigidBody>>, forces: ForceHelper) {
//!     for entity in &mut query {
//!         // Apply a force of 10 N in the positive Y direction to `entity`.
#![cfg_attr(
    feature = "2d",
    doc = "        forces.entity(entity).apply_force(Vec2::new(0.0, 10.0));"
)]
#![cfg_attr(
    feature = "3d",
    doc = "        forces.entity(entity).apply_force(Vec3::new(0.0, 10.0, 0.0));"
)]
//!     }
//! }
//! ```
//!
//! The force is applied continuously during the physics step, and cleared after the step is complete.
//! The [`ForceHelper`] manages everything for you, so there is no need to add or remove any components manually.
//!
//! The [`ForceHelper`] can also apply forces and impulses at a specific point in the world.
//! If the point is not aligned with the [`GlobalCenterOfMass`], it will also apply a torque to the body.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::{math::Vector, prelude::*};")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::{math::Vector, prelude::*};")]
#![cfg_attr(feature = "serialize", doc = "# use bevy::prelude::*;")]
//! #
//! # fn apply_impulses(query: Query<Entity, With<RigidBody>>, forces: ForceHelper) {
//! #     for entity in &mut query {
//! #         let force = Vector::default();
//! #         let point = Vector::default();
//! // Apply an impulse at a specific point in the world.
//! // Unlike forces, impulses are applied immediately to the velocity,
//! forces.entity(entity).apply_linear_impulse_at_point(force, point);
//! #     }
//! # }
//! ```
//!
//! As an example, you could implement radial gravity that pulls rigid bodies towards the world origin
//! with a system like the following:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # #[cfg(feature = "f32")]
//! fn radial_gravity(query: Query<(Entity, &GlobalTransform), With<RigidBody>>, forces: ForceHelper) {
//!     for (entity, global_transform) in &mut query {
//!         // Compute the direction towards the center of the world.
//!         let direction = -global_transform.translation().normalize_or_zero();
//!         // Apply a linear acceleration of 9.81 m/s² towards the center of the world.
#![cfg_attr(
    feature = "2d",
    doc = "        forces.entity(entity).apply_linear_acceleration(direction.truncate() * 9.81);"
)]
#![cfg_attr(
    feature = "3d",
    doc = "        forces.entity(entity).apply_linear_acceleration(direction * 9.81);"
)]
//!     }
//! }
//! ```
//!
//! # Applying Forces vs. Modifying Velocity
//!
//! It is possible to achieve similar effects by directly modifying the [`LinearVelocity`]
//! or [`AngularVelocity`] components instead of using the force APIs. For example, you could
//! implement gravity by simply modifying the velocity of the rigid bodies in a system:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! // In `FixedUpdate`
//! # #[cfg(feature = "f32")]
//! fn gravity(mut bodies: Query<&mut LinearVelocity>, time: Res<Time>) {
//!     for mut velocity in &mut bodies {
//!         // Apply a constant acceleration of 9.81 m/s² in the negative Y direction.
//!         velocity.y -= 9.81 * time.delta_secs();
//!     }
//! }
//! ```
//!
//! However, using the force APIs has several advantages:
//!
//! - Forces and accelerations are spread out over several [substeps](SubstepCount),
//!   which provides higher simulation fidelity than modifying velocity once per time step.
//! - Forces and impulses consider mass properties and delta time for you, simplifying code.
//! - The force APIs make it straightforward to apply forces at specific points in the world.
//!
//! Modifying velocity directly can still be useful and simpler in some cases,
//! and the difference in simulation fidelity tends to be small for most applications.
//! Notably, it is currently not possible to apply forces to kinematic bodies, as they have infinite mass.
//!
//! Still, for convenience and best results, it is generally recommended to use the force APIs
//! for most cases where you want to apply forces, impulses, or acceleration to dynamic rigid bodies.

mod plugin;
mod system_param;

pub use plugin::{ForcePlugin, ForceSet};
pub use system_param::{EntityForces, ForceHelper};

use crate::prelude::*;
use bevy::prelude::*;

#[cfg(feature = "2d")]
pub(crate) trait FloatZero {
    const ZERO: Self;
}

#[cfg(feature = "2d")]
impl FloatZero for Scalar {
    const ZERO: Self = 0.0;
}

/// A component for applying a constant force to a dynamic rigid body in world space.
/// The unit is typically N or kg⋅m/s².
///
/// The force persists across time steps, and is accumulated with other forces
/// in the [`AccumulatedWorldForces`] component.
///
/// This can be useful for simulating constant forces like gravity, force fields, wind,
/// or other environmental effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant force of 10 N in the positive Y direction.
#[cfg_attr(feature = "2d", doc = "    ConstantForce::new(0.0, 10.0),")]
#[cfg_attr(feature = "3d", doc = "    ConstantForce::new(0.0, 10.0, 0.0),")]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`ForceHelper`]: A system parameter for applying forces, impulses, and acceleration to entities.
/// - [`ConstantLocalForce`]: Applies a constant force in local space.
/// - [`ConstantTorque`]: Applies a constant torque in world space.
/// - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
/// - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
/// - [`AccumulatedWorldForces`]: Stores the accumulated world forces and torques.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedWorldForces)]
pub struct ConstantForce(pub Vector);

impl ConstantForce {
    /// Creates a new [`ConstantForce`] with the given `x` and `y` components.
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`ConstantForce`] with the given `x`, `y`, and `z` components.
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant torque to a dynamic rigid body in world space.
/// The unit is typically N⋅m or kg⋅m²/s².
///
/// The torque persists across time steps, and is accumulated with other torques
/// in the [`AccumulatedWorldForces`] component.
///
/// This can be useful for simulating constant torques like the torque from a motor,
/// or other rotational effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant torque of 5 N⋅m in the positive Z direction.
#[cfg_attr(feature = "2d", doc = "    ConstantTorque(5.0),")]
#[cfg_attr(feature = "3d", doc = "    ConstantTorque::new(0.0, 0.0, 5.0),")]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`ForceHelper`]: A system parameter for applying forces, impulses, and acceleration to entities.
/// - [`ConstantLocalTorque`]: Applies a constant torque in local space.
/// - [`ConstantForce`]: Applies a constant force in world space.
/// - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
/// - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
/// - [`AccumulatedWorldForces`]: Stores the accumulated world forces and torques.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedWorldForces)]
pub struct ConstantTorque(pub AngularVector);

#[cfg(feature = "3d")]
impl ConstantTorque {
    /// Creates a new [`ConstantTorque`] with the given `x`, `y`, and `z` components.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant force to a dynamic rigid body in local space.
/// The unit is typically N or kg⋅m/s².
///
/// The force persists across time steps, and is accumulated with other forces
/// in the [`AccumulatedLocalForces`] component.
///
/// This can be useful for simulating constant forces like rocket thrust,
/// or other local force effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant force of 10 N in the positive Y direction in local space.
#[cfg_attr(feature = "2d", doc = "    ConstantLocalForce::new(0.0, 10.0),")]
#[cfg_attr(feature = "3d", doc = "    ConstantLocalForce::new(0.0, 10.0, 0.0),")]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`ForceHelper`]: A system parameter for applying forces, impulses, and acceleration to entities.
/// - [`ConstantForce`]: Applies a constant force in world space.
/// - [`ConstantLocalTorque`]: Applies a constant torque in local space.
/// - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
/// - [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space.
/// /// - [`AccumulatedLocalForces`]: Stores the accumulated local forces and torques.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedLocalForces)]
pub struct ConstantLocalForce(pub Vector);

impl ConstantLocalForce {
    /// Creates a new [`ConstantLocalForce`] with the given `x` and `y` components.
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`ConstantLocalForce`] with the given `x`, `y`, and `z` components.
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant torque to a dynamic rigid body in local space.
/// The unit is typically N⋅m or kg⋅m²/s²
///
/// The torque persists across time steps, and is accumulated with other torques
/// in the [`AccumulatedLocalForces`] component.
///
/// This can be useful for simulating constant torques like torque from a motor,
/// or other rotational effects in local space.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant torque of 5 N⋅m in the positive Z direction in local space.
#[cfg_attr(feature = "2d", doc = "    ConstantLocalTorque(5.0),")]
#[cfg_attr(feature = "3d", doc = "    ConstantLocalTorque::new(0.0, 0.0, 5.0),")]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`ForceHelper`]: A system parameter for applying forces, impulses, and acceleration to entities.
/// - [`ConstantTorque`]: Applies a constant torque in world space.
/// - [`ConstantLocalForce`]: Applies a constant force in local space.
/// - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
/// - [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedLocalForces)]
pub struct ConstantLocalTorque(pub AngularVector);

#[cfg(feature = "3d")]
impl ConstantLocalTorque {
    /// Creates a new [`ConstantLocalTorque`] with the given `x`, `y`, and `z` components.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant linear acceleration to a dynamic rigid body in world space.
/// The unit is typically m/s².
///
/// The acceleration persists across time steps, and is accumulated with other accelerations.
///
/// This can be useful for simulating constant accelerations like per-body gravity.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant linear acceleration of 9.81 m/s² in the negative Y direction.
///     // This is equivalent to using the `Gravity` resource, but only for this entity.
#[cfg_attr(
    feature = "2d",
    doc = "    ConstantLinearAcceleration::new(0.0, -9.81),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    ConstantLinearAcceleration::new(0.0, -9.81, 0.0),"
)]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`ForceHelper`]: A system parameter for applying forces, impulses, and acceleration to entities.
/// - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
/// - [`ConstantForce`]: Applies a constant force in world space.
/// - [`ConstantTorque`]: Applies a constant torque in world space.
/// - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantLinearAcceleration(pub Vector);

impl ConstantLinearAcceleration {
    /// Creates a new [`ConstantLinearAcceleration`] with the given `x` and `y` components.
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`ConstantLinearAcceleration`] with the given `x`, `y`, and `z` components.
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant angular acceleration to a dynamic rigid body in world space.
/// The unit is typically rad/s².
///
/// The acceleration persists across time steps, and is accumulated with other accelerations.
///
/// This can be useful for simulating constant angular accelerations like rotation from a motor,
/// or other rotational effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant angular acceleration of 1.0 rad/s² in the positive Z direction.
#[cfg_attr(feature = "2d", doc = "    ConstantAngularAcceleration(1.0),")]
#[cfg_attr(
    feature = "3d",
    doc = "    ConstantAngularAcceleration::new(0.0, 0.0, 1.0),"
)]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`ForceHelper`]: A system parameter for applying forces, impulses, and acceleration to entities.
/// - [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space.
/// - [`ConstantForce`]: Applies a constant force in world space.
/// - [`ConstantTorque`]: Applies a constant torque in world space.
/// - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantAngularAcceleration(pub AngularVector);

#[cfg(feature = "3d")]
impl ConstantAngularAcceleration {
    /// Creates a new [`ConstantAngularAcceleration`] with the given `x`, `y`, and `z` components.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant linear acceleration to a dynamic rigid body in local space.
/// The unit is typically m/s².
///
/// The acceleration persists across time steps, and is accumulated with other accelerations.
///
/// This can be useful for simulating constant linear accelerations like rocket thrust,
/// or other local acceleration effects.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant linear acceleration of 10.0 m/s² in the positive Y direction in local space.
#[cfg_attr(
    feature = "2d",
    doc = "    ConstantLocalLinearAcceleration::new(0.0, 10.0),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    ConstantLocalLinearAcceleration::new(0.0, 10.0, 0.0),"
)]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`ForceHelper`]: A system parameter for applying forces, impulses, and acceleration to entities.
/// - [`ConstantLinearAcceleration`]: Applies a constant linear acceleration in world space.
/// - [`ConstantLocalForce`]: Applies a constant force in local space.
/// - [`ConstantLocalTorque`]: Applies a constant torque in local space.
/// - [`ConstantLocalAngularAcceleration`]: Applies a constant angular acceleration in local space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantLocalLinearAcceleration(pub Vector);

impl ConstantLocalLinearAcceleration {
    /// Creates a new [`ConstantLocalLinearAcceleration`] with the given `x` and `y` components.
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`ConstantLocalLinearAcceleration`] with the given `x`, `y`, and `z` components.
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

/// A component for applying a constant angular acceleration to a dynamic rigid body in local space.
/// The unit is typically rad/s².
///
/// The acceleration persists across time steps, and is accumulated with other accelerations.
///
/// This can be useful for simulating constant angular accelerations like rotation from a motor,
/// or other rotational effects in local space.
///
/// See the [module-level documentation](self) for more general information about forces in Avian.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.0),
///     // Apply a constant angular acceleration of 1.0 rad/s² in the positive Z direction in local space.
#[cfg_attr(feature = "2d", doc = "    ConstantLocalAngularAcceleration(1.0),")]
#[cfg_attr(
    feature = "3d",
    doc = "    ConstantLocalAngularAcceleration::new(0.0, 0.0, 1.0),"
)]
/// ));
/// # }
/// ```
///
/// # Related Types
///
/// - [`ForceHelper`]: A system parameter for applying forces, impulses, and acceleration to entities.
/// - [`ConstantAngularAcceleration`]: Applies a constant angular acceleration in world space.
/// - [`ConstantLocalForce`]: Applies a constant force in local space.
/// - [`ConstantLocalTorque`]: Applies a constant torque in local space.
/// - [`ConstantLocalLinearAcceleration`]: Applies a constant linear acceleration in local space.
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct ConstantLocalAngularAcceleration(pub AngularVector);

#[cfg(feature = "3d")]
impl ConstantLocalAngularAcceleration {
    /// Creates a new [`ConstantLocalAngularAcceleration`] with the given `x`, `y`, and `z` components.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
}

// TODO: Should accumulated forces and accelerations be `SparseSet` components?

/// A component with the user-applied world forces and torques
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero world forces have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedWorldForces {
    /// The accumulated force in world space.
    pub force: Vector,
    /// The accumulated torque in world space.
    pub torque: AngularVector,
}

/// A component with the user-applied local forces and torques
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero local forces have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedLocalForces {
    /// The accumulated force in local space.
    pub force: Vector,
    /// The accumulated torque in local space.
    pub torque: AngularVector,
}

/// A component with the user-applied local acceleration
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero local acceleration have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedLocalAcceleration {
    /// The accumulated linear acceleration in local space.
    pub linear: Vector,
    /// The accumulated angular acceleration in local space.
    #[cfg(feature = "2d")]
    pub angular: Scalar,
    /// The accumulated angular acceleration in local space.
    #[cfg(feature = "3d")]
    pub angular: Vector,
}
