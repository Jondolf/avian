use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;
use std::ops::{Deref, DerefMut};

pub(crate) trait FloatZero {
    const ZERO: Self;
}

impl FloatZero for Scalar {
    const ZERO: Self = 0.0;
}

/// An external force applied continuously to a dynamic [rigid body](RigidBody).
///
/// The force is stored in world space. If you want to apply forces in local space, you need to
/// use the body's rotation to [transform the local force into world space](#local-forces).
///
/// By default, the force persists across frames. You can clear the force manually using
/// [`clear`](Self::clear) or set `persistent` to false.
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
///     // Apply a force in world space every physics frame.
///     commands.spawn((RigidBody::Dynamic, ExternalForce::new(Vec3::Y)));
///
///     // Apply an initial force and automatically clear it every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple forces.
///     let mut force = ExternalForce::default();
///     force.apply_force(Vec3::Y).apply_force(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, force));
///
///     // Apply a force at a specific point relative to the given center of mass, also applying a torque.
///     // In this case, the torque would cause the body to rotate counterclockwise.
///     let mut force = ExternalForce::default();
///     force.apply_force_at_point(Vec3::Y, Vec3::X, Vec3::ZERO);
///     commands.spawn((RigidBody::Dynamic, force));
/// }
/// ```
///
/// ## Local forces
///
/// The force stored in `ExternalForce` is in world space.
///
/// If you want to apply a force in some direction relative to the body's frame of reference,
/// you need to rotate the force using the body's `Transform` or [`Rotation`].
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
///     // Spawn a rotated body and apply a force in the local up direction.
///     let transform = Transform::from_rotation(Quat::from_rotation_z(0.2));
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::new(transform.rotation * Vec3::Y),
///         transform,
///     ));
/// }
/// ```
///
/// Note that the actual force stored in `ExternalForce` is still in world space.
/// If you want to apply a force in the same local direction every frame,
/// consider setting `persistent` to `false` and running [`apply_force`](Self::apply_force) in a system.
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalForce2d {
    /// The total external force that will be applied.
    force: Vector2,
    /// True if the force persists across frames, and false if the force is automatically cleared every physics frame.
    ///
    /// If you clear the force manually, use the [`clear`](Self::clear) method. This will clear the force and
    /// the torque that is applied when the force is not applied at the center of mass.
    pub persistent: bool,
    /// The torque caused by forces applied at certain points using [`apply_force_at_point`](Self::apply_force_at_point).
    torque: Scalar,
}

impl Deref for ExternalForce2d {
    type Target = Vector2;

    fn deref(&self) -> &Self::Target {
        &self.force
    }
}

impl DerefMut for ExternalForce2d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.force
    }
}

impl Default for ExternalForce2d {
    fn default() -> Self {
        Self {
            force: Vector2::ZERO,
            persistent: true,
            torque: 0.0,
        }
    }
}

impl ExternalForce2d {
    /// Zero external force.
    pub const ZERO: Self = Self {
        force: Vector2::ZERO,
        persistent: true,
        torque: 0.0,
    };

    /// Creates a new [`ExternalForce`] component with a given world-space `force`.
    pub fn new(force: Vector2) -> Self {
        Self { force, ..default() }
    }

    /// Sets the world-space force. Note that the torque caused by any forces will not be reset.
    pub fn set_force(&mut self, force: Vector2) -> &mut Self {
        **self = force;
        self
    }

    /// Adds the given world-space `force` to the force that will be applied.
    pub fn apply_force(&mut self, force: Vector2) -> &mut Self {
        **self += force;
        self
    }

    /// Applies the given world-space `force` at a local `point`, which will also cause torque to be applied.
    pub fn apply_force_at_point(
        &mut self,
        force: Vector2,
        point: Vector2,
        center_of_mass: Vector2,
    ) -> &mut Self {
        **self += force;
        self.torque += (point - center_of_mass).perp_dot(force);
        self
    }

    /// Returns the force in world space.
    pub fn force(&self) -> Vector2 {
        self.force
    }

    /// Returns the torque caused by forces applied at certain points using
    /// [`apply_force_at_point`](Self::apply_force_at_point).
    pub fn torque(&self) -> Scalar {
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
        self.force = Vector2::ZERO;
        self.torque = 0.0;
    }
}

/// An external torque applied continuously to a dynamic [rigid body](RigidBody).
///
/// By default, the torque persists across frames. You can clear the torque manually using
/// [`clear`](Self::clear) or set `persistent` to false.
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
///     let mut torque = ExternalTorque::default();
///     torque.apply_torque(Vec3::Y).apply_torque(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, torque));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalTorque2d {
    /// The total external torque that will be applied.
    torque: Scalar,
    /// True if the torque persists across frames, and false if the torque is automatically cleared every physics frame.
    pub persistent: bool,
}

impl Deref for ExternalTorque2d {
    type Target = Scalar;

    fn deref(&self) -> &Self::Target {
        &self.torque
    }
}

impl DerefMut for ExternalTorque2d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.torque
    }
}

impl Default for ExternalTorque2d {
    fn default() -> Self {
        Self {
            torque: 0.0,
            persistent: true,
        }
    }
}

impl ExternalTorque2d {
    /// Zero external torque.
    pub const ZERO: Self = Self {
        torque: 0.0,
        persistent: true,
    };

    /// Creates a new [`ExternalTorque`] component with a given `torque`.
    pub fn new(torque: Scalar) -> Self {
        Self {
            torque,
            ..default()
        }
    }

    /// Sets the torque.
    pub fn set_torque(&mut self, torque: Scalar) -> &mut Self {
        **self = torque;
        self
    }

    /// Adds the given `torque` to the torque that will be applied.
    pub fn apply_torque(&mut self, torque: Scalar) -> &mut Self {
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
    pub fn torque(&self) -> Scalar {
        self.torque
    }

    /// Sets the torque to zero.
    pub fn clear(&mut self) {
        self.torque = 0.0;
    }
}

/// An external impulse applied instantly to a dynamic [rigid body](RigidBody).
///
/// The impulse is stored in world space. If you want to apply impulses in local space, you need to
/// use the body's rotation to [transform the local impulse into world space](#local-impulses).
///
/// By default, the impulse is cleared every frame. You can set `persistent` to true in order to persist
/// the impulse across frames.
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
///     // Apply an impulse in world space.
///     commands.spawn((RigidBody::Dynamic, ExternalImpulse::new(Vec3::Y)));
///
///     // Apply an impulse every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalImpulse::new(Vec3::Y).with_persistence(true),
///     ));
///
///     // Apply multiple impulses.
///     let mut impulse = ExternalImpulse::default();
///     impulse.apply_impulse(Vec3::Y).apply_impulse(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, impulse));
///
///     // Apply an impulse at a specific point relative to the given center of mass, also applying an angular impulse.
///     // In this case, the angular impulse would cause the body to rotate counterclockwise.
///     let mut impulse = ExternalImpulse::default();
///     impulse.apply_impulse_at_point(Vec3::Y, Vec3::X, Vec3::ZERO);
///     commands.spawn((RigidBody::Dynamic, impulse));
/// }
/// ```
///
/// ## Local impulses
///
/// The impulse stored in `ExternalImpulse` is in world space.
///
/// If you want to apply an impulse in some direction relative to the body's frame of reference,
/// you need to rotate the impulse using the body's `Transform` or [`Rotation`].
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
///     // Spawn a rotated body and apply an impulse in the local up direction.
///     let transform = Transform::from_rotation(Quat::from_rotation_z(0.2));
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalImpulse::new(transform.rotation * Vec3::Y),
///         transform,
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalImpulse2d {
    /// The total external impulse that will be applied.
    impulse: Vector2,
    /// True if the impulse persists across frames, and false if the impulse is automatically cleared every physics frame.
    ///
    /// If you clear the impulse manually, use the [`clear`](Self::clear) method. This will clear the impulse and
    /// the angular impulse that is applied when the impulse is not applied at the center of mass.
    pub persistent: bool,
    /// The angular impulse caused by impulses applied at certain points using [`apply_impulse_at_point`](Self::apply_impulse_at_point).
    angular_impulse: Scalar,
}

impl Deref for ExternalImpulse2d {
    type Target = Vector2;

    fn deref(&self) -> &Self::Target {
        &self.impulse
    }
}

impl DerefMut for ExternalImpulse2d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.impulse
    }
}

impl Default for ExternalImpulse2d {
    fn default() -> Self {
        Self {
            impulse: Vector2::ZERO,
            persistent: false,
            angular_impulse: 0.0,
        }
    }
}

impl ExternalImpulse2d {
    /// Zero external impulse.
    pub const ZERO: Self = Self {
        impulse: Vector2::ZERO,
        persistent: false,
        angular_impulse: 0.0,
    };

    /// Creates a new [`ExternalImpulse`] component with a given world-space `impulse`.
    pub fn new(impulse: Vector2) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    /// Sets the world-space impulse. Note that the angular impulse caused by any impulses will not be reset.
    pub fn set_impulse(&mut self, impulse: Vector2) -> &mut Self {
        **self = impulse;
        self
    }

    /// Adds the given world-space `impulse` to the impulse that will be applied.
    pub fn apply_impulse(&mut self, impulse: Vector2) -> &mut Self {
        **self += impulse;
        self
    }

    /// Applies the given world-space `impulse` at a local `point`, which will also cause an angular impulse to be applied.
    pub fn apply_impulse_at_point(
        &mut self,
        impulse: Vector2,
        point: Vector2,
        center_of_mass: Vector2,
    ) -> &mut Self {
        **self += impulse;
        self.angular_impulse += (point - center_of_mass).perp_dot(impulse);
        self
    }

    /// Returns the impulse in world space.
    pub fn impulse(&self) -> Vector2 {
        self.impulse
    }

    /// Returns the angular impulse caused by impulses applied at certain points using
    /// [`apply_impulse_at_point`](Self::apply_impulse_at_point).
    pub fn angular_impulse(&self) -> Scalar {
        self.angular_impulse
    }

    /// Determines if the impulse is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Sets the impulse and the potential angular impulse caused by the impulse to zero.
    pub fn clear(&mut self) {
        self.impulse = Vector2::ZERO;
        self.angular_impulse = 0.0;
    }
}

/// An external angular impulse applied instantly to a dynamic [rigid body](RigidBody).
///
/// By default, the angular impulse is cleared every frame. You can set `persistent` to true in order to persist
/// the impulse across frames.
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
///     // Apply an angular impulse.
///     commands.spawn((RigidBody::Dynamic, ExternalAngularImpulse::new(Vec3::Y)));
///
///     // Apply an angular impulse every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalAngularImpulse::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple angular impulses.
///     let mut angular_impulse = ExternalAngularImpulse::default();
///     angular_impulse.apply_impulse(Vec3::Y).apply_impulse(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, angular_impulse));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
#[doc(alias = "ExternalTorqueImpulse")]
pub struct ExternalAngularImpulse2d {
    /// The total external angular impulse that will be applied.
    impulse: Scalar,
    /// True if the angular impulse persists across frames, and false if
    /// the angular impulse is automatically cleared every physics frame.
    pub persistent: bool,
}

impl Deref for ExternalAngularImpulse2d {
    type Target = Scalar;

    fn deref(&self) -> &Self::Target {
        &self.impulse
    }
}

impl DerefMut for ExternalAngularImpulse2d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.impulse
    }
}

impl Default for ExternalAngularImpulse2d {
    fn default() -> Self {
        Self {
            impulse: 0.0,
            persistent: false,
        }
    }
}

impl ExternalAngularImpulse2d {
    /// Zero external angular impulse.
    pub const ZERO: Self = Self {
        impulse: 0.0,
        persistent: false,
    };

    /// Creates a new [`ExternalAngularImpulse`] component with a given `impulse`.
    pub fn new(impulse: Scalar) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    /// Sets the angular impulse.
    pub fn set_impulse(&mut self, impulse: Scalar) -> &mut Self {
        **self = impulse;
        self
    }

    /// Adds the given `impulse` to the angular impulse that will be applied.
    pub fn apply_impulse(&mut self, impulse: Scalar) -> &mut Self {
        **self += impulse;
        self
    }

    /// Determines if the angular impulse is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Returns the angular impulse.
    pub fn impulse(&self) -> Scalar {
        self.impulse
    }

    /// Sets the angular impulse to zero.
    pub fn clear(&mut self) {
        self.impulse = 0.0;
    }
}

/// An external force applied continuously to a dynamic [rigid body](RigidBody).
///
/// The force is stored in world space. If you want to apply forces in local space, you need to
/// use the body's rotation to [transform the local force into world space](#local-forces).
///
/// By default, the force persists across frames. You can clear the force manually using
/// [`clear`](Self::clear) or set `persistent` to false.
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
///     // Apply a force in world space every physics frame.
///     commands.spawn((RigidBody::Dynamic, ExternalForce::new(Vec3::Y)));
///
///     // Apply an initial force and automatically clear it every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple forces.
///     let mut force = ExternalForce::default();
///     force.apply_force(Vec3::Y).apply_force(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, force));
///
///     // Apply a force at a specific point relative to the given center of mass, also applying a torque.
///     // In this case, the torque would cause the body to rotate counterclockwise.
///     let mut force = ExternalForce::default();
///     force.apply_force_at_point(Vec3::Y, Vec3::X, Vec3::ZERO);
///     commands.spawn((RigidBody::Dynamic, force));
/// }
/// ```
///
/// ## Local forces
///
/// The force stored in `ExternalForce` is in world space.
///
/// If you want to apply a force in some direction relative to the body's frame of reference,
/// you need to rotate the force using the body's `Transform` or [`Rotation`].
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
///     // Spawn a rotated body and apply a force in the local up direction.
///     let transform = Transform::from_rotation(Quat::from_rotation_z(0.2));
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalForce::new(transform.rotation * Vec3::Y),
///         transform,
///     ));
/// }
/// ```
///
/// Note that the actual force stored in `ExternalForce` is still in world space.
/// If you want to apply a force in the same local direction every frame,
/// consider setting `persistent` to `false` and running [`apply_force`](Self::apply_force) in a system.
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalForce3d {
    /// The total external force that will be applied.
    force: Vector3,
    /// True if the force persists across frames, and false if the force is automatically cleared every physics frame.
    ///
    /// If you clear the force manually, use the [`clear`](Self::clear) method. This will clear the force and
    /// the torque that is applied when the force is not applied at the center of mass.
    pub persistent: bool,
    /// The torque caused by forces applied at certain points using [`apply_force_at_point`](Self::apply_force_at_point).
    torque: Vector3,
}

impl Deref for ExternalForce3d {
    type Target = Vector3;

    fn deref(&self) -> &Self::Target {
        &self.force
    }
}

impl DerefMut for ExternalForce3d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.force
    }
}

impl Default for ExternalForce3d {
    fn default() -> Self {
        Self {
            force: Vector3::ZERO,
            persistent: true,
            torque: 0.0,
        }
    }
}

impl ExternalForce3d {
    /// Zero external force.
    pub const ZERO: Self = Self {
        force: Vector3::ZERO,
        persistent: true,
        torque: 0.0,
    };

    /// Creates a new [`ExternalForce`] component with a given world-space `force`.
    pub fn new(force: Vector3) -> Self {
        Self { force, ..default() }
    }

    /// Sets the world-space force. Note that the torque caused by any forces will not be reset.
    pub fn set_force(&mut self, force: Vector3) -> &mut Self {
        **self = force;
        self
    }

    /// Adds the given world-space `force` to the force that will be applied.
    pub fn apply_force(&mut self, force: Vector3) -> &mut Self {
        **self += force;
        self
    }

    /// Applies the given world-space `force` at a local `point`, which will also cause torque to be applied.
    pub fn apply_force_at_point(
        &mut self,
        force: Vector3,
        point: Vector3,
        center_of_mass: Vector3,
    ) -> &mut Self {
        **self += force;
        self.torque += (point - center_of_mass).cross(force);
        self
    }

    /// Returns the force in world space.
    pub fn force(&self) -> Vector3 {
        self.force
    }

    /// Returns the torque caused by forces applied at certain points using
    /// [`apply_force_at_point`](Self::apply_force_at_point).
    pub fn torque(&self) -> Vector3 {
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
        self.force = Vector3::ZERO;
        self.torque = 0.0;
    }
}

/// An external torque applied continuously to a dynamic [rigid body](RigidBody).
///
/// By default, the torque persists across frames. You can clear the torque manually using
/// [`clear`](Self::clear) or set `persistent` to false.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
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
///     let mut torque = ExternalTorque::default();
///     torque.apply_torque(Vec3::Y).apply_torque(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, torque));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalTorque3d {
    /// The total external torque that will be applied.
    torque: Vector3,
    /// True if the torque persists across frames, and false if the torque is automatically cleared every physics frame.
    pub persistent: bool,
}

impl Deref for ExternalTorque3d {
    type Target = Vector3;

    fn deref(&self) -> &Self::Target {
        &self.torque
    }
}

impl DerefMut for ExternalTorque3d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.torque
    }
}

impl Default for ExternalTorque3d {
    fn default() -> Self {
        Self {
            torque: 0.0,
            persistent: true,
        }
    }
}

impl ExternalTorque3d {
    /// Zero external torque.
    pub const ZERO: Self = Self {
        torque: 0.0,
        persistent: true,
    };

    /// Creates a new [`ExternalTorque`] component with a given `torque`.
    pub fn new(torque: Vector3) -> Self {
        Self {
            torque,
            ..default()
        }
    }

    /// Sets the torque.
    pub fn set_torque(&mut self, torque: Vector3) -> &mut Self {
        **self = torque;
        self
    }

    /// Adds the given `torque` to the torque that will be applied.
    pub fn apply_torque(&mut self, torque: Vector3) -> &mut Self {
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
    pub fn torque(&self) -> Vector3 {
        self.torque
    }

    /// Sets the torque to zero.
    pub fn clear(&mut self) {
        self.torque = 0.0;
    }
}

/// An external impulse applied instantly to a dynamic [rigid body](RigidBody).
///
/// The impulse is stored in world space. If you want to apply impulses in local space, you need to
/// use the body's rotation to [transform the local impulse into world space](#local-impulses).
///
/// By default, the impulse is cleared every frame. You can set `persistent` to true in order to persist
/// the impulse across frames.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Apply an impulse in world space.
///     commands.spawn((RigidBody::Dynamic, ExternalImpulse::new(Vec3::Y)));
///
///     // Apply an impulse every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalImpulse::new(Vec3::Y).with_persistence(true),
///     ));
///
///     // Apply multiple impulses.
///     let mut impulse = ExternalImpulse::default();
///     impulse.apply_impulse(Vec3::Y).apply_impulse(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, impulse));
///
///     // Apply an impulse at a specific point relative to the given center of mass, also applying an angular impulse.
///     // In this case, the angular impulse would cause the body to rotate counterclockwise.
///     let mut impulse = ExternalImpulse::default();
///     impulse.apply_impulse_at_point(Vec3::Y, Vec3::X, Vec3::ZERO);
///     commands.spawn((RigidBody::Dynamic, impulse));
/// }
/// ```
///
/// ## Local impulses
///
/// The impulse stored in `ExternalImpulse` is in world space.
///
/// If you want to apply an impulse in some direction relative to the body's frame of reference,
/// you need to rotate the impulse using the body's `Transform` or [`Rotation`].
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Spawn a rotated body and apply an impulse in the local up direction.
///     let transform = Transform::from_rotation(Quat::from_rotation_z(0.2));
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalImpulse::new(transform.rotation * Vec3::Y),
///         transform,
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
pub struct ExternalImpulse3d {
    /// The total external impulse that will be applied.
    impulse: Vector3,
    /// True if the impulse persists across frames, and false if the impulse is automatically cleared every physics frame.
    ///
    /// If you clear the impulse manually, use the [`clear`](Self::clear) method. This will clear the impulse and
    /// the angular impulse that is applied when the impulse is not applied at the center of mass.
    pub persistent: bool,
    /// The angular impulse caused by impulses applied at certain points using [`apply_impulse_at_point`](Self::apply_impulse_at_point).
    angular_impulse: Vector3,
}

impl Deref for ExternalImpulse3d {
    type Target = Vector3;

    fn deref(&self) -> &Self::Target {
        &self.impulse
    }
}

impl DerefMut for ExternalImpulse3d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.impulse
    }
}

impl Default for ExternalImpulse3d {
    fn default() -> Self {
        Self {
            impulse: Vector3::ZERO,
            persistent: false,
            angular_impulse: 0.0,
        }
    }
}

impl ExternalImpulse3d {
    /// Zero external impulse.
    pub const ZERO: Self = Self {
        impulse: Vector3::ZERO,
        persistent: false,
        angular_impulse: 0.0,
    };

    /// Creates a new [`ExternalImpulse`] component with a given world-space `impulse`.
    pub fn new(impulse: Vector3) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    /// Sets the world-space impulse. Note that the angular impulse caused by any impulses will not be reset.
    pub fn set_impulse(&mut self, impulse: Vector3) -> &mut Self {
        **self = impulse;
        self
    }

    /// Adds the given world-space `impulse` to the impulse that will be applied.
    pub fn apply_impulse(&mut self, impulse: Vector3) -> &mut Self {
        **self += impulse;
        self
    }

    /// Applies the given world-space `impulse` at a local `point`, which will also cause an angular impulse to be applied.
    pub fn apply_impulse_at_point(
        &mut self,
        impulse: Vector3,
        point: Vector3,
        center_of_mass: Vector3,
    ) -> &mut Self {
        **self += impulse;
        self.angular_impulse += (point - center_of_mass).cross(impulse);
        self
    }

    /// Returns the impulse in world space.
    pub fn impulse(&self) -> Vector3 {
        self.impulse
    }

    /// Returns the angular impulse caused by impulses applied at certain points using
    /// [`apply_impulse_at_point`](Self::apply_impulse_at_point).
    pub fn angular_impulse(&self) -> Vector3 {
        self.angular_impulse
    }

    /// Determines if the impulse is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Sets the impulse and the potential angular impulse caused by the impulse to zero.
    pub fn clear(&mut self) {
        self.impulse = Vector3::ZERO;
        self.angular_impulse = 0.0;
    }
}

/// An external angular impulse applied instantly to a dynamic [rigid body](RigidBody).
///
/// By default, the angular impulse is cleared every frame. You can set `persistent` to true in order to persist
/// the impulse across frames.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Apply an angular impulse.
///     commands.spawn((RigidBody::Dynamic, ExternalAngularImpulse::new(Vec3::Y)));
///
///     // Apply an angular impulse every physics frame.
///     commands.spawn((
///         RigidBody::Dynamic,
///         ExternalAngularImpulse::new(Vec3::Y).with_persistence(false),
///     ));
///
///     // Apply multiple angular impulses.
///     let mut angular_impulse = ExternalAngularImpulse::default();
///     angular_impulse.apply_impulse(Vec3::Y).apply_impulse(Vec3::X);
///     commands.spawn((RigidBody::Dynamic, angular_impulse));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[reflect(Component)]
#[doc(alias = "ExternalTorqueImpulse")]
pub struct ExternalAngularImpulse3d {
    /// The total external angular impulse that will be applied.
    impulse: Vector3,
    /// True if the angular impulse persists across frames, and false if
    /// the angular impulse is automatically cleared every physics frame.
    pub persistent: bool,
}

impl Deref for ExternalAngularImpulse3d {
    type Target = Vector3;

    fn deref(&self) -> &Self::Target {
        &self.impulse
    }
}

impl DerefMut for ExternalAngularImpulse3d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.impulse
    }
}

impl Default for ExternalAngularImpulse3d {
    fn default() -> Self {
        Self {
            impulse: 0.0,
            persistent: false,
        }
    }
}

impl ExternalAngularImpulse3d {
    /// Zero external angular impulse.
    pub const ZERO: Self = Self {
        impulse: 0.0,
        persistent: false,
    };

    /// Creates a new [`ExternalAngularImpulse`] component with a given `impulse`.
    pub fn new(impulse: Vector3) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    /// Sets the angular impulse.
    pub fn set_impulse(&mut self, impulse: Vector3) -> &mut Self {
        **self = impulse;
        self
    }

    /// Adds the given `impulse` to the angular impulse that will be applied.
    pub fn apply_impulse(&mut self, impulse: Vector3) -> &mut Self {
        **self += impulse;
        self
    }

    /// Determines if the angular impulse is persistent or if it should be automatically cleared every physics frame.
    #[doc(alias = "clear_automatically")]
    pub fn with_persistence(mut self, is_persistent: bool) -> Self {
        self.persistent = is_persistent;
        self
    }

    /// Returns the angular impulse.
    pub fn impulse(&self) -> Vector3 {
        self.impulse
    }

    /// Sets the angular impulse to zero.
    pub fn clear(&mut self) {
        self.impulse = 0.0;
    }
}
