//! Forces, torques, linear impulses, and angular impulses
//! that can be applied to dynamic rigid bodies.

#![allow(missing_docs)]

use crate::prelude::*;
use bevy::{
    ecs::system::{
        lifetimeless::{Read, Write},
        SystemParam,
    },
    prelude::*,
};
use core::ops::{Deref, DerefMut};
use derive_more::From;

#[cfg(feature = "2d")]
pub(crate) type Torque = Scalar;

#[cfg(feature = "3d")]
pub(crate) type Torque = Vector;

#[cfg(feature = "2d")]
pub(crate) trait FloatZero {
    const ZERO: Self;
}

#[cfg(feature = "2d")]
impl FloatZero for Scalar {
    const ZERO: Self = 0.0;
}

#[derive(SystemParam)]
pub struct ForceHelper<'w, 's> {
    time: Res<'w, Time>,
    commands: Commands<'w, 's>,
    pub velocity_query: Query<'w, 's, (Write<LinearVelocity>, Write<AngularVelocity>)>,
    pub constant_force_query: Query<'w, 's, Write<ConstantForce>>,
    pub constant_torque_query: Query<'w, 's, Write<ConstantTorque>>,
    pub linear_acceleration_query: Query<'w, 's, Write<AccumulatedLinearAcceleration>>,
    pub angular_acceleration_query: Query<'w, 's, Write<AccumulatedAngularAcceleration>>,
    pub mass_properties_query: Query<
        'w,
        's,
        (
            Read<ComputedMass>,
            Read<GlobalAngularInertia>,
            Read<ComputedCenterOfMass>,
        ),
    >,
}

impl ForceHelper<'_, '_> {
    /// Adds a world-space [`ConstantForce`] that is applied continuously at the given
    /// point relative to the position of the entity until it is cleared.
    ///
    /// If the point is not a zero vector (the center of mass), the force will also apply a torque.
    ///
    /// If the [`ConstantForce`] component already exists for the entity, the new force is added to the existing force.
    /// Otherwise, the component is inserted. The same applies to the [`ConstantTorque`] component if torque is non-zero.
    ///
    /// The force is typically in Newtons, kg*m/s^2.
    pub fn add_constant_force(&mut self, entity: Entity, force: Vector, point: Vector) {
        let torque = cross(force, point);

        self.add_constant_center_force(entity, force);

        if torque != Torque::ZERO {
            self.add_constant_torque(entity, torque);
        }
    }

    /// Adds a world-space [`ConstantForce`] that is applied continuously at the center of mass until it is cleared.
    ///
    /// If the [`ConstantForce`] component already exists for the entity, the new force is added to the existing force.
    /// Otherwise, the component is inserted.
    ///
    /// The force is typically in Newtons, kg*m/s^2.
    pub fn add_constant_center_force(&mut self, entity: Entity, force: Vector) {
        if let Ok(mut constant_force) = self.constant_force_query.get_mut(entity) {
            constant_force.0 += force;
        } else {
            self.commands.entity(entity).insert(ConstantForce(force));
        }
    }

    /// Sets the [`ConstantForce`] of the given entity to the given force applied continuously
    /// at the given point relative to the position of the entity in world space.
    ///
    /// If the point is not a zero vector (the center of mass), the force will also apply a torque.
    ///
    /// If the [`ConstantForce`] component does not exist for the entity, it is inserted.
    /// The same applies to the [`ConstantTorque`] component if torque is non-zero.
    ///
    /// The force is typically in Newtons, kg*m/s^2.
    pub fn set_constant_force(&mut self, entity: Entity, force: Vector, point: Vector) {
        let torque = cross(force, point);

        self.set_constant_center_force(entity, force);

        if torque != Torque::ZERO {
            self.set_constant_torque(entity, torque);
        }
    }

    /// Sets the [`ConstantForce`] of the given entity to the given force in world space.
    ///
    /// If the [`ConstantForce`] component does not exist for the entity, it is inserted.
    ///
    /// The force is typically in Newtons, kg*m/s^2.
    pub fn set_constant_center_force(&mut self, entity: Entity, force: Vector) {
        if let Ok(mut constant_force) = self.constant_force_query.get_mut(entity) {
            constant_force.0 = force;
        } else {
            self.commands.entity(entity).insert(ConstantForce(force));
        }
    }

    /// Sets the [`ConstantTorque`] of the given entity to the given torque in world space.
    ///
    /// If the [`ConstantTorque`] component does not exist for the entity, it is inserted.
    ///
    /// The torque is typically in Newton-meters, kg*m^2/s^2.
    pub fn set_constant_torque(&mut self, entity: Entity, torque: Torque) {
        if let Ok(mut constant_torque) = self.constant_torque_query.get_mut(entity) {
            constant_torque.0 = torque;
        } else {
            self.commands.entity(entity).insert(ConstantTorque(torque));
        }
    }

    /// Adds a world-space [`ConstantTorque`] that is applied continuously until it is cleared.
    ///
    /// If the [`ConstantTorque`] component already exists for the entity, the new torque is added to the existing torque.
    /// Otherwise, the component is inserted.
    ///
    /// The torque is typically in Newton-meters, kg*m^2/s^2.
    pub fn add_constant_torque(&mut self, entity: Entity, torque: Torque) {
        if let Ok(mut constant_torque) = self.constant_torque_query.get_mut(entity) {
            constant_torque.0 += torque;
        } else {
            self.commands.entity(entity).insert(ConstantTorque(torque));
        }
    }

    /// Applies a linear impulse at the given point relative to the position of the entity in world space.
    ///
    /// If the point is not a zero vector (the center of mass), the impulse will also apply an angular impulse.
    ///
    /// The impulse is typically in Newton-seconds, kg*m/s.
    pub fn apply_linear_impulse(&mut self, entity: Entity, impulse: Vector, point: Vector) {
        let (mass, angular_inertia, _) = self.mass_properties_query.get(entity).unwrap();

        let linear_velocity = mass.inverse() * impulse;
        let angular_velocity = angular_inertia.inverse() * cross(point, impulse);

        self.apply_linear_center_impulse(entity, linear_velocity);
        self.apply_angular_impulse(entity, angular_velocity);
    }

    /// Applies a linear impulse at the center of mass of the given entity.
    ///
    /// The impulse is typically in Newton-seconds, kg*m/s.
    pub fn apply_linear_center_impulse(&mut self, entity: Entity, impulse: Vector) {
        let delta_time = self.time.delta_secs_f64().adjust_precision();
        let (mut linear_velocity, _) = self.velocity_query.get_mut(entity).unwrap();
        linear_velocity.0 += impulse / delta_time;
    }

    /// Applies an angular impulse to the given entity.
    ///
    /// The impulse is typically in Newton-meter-seconds, kg*m^2/s.
    pub fn apply_angular_impulse(&mut self, entity: Entity, impulse: Torque) {
        let delta_time = self.time.delta_secs_f64().adjust_precision();
        let (_, mut angular_velocity) = self.velocity_query.get_mut(entity).unwrap();
        angular_velocity.0 += impulse / delta_time;
    }

    /// Applies a force at the given point relative to the position of the entity in world space.
    ///
    /// If the point is not a zero vector (the center of mass), the force will also apply a torque.
    ///
    /// The force is typically in Newtons, kg*m/s^2.
    pub fn apply_force(&mut self, entity: Entity, force: Vector, point: Vector) {
        let (mass, angular_inertia, _) = self.mass_properties_query.get(entity).unwrap();

        let linear_acceleration = mass.inverse() * force;
        let angular_acceleration = angular_inertia.inverse() * cross(point, force);

        self.apply_linear_acceleration(entity, linear_acceleration);
        self.apply_angular_acceleration(entity, angular_acceleration);
    }

    /// Applies a force at the center of mass of the given entity.
    ///
    /// The force is typically in Newtons, kg*m/s^2.
    pub fn apply_center_force(&mut self, entity: Entity, force: Vector) {
        let mass = self.mass_properties_query.get(entity).unwrap().0;
        let acceleration = mass.inverse() * force;
        self.apply_linear_acceleration(entity, acceleration);
    }

    /// Applies a torque to the given entity.
    ///
    /// The torque is typically in Newton-meters, kg*m^2/s^2.
    pub fn apply_torque(&mut self, entity: Entity, torque: Torque) {
        let angular_inertia = self.mass_properties_query.get(entity).unwrap().1;
        let angular_acceleration = angular_inertia.inverse() * torque;
        self.apply_angular_acceleration(entity, angular_acceleration);
    }

    /// Applies a linear acceleration to the given entity, ignoring mass.
    ///
    /// The acceleration is typically in m/s^2.
    pub fn apply_linear_acceleration(&mut self, entity: Entity, acceleration: Vector) {
        if let Ok(mut accumulated_acceleration) = self.linear_acceleration_query.get_mut(entity) {
            accumulated_acceleration.0 += acceleration;
        } else {
            self.commands
                .entity(entity)
                .insert(AccumulatedLinearAcceleration(acceleration));
        }
    }

    /// Applies an angular acceleration to the given entity, ignoring angular inertia.
    ///
    /// The acceleration is typically in rad/s^2.
    pub fn apply_angular_acceleration(&mut self, entity: Entity, acceleration: Torque) {
        if let Ok(mut accumulated_acceleration) = self.linear_acceleration_query.get_mut(entity) {
            accumulated_acceleration.0 += acceleration;
        } else {
            self.commands
                .entity(entity)
                .insert(AccumulatedAngularAcceleration(acceleration));
        }
    }
}

#[derive(Component, Debug, Default, PartialEq, Reflect)]
#[require(AccumulatedLinearAcceleration)]
pub struct ConstantForce(pub Vector);

#[derive(Component, Debug, Default, PartialEq, Reflect)]
#[require(AccumulatedAngularAcceleration)]
pub struct ConstantTorque(pub Torque);

#[derive(Component, Debug, Default, PartialEq, Reflect)]
pub struct AccumulatedLinearAcceleration(pub Vector);

impl AccumulatedLinearAcceleration {
    /// Zero accumulated linear acceleration.
    pub const ZERO: Self = Self(Vector::ZERO);
}

#[derive(Component, Debug, Default, PartialEq, Reflect)]
pub struct AccumulatedAngularAcceleration(pub Torque);

impl AccumulatedAngularAcceleration {
    /// Zero accumulated angular acceleration.
    pub const ZERO: Self = Self(Torque::ZERO);
}

pub struct ForcePlugin;

impl Plugin for ForcePlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<(
            ConstantForce,
            ConstantTorque,
            AccumulatedLinearAcceleration,
            AccumulatedAngularAcceleration,
        )>();
        app.configure_sets(
            PhysicsSchedule,
            (
                PhysicsStepSet::NarrowPhase,
                ConstantForceSet,
                PhysicsStepSet::Solver,
            )
                .chain(),
        );

        app.add_systems(
            PhysicsSchedule,
            (apply_constant_force, apply_constant_torque).in_set(ConstantForceSet),
        );

        app.add_systems(
            PhysicsSchedule,
            (clear_linear_acceleration, clear_angular_acceleration).after(PhysicsStepSet::Solver),
        );
    }
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ConstantForceSet;

fn apply_constant_force(
    mut query: Query<(
        &ComputedMass,
        &ConstantForce,
        &mut AccumulatedLinearAcceleration,
    )>,
) {
    for (mass, constant_force, mut accumulated_acceleration) in &mut query {
        accumulated_acceleration.0 += mass.inverse() * constant_force.0;
    }
}

fn apply_constant_torque(
    mut query: Query<(
        &GlobalAngularInertia,
        &ConstantTorque,
        &mut AccumulatedAngularAcceleration,
    )>,
) {
    for (angular_inertia, constant_torque, mut accumulated_acceleration) in &mut query {
        accumulated_acceleration.0 += angular_inertia.inverse() * constant_torque.0;
    }
}

fn clear_linear_acceleration(mut query: Query<&mut AccumulatedLinearAcceleration>) {
    for mut accumulated_acceleration in &mut query {
        accumulated_acceleration.0 = Vector::ZERO;
    }
}

fn clear_angular_acceleration(mut query: Query<&mut AccumulatedAngularAcceleration>) {
    for mut accumulated_acceleration in &mut query {
        accumulated_acceleration.0 = Torque::ZERO;
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
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
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
/// # Local Forces
///
/// The force stored in `ExternalForce` is in world space.
///
/// If you want to apply a force in some direction relative to the body's frame of reference,
/// you need to rotate the force using the body's `Transform` or [`Rotation`].
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ExternalForce {
    /// The total external force that will be applied.
    force: Vector,
    /// True if the force persists across frames, and false if the force is automatically cleared every physics frame.
    ///
    /// If you clear the force manually, use the [`clear`](Self::clear) method. This will clear the force and
    /// the torque that is applied when the force is not applied at the center of mass.
    pub persistent: bool,
    /// The torque caused by forces applied at certain points using [`apply_force_at_point`](Self::apply_force_at_point).
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

    /// Creates a new [`ExternalForce`] component with a given world-space `force`.
    pub fn new(force: Vector) -> Self {
        Self { force, ..default() }
    }

    /// Sets the world-space force. Note that the torque caused by any forces will not be reset.
    pub fn set_force(&mut self, force: Vector) -> &mut Self {
        **self = force;
        self
    }

    /// Adds the given world-space `force` to the force that will be applied.
    pub fn apply_force(&mut self, force: Vector) -> &mut Self {
        **self += force;
        self
    }

    /// Applies the given `force` at the specified `point`, which will also cause torque to be applied.
    ///
    /// The force, point, and center of mass must be given in world space.
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

    /// Returns the force in world space.
    pub fn force(&self) -> Vector {
        self.force
    }

    /// Returns the torque caused by forces applied at certain points using
    /// [`apply_force_at_point`](Self::apply_force_at_point).
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

/// An external torque applied continuously to a dynamic [rigid body](RigidBody).
///
/// By default, the torque persists across frames. You can clear the torque manually using
/// [`clear`](Self::clear) or set `persistent` to false.
///
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ExternalTorque {
    /// The total external torque that will be applied.
    torque: Torque,
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

/// An external impulse applied instantly to a dynamic [rigid body](RigidBody).
///
/// The impulse is stored in world space. If you want to apply impulses in local space, you need to
/// use the body's rotation to [transform the local impulse into world space](#local-impulses).
///
/// By default, the impulse is cleared every frame. You can set `persistent` to true in order to persist
/// the impulse across frames.
///
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
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
/// # Local Impulses
///
/// The impulse stored in `ExternalImpulse` is in world space.
///
/// If you want to apply an impulse in some direction relative to the body's frame of reference,
/// you need to rotate the impulse using the body's `Transform` or [`Rotation`].
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ExternalImpulse {
    /// The total external impulse that will be applied.
    impulse: Vector,
    /// True if the impulse persists across frames, and false if the impulse is automatically cleared every physics frame.
    ///
    /// If you clear the impulse manually, use the [`clear`](Self::clear) method. This will clear the impulse and
    /// the angular impulse that is applied when the impulse is not applied at the center of mass.
    pub persistent: bool,
    /// The angular impulse caused by impulses applied at certain points using [`apply_impulse_at_point`](Self::apply_impulse_at_point).
    angular_impulse: Torque,
}

impl Deref for ExternalImpulse {
    type Target = Vector;

    fn deref(&self) -> &Self::Target {
        &self.impulse
    }
}

impl DerefMut for ExternalImpulse {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.impulse
    }
}

impl Default for ExternalImpulse {
    fn default() -> Self {
        Self {
            impulse: Vector::ZERO,
            persistent: false,
            angular_impulse: Torque::ZERO,
        }
    }
}

impl ExternalImpulse {
    /// Zero external impulse.
    pub const ZERO: Self = Self {
        impulse: Vector::ZERO,
        persistent: false,
        angular_impulse: Torque::ZERO,
    };

    /// Creates a new [`ExternalImpulse`] component with a given world-space `impulse`.
    pub fn new(impulse: Vector) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    /// Sets the world-space impulse. Note that the angular impulse caused by any impulses will not be reset.
    pub fn set_impulse(&mut self, impulse: Vector) -> &mut Self {
        **self = impulse;
        self
    }

    /// Adds the given world-space `impulse` to the impulse that will be applied.
    pub fn apply_impulse(&mut self, impulse: Vector) -> &mut Self {
        **self += impulse;
        self
    }

    /// Applies the given `impulse` at the specified `point`, which will also cause an angular impulse to be applied.
    ///
    /// The impulse, point, and center of mass must be given in world space.
    pub fn apply_impulse_at_point(
        &mut self,
        impulse: Vector,
        point: Vector,
        center_of_mass: Vector,
    ) -> &mut Self {
        **self += impulse;
        #[cfg(feature = "2d")]
        {
            self.angular_impulse += (point - center_of_mass).perp_dot(impulse);
        }
        #[cfg(feature = "3d")]
        {
            self.angular_impulse += (point - center_of_mass).cross(impulse);
        }
        self
    }

    /// Returns the impulse in world space.
    pub fn impulse(&self) -> Vector {
        self.impulse
    }

    /// Returns the angular impulse caused by impulses applied at certain points using
    /// [`apply_impulse_at_point`](Self::apply_impulse_at_point).
    pub fn angular_impulse(&self) -> Torque {
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
        self.impulse = Vector::ZERO;
        self.angular_impulse = Torque::ZERO;
    }
}

/// An external angular impulse applied instantly to a dynamic [rigid body](RigidBody).
///
/// By default, the angular impulse is cleared every frame. You can set `persistent` to true in order to persist
/// the impulse across frames.
///
/// # Example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
#[doc(alias = "ExternalTorqueImpulse")]
pub struct ExternalAngularImpulse {
    /// The total external angular impulse that will be applied.
    impulse: Torque,
    /// True if the angular impulse persists across frames, and false if
    /// the angular impulse is automatically cleared every physics frame.
    pub persistent: bool,
}

impl Deref for ExternalAngularImpulse {
    type Target = Torque;

    fn deref(&self) -> &Self::Target {
        &self.impulse
    }
}

impl DerefMut for ExternalAngularImpulse {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.impulse
    }
}

impl Default for ExternalAngularImpulse {
    fn default() -> Self {
        Self {
            impulse: Torque::ZERO,
            persistent: false,
        }
    }
}

impl ExternalAngularImpulse {
    /// Zero external angular impulse.
    pub const ZERO: Self = Self {
        impulse: Torque::ZERO,
        persistent: false,
    };

    /// Creates a new [`ExternalAngularImpulse`] component with a given `impulse`.
    pub fn new(impulse: Torque) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    /// Sets the angular impulse.
    pub fn set_impulse(&mut self, impulse: Torque) -> &mut Self {
        **self = impulse;
        self
    }

    /// Adds the given `impulse` to the angular impulse that will be applied.
    pub fn apply_impulse(&mut self, impulse: Torque) -> &mut Self {
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
    pub fn impulse(&self) -> Torque {
        self.impulse
    }

    /// Sets the angular impulse to zero.
    pub fn clear(&mut self) {
        self.impulse = Torque::ZERO;
    }
}
