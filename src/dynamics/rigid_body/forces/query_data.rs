use crate::{
    dynamics::integrator::VelocityIntegrationData,
    prelude::{mass_properties::components::GlobalCenterOfMass, *},
};
use bevy::ecs::{
    query::{Has, QueryData},
    system::lifetimeless::{Read, Write},
};

use super::AccumulatedLocalAcceleration;

/// A helper [`QueryData`] for applying forces, impulses, and accelerations to dynamic [rigid bodies](RigidBody).
///
/// For constant forces that persist across time steps, consider using components like [`ConstantForce`] instead.
///
/// See the [module-level documentation](crate::dynamics::rigid_body::forces) for more general information about forces in Avian.
///
/// # Usage
///
/// To use [`Forces`], add it to a [`Query`](bevy::prelude::Query) (without `&` or `&mut`),
/// and use the associated methods to apply forces, impulses, and accelerations to the rigid bodies.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # #[cfg(feature = "f32")]
/// fn apply_forces(mut query: Query<Forces>) {
///     for mut forces in &mut query {
///         // Apply a force of 10 N in the positive Y direction to the entity.
#[cfg_attr(
    feature = "2d",
    doc = "        forces.apply_force(Vec2::new(0.0, 10.0));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        forces.apply_force(Vec3::new(0.0, 10.0, 0.0));"
)]
///     }
/// }
/// ```
///
/// The force is applied continuously during the physics step, and cleared automatically after the step is complete.
///
/// By default, applying forces to [sleeping](Sleeping) bodies will wake them up. If this is not desired,
/// the [`non_waking`](ForcesItem::non_waking) method can be used to fetch a [`NonWakingForcesItem`]
/// that allows applying forces to a body without waking it up.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::{math::Vector, prelude::*};")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::{math::Vector, prelude::*};")]
/// # use bevy::prelude::*;
/// #
/// # fn apply_impulses(mut query: Query<Forces>) {
/// #     for mut forces in &mut query {
/// #         let force = Vector::default();
/// // Apply a force without waking up the body if it is sleeping.
/// forces.non_waking().apply_force(force);
/// #     }
/// # }
/// ```
///
/// [`Forces`] can also apply forces and impulses at a specific point in the world. If the point is not aligned
/// with the [`GlobalCenterOfMass`], it will apply a torque to the body.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::{math::Vector, prelude::*};")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::{math::Vector, prelude::*};")]
/// # use bevy::prelude::*;
/// #
/// # fn apply_impulses(mut query: Query<Forces>) {
/// #     for mut forces in &mut query {
/// #         let force = Vector::default();
/// #         let point = Vector::default();
/// // Apply an impulse at a specific point in the world.
/// // Unlike forces, impulses are applied immediately to the velocity.
/// forces.apply_linear_impulse_at_point(force, point);
/// #     }
/// # }
/// ```
///
/// As an example, you could implement radial gravity that pulls rigid bodies towards the world origin
/// with a system like the following:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # #[cfg(feature = "f32")]
/// fn radial_gravity(mut query: Query<(Forces, &GlobalTransform)>) {
///     for (mut forces, global_transform) in &mut query {
///         // Compute the direction towards the center of the world.
///         let direction = -global_transform.translation().normalize_or_zero();
///         // Apply a linear acceleration of 9.81 m/s² towards the center of the world.
#[cfg_attr(
    feature = "2d",
    doc = "        forces.apply_linear_acceleration(direction.truncate() * 9.81);"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        forces.apply_linear_acceleration(direction * 9.81);"
)]
///     }
/// }
/// ```
#[derive(QueryData)]
#[query_data(mutable)]
pub struct Forces {
    rotation: Read<Rotation>,
    linear_velocity: Write<LinearVelocity>,
    angular_velocity: Write<AngularVelocity>,
    mass: Read<ComputedMass>,
    angular_inertia: Read<ComputedAngularInertia>,
    global_center_of_mass: Read<GlobalCenterOfMass>,
    locked_axes: Option<Read<LockedAxes>>,
    integration: Write<VelocityIntegrationData>,
    accumulated_local_acceleration: Write<AccumulatedLocalAcceleration>,
    time_sleeping: Write<TimeSleeping>,
    is_sleeping: Has<Sleeping>,
}

/// A [`ForcesItem`] that does not wake up the body when applying forces, torques, impulses, or accelerations.
/// Returned by [`ForcesItem::non_waking`].
///
/// See the documentation of [`Forces`] for more information on how to apply forces in Avian.
pub struct NonWakingForcesItem<'a>(pub ForcesItem<'a>);

impl ForcesItem<'_> {
    /// Reborrows `self` as a new instance of [`ForcesItem`].
    pub fn reborrow(&mut self) -> ForcesItem<'_> {
        ForcesItem {
            rotation: self.rotation,
            linear_velocity: self.linear_velocity.reborrow(),
            angular_velocity: self.angular_velocity.reborrow(),
            mass: self.mass,
            angular_inertia: self.angular_inertia,
            global_center_of_mass: self.global_center_of_mass,
            locked_axes: self.locked_axes,
            integration: self.integration.reborrow(),
            accumulated_local_acceleration: self.accumulated_local_acceleration.reborrow(),
            time_sleeping: self.time_sleeping.reborrow(),
            is_sleeping: self.is_sleeping,
        }
    }

    /// Returns a [`NonWakingForcesItem`] that allows applying forces, impulses, and accelerations
    /// without waking up the body if it is sleeping.
    #[inline]
    #[must_use]
    pub fn non_waking(&mut self) -> NonWakingForcesItem<'_> {
        NonWakingForcesItem(self.reborrow())
    }
}

impl<'a> NonWakingForcesItem<'a> {
    /// Returns a [`ForcesItem`] that will wake up the body when applying forces, impulses, or accelerations.
    #[inline]
    #[must_use]
    pub fn waking(self) -> ForcesItem<'a> {
        self.0
    }
}

impl RigidBodyForces for ForcesItem<'_> {}
impl RigidBodyForces for NonWakingForcesItem<'_> {}

/// A trait for applying forces, impulses, and accelerations to a dynamic [rigid body](RigidBody).
///
/// This is implemented as a shared interface for the [`ForcesItem`] and [`NonWakingForcesItem`]
/// returned by [`Forces`].
///
/// See the documentation of [`Forces`] for more information on how to apply forces in Avian.
#[expect(
    private_bounds,
    reason = "The `data` method should not be publicly accessible."
)]
pub trait RigidBodyForces: RigidBodyForcesInternal {
    /// Applies a force at the center of mass in world space. The unit is typically N or kg⋅m/s².
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero force will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_force(&mut self, force: Vector) {
        let force = self.inverse_mass() * force;
        self.apply_linear_acceleration(force);
    }

    /// Applies a force at the given point in world space. The unit is typically N or kg⋅m/s².
    ///
    /// If the point is not at the center of mass, the force will also generate a torque.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero force will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_force_at_point(&mut self, force: Vector, world_point: Vector) {
        // Note: This does not consider the rotation of the body during substeps,
        //       so the torque may not be accurate if the body is rotating quickly.
        self.apply_force(force);
        self.apply_torque(cross(world_point - self.global_center_of_mass(), force));
    }

    /// Applies a force at the center of mass in local space. The unit is typically N or kg⋅m/s².
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero force will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_local_force(&mut self, force: Vector) {
        let acceleration = self.inverse_mass() * force;
        self.apply_local_linear_acceleration(acceleration);
    }

    /// Applies a torque in world space. The unit is typically N⋅m or kg⋅m²/s².
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero torque will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_torque(&mut self, torque: AngularVector) {
        let acceleration = self.global_inverse_angular_inertia() * torque;
        self.apply_angular_acceleration(acceleration);
    }

    /// Applies a torque in local space. The unit is typically N⋅m or kg⋅m²/s².
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero torque will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[cfg(feature = "3d")]
    #[inline]
    fn apply_local_torque(&mut self, torque: AngularVector) {
        let acceleration = self.inverse_angular_inertia() * torque;
        self.apply_local_angular_acceleration(acceleration);
    }

    /// Applies a linear impulse at the center of mass in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    ///
    /// By default, a non-zero impulse will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_linear_impulse(&mut self, impulse: Vector) {
        let effective_inverse_mass = self
            .locked_axes()
            .apply_to_vec(Vector::splat(self.inverse_mass()));
        let delta_vel = effective_inverse_mass * impulse;

        if impulse != Vector::ZERO && self.try_wake_up() {
            *self.linear_velocity_mut() += delta_vel;
        }
    }

    /// Applies a linear impulse at the given point in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// If the point is not at the center of mass, the impulse will also generate an angular impulse.
    ///
    /// The impulse modifies the [`LinearVelocity`] and [`AngularVelocity`] of the body immediately.
    ///
    /// By default, a non-zero impulse will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_linear_impulse_at_point(&mut self, impulse: Vector, world_point: Vector) {
        self.apply_linear_impulse(impulse);
        self.apply_angular_impulse(cross(world_point - self.global_center_of_mass(), impulse));
    }

    /// Applies a linear impulse in local space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    ///
    /// By default, a non-zero impulse will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_local_linear_impulse(&mut self, impulse: Vector) {
        let world_impulse = self.rotation() * impulse;
        self.apply_linear_impulse(world_impulse);
    }

    /// Applies an angular impulse in world space. The unit is typically N⋅m⋅s or kg⋅m²/s.
    ///
    /// The impulse modifies the [`AngularVelocity`] of the body immediately.
    ///
    /// By default, a non-zero impulse will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_angular_impulse(&mut self, impulse: AngularVector) {
        let effective_inverse_angular_inertia = self
            .locked_axes()
            .apply_to_angular_inertia(self.global_inverse_angular_inertia());
        let delta_vel = effective_inverse_angular_inertia * impulse;

        if impulse != AngularVector::ZERO && self.try_wake_up() {
            *self.angular_velocity_mut() += delta_vel;
        }
    }

    /// Applies an angular impulse in local space. The unit is typically N⋅m⋅s or kg⋅m²/s.
    ///
    /// The impulse modifies the [`AngularVelocity`] of the body immediately.
    ///
    /// By default, a non-zero impulse will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[cfg(feature = "3d")]
    #[inline]
    fn apply_local_angular_impulse(&mut self, impulse: AngularVector) {
        let world_impulse = self.rotation() * impulse;
        self.apply_angular_impulse(world_impulse);
    }

    /// Applies a linear acceleration, ignoring mass. The unit is typically m/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero acceleration will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_linear_acceleration(&mut self, acceleration: Vector) {
        if acceleration != Vector::ZERO && self.try_wake_up() {
            self.integration_data_mut()
                .apply_linear_acceleration(acceleration);
        }
    }

    /// Applies a linear acceleration in local space, ignoring mass. The unit is typically m/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero acceleration will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_local_linear_acceleration(&mut self, acceleration: Vector) {
        if acceleration != Vector::ZERO && self.try_wake_up() {
            self.accumulated_local_acceleration_mut().linear += acceleration;
        }
    }

    /// Applies an angular acceleration, ignoring angular inertia. The unit is rad/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero acceleration will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[inline]
    fn apply_angular_acceleration(&mut self, acceleration: AngularVector) {
        if acceleration != AngularVector::ZERO && self.try_wake_up() {
            self.integration_data_mut()
                .apply_angular_acceleration(acceleration);
        }
    }

    /// Applies an angular acceleration in local space, ignoring angular inertia. The unit is rad/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    ///
    /// By default, a non-zero acceleration will wake up the body if it is sleeping. This can be prevented
    /// by first calling [`ForcesItem::non_waking`] to get a [`NonWakingForcesItem`].
    #[cfg(feature = "3d")]
    #[inline]
    fn apply_local_angular_acceleration(&mut self, acceleration: AngularVector) {
        if acceleration != AngularVector::ZERO && self.try_wake_up() {
            self.accumulated_local_acceleration_mut().angular += acceleration;
        }
    }

    /// Returns the linear acceleration that the body has accumulated
    /// before the physics step in world space, including acceleration
    /// caused by forces.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only forces and accelerations applied through [`Forces`] are included.
    #[inline]
    fn accumulated_linear_acceleration(&self) -> Vector {
        // The linear increment is treated as linear acceleration until the integration step.
        let world_linear_acceleration = self.integration_data().linear_increment;
        let local_linear_acceleration = self.accumulated_local_acceleration().linear;

        // Return the total world-space linear acceleration.
        self.locked_axes()
            .apply_to_vec(world_linear_acceleration + self.rotation() * local_linear_acceleration)
    }

    /// Returns the angular acceleration that the body has accumulated
    /// before the physics step in world space, including acceleration
    /// caused by torques.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only torques and accelerations applied through [`Forces`] are included.
    #[cfg(feature = "2d")]
    #[inline]
    fn accumulated_angular_acceleration(&self) -> AngularVector {
        // The angular increment is treated as angular acceleration until the integration step.
        self.locked_axes()
            .apply_to_angular_velocity(self.integration_data().angular_increment)
    }

    /// Returns the angular acceleration that the body has accumulated
    /// before the physics step in world space, including acceleration
    /// caused by torques.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only torques and accelerations applied through [`Forces`] are included.
    #[cfg(feature = "3d")]
    #[inline]
    fn accumulated_angular_acceleration(&self) -> AngularVector {
        // The angular increment is treated as angular acceleration until the integration step.
        let world_angular_acceleration = self.integration_data().angular_increment;
        let local_angular_acceleration = self.accumulated_local_acceleration().angular;

        // Return the total world-space angular acceleration.
        self.locked_axes().apply_to_angular_velocity(
            world_angular_acceleration + self.rotation() * local_angular_acceleration,
        )
    }

    /// Resets the accumulated linear acceleration to zero.
    #[inline]
    fn reset_accumulated_linear_acceleration(&mut self) {
        self.integration_data_mut().linear_increment = Vector::ZERO;
        self.accumulated_local_acceleration_mut().linear = Vector::ZERO;
    }

    /// Resets the accumulated angular acceleration to zero.
    #[inline]
    fn reset_accumulated_angular_acceleration(&mut self) {
        self.integration_data_mut().angular_increment = AngularVector::ZERO;
        #[cfg(feature = "3d")]
        {
            self.accumulated_local_acceleration_mut().angular = AngularVector::ZERO;
        }
    }

    /// Returns the [`LinearVelocity`] of the body in world space.
    #[inline]
    fn linear_velocity(&self) -> Vector {
        self.lin_vel()
    }

    /// Returns a mutable reference to the [`LinearVelocity`] of the body in world space.
    #[inline]
    fn linear_velocity_mut(&mut self) -> &mut Vector {
        self.lin_vel_mut()
    }

    /// Returns the [`AngularVelocity`] of the body in world space.
    #[inline]
    fn angular_velocity(&self) -> AngularVector {
        self.ang_vel()
    }

    /// Returns a mutable reference to the [`AngularVelocity`] of the body in world space.
    #[inline]
    fn angular_velocity_mut(&mut self) -> &mut AngularVector {
        self.ang_vel_mut()
    }
}

/// A trait to provide internal getters and helpers for [`RigidBodyForces`].
trait RigidBodyForcesInternal {
    fn rotation(&self) -> &Rotation;
    fn lin_vel(&self) -> Vector;
    fn lin_vel_mut(&mut self) -> &mut Vector;
    fn ang_vel(&self) -> AngularVector;
    fn ang_vel_mut(&mut self) -> &mut AngularVector;
    fn inverse_mass(&self) -> Scalar;
    #[cfg(feature = "3d")]
    fn inverse_angular_inertia(&self) -> SymmetricTensor;
    fn global_inverse_angular_inertia(&self) -> SymmetricTensor;
    fn global_center_of_mass(&self) -> Vector;
    fn locked_axes(&self) -> LockedAxes;
    fn integration_data(&self) -> &VelocityIntegrationData;
    fn integration_data_mut(&mut self) -> &mut VelocityIntegrationData;
    fn accumulated_local_acceleration(&self) -> &AccumulatedLocalAcceleration;
    fn accumulated_local_acceleration_mut(&mut self) -> &mut AccumulatedLocalAcceleration;
    fn try_wake_up(&mut self) -> bool;
}

impl RigidBodyForcesInternal for ForcesItem<'_> {
    #[inline]
    fn rotation(&self) -> &Rotation {
        &self.rotation
    }
    #[inline]
    fn lin_vel(&self) -> Vector {
        self.linear_velocity.0
    }
    #[inline]
    fn lin_vel_mut(&mut self) -> &mut Vector {
        &mut self.linear_velocity.0
    }
    #[inline]
    fn ang_vel(&self) -> AngularVector {
        self.angular_velocity.0
    }
    #[inline]
    fn ang_vel_mut(&mut self) -> &mut AngularVector {
        &mut self.angular_velocity.0
    }
    #[inline]
    fn inverse_mass(&self) -> Scalar {
        self.mass.inverse()
    }
    #[inline]
    #[cfg(feature = "3d")]
    fn inverse_angular_inertia(&self) -> SymmetricTensor {
        self.angular_inertia.inverse()
    }
    #[inline]
    fn global_inverse_angular_inertia(&self) -> SymmetricTensor {
        #[cfg(feature = "2d")]
        let global_angular_inertia = *self.angular_inertia;
        #[cfg(feature = "3d")]
        let global_angular_inertia = self.angular_inertia.rotated(self.rotation.0);
        self.locked_axes()
            .apply_to_angular_inertia(global_angular_inertia)
            .inverse()
    }
    #[inline]
    fn global_center_of_mass(&self) -> Vector {
        self.global_center_of_mass.get()
    }
    #[inline]
    fn locked_axes(&self) -> LockedAxes {
        self.locked_axes.copied().unwrap_or_default()
    }
    #[inline]
    fn integration_data(&self) -> &VelocityIntegrationData {
        &self.integration
    }
    #[inline]
    fn integration_data_mut(&mut self) -> &mut VelocityIntegrationData {
        &mut self.integration
    }
    #[inline]
    fn accumulated_local_acceleration(&self) -> &AccumulatedLocalAcceleration {
        &self.accumulated_local_acceleration
    }
    #[inline]
    fn accumulated_local_acceleration_mut(&mut self) -> &mut AccumulatedLocalAcceleration {
        &mut self.accumulated_local_acceleration
    }
    #[inline]
    fn try_wake_up(&mut self) -> bool {
        // Wake up the body. Return `true` to indicate that the body is awake.
        self.time_sleeping.reset();
        true
    }
}

impl RigidBodyForcesInternal for NonWakingForcesItem<'_> {
    #[inline]
    fn rotation(&self) -> &Rotation {
        self.0.rotation()
    }
    #[inline]
    fn lin_vel(&self) -> Vector {
        self.0.lin_vel()
    }
    #[inline]
    fn lin_vel_mut(&mut self) -> &mut Vector {
        self.0.lin_vel_mut()
    }
    #[inline]
    fn ang_vel(&self) -> AngularVector {
        self.0.ang_vel()
    }
    #[inline]
    fn ang_vel_mut(&mut self) -> &mut AngularVector {
        self.0.ang_vel_mut()
    }
    #[inline]
    fn inverse_mass(&self) -> Scalar {
        self.0.inverse_mass()
    }
    #[inline]
    #[cfg(feature = "3d")]
    fn inverse_angular_inertia(&self) -> SymmetricTensor {
        self.0.inverse_angular_inertia()
    }
    #[inline]
    fn global_inverse_angular_inertia(&self) -> SymmetricTensor {
        self.0.global_inverse_angular_inertia()
    }
    #[inline]
    fn global_center_of_mass(&self) -> Vector {
        self.0.global_center_of_mass()
    }
    #[inline]
    fn locked_axes(&self) -> LockedAxes {
        self.0.locked_axes()
    }
    #[inline]
    fn integration_data(&self) -> &VelocityIntegrationData {
        self.0.integration_data()
    }
    #[inline]
    fn integration_data_mut(&mut self) -> &mut VelocityIntegrationData {
        self.0.integration_data_mut()
    }
    #[inline]
    fn accumulated_local_acceleration(&self) -> &AccumulatedLocalAcceleration {
        self.0.accumulated_local_acceleration()
    }
    #[inline]
    fn accumulated_local_acceleration_mut(&mut self) -> &mut AccumulatedLocalAcceleration {
        self.0.accumulated_local_acceleration_mut()
    }
    #[inline]
    fn try_wake_up(&mut self) -> bool {
        // Don't wake up the body.
        // Return `true` if the body is already awake and forces should be applied.
        !self.0.is_sleeping
    }
}
