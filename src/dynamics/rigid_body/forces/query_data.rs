use crate::{
    dynamics::{integrator::VelocityIntegrationData, solver::solver_body::SolverBodyInertia},
    prelude::{mass_properties::components::GlobalCenterOfMass, *},
};
use bevy::ecs::{
    query::QueryData,
    system::lifetimeless::{Read, Write},
};

use super::AccumulatedLocalAcceleration;

/// A helper [`QueryData`] for applying forces, torques, impulses, and accelerations to dynamic [rigid bodies](RigidBody).
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
#[cfg_attr(feature = "serialize", doc = "# use bevy::prelude::*;")]
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
/// [`Forces`] can also apply forces and impulses at a specific point in the world. If the point is not aligned
/// with the [`GlobalCenterOfMass`], it will also apply a torque to the body.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::{math::Vector, prelude::*};")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::{math::Vector, prelude::*};")]
#[cfg_attr(feature = "serialize", doc = "# use bevy::prelude::*;")]
/// #
/// # fn apply_impulses(mut query: Query<Forces>) {
/// #     for mut forces in &mut query {
/// #         let force = Vector::default();
/// #         let point = Vector::default();
/// // Apply an impulse at a specific point in the world.
/// // Unlike forces, impulses are applied immediately to the velocity,
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
    inertia: Read<SolverBodyInertia>,
    integration: Write<VelocityIntegrationData>,
    accumulated_local_acceleration: Write<AccumulatedLocalAcceleration>,
}

impl ForcesItem<'_> {
    /// Applies a force at the center of mass in world space. The unit is typically N or kg⋅m/s².
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_force(&mut self, force: Vector) {
        self.integration.linear_increment += self.mass.inverse() * force;
    }

    /// Applies a force at the given point in world space. The unit is typically N or kg⋅m/s².
    ///
    /// If the point is not at the center of mass, the force will also generate a torque.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_force_at_point(&mut self, force: Vector, world_point: Vector) {
        // Note: This does not consider the rotation of the body during substeps,
        //       so the torque may not be accurate if the body is rotating quickly.
        self.apply_force(force);
        self.apply_torque(cross(world_point - self.global_center_of_mass.get(), force));
    }

    /// Applies a force at the center of mass in local space. The unit is typically N or kg⋅m/s².
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_local_force(&mut self, force: Vector) {
        self.accumulated_local_acceleration.linear += self.mass.inverse() * force;
    }

    /// Applies a torque in world space. The unit is typically N⋅m or kg⋅m²/s².
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_torque(&mut self, torque: AngularVector) {
        self.integration.angular_increment += self.inertia.effective_inv_angular_inertia() * torque;
    }

    /// Applies a torque in local space. The unit is typically N⋅m or kg⋅m²/s².
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    ///
    /// **Note:** This does not consider the rotation of the body during substeps,
    ///           so the torque may not be accurate if the body is rotating quickly.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn apply_local_torque(&mut self, torque: AngularVector) {
        self.accumulated_local_acceleration.angular += self.angular_inertia.inverse() * torque;
    }

    /// Applies a linear impulse at the center of mass in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    #[inline]
    pub fn apply_linear_impulse(&mut self, impulse: Vector) {
        self.linear_velocity.0 += self.inertia.effective_inv_mass() * impulse;
    }

    /// Applies a linear impulse at the given point in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// If the point is not at the center of mass, the impulse will also generate an angular impulse.
    ///
    /// The impulse modifies the [`LinearVelocity`] and [`AngularVelocity`] of the body immediately.
    #[inline]
    pub fn apply_linear_impulse_at_point(&mut self, impulse: Vector, world_point: Vector) {
        self.apply_linear_impulse(impulse);
        self.apply_angular_impulse(cross(
            world_point - self.global_center_of_mass.get(),
            impulse,
        ));
    }

    /// Applies a linear impulse in local space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    #[inline]
    pub fn apply_local_linear_impulse(&mut self, impulse: Vector) {
        let world_impulse = self.rotation * impulse;
        self.apply_linear_impulse(world_impulse);
    }

    /// Applies an angular impulse in world space. The unit is typically N⋅m⋅s or kg⋅m²/s.
    ///
    /// The impulse modifies the [`AngularVelocity`] of the body immediately.
    #[inline]
    pub fn apply_angular_impulse(&mut self, impulse: AngularVector) {
        self.angular_velocity.0 += self.inertia.effective_inv_angular_inertia() * impulse;
    }

    /// Applies an angular impulse in local space. The unit is typically N⋅m⋅s or kg⋅m²/s.
    ///
    /// The impulse modifies the [`AngularVelocity`] of the body immediately.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn apply_local_angular_impulse(&mut self, impulse: AngularVector) {
        let world_impulse = self.rotation * impulse;
        self.apply_angular_impulse(world_impulse);
    }

    /// Applies a linear acceleration, ignoring mass. The unit is typically m/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_linear_acceleration(&mut self, acceleration: Vector) {
        self.integration.apply_linear_acceleration(acceleration);
    }

    /// Applies a linear acceleration in local space, ignoring mass. The unit is typically m/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_local_linear_acceleration(&mut self, acceleration: Vector) {
        self.accumulated_local_acceleration.linear += acceleration;
    }

    /// Applies an angular acceleration, ignoring angular inertia. The unit is rad/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_angular_acceleration(&mut self, acceleration: AngularVector) {
        self.integration.apply_angular_acceleration(acceleration);
    }

    /// Applies an angular acceleration in local space, ignoring angular inertia. The unit is rad/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn apply_local_angular_acceleration(&mut self, acceleration: AngularVector) {
        self.accumulated_local_acceleration.angular += acceleration;
    }

    /// Returns the linear acceleration that the body has accumulated
    /// before the physics step in world space, including acceleration
    /// caused by forces.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only forces and accelerations applied through [`Forces`] are included.
    #[inline]
    pub fn accumulated_linear_acceleration(&self) -> Vector {
        // The linear increment is treated as linear acceleration until the integration step.
        let world_linear_acceleration = self.integration.linear_increment;
        let local_linear_acceleration = self.accumulated_local_acceleration.linear;

        // Return the total world-space linear acceleration.
        self.inertia
            .flags()
            .locked_axes()
            .apply_to_vec(world_linear_acceleration + self.rotation * local_linear_acceleration)
    }

    /// Returns the angular acceleration that the body has accumulated
    /// before the physics step in world space, including acceleration
    /// caused by torques.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only torques and accelerations applied through [`Forces`] are included.
    #[cfg(feature = "2d")]
    #[inline]
    pub fn accumulated_angular_acceleration(&self) -> AngularVector {
        // The angular increment is treated as angular acceleration until the integration step.
        self.inertia
            .flags()
            .locked_axes()
            .apply_to_angular_velocity(self.integration.angular_increment)
    }

    /// Returns the angular acceleration that the body has accumulated
    /// before the physics step in world space, including acceleration
    /// caused by torques.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only torques and accelerations applied through [`Forces`] are included.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn accumulated_angular_acceleration(&self) -> AngularVector {
        // The angular increment is treated as angular acceleration until the integration step.
        let world_angular_acceleration = self.integration.angular_increment;
        let local_angular_acceleration = self.accumulated_local_acceleration.angular;

        // Return the total world-space angular acceleration.
        self.inertia
            .flags()
            .locked_axes()
            .apply_to_angular_velocity(
                world_angular_acceleration + self.rotation * local_angular_acceleration,
            )
    }

    /// Resets the accumulated linear acceleration to zero.
    #[inline]
    pub fn reset_accumulated_linear_acceleration(&mut self) {
        self.integration.linear_increment = Vector::ZERO;
        self.accumulated_local_acceleration.linear = Vector::ZERO;
    }

    /// Resets the accumulated angular acceleration to zero.
    #[inline]
    pub fn reset_accumulated_angular_acceleration(&mut self) {
        self.integration.angular_increment = AngularVector::ZERO;
        #[cfg(feature = "3d")]
        {
            self.accumulated_local_acceleration.angular = AngularVector::ZERO;
        }
    }

    /// Returns the [`LinearVelocity`] of the body in world space.
    #[inline]
    pub fn linear_velocity(&self) -> Vector {
        self.linear_velocity.0
    }

    /// Returns a mutable reference to the [`LinearVelocity`] of the body in world space.
    #[inline]
    pub fn linear_velocity_mut(&mut self) -> &mut Vector {
        &mut self.linear_velocity.0
    }

    /// Returns the [`AngularVelocity`] of the body in world space.
    #[inline]
    pub fn angular_velocity(&self) -> AngularVector {
        self.angular_velocity.0
    }

    /// Returns a mutable reference to the [`AngularVelocity`] of the body in world space.
    #[inline]
    pub fn angular_velocity_mut(&mut self) -> &mut AngularVector {
        &mut self.angular_velocity.0
    }
}
