//! Forces, torques, linear impulses, and angular impulses
//! that can be applied to dynamic rigid bodies.

#![allow(missing_docs)]

use crate::{
    dynamics::{
        integrator::VelocityIntegrationData,
        solver::solver_body::{SolverBody, SolverBodyInertia},
    },
    prelude::*,
};
use bevy::{
    ecs::{
        query::QueryData,
        system::lifetimeless::{Read, Write},
    },
    prelude::*,
};

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

// TODO: Docs
#[derive(QueryData)]
#[query_data(mutable)]
pub struct RigidBodyForces {
    position: Read<Position>,
    rotation: Read<Rotation>,
    linear_velocity: Write<LinearVelocity>,
    angular_velocity: Write<AngularVelocity>,
    mass_props: Read<SolverBodyInertia>,
    center_of_mass: Read<ComputedCenterOfMass>,
    locked_axes: Option<Read<LockedAxes>>,
    integration: Write<VelocityIntegrationData>,
    accumulated_forces: Write<AccumulatedForces>,
}

// TODO: Helpers for velocity
impl RigidBodyForcesItem<'_> {
    /// Applies a linear impulse in local space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    pub fn apply_local_linear_impulse(&mut self, impulse: Vector) {
        let world_impulse = *self.rotation * impulse;
        self.apply_linear_impulse(world_impulse);
    }

    /// Applies a linear impulse at the center of mass in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    pub fn apply_linear_impulse(&mut self, impulse: Vector) {
        self.linear_velocity.0 += self.mass_props.effective_inv_mass() * impulse;
    }

    /// Applies a linear impulse at the given point in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// If the point is not at the center of mass, the impulse will also generate an angular impulse.
    ///
    /// The impulse modifies the [`LinearVelocity`] and [`AngularVelocity`] of the body immediately.
    pub fn apply_linear_impulse_at_point(&mut self, impulse: Vector, world_point: Vector) {
        let world_center_of_mass = self.position.0 + *self.rotation * self.center_of_mass.0;
        self.apply_linear_impulse(impulse);
        self.apply_angular_impulse(cross(world_point - world_center_of_mass, impulse));
    }

    /// Applies an angular impulse in world space. The unit is typically N⋅m⋅s or kg⋅m^2/s.
    ///
    /// The impulse modifies the [`AngularVelocity`] of the body immediately.
    pub fn apply_angular_impulse(&mut self, impulse: Torque) {
        self.angular_velocity.0 += self.mass_props.effective_inv_angular_inertia() * impulse;
    }

    /// Applies a force at the center of mass in local space. The unit is typically N or kg⋅m/s^2.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    // TODO: Do we also want `apply_local_force`? It's not clear what the point should be relative to.
    pub fn apply_local_force(&mut self, force: Vector) {
        let world_force = *self.rotation * force;
        self.apply_force(world_force);
    }

    /// Applies a force at the center of mass in world space. The unit is typically N or kg⋅m/s^2.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    pub fn apply_force(&mut self, force: Vector) {
        self.accumulated_forces.force += force;
    }

    /// Applies a force at the given point in world space. The unit is typically N or kg⋅m/s^2.
    ///
    /// If the point is not at the center of mass, the force will also generate a torque.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    pub fn apply_force_at_point(&mut self, force: Vector, world_point: Vector) {
        // Note: This does not consider the rotation of the body during substeps,
        //       so the torque may not be accurate if the body is rotating quickly.
        let world_center_of_mass = self.position.0 + *self.rotation * self.center_of_mass.0;
        self.apply_force(force);
        self.apply_torque(cross(world_point - world_center_of_mass, force));
    }

    /// Applies a torque in local space. The unit is typically N⋅m or kg⋅m^2/s^2.
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    ///
    /// **Note:** This does not consider the rotation of the body during substeps,
    ///           so the torque may not be accurate if the body is rotating quickly.
    #[cfg(feature = "3d")]
    pub fn apply_local_torque(&mut self, torque: Torque) {
        // NOTE: This does not consider the rotation of the body during substeps,
        //       so the torque may not be accurate if the body is rotating quickly.
        let world_torque = self.rotation.0 * torque;
        self.apply_torque(world_torque);
    }

    /// Applies a torque in world space. The unit is typically N⋅m or kg⋅m^2/s^2.
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    pub fn apply_torque(&mut self, torque: Torque) {
        self.accumulated_forces.torque += torque;
    }

    /// Applies a linear acceleration in local space, ignoring mass. The unit is typically m/s^2.
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    pub fn apply_local_linear_acceleration(&mut self, acceleration: Vector) {
        // NOTE: This does not consider the rotation of the body during substeps,
        //       so the torque may not be accurate if the body is rotating quickly.
        let world_acceleration = *self.rotation * acceleration;
        self.apply_linear_acceleration(world_acceleration);
    }

    /// Applies a linear acceleration, ignoring mass. The unit is typically m/s^2.
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    pub fn apply_linear_acceleration(&mut self, acceleration: Vector) {
        // TODO: Technically we don't need to apply locked axes here since it's applied in the solver.
        let locked_axes = self.locked_axes.copied().unwrap_or_default();
        self.integration
            .apply_linear_acceleration(locked_axes.apply_to_vec(acceleration));
    }

    /// Applies an angular acceleration in local space, ignoring angular inertia. The unit is rad/s^2.
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[cfg(feature = "3d")]
    pub fn apply_local_angular_acceleration(
        &mut self,
        #[cfg(feature = "2d")] acceleration: f32,
        #[cfg(feature = "3d")] acceleration: Vector,
    ) {
        // NOTE: This doesn't consider rotation changes during substeps.
        let world_acceleration = *self.rotation * acceleration;
        self.apply_angular_acceleration(world_acceleration);
    }

    /// Applies an angular acceleration, ignoring angular inertia. The unit is rad/s^2.
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    pub fn apply_angular_acceleration(
        &mut self,
        #[cfg(feature = "2d")] acceleration: f32,
        #[cfg(feature = "3d")] acceleration: Vector,
    ) {
        // TODO: Technically we don't need to apply locked axes here since it's applied in the solver.
        let locked_axes = self.locked_axes.copied().unwrap_or_default();
        self.integration
            .apply_angular_acceleration(locked_axes.apply_to_angular_velocity(acceleration));
    }

    /// Returns the external forces that the body has accumulated before the physics step.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only forces applied through [`RigidBodyForces`] are included.
    pub fn accumulated_force(&self) -> Vector {
        self.accumulated_forces.force
    }

    /// Returns the external torque that the body has accumulated before the physics step.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only torques applied through [`RigidBodyForces`] are included.
    pub fn accumulated_torque(&self) -> Torque {
        self.accumulated_forces.torque
    }

    /// Returns the linear acceleration that the body has accumulated before the physics step.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only accelerations applied through [`RigidBodyForces`] are included.
    pub fn accumulated_linear_acceleration(&self) -> Vector {
        // The linear increment is treated as linear acceleration until the integration step.
        self.integration.linear_increment
    }

    /// Returns the angular acceleration that the body has accumulated before the physics step.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only accelerations applied through [`RigidBodyForces`] are included.
    pub fn accumulated_angular_acceleration(&self) -> Torque {
        // The angular increment is treated as angular acceleration until the integration step.
        self.integration.angular_increment
    }

    /// Resets the accumulated forces to zero.
    pub fn reset_accumulated_force(&mut self) {
        self.accumulated_forces.force = Vector::ZERO;
    }

    /// Resets the accumulated torque to zero.
    pub fn reset_accumulated_torque(&mut self) {
        self.accumulated_forces.torque = Torque::ZERO;
    }

    /// Resets the accumulated linear acceleration to zero.
    pub fn reset_accumulated_linear_acceleration(&mut self) {
        self.integration.linear_increment = Vector::ZERO;
    }

    /// Resets the accumulated angular acceleration to zero.
    pub fn reset_accumulated_angular_acceleration(&mut self) {
        self.integration.angular_increment = Torque::ZERO;
    }
}

// TODO: Should we insert this when a force is applied, and automatically
//       remove it when forces are no longer applied?
/// A component with the accumulated user-applied forces and torques of a rigid body.
#[derive(Component, Debug, Default, PartialEq, Reflect)]
pub struct AccumulatedForces {
    pub force: Vector,
    pub torque: Torque,
}

pub struct ForcePlugin;

impl Plugin for ForcePlugin {
    fn build(&self, app: &mut App) {
        // Add `AccumulatedForces` to all `SolverBody`s.
        app.register_required_components::<SolverBody, AccumulatedForces>();

        app.register_type::<AccumulatedForces>();

        app.configure_sets(
            PhysicsSchedule,
            ForceSet::Clear.in_set(SolverSet::PostSubstep),
        );

        app.add_systems(
            PhysicsSchedule,
            clear_accumulated_forces.in_set(ForceSet::Clear),
        );
    }
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ForceSet {
    Clear,
}

fn clear_accumulated_forces(mut query: Query<&mut AccumulatedForces>) {
    for mut forces in &mut query {
        forces.force = Vector::ZERO;
        forces.torque = Torque::ZERO;
    }
}
