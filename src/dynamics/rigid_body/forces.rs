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
    // These components are mostly internal and not too user-facing,
    // so they are unlikely to conflict with typical user queries.
    position: Read<Position>,
    rotation: Read<Rotation>,
    body: Write<SolverBody>,
    mass_props: Read<SolverBodyInertia>,
    locked_axes: Option<Read<LockedAxes>>,
    integration: Write<VelocityIntegrationData>,
    accumulated_forces: Write<AccumulatedForces>,
}

// TODO
// - Local forces, torques, impulses, and acceleration.
// - Getters for the accumulated forces, torques, and accelerations.
// - Setters for the accumulated forces, torques, and accelerations.
// - Methods to clear the accumulated forces, torques, and accelerations.
impl RigidBodyForcesItem<'_> {
    /// Applies a linear impulse at the given point in world space.
    ///
    /// If the point is not at the center of mass, the impulse will also apply an angular impulse.
    ///
    /// The impulse is typically in Newton-seconds, kg*m/s.
    pub fn apply_linear_impulse(&mut self, impulse: Vector, global_point: Vector) {
        self.apply_linear_center_impulse(impulse);
        self.apply_angular_impulse(cross(global_point - self.position.0, impulse));
    }

    /// Applies a linear impulse at the center of mass of the given entity.
    ///
    /// The impulse is typically in Newton-seconds, kg*m/s.
    pub fn apply_linear_center_impulse(&mut self, impulse: Vector) {
        self.body.linear_velocity += self.mass_props.effective_inv_mass() * impulse;
    }

    /// Applies an angular impulse to the given entity.
    ///
    /// The impulse is typically in Newton-meter-seconds, kg*m^2/s.
    pub fn apply_angular_impulse(&mut self, impulse: Torque) {
        self.body.angular_velocity += self.mass_props.effective_inv_angular_inertia() * impulse;
    }

    /// Applies a force at the given point in world space.
    ///
    /// If the point is not at the center of mass, the force will also apply a torque.
    ///
    /// The force is typically in Newtons, kg*m/s^2.
    pub fn apply_force(&mut self, force: Vector, global_point: Vector) {
        self.apply_center_force(force);
        self.apply_torque(cross(global_point - self.position.0, force));
    }

    /// Applies a force at the center of mass of the given entity.
    ///
    /// The force is typically in Newtons, kg*m/s^2.
    pub fn apply_center_force(&mut self, force: Vector) {
        self.accumulated_forces.force += force;
    }

    /// Applies a torque to the given entity.
    ///
    /// The torque is typically in Newton-meters, kg*m^2/s^2.
    pub fn apply_torque(&mut self, torque: Torque) {
        self.accumulated_forces.torque += torque;
    }

    /// Applies a linear acceleration to the given entity, ignoring mass.
    ///
    /// The acceleration is typically in m/s^2.
    pub fn apply_linear_acceleration(&mut self, acceleration: Vector) {
        // TODO: Technically we don't need to apply locked axes here since it's applied in the solver.
        let locked_axes = self.locked_axes.copied().unwrap_or_default();
        self.integration
            .apply_linear_acceleration(locked_axes.apply_to_vec(acceleration));
    }

    /// Applies an angular acceleration to the given entity, ignoring angular inertia.
    ///
    /// The acceleration is in rad/s^2.
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
