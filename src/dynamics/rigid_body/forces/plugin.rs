use crate::{
    dynamics::{
        integrator::{self, IntegrationSet, VelocityIntegrationData},
        solver::{
            solver_body::{SolverBody, SolverBodyInertia},
            SolverDiagnostics,
        },
    },
    prelude::*,
};
use bevy::prelude::*;

use super::{AccumulatedLocalAcceleration, AccumulatedLocalForces, AccumulatedWorldForces};

/// A plugin for managing and applying external forces, torques, and accelerations for [rigid bodies](RigidBody).
///
/// See the [module-level documentation](crate::dynamics::rigid_body::forces) for more general information about forces in Avian.
pub struct ForcePlugin;

impl Plugin for ForcePlugin {
    fn build(&self, app: &mut App) {
        // Register types.
        app.register_type::<(
            ConstantForce,
            ConstantTorque,
            ConstantLinearAcceleration,
            ConstantAngularAcceleration,
            ConstantLocalForce,
            ConstantLocalTorque,
            ConstantLocalLinearAcceleration,
            ConstantLocalAngularAcceleration,
            AccumulatedWorldForces,
            AccumulatedLocalForces,
            AccumulatedLocalAcceleration,
        )>();

        // Set up system sets.
        app.configure_sets(
            PhysicsSchedule,
            (
                (ForceSet::ApplyConstantForces, ForceSet::ApplyWorldForces)
                    .chain()
                    .in_set(IntegrationSet::UpdateVelocityIncrements)
                    .before(integrator::pre_process_velocity_increments),
                ForceSet::Clear.in_set(SolverSet::PostSubstep),
            ),
        );
        app.configure_sets(
            SubstepSchedule,
            ForceSet::ApplyLocalForces
                .in_set(IntegrationSet::Velocity)
                .before(integrator::integrate_velocities),
        );

        // Accumulate constant forces, torques, and accelerations.
        app.add_systems(
            PhysicsSchedule,
            (
                apply_constant_forces,
                apply_constant_torques,
                apply_constant_linear_acceleration,
                apply_constant_angular_acceleration,
                apply_constant_local_forces,
                apply_constant_local_torques,
                apply_constant_local_linear_acceleration,
                apply_constant_local_angular_acceleration,
            )
                .chain()
                .in_set(ForceSet::ApplyConstantForces),
        );

        // Apply world forces.
        app.add_systems(
            PhysicsSchedule,
            apply_world_forces.in_set(ForceSet::ApplyWorldForces),
        );

        // Apply local forces and accelerations.
        // This is done in the substepping loop, because the orientations of bodies can change between substeps.
        app.add_systems(
            SubstepSchedule,
            (apply_local_forces, apply_local_acceleration)
                .chain()
                .in_set(ForceSet::ApplyLocalForces),
        );

        // Clear accumulated forces and accelerations.
        app.add_systems(
            PhysicsSchedule,
            (
                clear_accumulated_world_forces,
                clear_accumulated_local_forces,
                clear_accumulated_local_acceleration,
            )
                .in_set(ForceSet::Clear),
        );
    }
}

/// System sets for managing and applying forces, torques, and accelerations for [rigid bodies](RigidBody).
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ForceSet {
    /// Adds [`ConstantForce`], [`ConstantTorque`], [`ConstantLinearAcceleration`], and [`ConstantAngularAcceleration`]
    /// to [`AccumulatedWorldForces`] and [`ConstantLocalForce`], [`ConstantLocalTorque`], [`ConstantLocalLinearAcceleration`],
    /// and [`ConstantLocalAngularAcceleration`] to [`AccumulatedLocalForces`].
    ApplyConstantForces,
    /// Applies [`AccumulatedWorldForces`] to the linear and angular velocities of bodies.
    ApplyWorldForces,
    /// Applies [`AccumulatedLocalForces`] to the linear and angular velocities of bodies.
    ApplyLocalForces,
    /// Clears [`AccumulatedWorldForces`], [`AccumulatedLocalForces`], and [`AccumulatedLocalAcceleration`] for all bodies.
    Clear,
}

/// Applies [`ConstantForce`] to the accumulated forces.
fn apply_constant_forces(mut bodies: Query<(&mut AccumulatedWorldForces, &ConstantForce)>) {
    for (mut forces, constant_force) in &mut bodies {
        forces.force += constant_force.0;
    }
}

/// Applies [`ConstantTorque`] to the accumulated torques.
fn apply_constant_torques(mut bodies: Query<(&mut AccumulatedWorldForces, &ConstantTorque)>) {
    for (mut forces, constant_torque) in &mut bodies {
        forces.torque += constant_torque.0;
    }
}

/// Applies [`ConstantLinearAcceleration`] to the linear velocity increments.
fn apply_constant_linear_acceleration(
    mut bodies: Query<(&mut VelocityIntegrationData, &ConstantLinearAcceleration)>,
) {
    for (mut integration, constant_acceleration) in &mut bodies {
        integration.linear_increment += constant_acceleration.0;
    }
}

/// Applies [`ConstantAngularAcceleration`] to the angular velocity increments.
fn apply_constant_angular_acceleration(
    mut bodies: Query<(&mut VelocityIntegrationData, &ConstantAngularAcceleration)>,
) {
    for (mut integration, constant_acceleration) in &mut bodies {
        integration.angular_increment += constant_acceleration.0;
    }
}

/// Applies [`ConstantLocalForce`] to the accumulated forces.
fn apply_constant_local_forces(
    mut bodies: Query<(&mut AccumulatedLocalForces, &ConstantLocalForce)>,
) {
    for (mut forces, constant_force) in &mut bodies {
        forces.force += constant_force.0;
    }
}

/// Applies [`ConstantLocalTorque`] to the accumulated torques.
fn apply_constant_local_torques(
    mut bodies: Query<(&mut AccumulatedLocalForces, &ConstantLocalTorque)>,
) {
    for (mut forces, constant_torque) in &mut bodies {
        forces.torque += constant_torque.0;
    }
}

/// Applies [`ConstantLocalLinearAcceleration`] to the accumulated local acceleration.
fn apply_constant_local_linear_acceleration(
    mut bodies: Query<(
        &mut AccumulatedLocalAcceleration,
        &ConstantLocalLinearAcceleration,
    )>,
) {
    for (mut acceleration, constant_acceleration) in &mut bodies {
        acceleration.linear += constant_acceleration.0;
    }
}

/// Applies [`ConstantLocalAngularAcceleration`] to the accumulated local acceleration.
fn apply_constant_local_angular_acceleration(
    mut bodies: Query<(
        &mut AccumulatedLocalAcceleration,
        &ConstantLocalAngularAcceleration,
    )>,
) {
    for (mut acceleration, constant_acceleration) in &mut bodies {
        acceleration.angular += constant_acceleration.0;
    }
}

/// Applies [`AccumulatedWorldForces`] to the linear and angular velocity of bodies.
fn apply_world_forces(
    mut bodies: Query<(
        &mut VelocityIntegrationData,
        &AccumulatedWorldForces,
        &SolverBodyInertia,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    // TODO: Do we want to skip kinematic bodies here?
    bodies
        .par_iter_mut()
        .for_each(|(mut integration, forces, mass_props)| {
            // NOTE: The velocity increments are treated as accelerations at this point.

            // Apply external forces and torques.
            // NOTE: We ignore changes in the inertia tensor, keeping angular acceleration constant across substeps.
            //       This may not be entirely correct, but is generally acceptable for world-space torque.
            integration.linear_increment += mass_props.effective_inv_mass() * forces.force;
            integration.angular_increment +=
                mass_props.effective_inv_angular_inertia() * forces.torque;

            // The `IntegrationPlugin` will take care of applying the time step
            // and locked axes to the velocity increments.
        });

    diagnostics.update_velocity_increments += start.elapsed();
}

/// Applies [`AccumulatedLocalForces`] to the linear and angular velocity of bodies.
fn apply_local_forces(
    mut bodies: Query<
        (
            &mut SolverBody,
            &AccumulatedLocalForces,
            &Rotation,
            &SolverBodyInertia,
        ),
        With<SolverBody>,
    >,
    time: Res<Time<Substeps>>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_secs_f64() as Scalar;

    bodies
        .par_iter_mut()
        .for_each(|(mut body, forces, rotation, mass_props)| {
            let rotation = body.delta_rotation * *rotation;
            let locked_axes = body.flags.locked_axes();

            // NOTE:
            //
            // We could have a `LocalVelocityIncrements` component and apply these forces and torques
            // to that once per time step rather than once per substep. However, that would have two caveats:
            //
            // 1. We need to store and manage both `AccumulatedLocalForces` and `LocalVelocityIncrements`
            //    when local forces are applied.
            // 2. Changes in the inertia tensor during substeps would not be considered.
            //    (though we currently accept this for world-space torque)

            // Compute the world-space accelerations with locked axes applied.
            let linear_acceleration = locked_axes
                .apply_to_vec(mass_props.effective_inv_mass() * (rotation * forces.force));
            #[cfg(feature = "3d")]
            let angular_acceleration = locked_axes.apply_to_angular_velocity(
                mass_props.effective_inv_angular_inertia() * (rotation * forces.torque),
            );

            // Apply external forces and torques.
            body.linear_velocity += linear_acceleration * delta_secs;
            #[cfg(feature = "3d")]
            {
                body.angular_velocity += angular_acceleration * delta_secs;
            }
        });

    diagnostics.integrate_velocities += start.elapsed();
}

/// Applies [`AccumulatedLocalAcceleration`] to the linear and angular velocity of bodies.
///
/// This should run in the substepping loop, just before [`IntegrationSet::Velocity`].
fn apply_local_acceleration(
    mut bodies: Query<(
        &mut SolverBody,
        &AccumulatedLocalAcceleration,
        &Rotation,
        Option<&LockedAxes>,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
    time: Res<Time<Substeps>>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_secs_f64() as Scalar;

    bodies
        .par_iter_mut()
        .for_each(|(mut body, acceleration, rotation, locked_axes)| {
            let rotation = body.delta_rotation * *rotation;
            let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

            // Compute the world space velocity increments with locked axes applied.
            let world_linear_acceleration =
                locked_axes.apply_to_vec(rotation * acceleration.linear);
            #[cfg(feature = "3d")]
            let world_angular_acceleration =
                locked_axes.apply_to_vec(rotation * acceleration.angular);

            // Apply acceleration.
            body.linear_velocity += world_linear_acceleration * delta_secs;
            #[cfg(feature = "3d")]
            {
                body.angular_velocity += world_angular_acceleration * delta_secs;
            }
        });

    diagnostics.integrate_velocities += start.elapsed();
}

/// Clears [`AccumulatedWorldForces`] for all rigid bodies.
///
/// Continuously applied forces and torques are only reset to zero,
/// while forces and torques that were already zero for an entire time step
/// are removed from the entities.
fn clear_accumulated_world_forces(
    mut commands: Commands,
    mut query: Query<(
        Entity,
        &mut AccumulatedWorldForces,
        Has<ConstantForce>,
        Has<ConstantTorque>,
    )>,
) {
    // Initialize a buffer for entities to remove the component from.
    let mut entity_buffer = Vec::new();

    for (entity, mut forces, has_constant_force, has_constant_torque) in &mut query {
        if forces.force != Vector::ZERO || forces.torque != AngularVector::ZERO {
            // The force or torque was not zero, so these may be continuously applied forces.
            // Just reset the forces and keep the component.
            forces.force = Vector::ZERO;
            forces.torque = AngularVector::ZERO;
        } else if !has_constant_force && !has_constant_torque {
            // No forces or torques were applied for an entire time step, so we can remove the component.
            entity_buffer.push(entity);
        }
    }

    if entity_buffer.is_empty() {
        return;
    }

    // Remove the component from all entities that had no forces or torques applied.
    commands.queue(|world: &mut World| {
        entity_buffer.into_iter().for_each(|entity| {
            world.entity_mut(entity).remove::<AccumulatedWorldForces>();
        });
    });
}

/// Clears [`AccumulatedLocalForces`] for all rigid bodies.
///
/// Continuously applied forces and torques are only reset to zero,
/// while forces and torques that were already zero for an entire time step
/// are removed from the entities.
fn clear_accumulated_local_forces(
    mut commands: Commands,
    mut query: Query<(
        Entity,
        &mut AccumulatedLocalForces,
        Has<ConstantLocalForce>,
        Has<ConstantLocalTorque>,
    )>,
) {
    // Initialize a buffer for entities to remove the component from.
    let mut entity_buffer = Vec::new();

    for (entity, mut forces, has_constant_force, has_constant_torque) in &mut query {
        if forces.force != Vector::ZERO || forces.torque != AngularVector::ZERO {
            // The force or torque was not zero, so these may be continuously applied forces.
            // Just reset the forces and keep the component.
            forces.force = Vector::ZERO;
            forces.torque = AngularVector::ZERO;
        } else if !has_constant_force && !has_constant_torque {
            // No forces or torques were applied for an entire time step, so we can remove the component.
            entity_buffer.push(entity);
        }
    }

    if entity_buffer.is_empty() {
        return;
    }

    // Remove the component from all entities that had no forces or torques applied.
    commands.queue(|world: &mut World| {
        entity_buffer.into_iter().for_each(|entity| {
            world.entity_mut(entity).remove::<AccumulatedLocalForces>();
        });
    });
}

fn clear_accumulated_local_acceleration(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AccumulatedLocalAcceleration)>,
) {
    // Initialize a buffer for entities to remove the component from.
    let mut entity_buffer = Vec::new();

    for (entity, mut acceleration) in &mut query {
        #[cfg(feature = "2d")]
        let non_zero_acceleration = acceleration.linear != Vector::ZERO;
        #[cfg(feature = "3d")]
        let non_zero_acceleration =
            acceleration.linear != Vector::ZERO || acceleration.angular != Vector::ZERO;
        if non_zero_acceleration {
            // The acceleration was not zero, so this may be a continuously applied acceleration.
            // Just reset the acceleration and keep the component.
            acceleration.linear = Vector::ZERO;
            #[cfg(feature = "3d")]
            {
                acceleration.angular = Vector::ZERO;
            }
        } else {
            // No acceleration was applied for an entire time step, so we can remove the component.
            entity_buffer.push(entity);
        }
    }

    if entity_buffer.is_empty() {
        return;
    }

    // Remove the component from all entities that had no acceleration applied.
    commands.queue(|world: &mut World| {
        entity_buffer.into_iter().for_each(|entity| {
            world
                .entity_mut(entity)
                .remove::<AccumulatedLocalAcceleration>();
        });
    });
}
