//! Sets up the physics engine by initializing the necessary schedules, sets and resources.
//!
//! See [`PhysicsSetupPlugin`].

mod time;

use std::time::Duration;

pub use time::*;

use super::sync::PreviousGlobalTransform;
use crate::prelude::*;
use bevy::{
    ecs::schedule::{ExecutorKind, ScheduleBuildSettings},
    prelude::*,
    transform::TransformSystem,
    utils::intern::Interned,
};

/// Sets up the physics engine by initializing the necessary schedules, sets and resources.
///
/// This plugin does *not* initialize any other plugins or physics systems.
/// For that, add the plugins in [`PhysicsPlugins`], or even [create your own plugins](PhysicsPlugins#custom-plugins) using
/// the schedules and sets provided by this setup plugin.
///
/// ## Schedules and sets
///
/// This plugin initializes and configures the following schedules and system sets:
///
/// - [`PhysicsSet`]: High-level system sets for the main phases of the physics engine.
/// You can use these to schedule your own systems before or after physics is run without
/// having to worry about implementation details.
/// - [`PhysicsSchedule`]: Responsible for advancing the simulation in [`PhysicsSet::StepSimulation`].
/// - [`PhysicsStepSet`]: System sets for the steps of the actual physics simulation loop, like
/// the broad phase and the substepping loop.
/// - [`SubstepSchedule`]: Responsible for running the substepping loop in [`PhysicsStepSet::Substeps`].
/// - [`SubstepSet`]: System sets for the steps of the substepping loop, like position integration and
/// the constraint solver.
/// - [`PostProcessCollisions`]: Responsible for running collision post-processing systems.
/// Empty by default.
pub struct PhysicsSetupPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PhysicsSetupPlugin {
    /// Creates a [`PhysicsSetupPlugin`] using the given schedule for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PhysicsSetupPlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

impl Plugin for PhysicsSetupPlugin {
    fn build(&self, app: &mut App) {
        // Init resources and register component types
        app.init_resource::<Time<Physics>>()
            .insert_resource(Time::new_with(Substeps))
            .init_resource::<SubstepCount>()
            .init_resource::<BroadCollisionPairs>()
            .init_resource::<SleepingThreshold>()
            .init_resource::<DeactivationTime>()
            .init_resource::<Gravity>()
            .register_type::<Time<Physics>>()
            .register_type::<Time<Substeps>>()
            .register_type::<SubstepCount>()
            .register_type::<BroadCollisionPairs>()
            .register_type::<SleepingThreshold>()
            .register_type::<DeactivationTime>()
            .register_type::<Gravity>()
            .register_type::<RigidBody>()
            .register_type::<Sleeping>()
            .register_type::<SleepingDisabled>()
            .register_type::<TimeSleeping>()
            .register_type::<Position>()
            .register_type::<Rotation>()
            .register_type::<PreviousPosition>()
            .register_type::<PreviousRotation>()
            .register_type::<PreviousGlobalTransform>()
            .register_type::<AccumulatedTranslation>()
            .register_type::<LinearVelocity>()
            .register_type::<AngularVelocity>()
            .register_type::<PreSolveLinearVelocity>()
            .register_type::<PreSolveAngularVelocity>()
            .register_type::<Restitution>()
            .register_type::<Friction>()
            .register_type::<LinearDamping>()
            .register_type::<AngularDamping>()
            .register_type::<ExternalForce>()
            .register_type::<ExternalTorque>()
            .register_type::<ExternalImpulse>()
            .register_type::<ExternalAngularImpulse>()
            .register_type::<GravityScale>()
            .register_type::<Mass>()
            .register_type::<InverseMass>()
            .register_type::<Inertia>()
            .register_type::<InverseInertia>()
            .register_type::<CenterOfMass>()
            .register_type::<ColliderDensity>()
            .register_type::<ColliderMassProperties>()
            .register_type::<LockedAxes>()
            .register_type::<ColliderParent>()
            .register_type::<Dominance>()
            .register_type::<CollisionLayers>()
            .register_type::<CollidingEntities>()
            .register_type::<CoefficientCombine>()
            .register_type::<Sensor>()
            .register_type::<ColliderTransform>()
            .register_type::<PreviousColliderTransform>();

        // Configure higher level system sets for the given schedule
        let schedule = self.schedule;
        app.configure_sets(
            schedule,
            (
                PhysicsSet::Prepare,
                PhysicsSet::StepSimulation,
                PhysicsSet::Sync,
            )
                .chain()
                .before(TransformSystem::TransformPropagate),
        );

        // Set up the physics schedule, the schedule that advances the physics simulation
        app.edit_schedule(PhysicsSchedule, |schedule| {
            schedule
                .set_executor_kind(ExecutorKind::SingleThreaded)
                .set_build_settings(ScheduleBuildSettings {
                    ambiguity_detection: LogLevel::Error,
                    ..default()
                });

            schedule.configure_sets(
                (
                    PhysicsStepSet::BroadPhase,
                    PhysicsStepSet::Substeps,
                    PhysicsStepSet::ReportContacts,
                    PhysicsStepSet::Sleeping,
                    PhysicsStepSet::SpatialQuery,
                )
                    .chain(),
            );
        });

        app.add_systems(
            schedule,
            run_physics_schedule.in_set(PhysicsSet::StepSimulation),
        );

        // Set up the substep schedule, the schedule that runs the inner substepping loop
        app.edit_schedule(SubstepSchedule, |schedule| {
            schedule
                .set_executor_kind(ExecutorKind::SingleThreaded)
                .set_build_settings(ScheduleBuildSettings {
                    ambiguity_detection: LogLevel::Error,
                    ..default()
                });

            schedule.configure_sets(
                (
                    SubstepSet::Integrate,
                    SubstepSet::NarrowPhase,
                    SubstepSet::PostProcessCollisions,
                    SubstepSet::SolveConstraints,
                    SubstepSet::SolveUserConstraints,
                    SubstepSet::UpdateVelocities,
                    SubstepSet::SolveVelocities,
                    SubstepSet::StoreImpulses,
                    SubstepSet::ApplyTranslation,
                )
                    .chain(),
            );
        });

        app.add_systems(
            PhysicsSchedule,
            run_substep_schedule.in_set(PhysicsStepSet::Substeps),
        );

        // Set up the PostProcessCollisions schedule for user-defined systems
        // that filter and modify collisions.
        app.edit_schedule(PostProcessCollisions, |schedule| {
            schedule
                .set_executor_kind(ExecutorKind::SingleThreaded)
                .set_build_settings(ScheduleBuildSettings {
                    ambiguity_detection: LogLevel::Error,
                    ..default()
                });
        });

        app.add_systems(
            SubstepSchedule,
            run_post_process_collisions_schedule.in_set(SubstepSet::PostProcessCollisions),
        );
    }
}

/// True if a system is running for the first time.
struct IsFirstRun(bool);

impl Default for IsFirstRun {
    fn default() -> Self {
        Self(true)
    }
}

/// Runs the [`PhysicsSchedule`].
fn run_physics_schedule(world: &mut World, mut is_first_run: Local<IsFirstRun>) {
    let _ = world.try_schedule_scope(PhysicsSchedule, |world, schedule| {
        let real_delta = world.resource::<Time<Real>>().delta();
        let old_delta = world.resource::<Time<Physics>>().delta();
        let is_paused = world.resource::<Time<Physics>>().is_paused();
        let old_clock = world.resource::<Time>().as_generic();
        let physics_clock = world.resource_mut::<Time<Physics>>();

        // Get the scaled timestep delta time based on the timestep mode.
        let timestep = match physics_clock.timestep_mode() {
            TimestepMode::Fixed { delta, .. } => delta.mul_f64(physics_clock.relative_speed_f64()),
            TimestepMode::FixedOnce { delta } => delta.mul_f64(physics_clock.relative_speed_f64()),
            TimestepMode::Variable { max_delta } => {
                let scaled_delta = real_delta.mul_f64(physics_clock.relative_speed_f64());
                scaled_delta.min(max_delta)
            }
        };

        // How many steps should be run during this frame.
        // For `TimestepMode::Fixed`, this is computed using the accumulated overstep.
        let mut queued_steps = 1;

        if !is_first_run.0 {
            if let TimestepMode::Fixed {
                delta,
                overstep,
                max_delta_overstep,
            } = world.resource_mut::<Time<Physics>>().timestep_mode_mut()
            {
                // If paused, add the `Physics` delta time, otherwise add real time.
                if is_paused {
                    *overstep += old_delta;
                } else {
                    *overstep += real_delta.min(*max_delta_overstep);
                }

                // Consume as many steps as possible with the fixed `delta`.
                queued_steps = (overstep.as_secs_f64() / delta.as_secs_f64()) as usize;
                *overstep -= delta.mul_f64(queued_steps as f64);
            }
        }

        // Advance physics clock by timestep if not paused.
        if !is_paused {
            world.resource_mut::<Time<Physics>>().advance_by(timestep);
        }

        if world.resource::<Time<Physics>>().delta() >= timestep {
            // Set generic `Time` resource to `Time<Physics>`.
            *world.resource_mut::<Time>() = world.resource::<Time<Physics>>().as_generic();

            // Advance simulation by the number of queued steps.
            for _ in 0..queued_steps {
                trace!("running PhysicsSchedule");
                schedule.run(world);
            }
        }

        // If physics is paused, reset delta time to stop simulation
        // unless users manually advance `Time<Physics>`.
        if is_paused {
            world
                .resource_mut::<Time<Physics>>()
                .advance_by(Duration::ZERO);
        }

        // Set generic `Time` resource back to the clock that was active before physics.
        *world.resource_mut::<Time>() = old_clock;
    });

    is_first_run.0 = false;
}

/// Runs the [`SubstepSchedule`].
fn run_substep_schedule(world: &mut World) {
    let delta = world.resource::<Time<Physics>>().delta();
    let SubstepCount(substeps) = *world.resource::<SubstepCount>();
    let sub_delta = delta.div_f64(substeps as f64);

    let mut sub_delta_time = world.resource_mut::<Time<Substeps>>();
    sub_delta_time.advance_by(sub_delta);

    let _ = world.try_schedule_scope(SubstepSchedule, |world, schedule| {
        for i in 0..substeps {
            trace!("running SubstepSchedule: {i}");
            *world.resource_mut::<Time>() = world.resource::<Time<Substeps>>().as_generic();
            schedule.run(world);
        }
    });

    // Set generic `Time` resource back to `Time<Physics>`.
    // Later, it's set back to the default clock after the `PhysicsSchedule`.
    *world.resource_mut::<Time>() = world.resource::<Time<Physics>>().as_generic();
}

/// Runs the [`PostProcessCollisions`] schedule.
fn run_post_process_collisions_schedule(world: &mut World) {
    trace!("running PostProcessCollisions");
    world.run_schedule(PostProcessCollisions);
}
