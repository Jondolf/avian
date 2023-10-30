//! Sets up the physics engine by initializing the necessary schedules, sets and resources.
//!
//! See [`PhysicsSetupPlugin`].

use bevy::{
    ecs::schedule::{ExecutorKind, ScheduleBuildSettings},
    transform::TransformSystem,
};

use crate::prelude::*;

use super::sync::PreviousGlobalTransform;
use bevy::utils::intern::Interned;

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
        app.init_resource::<PhysicsTimestep>()
            .init_resource::<PhysicsTimescale>()
            .init_resource::<DeltaTime>()
            .init_resource::<SubDeltaTime>()
            .init_resource::<SubstepCount>()
            .init_resource::<BroadCollisionPairs>()
            .init_resource::<SleepingThreshold>()
            .init_resource::<DeactivationTime>()
            .init_resource::<Gravity>()
            .register_type::<PhysicsTimestep>()
            .register_type::<PhysicsTimescale>()
            .register_type::<DeltaTime>()
            .register_type::<SubDeltaTime>()
            .register_type::<SubstepCount>()
            .register_type::<BroadCollisionPairs>()
            .register_type::<SleepingThreshold>()
            .register_type::<DeactivationTime>()
            .register_type::<PhysicsLoop>()
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

        // Check and store if schedule is configured to run in FixedUpdate.
        let fixed_update = schedule == FixedUpdate.intern();

        app.insert_resource(PhysicsLoop {
            fixed_update,
            ..Default::default()
        });

        // Create physics schedule, the schedule that advances the physics simulation
        let mut physics_schedule = Schedule::new(PhysicsSchedule);

        physics_schedule
            .set_executor_kind(ExecutorKind::SingleThreaded)
            .set_build_settings(ScheduleBuildSettings {
                ambiguity_detection: LogLevel::Error,
                ..default()
            });

        physics_schedule.configure_sets(
            (
                PhysicsStepSet::BroadPhase,
                PhysicsStepSet::Substeps,
                PhysicsStepSet::ReportContacts,
                PhysicsStepSet::Sleeping,
                PhysicsStepSet::SpatialQuery,
            )
                .chain(),
        );

        app.add_schedule(physics_schedule);

        app.add_systems(
            schedule,
            run_physics_schedule.in_set(PhysicsSet::StepSimulation),
        );

        // Create substep schedule, the schedule that runs the inner substepping loop
        let mut substep_schedule = Schedule::new(SubstepSchedule);

        substep_schedule
            .set_executor_kind(ExecutorKind::SingleThreaded)
            .set_build_settings(ScheduleBuildSettings {
                ambiguity_detection: LogLevel::Error,
                ..default()
            });

        substep_schedule.configure_sets(
            (
                SubstepSet::Integrate,
                SubstepSet::NarrowPhase,
                SubstepSet::PostProcessCollisions,
                SubstepSet::SolveConstraints,
                SubstepSet::SolveUserConstraints,
                SubstepSet::UpdateVelocities,
                SubstepSet::SolveVelocities,
                SubstepSet::ApplyTranslation,
            )
                .chain(),
        );

        app.add_schedule(substep_schedule);

        app.add_systems(
            PhysicsSchedule,
            run_substep_schedule.in_set(PhysicsStepSet::Substeps),
        );

        // Create the PostProcessCollisions schedule for user-defined systems
        // that filter and modify collisions.
        let mut post_process_collisions_schedule = Schedule::new(PostProcessCollisions);

        post_process_collisions_schedule
            .set_executor_kind(ExecutorKind::SingleThreaded)
            .set_build_settings(ScheduleBuildSettings {
                ambiguity_detection: LogLevel::Error,
                ..default()
            });

        app.add_schedule(post_process_collisions_schedule);

        app.add_systems(
            SubstepSchedule,
            run_post_process_collisions_schedule.in_set(SubstepSet::PostProcessCollisions),
        );
    }
}

/// Controls the physics simulation loop.
///
/// ## Example
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         // `pause` is a provided system that calls `PhysicsLoop::pause`
#[cfg_attr(
    feature = "2d",
    doc = "        .add_systems(Startup, bevy_xpbd_2d::pause)"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        .add_systems(Startup, bevy_xpbd_3d::pause)"
)]
///         .add_systems(Update, step_manually)
///         .run();
/// }
///
/// fn step_manually(input: Res<Input<KeyCode>>, mut physics_loop: ResMut<PhysicsLoop>) {
///     // Advance the simulation by one frame every time Space is pressed
///     if input.just_pressed(KeyCode::Space) {
///         physics_loop.step();
///     }
/// }
/// ```
#[derive(Reflect, Resource, Debug, Default)]
#[reflect(Resource)]
pub struct PhysicsLoop {
    /// Time accumulated into the physics loop. This is consumed by the [`PhysicsSchedule`].
    pub accumulator: Scalar,
    /// Number of steps queued by the user. Time will be added to the accumulator according to the number of queued step.
    pub queued_steps: u32,
    /// If [`PhysicsSchedule`] runs in [`FixedUpdate`]. Determines the delta time for the simulation.
    pub(crate) fixed_update: bool,
    /// Determines if the simulation is paused.
    pub paused: bool,
}

impl PhysicsLoop {
    /// Add a step to be run on the next run of the [`PhysicsSchedule`].
    pub fn step(&mut self) {
        self.queued_steps += 1;
    }
    /// Pause the simulation.
    pub fn pause(&mut self) {
        self.paused = true;
    }
    /// Resume the simulation.
    pub fn resume(&mut self) {
        self.paused = false;
    }
}

/// Pause the simulation.
pub fn pause(mut physics_loop: ResMut<PhysicsLoop>) {
    physics_loop.pause();
}

/// Resume the simulation.
pub fn resume(mut physics_loop: ResMut<PhysicsLoop>) {
    physics_loop.resume();
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
    let mut physics_loop = world
        .remove_resource::<PhysicsLoop>()
        .expect("no PhysicsLoop resource");

    #[cfg(feature = "f32")]
    let mut delta_seconds = if physics_loop.fixed_update {
        world.resource::<Time<Fixed>>().delta_seconds()
    } else {
        world.resource::<Time>().delta_seconds()
    };

    #[cfg(feature = "f64")]
    let mut delta_seconds = if physics_loop.fixed_update {
        world.resource::<Time<Fixed>>().delta_seconds_f64()
    } else {
        world.resource::<Time>().delta_seconds_f64()
    };

    let time_step = *world.resource::<PhysicsTimestep>();
    let time_scale = world.resource::<PhysicsTimescale>().0.max(0.0);

    // Update `DeltaTime` according to the `PhysicsTimestep` configuration
    let (raw_dt, accumulate) = match time_step {
        PhysicsTimestep::Fixed(fixed_delta_seconds) => (fixed_delta_seconds, true),
        PhysicsTimestep::FixedOnce(fixed_delta_seconds) => (fixed_delta_seconds, false),
        PhysicsTimestep::Variable { max_dt } => (delta_seconds.min(max_dt), true),
    };

    // On the first run of the schedule, `delta_seconds` could be 0.0.
    // In that case, replace it with the fixed timestep amount.
    // With a variable timestep, the physics accumulator might not increase
    // until the second run of the system.
    if is_first_run.0 {
        delta_seconds = raw_dt;
        is_first_run.0 = false;
    }

    let dt = raw_dt * time_scale;
    world.resource_mut::<DeltaTime>().0 = dt;

    match accumulate {
        false if physics_loop.paused && physics_loop.queued_steps == 0 => {}
        false => {
            if physics_loop.queued_steps > 0 {
                physics_loop.queued_steps -= 1;
            }
            debug!("running PhysicsSchedule");
            world.run_schedule(PhysicsSchedule);
        }
        true => {
            // Add time to the accumulator
            if physics_loop.paused {
                physics_loop.accumulator += dt * physics_loop.queued_steps as Scalar;
                physics_loop.queued_steps = 0;
            } else {
                physics_loop.accumulator += delta_seconds * time_scale;
            }

            // Step the simulation until the accumulator has been consumed.
            // Note that a small remainder may be passed on to the next run of the physics schedule.
            while physics_loop.accumulator >= dt && dt > 0.0 {
                debug!("running PhysicsSchedule");
                world.run_schedule(PhysicsSchedule);
                physics_loop.accumulator -= dt;
            }
        }
    }

    world.insert_resource(physics_loop);
}

/// Runs the [`SubstepSchedule`].
fn run_substep_schedule(world: &mut World) {
    let SubstepCount(substeps) = *world.resource::<SubstepCount>();
    let dt = world.resource::<DeltaTime>().0;

    // Update `SubDeltaTime`
    let mut sub_delta_time = world.resource_mut::<SubDeltaTime>();
    sub_delta_time.0 = dt / substeps as Scalar;

    for i in 0..substeps {
        debug!("running SubstepSchedule: {i}");
        world.run_schedule(SubstepSchedule);
    }
}

/// Runs the [`PostProcessCollisions`] schedule.
fn run_post_process_collisions_schedule(world: &mut World) {
    debug!("running PostProcessCollisions");
    world.run_schedule(PostProcessCollisions);
}
