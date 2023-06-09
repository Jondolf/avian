//! Sets up the physics engine by initializing the necessary schedules, sets and resources. See [`PhysicsSetupPlugin`].

use crate::prelude::*;

/// Sets up the physics engine by initializing the necessary schedules, sets and resources.
///
/// This plugin does *not* initialize any other plugins or physics systems.
/// For that, add the plugins in [`PhysicsPlugins`], or even create your own plugins using
/// the schedules and sets provided by this setup plugin.
///
/// ## Schedules and sets
///
/// The [`PhysicsSchedule`] is responsible for the high-level physics schedule that runs once per physics frame.
/// It has the following sets, in order:
///
/// 1. [`PhysicsSet::Prepare`],
/// 2. [`PhysicsSet::BroadPhase`],
/// 3. [`PhysicsSet::Substeps`],
/// 4. [`PhysicsSet::Sleeping`],
/// 5. [`PhysicsSet::Sync`],
///
/// The [`SubstepSchedule`] handles physics substepping. It is run [`NumSubsteps`] times in [`PhysicsSet::Substeps`],
/// and it typically handles things like collision detection and constraint solving.
///
/// Substepping sets are added by the solver plugin if it is enabled. See [`SolverPlugin`] for more information.
pub struct PhysicsSetupPlugin;

impl Plugin for PhysicsSetupPlugin {
    fn build(&self, app: &mut App) {
        // Init resources and register component types
        app.init_resource::<PhysicsTimestep>()
            .init_resource::<DeltaTime>()
            .init_resource::<SubDeltaTime>()
            .init_resource::<NumSubsteps>()
            .init_resource::<NumPosIters>()
            .init_resource::<SleepingThreshold>()
            .init_resource::<DeactivationTime>()
            .init_resource::<PhysicsLoop>()
            .init_resource::<Gravity>()
            .register_type::<PhysicsTimestep>()
            .register_type::<DeltaTime>()
            .register_type::<SubDeltaTime>()
            .register_type::<NumSubsteps>()
            .register_type::<NumPosIters>()
            .register_type::<SleepingThreshold>()
            .register_type::<DeactivationTime>()
            .register_type::<PhysicsLoop>()
            .register_type::<Gravity>()
            .register_type::<RigidBody>()
            .register_type::<Sleeping>()
            .register_type::<SleepingDisabled>()
            .register_type::<TimeSleeping>()
            .register_type::<Pos>()
            .register_type::<Rot>()
            .register_type::<PrevPos>()
            .register_type::<PrevRot>()
            .register_type::<LinVel>()
            .register_type::<AngVel>()
            .register_type::<PreSolveLinVel>()
            .register_type::<PreSolveAngVel>()
            .register_type::<Restitution>()
            .register_type::<Friction>()
            .register_type::<ExternalForce>()
            .register_type::<ExternalTorque>()
            .register_type::<Mass>()
            .register_type::<InvMass>()
            .register_type::<Inertia>()
            .register_type::<InvInertia>()
            .register_type::<LocalCom>();

        let mut physics_schedule = Schedule::default();

        physics_schedule.set_build_settings(bevy::ecs::schedule::ScheduleBuildSettings {
            ambiguity_detection: LogLevel::Error,
            ..default()
        });

        physics_schedule.configure_sets(
            (
                PhysicsSet::Prepare,
                PhysicsSet::BroadPhase,
                PhysicsSet::Substeps,
                PhysicsSet::Sleeping,
                PhysicsSet::Sync,
            )
                .chain(),
        );

        app.add_schedule(PhysicsSchedule, physics_schedule);

        let mut substep_schedule = Schedule::default();

        substep_schedule.set_build_settings(bevy::ecs::schedule::ScheduleBuildSettings {
            ambiguity_detection: LogLevel::Error,
            ..default()
        });

        app.add_schedule(SubstepSchedule, substep_schedule);

        // Add system set for running physics schedule
        app.configure_set(
            FixedUpdateSet
                .before(CoreSet::Update)
                .in_base_set(CoreSet::PreUpdate),
        );
        app.add_system(run_physics_schedule.in_set(FixedUpdateSet));

        app.add_system(
            run_substep_schedule
                .in_set(PhysicsSet::Substeps)
                .in_schedule(PhysicsSchedule),
        );

        #[cfg(feature = "debug-render-aabbs")]
        {
            app.add_system(draw_aabbs);
        }
    }
}

/// The high-level physics schedule that runs once per physics frame.
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct PhysicsSchedule;

/// The substepping schedule. The number of substeps per physics step is
/// configured through the [`NumSubsteps`] resource.
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct SubstepSchedule;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
struct FixedUpdateSet;

/// Data related to the physics simulation loop.
#[derive(Reflect, Resource, Debug, Default)]
#[reflect(Resource)]
pub struct PhysicsLoop {
    /// Time accumulated into the physics loop. This is consumed by the [`PhysicsSchedule`].
    pub(crate) accumulator: Scalar,
    /// Number of steps queued by the user. Time will be added to the accumulator according to the number of queued step.
    pub(crate) queued_steps: u32,
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

/// Runs the [`PhysicsSchedule`].
fn run_physics_schedule(world: &mut World) {
    let mut physics_loop = world
        .remove_resource::<PhysicsLoop>()
        .expect("no PhysicsLoop resource");

    #[cfg(feature = "f32")]
    let delta_seconds = world.resource::<Time>().delta_seconds();
    #[cfg(feature = "f64")]
    let delta_seconds = world.resource::<Time>().delta_seconds_f64();

    let time_step = *world.resource::<PhysicsTimestep>();

    // Update `DeltaTime` according to the `PhysicsTimestep` configuration
    let dt = match time_step {
        PhysicsTimestep::Fixed(fixed_delta_seconds) => fixed_delta_seconds,
        PhysicsTimestep::Variable { max_dt } => delta_seconds.min(max_dt),
    };
    world.resource_mut::<DeltaTime>().0 = dt;

    // Add time to the accumulator
    if physics_loop.paused {
        physics_loop.accumulator += dt * physics_loop.queued_steps as Scalar;
        physics_loop.queued_steps = 0;
    } else {
        physics_loop.accumulator += delta_seconds;
    }

    // Step the simulation until the accumulator has been consumed.
    // Note that a small remainder may be passed on to the next run of the physics schedule.
    while physics_loop.accumulator >= dt && dt > 0.0 {
        debug!("running PhysicsSchedule");
        world.run_schedule(PhysicsSchedule);
        physics_loop.accumulator -= dt;
    }

    world.insert_resource(physics_loop);
}

/// Runs the [`SubstepSchedule`].
fn run_substep_schedule(world: &mut World) {
    let NumSubsteps(substeps) = *world.resource::<NumSubsteps>();
    let dt = world.resource::<DeltaTime>().0;

    // Update `SubDeltaTime`
    let mut sub_delta_time = world.resource_mut::<SubDeltaTime>();
    sub_delta_time.0 = dt / substeps as Scalar;

    for i in 0..substeps {
        debug!("running SubstepSchedule: {i}");
        world.run_schedule(SubstepSchedule);
    }
}
