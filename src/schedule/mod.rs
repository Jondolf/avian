//! Sets up the default scheduling, system set configuration, and time resources for physics.
//!
//! See [`PhysicsSchedulePlugin`].

mod time;
pub use time::*;

use std::time::Duration;

// For doc links
#[allow(unused_imports)]
use crate::prelude::*;

use bevy::{
    ecs::intern::Interned,
    ecs::schedule::{ExecutorKind, LogLevel, ScheduleBuildSettings, ScheduleLabel},
    prelude::*,
    transform::TransformSystem,
};

/// Sets up the default scheduling, system set configuration, and time resources for physics.
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
/// - [`SubstepSchedule`]: Responsible for running the substepping loop in [`SolverSet::Substep`].
pub struct PhysicsSchedulePlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PhysicsSchedulePlugin {
    /// Creates a [`PhysicsSchedulePlugin`] using the given schedule for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PhysicsSchedulePlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

impl Plugin for PhysicsSchedulePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Time<Physics>>()
            .insert_resource(Time::new_with(Substeps))
            .init_resource::<SubstepCount>();

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
                    PhysicsStepSet::First,
                    PhysicsStepSet::BroadPhase,
                    PhysicsStepSet::NarrowPhase,
                    PhysicsStepSet::Solver,
                    PhysicsStepSet::ReportContacts,
                    PhysicsStepSet::Sleeping,
                    PhysicsStepSet::SpatialQuery,
                    PhysicsStepSet::Last,
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
        });

        // TODO: This should probably just be in the SolverPlugin.
        app.add_systems(
            PhysicsSchedule,
            run_substep_schedule.in_set(SolverSet::Substep),
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

/// Responsible for advancing the physics simulation. This is run in [`PhysicsSet::StepSimulation`].
///
/// See [`PhysicsStepSet`] for the system sets that are run in this schedule.
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct PhysicsSchedule;

/// The substepping schedule that runs in [`SolverSet::Substep`].
/// The number of substeps per physics step is configured through the [`SubstepCount`] resource.
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct SubstepSchedule;

/// A schedule where you can add systems to filter or modify collisions
/// using the [`Collisions`] resource.
///
/// The schedule is empty by default and runs in
/// [`NarrowPhaseSet::PostProcess`](collision::narrow_phase::NarrowPhaseSet::PostProcess).
///
/// ## Example
///
/// Below is an example of how you could add a system that filters collisions.
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// #[derive(Component)]
/// struct Invulnerable;
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         .add_systems(PostProcessCollisions, filter_collisions)
///         .run();
/// }
///
/// fn filter_collisions(mut collisions: ResMut<Collisions>, query: Query<(), With<Invulnerable>>) {
///     // Remove collisions where one of the colliders has an `Invulnerable` component.
///     // In a real project, this could be done more efficiently with collision layers.
///     collisions.retain(|contacts| {
///         !query.contains(contacts.entity1) && !query.contains(contacts.entity2)
///     });
/// }
/// ```
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct PostProcessCollisions;

/// High-level system sets for the main phases of the physics engine.
/// You can use these to schedule your own systems before or after physics is run without
/// having to worry about implementation details.
///
/// 1. `Prepare`: Responsible for initializing [rigid bodies](RigidBody) and [colliders](Collider) and
/// updating several components.
/// 2. `StepSimulation`: Responsible for advancing the simulation by running the steps in [`PhysicsStepSet`].
/// 3. `Sync`: Responsible for synchronizing physics components with other data, like keeping [`Position`]
/// and [`Rotation`] in sync with `Transform`.
///
/// ## See also
///
/// - [`PhysicsSchedule`]: Responsible for advancing the simulation in [`PhysicsSet::StepSimulation`].
/// - [`PhysicsStepSet`]: System sets for the steps of the actual physics simulation loop, like
///   the broad phase and the substepping loop.
/// - [`SubstepSchedule`]: Responsible for running the substepping loop in [`PhysicsStepSet::Solver`].
/// - [`PostProcessCollisions`]: Responsible for running the post-process collisions group in
///   [`NarrowPhaseSet::PostProcess`](collision::narrow_phase::NarrowPhaseSet::PostProcess).
///   Empty by default.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsSet {
    /// Responsible for initializing [rigid bodies](RigidBody) and [colliders](Collider) and
    /// updating several components.
    ///
    /// See [`PreparePlugin`].
    Prepare,
    /// Responsible for advancing the simulation by running the steps in [`PhysicsStepSet`].
    /// Systems in this set are run in the [`PhysicsSchedule`].
    StepSimulation,
    /// Responsible for synchronizing physics components with other data, like keeping [`Position`]
    /// and [`Rotation`] in sync with `Transform`.
    ///
    /// See [`SyncPlugin`].
    Sync,
}

/// System sets for the main steps in the physics simulation loop. These are typically run in the [`PhysicsSchedule`].
///
/// 1. First (empty by default)
/// 2. Broad phase
/// 3. Narrow phase
/// 4. Solver
/// 5. Report contacts (send collision events)
/// 6. Sleeping
/// 7. Spatial queries
/// 8. Last (empty by default)
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsStepSet {
    /// Runs at the start of the [`PhysicsSchedule`]. Empty by default.
    First,
    /// Responsible for collecting pairs of potentially colliding entities into [`BroadCollisionPairs`] using
    /// [AABB](ColliderAabb) intersection tests.
    ///
    /// See [`BroadPhasePlugin`].
    BroadPhase,
    /// Responsible for computing contacts between entities and sending collision events.
    ///
    /// See [`NarrowPhasePlugin`].
    NarrowPhase,
    /// Responsible for running the solver and its substepping loop.
    ///
    /// See [`SolverPlugin`] and [`SubstepSchedule`].
    Solver,
    /// Responsible for sending collision events and updating [`CollidingEntities`].
    ///
    /// See [`ContactReportingPlugin`].
    ReportContacts,
    /// Responsible for controlling when bodies should be deactivated and marked as [`Sleeping`].
    ///
    /// See [`SleepingPlugin`].
    Sleeping,
    /// Responsible for spatial queries like [raycasting](`RayCaster`) and shapecasting.
    ///
    /// See [`SpatialQueryPlugin`].
    SpatialQuery,
    /// Runs at the end of the [`PhysicsSchedule`]. Empty by default.
    Last,
}

/// The number of substeps used in the simulation.
///
/// A higher number of substeps reduces the value of [`Time`],
/// which results in a more accurate simulation, but also reduces performance. The default
/// substep count is currently 6.
///
/// If you use a very high substep count and encounter stability issues, consider enabling the `f64`
/// feature as shown in the [getting started guide](crate#getting-started) to avoid floating point
/// precision problems.
///
/// ## Example
///
/// You can change the number of substeps by inserting the [`SubstepCount`] resource:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         .insert_resource(SubstepCount(12))
///         .run();
/// }
/// ```
#[derive(Debug, Reflect, Resource, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Resource, PartialEq)]
pub struct SubstepCount(pub u32);

impl Default for SubstepCount {
    fn default() -> Self {
        Self(6)
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
