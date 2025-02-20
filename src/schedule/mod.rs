//! Sets up the default scheduling, system set configuration, and time resources for physics.
//!
//! See [`PhysicsSchedulePlugin`].

mod time;
use dynamics::solver::schedule::SubstepCount;
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
/// # Schedules and Sets
///
/// This plugin initializes and configures the following schedules and system sets:
///
/// - [`PhysicsSet`]: High-level system sets for the main phases of the physics engine.
///   You can use these to schedule your own systems before or after physics is run without
///   having to worry about implementation details.
/// - [`PhysicsSchedule`]: Responsible for advancing the simulation in [`PhysicsSet::StepSimulation`].
/// - [`PhysicsStepSet`]: System sets for the steps of the actual physics simulation loop.
pub struct PhysicsSchedulePlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PhysicsSchedulePlugin {
    /// Creates a [`PhysicsSchedulePlugin`] using the given schedule for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PhysicsSchedulePlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

impl Plugin for PhysicsSchedulePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Time<Physics>>()
            .insert_resource(Time::new_with(Substeps))
            .init_resource::<SubstepCount>();

        // TODO: Where should this be initialized?
        app.init_resource::<PhysicsLengthUnit>();

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
                    PhysicsStepSet::First,
                    PhysicsStepSet::BroadPhase,
                    PhysicsStepSet::NarrowPhase,
                    PhysicsStepSet::Solver,
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

// TODO: Remove this in favor of collision hooks.
/// A schedule where you can add systems to filter or modify collisions
/// using the [`Collisions`] resource.
///
/// The schedule is empty by default and runs in
/// [`NarrowPhaseSet::PostProcess`](collision::narrow_phase::NarrowPhaseSet::PostProcess).
///
/// # Example
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
///    updating several components.
/// 2. `StepSimulation`: Responsible for advancing the simulation by running the steps in [`PhysicsStepSet`].
/// 3. `Sync`: Responsible for synchronizing physics components with other data, like keeping [`Position`]
///    and [`Rotation`] in sync with `Transform`.
///
/// # See Also
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
/// 5. Sleeping
/// 6. Spatial queries
/// 7. Last (empty by default)
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

/// Runs the [`PhysicsSchedule`].
fn run_physics_schedule(world: &mut World, mut is_first_run: Local<IsFirstRun>) {
    let _ = world.try_schedule_scope(PhysicsSchedule, |world, schedule| {
        let is_paused = world.resource::<Time<Physics>>().is_paused();
        let old_clock = world.resource::<Time>().as_generic();
        let physics_clock = world.resource_mut::<Time<Physics>>();

        // Get the scaled timestep delta time based on the timestep mode.
        let timestep = old_clock
            .delta()
            .mul_f64(physics_clock.relative_speed_f64());

        // Advance the physics clock by the timestep if not paused.
        if !is_paused {
            world.resource_mut::<Time<Physics>>().advance_by(timestep);

            // Advance the substep clock already so that systems running
            // before the substepping loop have the right delta.
            let SubstepCount(substeps) = *world.resource::<SubstepCount>();
            let sub_delta = timestep.div_f64(substeps as f64);
            world.resource_mut::<Time<Substeps>>().advance_by(sub_delta);
        }

        // Set the generic `Time` resource to `Time<Physics>`.
        *world.resource_mut::<Time>() = world.resource::<Time<Physics>>().as_generic();

        // Advance the simulation.
        if !world.resource::<Time>().delta().is_zero() {
            trace!("running PhysicsSchedule");
            schedule.run(world);
        }

        // If physics is paused, reset delta time to stop the simulation
        // unless users manually advance `Time<Physics>`.
        if is_paused {
            world
                .resource_mut::<Time<Physics>>()
                .advance_by(Duration::ZERO);
        }

        // Set the generic `Time` resource back to the clock that was active before physics.
        *world.resource_mut::<Time>() = old_clock;
    });

    is_first_run.0 = false;
}
