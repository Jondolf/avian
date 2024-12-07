//! Sets up the default scheduling, system set configuration, and time resources
//! for the physics solver and substepping loop.
//!
//! See [`SolverSchedulePlugin`].

use crate::prelude::*;
use bevy::{
    ecs::schedule::{ExecutorKind, LogLevel, ScheduleBuildSettings, ScheduleLabel},
    prelude::*,
};
use dynamics::integrator::IntegrationSet;

/// Sets up the default scheduling, system set configuration, and time resources for the physics solver.
#[derive(Debug, Default)]
pub struct SolverSchedulePlugin;

impl Plugin for SolverSchedulePlugin {
    fn build(&self, app: &mut App) {
        // Register types.
        app.register_type::<(Time<Substeps>, SubstepCount)>();

        // Initialize resources.
        app.insert_resource(Time::new_with(Substeps))
            .init_resource::<SubstepCount>();

        // Get the `PhysicsSchedule`, and panic if it doesn't exist.
        let physics = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        // See `SolverSet` for what each system set is responsible for.
        physics.configure_sets(
            (
                SolverSet::PreSubstep,
                SolverSet::Substep,
                SolverSet::PostSubstep,
                SolverSet::Restitution,
                SolverSet::ApplyTranslation,
                SolverSet::StoreContactImpulses,
            )
                .chain()
                .in_set(PhysicsStepSet::Solver),
        );

        // Run the substepping loop.
        physics.add_systems(run_substep_schedule.in_set(SolverSet::Substep));

        // Set up the substep schedule, the schedule that runs systems in the inner substepping loop.
        app.edit_schedule(SubstepSchedule, |schedule| {
            schedule
                .set_executor_kind(ExecutorKind::SingleThreaded)
                .set_build_settings(ScheduleBuildSettings {
                    ambiguity_detection: LogLevel::Error,
                    ..default()
                })
                .configure_sets(
                    (
                        IntegrationSet::Velocity,
                        SubstepSolverSet::WarmStart,
                        SubstepSolverSet::SolveConstraints,
                        IntegrationSet::Position,
                        SubstepSolverSet::Relax,
                        SubstepSolverSet::SolveXpbdConstraints,
                        SubstepSolverSet::SolveUserConstraints,
                        SubstepSolverSet::XpbdVelocityProjection,
                    )
                        .chain(),
                );
        });
    }
}

/// The substepping schedule that runs in [`SolverSet::Substep`].
/// The number of substeps per physics step is configured through the [`SubstepCount`] resource.
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct SubstepSchedule;

/// System sets for the constraint solver.
///
/// # Steps
///
/// Below is the core solver loop.
///
/// 1. Generate and prepare constraints ([`NarrowPhaseSet::GenerateConstraints`](collision::narrow_phase::NarrowPhaseSet::GenerateConstraints))
/// 2. Substepping loop (runs the [`SubstepSchedule`] [`SubstepCount`] times; see [`SolverSet::Substep`])
/// 3. Apply restitution ([`SolverSet::Restitution`])
/// 4. Finalize positions by applying [`AccumulatedTranslation`] ([`SolverSet::ApplyTranslation`])
/// 5. Store contact impulses for next frame's warm starting ([`SolverSet::StoreContactImpulses`])
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SolverSet {
    /// A system set for systems running just before the substepping loop.
    PreSubstep,
    /// A system set for the substepping loop.
    Substep,
    /// A system set for systems running just after the substepping loop.
    PostSubstep,
    /// Applies [restitution](Restitution) for bodies after solving overlap.
    Restitution,
    /// Finalizes the positions of bodies by applying the [`AccumulatedTranslation`].
    ///
    /// Constraints don't modify the positions of bodies directly and instead adds
    /// to this translation to improve numerical stability when bodies are far from the world origin.
    ApplyTranslation,
    /// Copies contact impulses from [`ContactConstraints`] to the contacts in [`Collisions`].
    /// They will be used for [warm starting](SubstepSolverSet::WarmStart) the next frame or substep.
    ///
    /// [`ContactConstraints`]: super::ContactConstraints
    StoreContactImpulses,
}

/// System sets for the substepped part of the constraint solver.
///
/// # Steps
///
/// 1. Integrate velocity ([`IntegrationSet::Velocity`])
/// 2. Warm start ([`SubstepSolverSet::WarmStart`])
/// 3. Solve constraints with bias ([`SubstepSolverSet::SolveConstraints`])
/// 4. Integrate positions ([`IntegrationSet::Position`])
/// 5. Solve constraints without bias to relax velocities ([`SubstepSolverSet::Relax`])
/// 6. Solve joints using Extended Position-Based Dynamics (XPBD). ([`SubstepSolverSet::SolveXpbdConstraints`])
/// 7. Solve user-defined constraints. ([`SubstepSolverSet::SolveUserConstraints`])
/// 8. Update velocities after XPBD constraint solving. ([`SubstepSolverSet::XpbdVelocityProjection`])
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SubstepSolverSet {
    /// Warm starts the solver by applying the impulses from the previous frame or substep.
    ///
    /// This significantly improves convergence, but by itself can lead to overshooting.
    /// Overshooting is reduced by [relaxing](SubstepSolverSet::Relax) the biased velocities
    /// by running the solver a second time *without* bias.
    WarmStart,
    /// Solves velocity constraints using a position bias that boosts the response
    /// to account for the constraint error.
    SolveConstraints,
    /// Solves velocity constraints without a position bias to relax the biased velocities
    /// and impulses. This reduces overshooting caused by [warm starting](SubstepSolverSet::WarmStart).
    Relax,
    /// Solves joints using Extended Position-Based Dynamics (XPBD).
    SolveXpbdConstraints,
    /// A system set for user constraints.
    SolveUserConstraints,
    /// Performs velocity updates after XPBD constraint solving.
    XpbdVelocityProjection,
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
/// # Example
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
