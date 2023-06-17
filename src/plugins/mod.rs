//! The steps used in the simulation loop.

pub mod broad_phase;
#[cfg(feature = "debug-plugin")]
pub mod debug;
pub mod integrator;
pub mod prepare;
pub mod setup;
pub mod sleeping;
pub mod solver;
pub mod sync;

pub use broad_phase::BroadPhasePlugin;
#[cfg(feature = "debug-plugin")]
pub use debug::PhysicsDebugPlugin;
pub use integrator::IntegratorPlugin;
pub use prepare::PreparePlugin;
pub use setup::*;
pub use sleeping::SleepingPlugin;
pub use solver::{solve_constraint, SolverPlugin};
pub use sync::SyncPlugin;

#[allow(unused_imports)]
use crate::prelude::*; // For doc comments
use bevy::prelude::*;

/// This plugin group will add the following physics plugins:
///
/// - [`PhysicsSetupPlugin`]
/// - [`PreparePlugin`]
/// - [`BroadPhasePlugin`]
/// - [`IntegratorPlugin`]
/// - [`SolverPlugin`]
/// - [`SleepingPlugin`]
/// - [`SyncPlugin`]
/// - `PhysicsDebugPlugin` (with `debug-plugin` feature enabled)
///
/// Note that the [`PhysicsSetupPlugin`] initializes all of the schedules, sets and resources required
/// by the other plugins, so it is necessary.
///
/// Other than that, you can disable and configure the plugins freely, and even plug in your own implementations.
/// For example, you could replace the broad phase with your own specialized solution, or create your own sync plugin
/// that synchronizes the state of the physics world to something else than Bevy's transforms.
///
/// Refer to the documentation of the other plugins for more detailed information about the default implementations.
pub struct PhysicsPlugins;

impl PluginGroup for PhysicsPlugins {
    fn build(self) -> PluginGroupBuilder {
        #[allow(unused_mut)]
        let mut builder = PluginGroupBuilder::start::<Self>();

        #[cfg(feature = "debug-plugin")]
        {
            builder = builder.add(PhysicsDebugPlugin);
        }

        builder
            .add(PhysicsSetupPlugin)
            .add(PreparePlugin)
            .add(BroadPhasePlugin)
            .add(IntegratorPlugin)
            .add(SolverPlugin)
            .add(SleepingPlugin)
            .add(SyncPlugin)
    }
}

/// The main steps in the physics simulation loop.
///
/// 1. Prepare
/// 2. Broad phase
/// 3. Substeps
///     1. Integrate
///     2. Solve positional and angular constraints
///     3. Update velocities
///     4. Solve velocity constraints (dynamic friction and restitution)
/// 4. Sleeping
/// 5. Sync data
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsSet {
    /// In the preparation step, necessary preparations and updates will be run before the rest of the physics simulation loop.
    Prepare,
    /// During the broad phase, potential collisions will be collected into the [`BroadCollisionPairs`] resource using simple AABB intersection checks.
    ///
    /// The broad phase speeds up collision detection, as the number of accurate collision checks is greatly reduced.
    BroadPhase,
    /// Substepping is an inner loop inside a physics step. See [`SubstepSet`] and [`SubstepSchedule`].
    Substeps,
    /// The sleeping step controls when bodies are active. This improves performance and helps prevent jitter. See [`Sleeping`].
    Sleeping,
    /// In the sync step, Bevy [`Transform`]s are synchronized with the physics world.
    Sync,
}

/// The steps in the inner substepping loop.
///
/// 1. Integrate
/// 2. Solve positional and angular constraints
/// 3. Update velocities
/// 4. Solve velocity constraints (dynamic friction and restitution)
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SubstepSet {
    /// In the integration step, the position and velocity of each particle and body is explicitly integrated, taking only external forces like gravity (and forces applied by the user) into account.
    Integrate,
    /// The solver iterates through constraints and solves them.
    /// This step is also responsible for narrow phase collision detection, as it creates a [`PenetrationConstraint`] for each contact.
    ///
    /// **Note**: If you want to create your own constraints, you should add them in [`SubstepSet::SolveUserConstraints`]
    /// to avoid system order ambiguities.
    SolveConstraints,
    /// The position solver iterates through custom constraints created by the user and solves them.
    ///
    /// You can create new constraints by implementing [`XpbdConstraint`] for a component and adding
    /// the constraint system to this set. See [`solve_constraint`].
    SolveUserConstraints,
    /// In the velocity update step, new velocities are derived for all particles and bodies after the position solving step.
    UpdateVel,
    /// During the velocity solving step, a velocity update caused by properties like restitution and friction will be applied to all particles and bodies.
    SolveVel,
}
