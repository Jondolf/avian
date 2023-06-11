//! The steps used in the simulation loop.

pub mod broad_phase;
pub mod integrator;
pub mod prepare;
pub mod sleeping;
pub mod solver;
pub mod sync;

pub use broad_phase::BroadPhasePlugin;
pub use integrator::IntegratorPlugin;
pub use prepare::PreparePlugin;
pub use sleeping::SleepingPlugin;
pub use solver::{solve_constraint, SolverPlugin};
pub use sync::SyncPlugin;

#[allow(unused_imports)]
use crate::prelude::{steps::broad_phase::BroadCollisionPairs, *}; // For doc comments
use bevy::prelude::*;

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
    /// Substepping is an inner loop inside a physics step. See [`SubsteppingSet`] and [`XpbdSubstepSchedule`].
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
pub enum SubsteppingSet {
    /// In the integration step, the position and velocity of each particle and body is explicitly integrated, taking only external forces like gravity (and forces applied by the user) into account.
    Integrate,
    /// The position solving step iterates through constraints, and moves particles and bodies accordingly. This step is also responsible for narrow phase collision detection, as it creates non-penetration constraints for colliding bodies.
    SolvePos,
    /// In the velocity update step, new velocities are derived for all particles and bodies after the position solving step.
    UpdateVel,
    /// During the velocity solving step, a velocity update caused by properties like restitution and friction will be applied to all particles and bodies.
    SolveVel,
}
