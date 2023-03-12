pub mod broad_phase;
pub mod integrator;
pub mod prepare;
pub mod solver;

pub use broad_phase::BroadPhasePlugin;
pub use integrator::IntegratorPlugin;
pub use prepare::PreparePlugin;
pub use solver::SolverPlugin;

use bevy::prelude::SystemSet;

/// The main steps in the physics simulation loop.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsSet {
    /// In the preparation step, necessary preparations and updates will be run before the rest of the physics simulation loop.
    /// For example, bevy `Transform`s are synchronized with the physics world, AABBs are updated etc.
    Prepare,
    /// During the broad phase, potential collisions will be collected into the [`BroadCollisions`] resource using simple AABB intersection checks. These will be further checked for collision in the [`PhysicsStep::NarrowPhase`].
    ///
    /// The broad phase speeds up collision detection, as the number of accurate collision checks is greatly reduced.
    BroadPhase,
    /// Substepping is an inner loop inside a physics step, see [`PhysicsSubstep`] and [`XpbdSubstepSchedule`]
    Substeps,
}

/// The steps in the inner substepping loop
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
