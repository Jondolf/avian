pub mod broad_phase;
pub mod integrator;
pub mod narrow_phase;
pub mod prepare;
pub mod solver;

pub use broad_phase::BroadPhasePlugin;
pub use integrator::IntegratorPlugin;
pub use narrow_phase::NarrowPhasePlugin;
pub use prepare::PreparePlugin;
pub use solver::SolverPlugin;

use bevy::prelude::SystemLabel;

/// The main steps in the physics simulation loop.
#[derive(SystemLabel, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsStep {
    /// In the preparation step, necessary preparations and updates will be run before the rest of the physics simulation loop.
    /// For example, bevy `Transform`s are synchronized with the physics world, AABBs are updated etc.
    Prepare,
    /// During the broad phase, pairs of potentially colliding entities will be collected into the [`BroadCollisionPairs`] resource using simple AABB intersection checks. These will be further checked for collision in the [`PhysicsStep::NarrowPhase`].
    ///
    /// The broad phase speeds up collision detection, as the number of accurate collision checks is greatly reduced.
    BroadPhase,
    /// During the narrow phase, pairs of entities collected in the [`PhysicsStep::BroadPhase`] will be checked for collisions. The data for each collision is stored in the [`Collisions`] resource.
    NarrowPhase,
    /// In the integration step, the position and velocity of each particle and body is explicitly integrated, taking only external forces like gravity (and forces applied by the user) into account.
    Integrate,
    /// The position solving step iterates through constraints, and moves particles and bodies accordingly. This is used for things like joints and resolving collisions through non-penetration constraints.
    SolvePos,
    /// In the velocity update step, new velocities are derived for all particles and bodies after the position solving step.
    UpdateVel,
    /// During the velocity solving step, a velocity update caused by properties like restitution and friction will be applied to all particles and bodies.
    SolveVel,
}
