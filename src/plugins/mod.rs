//! The building blocks of Bevy XPBD.
//!
//! Bevy XPBD consists of multiple different plugins, each with their own responsibilities. These plugins
//! are grouped into the [`PhysicsPlugins`] plugin group, which allows you to easily configure and
//! disable any of the existing plugins.
//!
//! This means that users can simply disable existing functionality and replace it
//! with specialized solutions, while keeping the rest of the engine's features. This can be important
//! in many games and applications that require special optimizations or features.
//!
//! Currently, the only plugin that is truly necessary is the [`PhysicsSetupPlugin`] that initializes the
//! engine's resources, schedules and sets, and even that could be implemented manually if you wanted
//! a custom scheduler for example.
//!
//! ## See also
//!
//! - [All of the current plugins and their responsibilities](PhysicsPlugins)
//! - [Creating custom plugins](PhysicsPlugins#custom-plugins)
//! - [`PhysicsSchedule`] and [`PhysicsSet`]
//! - [`SubstepSchedule`] and [`SubstepSet`]

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

/// A plugin group containing all of Bevy XPBD's plugins.
///
/// By default, the following plugins will be added:
///
/// - [`PhysicsSetupPlugin`]: Sets up the physics engine by initializing the necessary schedules, sets and resources.
/// - [`PreparePlugin`]: Runs systems at the start of each physics frame; initializes [rigid bodies](RigidBody)
/// and [colliders](Collider) and updates components.
/// - [`BroadPhasePlugin`]: Collects pairs of potentially colliding entities into [`BroadCollisionPairs`] using
/// [AABB](ColliderAabb) intersection checks.
/// - [`IntegratorPlugin`]: Integrates Newton's 2nd law of motion, applying forces and moving entities according to their velocities.
/// - [`SolverPlugin`]: Solves positional and angular [constraints], updates velocities and solves velocity constraints
/// (dynamic [friction](Friction) and [restitution](Restitution)).
/// - [`SleepingPlugin`]: Controls when bodies should be deactivated and marked as [`Sleeping`] to improve performance.
/// - [`SyncPlugin`]: Synchronizes the engine's [`Position`]s and [`Rotation`]s with Bevy's [`Transform`]s.
/// - `PhysicsDebugPlugin`: Renders physics objects and events like [AABBs](ColliderAabb) and [contacts](Collision)
/// for debugging purposes (only with `debug-plugin` feature enabled).
///
/// Refer to the documentation of the plugins for more information about their responsibilities and implementations.
///
/// You can also find more information regarding the engine's general plugin architecture [here](plugins).
///
/// ## Custom plugins
///
/// First, create a new plugin. If you want to run your systems in the engine's schedules, get either the [`PhysicsSchedule`]
/// or the [`SubstepSchedule`]. Then you can add your systems to that schedule and control system ordering with
/// [`PhysicsSet`] or [`SubstepSet`].
///
/// Here we will create a custom broad phase plugin that will replace the default [`BroadPhasePlugin`]:
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// pub struct CustomBroadPhasePlugin;
///
/// impl Plugin for CustomBroadPhasePlugin {
///     fn build(&self, app: &mut App) {
///         // Make sure the PhysicsSchedule is available
///         let physics_schedule = app
///             .get_schedule_mut(PhysicsSchedule)
///             .expect("add PhysicsSchedule first");
///
///         // Add the system into the broad phase system set
///         physics_schedule.add_system(collect_collision_pairs.in_set(PhysicsSet::BroadPhase));
///     }
/// }
///
/// fn collect_collision_pairs() {
///     // Implementation goes here
/// }
/// ```
///
/// Next, when creating your app, simply disable the default [`BroadPhasePlugin`] and add your custom plugin:
///
/// ```no_run
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # struct CustomBroadPhasePlugin;
/// # impl Plugin for CustomBroadPhasePlugin {
/// #     fn build(&self, app: &mut App) {}
/// # }
/// #
/// fn main() {
///     let mut app = App::new();
///
///     app.add_plugins(DefaultPlugins);
///
///     // Add PhysicsPlugins and replace default broad phase with our custom broad phase
///     app.add_plugins(
///         PhysicsPlugins
///             .build()
///             .disable::<BroadPhasePlugin>()
///             .add(CustomBroadPhasePlugin),
///     );
///
///     app.run();
/// }
/// ```
///
/// You can find a full working example
/// [here](https://github.com/Jondolf/bevy_xpbd/blob/main/crates/bevy_xpbd_3d/examples/custom_broad_phase.rs).
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

/// System sets for the main steps in the physics simulation loop. These are typically run in the [`PhysicsSchedule`].
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

/// System sets for the the steps in the inner substepping loop. These are typically run in the [`SubstepSchedule`].
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
    UpdateVelocities,
    /// During the velocity solving step, a velocity update caused by properties like restitution and friction will be applied to all particles and bodies.
    SolveVelocities,
}
