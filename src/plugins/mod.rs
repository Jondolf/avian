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
//! - [`PhysicsSet`]
//! - [`PhysicsSchedule`] and [`PhysicsStepSet`]
//! - [`SubstepSchedule`] and [`SubstepSet`]

pub mod collision;
#[cfg(feature = "debug-plugin")]
pub mod debug;
pub mod integrator;
pub mod prepare;
pub mod setup;
pub mod sleeping;
pub mod solver;
pub mod spatial_query;
pub mod sync;

use bevy::utils::intern::Interned;
pub use collision::{
    broad_phase::BroadPhasePlugin, collider_backend::*, contact_reporting::ContactReportingPlugin,
    narrow_phase::NarrowPhasePlugin,
};
#[cfg(feature = "debug-plugin")]
pub use debug::PhysicsDebugPlugin;
pub use integrator::IntegratorPlugin;
pub use prepare::PreparePlugin;
pub use setup::PhysicsSetupPlugin;
pub use sleeping::SleepingPlugin;
pub use solver::SolverPlugin;
pub use spatial_query::SpatialQueryPlugin;
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
/// - [`ColliderBackendPlugin`]: Handles generic collider backend logic, like initializing colliders and AABBs
/// and updating related components.
/// - [`BroadPhasePlugin`]: Collects pairs of potentially colliding entities into [`BroadCollisionPairs`] using
/// [AABB](ColliderAabb) intersection checks.
/// - [`NarrowPhasePlugin`]: Computes contacts between entities and sends collision events.
/// - [`ContactReportingPlugin`]: Sends collision events and updates [`CollidingEntities`].
/// - [`IntegratorPlugin`]: Integrates Newton's 2nd law of motion, applying forces and moving entities according to their velocities.
/// - [`SolverPlugin`]: Solves positional and angular [constraints], updates velocities and solves velocity constraints
/// (dynamic [friction](Friction) and [restitution](Restitution)).
/// - [`SleepingPlugin`]: Controls when bodies should be deactivated and marked as [`Sleeping`] to improve performance.
/// - [`SpatialQueryPlugin`]: Handles spatial queries like [raycasting](RayCaster) and shapecasting.
/// - [`SyncPlugin`]: Keeps [`Position`] and [`Rotation`] in sync with `Transform`.
/// - `PhysicsDebugPlugin`: Renders physics objects and events like [AABBs](ColliderAabb) and [contacts](Collision)
/// for debugging purposes (only with `debug-plugin` feature enabled).
///
/// Refer to the documentation of the plugins for more information about their responsibilities and implementations.
///
/// You can also find more information regarding the engine's general plugin architecture [here](plugins).
///
/// ## Custom schedule
///
/// You can run the [`PhysicsSchedule`] in any schedule you want by specifying the schedule when adding the plugin group:
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::new(FixedUpdate)))
///         .run();
/// }
/// ```
///
/// Note that using `FixedUpdate` with a fixed [physics timestep](Physics) can produce unexpected results due to two separate
/// fixed timesteps. However, using `FixedUpdate` can be useful for [networking usage](crate#can-the-engine-be-used-on-servers)
/// when you need to keep the client and server in sync.
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
#[cfg_attr(
    feature = "2d",
    doc = "use bevy_xpbd_2d::{prelude::*, PhysicsSchedule, PhysicsStepSet};"
)]
#[cfg_attr(
    feature = "3d",
    doc = "use bevy_xpbd_3d::{prelude::*, PhysicsSchedule, PhysicsStepSet};"
)]
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
///         physics_schedule.add_systems(collect_collision_pairs.in_set(PhysicsStepSet::BroadPhase));
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
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
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
///         PhysicsPlugins::default()
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
pub struct PhysicsPlugins {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PhysicsPlugins {
    /// Creates a [`PhysicsPlugins`] plugin group using the given schedule for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PhysicsPlugins {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

impl PluginGroup for PhysicsPlugins {
    fn build(self) -> PluginGroupBuilder {
        PluginGroupBuilder::start::<Self>()
            .add(PhysicsSetupPlugin::new(self.schedule))
            .add(PreparePlugin::new(self.schedule))
            .add(ColliderBackendPlugin::<Collider>::new(self.schedule))
            .add(BroadPhasePlugin)
            .add(NarrowPhasePlugin::<Collider>::default())
            .add(ContactReportingPlugin)
            .add(IntegratorPlugin)
            .add(SolverPlugin)
            .add(SleepingPlugin)
            .add(SpatialQueryPlugin::new(self.schedule))
            .add(SyncPlugin::new(self.schedule))
    }
}
