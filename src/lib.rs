//! # Bevy XPBD
//!
//! **Bevy XPBD** is a 2D and 3D physics engine based on [*Extended Position Based Dynamics* (XPBD)](#what-is-xpbd)
//! for the [Bevy game engine](https://bevyengine.org/).
//!
//! ## Design
//!
//! Below are some of the core design principles used in Bevy XPBD.
//!
//! - Made with Bevy, for Bevy. No wrappers around existing engines.
//! - Provide an ergonomic and familiar API. Ergonomics is key for a good experience.
//! - Utilize the ECS as much as possible. The engine should feel like a part of Bevy, and it shouldn't
//! need to maintain a separate physics world.
//! - Use a highly modular [plugin architecture](plugins). Users should be able to
//! replace parts of the engine with their own implementations.
//! - Have good documentation. A physics engine is pointless if you don't know how to use it.
//!
//! ## Features
//!
//! Below are some of the features of Bevy XPBD.
//!
//! - Dynamic, kinematic and static [rigid bodies](RigidBody)
//! - [Colliders](Collider) powered by [parry](parry)
//!     - Collision events: [`Collision`], [`CollisionStarted`], [`CollisionEnded`]
//!     - Access to [colliding entities](CollidingEntities)
//!     - [Sensor colliders](Sensor)
//!     - [Collision layers](CollisionLayers)
//!     - [Contact and time of impact queries](narrow_phase::contact_query)
//! - Material properties like [restitution](Restitution) and [friction](Friction)
//! - [Linear damping](LinearDamping) and [angular damping](AngularDamping) for simulating drag
//! - External [forces](ExternalForce), [torque](ExternalTorque), [impulses](ExternalImpulse) and
//! [angular impulses](ExternalAngularImpulse)
//! - [Gravity] and [gravity scale](GravityScale)
//! - [Locking](LockedAxes) translational and rotational axes
//! - [Dominance]
//! - [Joints](joints)
//! - Built-in [constraints] and support for [custom constraints](constraints#custom-constraints)
//! - [Spatial queries](spatial_query)
//!     - [Ray casting](spatial_query#ray-casting)
//!     - [Shape casting](spatial_query#shape-casting)
//!     - [Point projection](spatial_query#point-projection)
//!     - [Intersection tests](spatial_query#intersection-tests)
//! - Debug rendering [colliders](Collider), [AABBs](ColliderAabb), [contacts](Contact), [joints] and axes
//! (with `debug-plugin` feature)
//! - Automatically deactivating bodies with [sleeping](Sleeping)
//! - Configurable [timesteps](PhysicsTimestep), [time scale](PhysicsTimescale) and [substepping](SubstepCount)
//! - `f32`/`f64` precision (`f32` by default)
//!
//! ## Getting started
//!
//! This short guide should help you get started with Bevy XPBD.
//!
//! ### Add the dependency
//!
//! First, add `bevy_xpbd_2d` or `bevy_xpbd_3d` to your dependencies in `Cargo.toml`:
//!  
//! ```toml
//! # For 2D applications:
//! [dependencies]
//! bevy_xpbd_2d = "0.2"
//!
//! # For 3D applications:
//! [dependencies]
//! bevy_xpbd_3d = "0.2"
//!
//! # If you want to use the most up-to-date version, you can follow the main branch:
//! [dependencies]
//! bevy_xpbd_3d = { git = "https://github.com/Jondolf/bevy_xpbd", branch = "main" }
//! ```
//!
//! By default, Bevy XPBD uses `f32` numbers. If you encounter instability or use a large number
//! of [substeps](SubstepCount), you might want to use `f64` instead. You can change these kinds
//! of features by disabling the default features and manually specifying the feature flags you want:
//!
//! ```toml
//! [dependencies]
//! # Add 3D Bevy XPBD with double-precision floating point numbers
//! bevy_xpbd_3d = { version = "0.2", default-features = false, features = ["3d", "f64"] }
//! ```
//!
//! ### Feature flags
//!
//! Default features: `2d`/`3d`, `f32` and `collider-from-mesh`
//!
//! - `2d` enables simulation on the `x` and `y` axes. Enabled by default for `bevy_xpbd_2d`. Incompatible with `3d`.
//! - `3d` enables simulation on the `x`, `y` and `z` axes. Enabled by default for `bevy_xpbd_3d`. Incompatible with `2d`.
//! - `f32` enables using `f32` numbers. Incompatible with `f64`.
//! - `f64` enables using `f64` numbers. Recommended when encountering stability problems, especially with
//! small timesteps. Incompatible with `f32`.
//! - `debug-plugin` enables the `PhysicsDebugPlugin` used for rendering physics objects and properties, like
//! [colliders](Collider), [AABBs](ColliderAabb) and [contacts](Contact).
//! - `collider-from-mesh` allows you to create [colliders](Collider) from Bevy meshes. Enables `bevy_render`.
//! - `simd` enables [SIMD](https://en.wikipedia.org/wiki/Single_instruction,_multiple_data) optimizations.
//! - `parallel` enables multithreading. This improves performance for larger simulations but can add unnecessary
//! overhead for smaller ones.
//! - `enhanced-determinism` enables increased determinism. (Note: cross-platform determinism doesn't work yet, even
//! with this feature enabled)
//!
//! ### Install the plugin
//!
//! Bevy XPBD is designed to be very modular. It is built from many different [plugins] that
//! manage different parts of the engine. These plugins can be easily initialized and configured through
//! the [`PhysicsPlugins`] plugin group.
//!
//! ```no_run
//! use bevy::prelude::*;
//! # #[cfg(feature = "2d")]
//! # use bevy_xpbd_2d::prelude::*;
//! # #[cfg(feature = "3d")]
//! use bevy_xpbd_3d::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
//!         // ...your other plugins, systems and resources
//!         .run();
//! }
//! ```
//!
//! Now you can use all of Bevy XPBD's [components] and [resources] to build whatever you want!
//!
//! For example, adding a [rigid body](RigidBody) with a [collider](Collider) is as simple as spawning an entity
//! with the [`RigidBody`] and [`Collider`] components:
//!
//! ```
//! # use bevy::prelude::*;
//! # #[cfg(feature = "2d")]
//! # use bevy_xpbd_2d::prelude::*;
//! # #[cfg(feature = "3d")]
//! # use bevy_xpbd_3d::prelude::*;
//! #
//! fn setup(mut commands: Commands) {
//!     commands.spawn((RigidBody::Dynamic, Collider::ball(0.5)));
//! }
//! ```
//!
//! To learn more about using Bevy XPBD, consider taking a look at the official [examples](#examples) and
//! how to accomplish some [common tasks](#common-tasks).
//!
//! To learn more about the structure of the engine, consider taking a look at the [plugins] and
//! [what XPBD actually is](#what-is-xpbd).
//!
//! ### Examples
//!
//! You can find 2D examples in [`crates/bevy_xpbd_2d/examples`](https://github.com/Jondolf/bevy_xpbd/tree/main/crates/bevy_xpbd_2d/examples)
//! and 3D examples in [`crates/bevy_xpbd_3d/examples`](https://github.com/Jondolf/bevy_xpbd/tree/main/crates/bevy_xpbd_3d/examples).
//!
//! You can run the examples with default features as you normally would.
//! For example, running the `cubes` example looks like this:
//!
//! ```bash
//! cargo run --example cubes
//! ```
//!
//! Note that the examples support both `f32` and `f64` precisions, so the code contains some feature-dependent types like `Scalar` and `Vector`.
//! In actual usage these are not needed, so you can just use `f32` or `f64` types depending on the features you have chosen.
//!
//! By default the examples use `f32`, so if you want to run the `f64` versions, you need to disable the default features and manually choose
//! the dimension and precision:
//!
//! ```bash
//! cargo run --example cubes --no-default-features --features "3d f64"
//! ```
//!
//! ### Common tasks
//!
//! - [Create a rigid body](RigidBody)
//! - [Define mass properties](RigidBody#adding-mass-properties)
//! - [Add a collider](Collider)
//! - [Listen to collision events](Collider#collision-events)
//! - [Define collision layers](CollisionLayers#creation)
//! - [Configure restitution (bounciness)](Restitution)
//! - [Configure friction](Friction)
//! - [Configure gravity](Gravity)
//! - [Apply external forces](ExternalForce)
//! - [Apply external torque](ExternalTorque)
//! - [Lock translational and rotational axes](LockedAxes)
//! - [Use joints](joints#using-joints)
//! - [Perform spatial queries](spatial_query)
//!     - [Ray casting](spatial_query#ray-casting)
//!     - [Shape casting](spatial_query#shape-casting)
//!     - [Point projection](spatial_query#point-projection)
//!     - [Intersection tests](spatial_query#intersection-tests)
//! - [Configure the physics timestep](PhysicsTimestep)
//! - [Configure the time scale](PhysicsTimescale)
//! - [Configure the substep count](SubstepCount)
//! - [Configure the schedule for running physics](PhysicsPlugins#custom-schedule)
//! - [Usage on servers](#can-the-engine-be-used-on-servers)
//! - [Create custom constraints](constraints#custom-constraints)
//! - [Replace built-in plugins with custom plugins](PhysicsPlugins#custom-plugins)
//!
//! ## Frequently asked questions
//!
//! ### How does Bevy XPBD compare to Rapier and bevy_rapier?
//!
//! Rapier is the biggest and most used physics engine in the Rust ecosystem, and it is currently
//! the most mature and feature-rich option.
//!
//! bevy_rapier is a great physics integration for Bevy, but it does have several problems:
//!
//! - It has to maintain a separate physics world and synchronize a ton of data with Bevy each frame
//! - The source code is difficult to inspect, as the vast majority of it is glue code and wrappers
//! for Bevy
//! - It has poor docs.rs documentation, and the documentation on rapier.rs is often outdated and
//! missing features
//! - It is hard to extend as it's not very modular or composable in design
//! - Overall, it doesn't have a native ECS-like feel outside of its public API
//!
//! Bevy XPBD on the other hand is built *for* Bevy *with* Bevy, and it uses the ECS for both the internals
//! and the public API. This removes the need for a separate physics world, reduces overhead, and makes
//! the source code much more approachable and easy to inspect for Bevy users.
//!
//! In part thanks to Bevy's modular architecture and the ECS, Bevy XPBD is also highly composable,
//! as it consists of several independent plugins and provides lots of options for configuration and extensions,
//! from [custom schedules](PhysicsPlugins#custom-schedule) and [plugins](PhysicsPlugins#custom-plugins) to
//! [custom joints](joints#custom-joints) and [constraints](constraints#custom-constraints).
//!
//! In terms of the physics implementation, Rapier uses an impulse/velocity based solver, while Bevy XPBD uses
//! [Extended Position Based Dynamics](#what-is-xpbd). On paper, XPBD should be more stable and robust,
//! but it hasn't been widely adopted in mainstream usage yet.
//!
//! One of the biggest disadvantages of Bevy XPBD is that it is still very young, so it can have lots of bugs,
//! some missing features, and fewer community resources and third party crates. However, it is growing quite
//! rapidly, and it is already pretty close to feature-parity with Rapier.
//!
//! At the end of the day, both engines are very solid options. If you are looking for a more mature and tested
//! physics integration, bevy_rapier is the better choice, but if you prefer an engine with less overhead
//! and a more native Bevy integration, consider using Bevy XPBD. Their core APIs are also quite similar,
//! so switching between them should be straightforward.
//!
//! ### Why is nothing happening?
//!
//! Make sure you have added the [`PhysicsPlugins`] plugin group and you have given your rigid bodies
//! a [`RigidBody`] component. See the [getting started](#getting-started) section.
//!
//! ### Why is everything moving so slowly?
//!
//! If your application is in 2D, you might be using pixels as length units. This will require you to use
//! larger velocities and forces than you would in 3D. Make sure you set [`Gravity`] to some larger value
//! as well, because its magnitude is 9.81 by default, which is tiny in pixels.
//!
//! Bevy XPBD doesn't have a "physics scale" yet, but it will most likely be added in the future
//! so that it's possible to define some kind of pixels per meter configuration.
//!
//! ### Why did my rigid body suddenly vanish?
//!
//! Make sure to [give your rigid bodies some mass](RigidBody#adding-mass-properties), either by adding a [`Collider`]
//! or a [`MassPropertiesBundle`]. If your bodies don't have any mass, any physical interaction is likely to
//! instantly give them infinite velocity.
//!
//! Bevy XPBD should automatically print warnings when it detects bodies with an invalid mass or inertia.
//!
//! ### Why is performance so bad?
//!
//! Make sure you are building your project in release mode using `cargo build --release`.
//!
//! You can further optimize builds by setting the number of codegen units in your `Cargo.toml` to 1,
//! although this will also increase build times.
//!
//! ```toml
//! [profile.release]
//! codegen-units = 1
//! ```
//!
//! Note that Bevy XPBD simply isn't very optimized yet, and it mostly runs on a single thread for now.
//! This will be addressed in future releases.
//!
//! ### Can the engine be used on servers?
//!
//! Yes! Networking often requires running the simulation in a specific schedule, and in Bevy XPBD you can
//! [set the schedule that runs physics](PhysicsPlugins#custom-schedule) and [configure the timestep](PhysicsTimestep)
//! to whatever you want.
//!
//! One configuration is to run the client in `FixedUpdate`, and to use [`PhysicsTimestep::FixedOnce`] on both the
//! server and the client to make sure the physics simulation is only advanced by one step each time the schedule runs.
//!
//! Note that while Bevy XPBD should be locally deterministic, it can produce slightly different results on different
//! machines.
//!
//! ### Something else?
//!
//! Physics engines are very large and Bevy XPBD is very young, so stability issues and bugs are to be expected.
//!
//! If you encounter issues, please consider first taking a look at the
//! [issues on GitHub](https://github.com/Jondolf/bevy_xpbd/issues) and
//! [open a new issue](https://github.com/Jondolf/bevy_xpbd/issues/new) if there already isn't one regarding your problem.
//!
//! You can also come and say hello on the [Bevy Discord server](https://discord.com/invite/gMUk5Ph).
//! There you can find a bevy_xpbd thread on the crate-help channel where you can ask questions.
//!
//! ## What is XPBD?
//!
//! *XPBD* or *Extended Position Based Dynamics* is a physics simulation method that extends
//! the traditional *PBD* to be more physically accurate and less dependent on time step size
//! and iteration count.
//!
//! Unlike force or impulse based physics simulation methods, XPBD mostly operates at the position-level,
//! which can produce more stable and reliable results, while allowing straightforward coupling
//! of [rigid bodies](RigidBody), soft bodies and fluids.
//!
//! ### Simulation loop
//!
//! At a high level, XPBD consists of a broad phase followed by a substepping loop that handles position
//! [integration](integrator), [constraint solving](solver), velocity updates, and a velocity solver that
//! handles dynamic friction and restitution.
//!
//! It looks roughly like this:
//!
//! ```ignore
//! while simulating:
//!     // Substep size
//!     h = ∆t / substep_count
//!
//!     // Broad phase
//!     broad_collision_pairs = collect_collision_pairs()
//!
//!     for substep_count:
//!         // Integrate
//!         for n particles and bodies:
//!             // Integrate position
//!             x_prev = x
//!             v = v + h * f_ext / m
//!             x = x + h * v
//!
//!             // Integrate rotation
//!             q_prev = q
//!             ω = ω + h * I^-1 * (τ_ext - (ω x (I * ω)))
//!             q = q + h * 0.5 * [ω_x, ω_y, ω_z, 0] * q
//!             q = q / |q|
//!
//!         // Narrow phase
//!         for pair in broad_collision_pairs:
//!             compute_contacts(pair)
//!
//!         // Solve constraints (contacts, joints etc.)
//!         solve_constraints(particles and bodies)
//!
//!         // Update velocities
//!         for n particles and bodies:
//!             v = (x - x_prev) / h
//!             ∆q = q * q_prev^-1
//!             ω = 2 * [∆q_x, ∆q_y, ∆q_z] / h
//!             ω = ∆q_w >= 0 ? ω : -ω
//!
//!         // Solve velocity constraints (dynamic friction and restitution)
//!         solve_velocities(particles and bodies)
//! ```
//!
//! where `h` is the [substep size](SubDeltaTime), `q` is the [rotation](Rotation) as a quaternion,
//! `ω` is the [angular velocity](AngularVelocity), `I` is the [angular inertia tensor](`Inertia`) and `τ` is the
//! [external torque](ExternalTorque).
//!
//! In Bevy XPBD, the simulation loop is handled by various plugins. The [`PhysicsSetupPlugin`] sets up
//! the Bevy schedules[^1][^2] and sets[^3][^4][^5], the [`BroadPhasePlugin`] manages the broad phase, the [`IntegratorPlugin`]
//! handles XPBD integration, and so on. You can find all of the plugins and their responsibilities [here](PhysicsPlugins).
//!
//! ### See also
//!
//! - [XPBD integration step](integrator)
//! - [Constraints and how to create them](constraints)
//! - [Schedules and sets used for the simulation loop](PhysicsSetupPlugin#schedules-and-sets)
//!
//! ### Learning resources
//!
//! If you want to learn more about XPBD, I recommend taking a look at some of the papers.
//! Especially the first one from 2020 was used heavily for the simulation loop and constraints in Bevy XPBD.
//!
//! - XPBD: Müller M, Macklin M, Chentanez N, Jeschke S, Kim T. 2020. *[Detailed Rigid Body Simulation with Extended Position Based Dynamics](https://matthias-research.github.io/pages/publications/PBDBodies.pdf)*.
//! - XPBD: Macklin M, Müller M, Chentanez N. 2016. *[XPBD: Position-Based Simulation of Compliant Constrained Dynamics](http://mmacklin.com/xpbd.pdf)*.
//!
//! The papers are quite academic, so you might instead prefer some videos and articles.
//! The first one by Ten Minute Physics (Matthias Müller, one of the XPBD researchers) is great for understanding
//! how XPBD differs from other simulation methods and how the constraints work.
//!
//! - Video: Ten Minute Physics. 2022. *[Getting ready to simulate the world with XPBD](https://youtu.be/jrociOAYqxA)*.
//! - Notes: Nolan Tait. *[Bevy Physics: XPBD](https://taintedcoders.com/bevy/xpbd/)*.
//! - Tutorial series: Johan Helsing. *[Tutorial: Making a physics engine with Bevy](https://johanhelsing.studio/posts/bevy-xpbd)*.
//! (inspired this project)
//!
//! ## License
//!
//! Bevy XPBD is free and open source. All code in the Bevy XPBD repository is dual-licensed under either:
//!
//! - MIT License ([LICENSE-MIT](https://github.com/Jondolf/bevy_xpbd/blob/main/LICENSE-MIT)
//! or <http://opensource.org/licenses/MIT>)
//! - Apache License, Version 2.0 ([LICENSE-APACHE](https://github.com/Jondolf/bevy_xpbd/blob/main/LICENSE-APACHE)
//! or <http://www.apache.org/licenses/LICENSE-2.0>)
//!
//! at your option.
//!
//! [^1]: [`PhysicsSchedule`]
//!
//! [^2]: [`SubstepSchedule`]
//!
//! [^3]: [`PhysicsSet`]
//!
//! [^4]: [`PhysicsStepSet`]
//!
//! [^5]: [`SubstepSet`]

#![allow(rustdoc::invalid_rust_codeblocks)]
#![warn(missing_docs)]

#[cfg(all(feature = "f32", feature = "f64"))]
compile_error!("feature \"f32\" and feature \"f64\" cannot be enabled at the same time");

#[cfg(all(feature = "2d", feature = "3d"))]
compile_error!("feature \"f2d\" and feature \"3d\" cannot be enabled at the same time");

#[cfg(all(feature = "2d", feature = "f32"))]
pub extern crate parry2d as parry;

#[cfg(all(feature = "2d", feature = "f64"))]
pub extern crate parry2d_f64 as parry;

#[cfg(all(feature = "3d", feature = "f32"))]
pub extern crate parry3d as parry;

#[cfg(all(feature = "3d", feature = "f64"))]
pub extern crate parry3d_f64 as parry;

pub mod components;
pub mod constraints;
pub mod math;
pub mod plugins;
pub mod resources;

/// Re-exports common components, bundles, resources, plugins and types.
pub mod prelude {
    pub use crate::{
        components::*,
        constraints::{joints::*, *},
        plugins::*,
        resources::*,
        PhysicsSet,
    };
    pub(crate) use crate::{math::*, *};
    pub use bevy_xpbd_derive::*;
}
pub use prelude::setup::{pause, resume};

mod utils;

#[cfg(test)]
mod tests;

use bevy::{
    app::PluginGroupBuilder,
    ecs::schedule::{LogLevel, ScheduleLabel},
    prelude::*,
};
use parry::math::Isometry;
#[allow(unused_imports)]
use prelude::*;

/// Responsible for advancing the physics simulation. This is run in [`PhysicsSet::StepSimulation`].
///
/// See [`PhysicsStepSet`] for the system sets that are run in this schedule.
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct PhysicsSchedule;

/// The substepping schedule that runs in [`PhysicsStepSet::Substeps`].
/// The number of substeps per physics step is configured through the [`SubstepCount`] resource.
///
/// See [`SubstepSet`] for the system sets that are run in this schedule.
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct SubstepSchedule;

/// The schedule that runs in [`SubstepSet::PostProcessCollisions`].
///
/// Empty by default.
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
/// the broad phase and the substepping loop.
/// - [`SubstepSchedule`]: Responsible for running the substepping loop in [`PhysicsStepSet::Substeps`].
/// - [`SubstepSet`]: System sets for the steps of the substepping loop, like position integration and
/// the constraint solver.
/// - [`PostProcessCollisions`]: Responsible for running the post-process collisions group in
/// [`SubstepSet::PostProcessCollisions`]. Empty by default.
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
/// 1. Broad phase
/// 2. Substeps
///     1. Integrate
///     2. Narrow phase
///     3. Solve positional and angular constraints
///     4. Update velocities
///     5. Solve velocity constraints (dynamic friction and restitution)
/// 3. Sleeping
/// 4. Spatial queries
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsStepSet {
    /// Responsible for collecting pairs of potentially colliding entities into [`BroadCollisionPairs`] using
    /// [AABB](ColliderAabb) intersection tests.
    ///
    /// See [`BroadPhasePlugin`].
    BroadPhase,
    /// Responsible for substepping, which is an inner loop inside a physics step.
    ///
    /// See [`SubstepSet`] and [`SubstepSchedule`].
    Substeps,
    /// Responsible for controlling when bodies should be deactivated and marked as [`Sleeping`].
    ///
    /// See [`SleepingPlugin`].
    Sleeping,
    /// Responsible for spatial queries like [ray casting](`RayCaster`) and shape casting.
    ///
    /// See [`SpatialQueryPlugin`].
    SpatialQuery,
}

/// System sets for the the steps in the inner substepping loop. These are typically run in the [`SubstepSchedule`].
///
/// 1. Integrate
/// 2. Narrow phase
/// 3. Post-process collisions
/// 4. Solve positional and angular constraints
/// 5. Update velocities
/// 6. Solve velocity constraints (dynamic friction and restitution)
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SubstepSet {
    /// Responsible for integrating Newton's 2nd law of motion,
    /// applying forces and moving entities according to their velocities.
    ///
    /// See [`IntegratorPlugin`].
    Integrate,
    /// Responsible for computing contacts between entities and sending collision events.
    ///
    /// See [`NarrowPhasePlugin`].
    NarrowPhase,
    /// Responsible for running the [`PostProcessCollisions`] schedule to allow user-defined systems
    /// to filter and modify collisions.
    ///
    /// If you want to modify or remove collisions after [`SubstepSet::NarrowPhase`], you can
    /// add custom systems to this set, or to [`PostProcessCollisions`].
    ///
    /// See [`NarrowPhasePlugin`].
    PostProcessCollisions,
    /// The [solver] iterates through [constraints] and solves them.
    ///
    /// **Note**: If you want to [create your own constraints](constraints#custom-constraints),
    /// you should add them in [`SubstepSet::SolveUserConstraints`]
    /// to avoid system order ambiguities.
    ///
    /// See [`SolverPlugin`].
    SolveConstraints,
    /// The [solver] iterates through custom [constraints] created by the user and solves them.
    ///
    /// You can [create new constraints](constraints#custom-constraints) by implementing [`XpbdConstraint`]
    /// for a component and adding the [constraint system](solve_constraint) to this set.
    ///
    /// See [`SolverPlugin`].
    SolveUserConstraints,
    /// Responsible for updating velocities after [constraint](constraints) solving.
    ///
    /// See [`SolverPlugin`].
    UpdateVelocities,
    /// Responsible for applying dynamic friction, restitution and joint damping at the end of the
    /// substepping loop.
    ///
    /// See [`SolverPlugin`].
    SolveVelocities,
    /// Responsible for applying translation accumulated during the substep.
    ///
    /// See [`SolverPlugin`].
    ApplyTranslation,
}
