//! # Bevy XPBD
//!
//! **Bevy XPBD** is a 2D and 3D physics engine based on [*Extended Position Based Dynamics* (XPBD)](#what-is-xpbd)
//! for the [Bevy game engine](https://bevyengine.org/).
//!
//! ## Design
//!
//! Below are some of the core design principles used in Bevy XPBD.
//!
//! - Made with Bevy, for Bevy.
//! - Use the ECS as much as possible. A Bevy physics engine shouldn't need to maintain a separate physics world.
//! - Provide an ergonomic and familiar API. Creating and using [rigid bodies](RigidBody) and [colliders](Collider) shouldn't be hard.
//! - Use a highly modular [plugin architecture](plugins). Many large projects require custom-made solutions, so users should be able to
//! replace parts of the engine with their own implementations.
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
//! - Material properties like [restitution](Restitution) and [friction](Friction)
//! - External [forces](ExternalForce) and [torque](ExternalTorque)
//! - [Gravity](Gravity)
//! - [Joints](joints)
//! - Built-in [constraints] and support for [custom constraints](constraints#custom-constraints)
//! - Automatically deactivating bodies with [sleeping](sleeping)
//! - Configurable [timesteps](PhysicsTimestep) and [substepping](SubstepCount)
//! - `f32`/`f64` precision (`f32` by default)
//!
//! ## Getting started
//!
//! This short guide should help you get started with Bevy XPBD.
//!
//! ### Adding the dependency
//!
//! For a 2D game, add the `bevy_xpbd_2d` crate to your `Cargo.toml` like this:
//!
//! ```toml
//! [dependencies]
//! bevy_xpbd_2d = "0.1"
//! ```
//!
//! Similarly for a 3D game, add `bevy_xpbd_3d`:
//!
//! ```toml
//! [dependencies]
//! bevy_xpbd_3d = "0.1"
//! ```
//!
//! By default, Bevy XPBD uses `f32` numbers. If you encounter instability or use small [timesteps](PhysicsTimestep),
//! you might want to use `f64` instead. You can change these kinds of features by disabling
//! the default features and manually specifying the feature flags you want:
//!
//! ```toml
//! [dependencies]
//! # Add 3D Bevy XPBD with double-precision floating point numbers
//! bevy_xpbd_3d = { version = "0.1", features = ["3d", "f64"], default-features = false }
//! ```
//!
//! ### Feature flags
//!
//! - `2d` enables simulation on the `x` and `y` axes. Enabled by default for `bevy_xpbd_2d`. Incompatible with `3d`.
//! - `3d` enables simulation on the `x`, `y` and `z` axes. Enabled by default for `bevy_xpbd_3d`. Incompatible with `2d`.
//! - `f32` enables using `f32` numbers. Incompatible with `f64`.
//! - `f64` enables using `f64` numbers. Recommended when encountering stability problems, especially with
//! small timesteps. Incompatible with `f32`.
//! - `debug-plugin` enables the `PhysicsDebugPlugin` used for rendering physics objects and events like [AABBs](ColliderAabb)
//! and [contacts](Contact).
//! - `simd` enables [SIMD](https://en.wikipedia.org/wiki/Single_instruction,_multiple_data) optimizations.
//! - `enhanced-determinism` enables increased determinism. (Note: cross-platform determinism doesn't work yet, even
//! with this feature enabled)
//!
//! ### Install the plugin
//!
//! Bevy XPBD is designed to be very modular. It is built from many different [plugins] that
//! manage different parts of the engine. These plugins can be easily initialized and configured through
//! the [`PhysicsPlugins`] plugin group.
//!
//! ```ignore
//! use bevy::prelude::*;
//! use bevy_xpbd_3d::prelude::*;
//!
//! App::new()
//!     .add_plugins(DefaultPlugins)
//!     .add_plugins(PhysicsPlugins)
//!     // ...your other plugins, systems and resources
//!     .run();
//! ```
//!
//! Now you can use all of Bevy XPBD's [components] and [resources] to build whatever you want!
//!
//! For example, adding a [rigid body](RigidBody) with a [collider](Collider) is as simple as spawning an entity
//! with the [`RigidBody`] and [`Collider`] components:
//!
//! ```ignore
//! commands.spawn((RigidBody::Dynamic, Collider::ball(0.5)));
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
//! Note that the examples support both f32 and f64 precisions, so the code contains some feature-dependent types like `Scalar` and `Vector`.
//! In actual usage these are not needed, so you can just use `f32` or `f64` types depending on the features you have chosen.
//!
//! By default the examples use `f64`, so if you want to run the `f32` versions, you need to disable the default features and manually choose
//! the dimension and precision:
//!
//! ```bash
//! cargo run --example cubes --no-default-features --features 3d,f32
//! ```
//!
//! ### Common tasks
//!
//! - [Create a rigid body](RigidBody)
//! - [Add a collider](Collider)
//! - [Listen to collision events](Collider#collision-events)
//! - [Definen collision layers](CollisionLayers)
//! - [Define mass properties](RigidBody#mass-properties)
//! - [Use joints](joints)
//! - [Configure gravity](Gravity)
//! - [Configure restitution](Restitution)
//! - [Configure friction](Friction)
//! - [Configure the physics timestep](PhysicsTimestep)
//! - [Configure the substep count](SubstepCount)
//! - [Create custom constraints](constraints#custom-constraints)
//! - [Replace built-in plugins with custom plugins](PhysicsPlugins#custom-plugins)
//!
//! ## What is XPBD?
//!
//! *XPBD* or *Extended Position Based Dynamics* is a physics simulation method that extends
//! the traditional *PBD* to be more physically accurate and less dependent on time step size
//! and iteration count.
//!
//! At a high level, XPBD consists of a broad phase followed by a substepping loop that handles position
//! [integration](integrator), [constraint solving](solver), and velocity updates. Unlike in force or impulse
//! based simulation methods, [constraints] operate directly on positions, which often provides more reliable
//! and stable results, and allows straightforward coupling of [rigid bodies](RigidBody), soft bodies and fluids.
//!
//! Below is a high level overview of the XPBD simulation loop.
//!
//! ```
//! while simulating:
//!     // Substep size
//!     h = ∆t / substep_count
//!
//!     // Broad phase
//!     collect_collision_pairs()
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
//!         // Solve constraints (1 iteration and many substeps recommended)
//!         for iteration_count:
//!             solve_constraints(particles and bodies)
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
//! where `h` is the [substep size](SubDeltaTime), `q` is the [rotation](Rot) as a quaternion,
//! `ω` is the [angular velocity](AngVel), `I` is the [angular inertia tensor](`Inertia`) and `τ` is the
//! [external torque](ExternalTorque).
//!
//! In Bevy XPBD, the simulation loop is handled by various plugins. The [`PhysicsSetupPlugin`] sets up
//! the Bevy schedules[^1][^2] and sets[^3][^4], the [`BroadPhasePlugin`] manages the broad phase, the [`IntegratorPlugin`] handles
//! XPBD integration, and so on. You can find all of the plugins and their responsibilities [here](PhysicsPlugins).
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
//! - Tutorial series: Johan Helsing. *[Tutorial: Making a physics engine with Bevy](https://johanhelsing.studio/posts/bevy-xpbd)*.
//! (inspired this project)
//!
//! License
//!
//! Bevy XPBD is free and open source. All code in this repository is dual-licensed under either:
//!
//! - MIT License ([LICENSE-MIT](https://github.com/Jondolf/bevy_xpbd/LICENSE-MIT)
//! or http://opensource.org/licenses/MIT)
//! - Apache License, Version 2.0 ([LICENSE-APACHE](https://github.com/Jondolf/bevy_xpbd/LICENSE-APACHE)
//! or http://www.apache.org/licenses/LICENSE-2.0)
//!
//! at your option.
//!
//! [^1]: [`PhysicsSchedule`]
//!
//! [^2]: [`SubstepSchedule`]
//!
//! [^3]: [`PhysicsSet`]
//!
//! [^4]: [`SubstepSet`]

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

pub mod collision;
pub mod components;
pub mod constraints;
pub mod math;
pub mod plugins;
pub mod resources;

/// Reimports common components, bundles, resources, plugins and types.
pub mod prelude {
    pub use crate::{
        collision::*,
        components::*,
        constraints::{joints::*, *},
        math::*,
        plugins::*,
        resources::*,
        *,
    };
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
use prelude::*;
