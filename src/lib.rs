//! # Bevy XPBD
//!
//! **Bevy XPBD** is a 2D and 3D physics engine based on
//! [*Extended Position Based Dynamics* (XPBD)](#what-is-xpbd) for
//! the [Bevy game engine](https://bevyengine.org/).
//!
//! Check out the [GitHub repository](https://github.com/Jondolf/bevy_xpbd)
//! for more information about the design, read the [Getting started](#getting-started)
//! guide below to get up to speed, and take a look at the [Table of contents](#table-of-contents)
//! for an overview of the engine's features and their documentation.
//!
//! You can also check out the [FAQ](#frequently-asked-questions), and if you encounter
//! any further problems, consider saying hello on the [Bevy Discord](https://discord.gg/bevy)!
//!
//! ## Getting started
//!
//! This short guide should help you get started with Bevy XPBD.
//!
//! ### Add the dependency
//!
//! First, add `bevy_xpbd_2d` or `bevy_xpbd_3d` to the dependencies in your `Cargo.toml`:
//!  
//! ```toml
//! # For 2D applications:
//! [dependencies]
//! bevy_xpbd_2d = "0.3"
//!
//! # For 3D applications:
//! [dependencies]
//! bevy_xpbd_3d = "0.3"
//!
//! # If you want to use the most up-to-date version, you can follow the main branch:
//! [dependencies]
//! bevy_xpbd_3d = { git = "https://github.com/Jondolf/bevy_xpbd", branch = "main" }
//! ```
//!
//! You can specify features by disabling the default features and manually adding
//! the feature flags you want:
//!
//! ```toml
//! [dependencies]
//! # Add 3D Bevy XPBD with double-precision floating point numbers
//! bevy_xpbd_3d = { version = "0.3", default-features = false, features = ["3d", "f64"] }
//! ```
//!
//! ### Feature flags
//!
//! | Feature                | Description                                                                                                                      | Default feature         |
//! | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------- | ----------------------- |
//! | `2d`                   | Enables 2D physics. Incompatible with `3d`.                                                                                      | Yes (`bevy_xpbd_2d`)    |
//! | `3d`                   | Enables 3D physics. Incompatible with `2d`.                                                                                      | Yes (`bevy_xpbd_3d`)    |
//! | `f32`                  | Enables `f32` precision for physics. Incompatible with `f64`.                                                                    | Yes                     |
//! | `f64`                  | Enables `f64` precision for physics. Incompatible with `f32`.                                                                    | No                      |
#![cfg_attr(
    feature = "3d",
    doc = "| `collider-from-mesh`   | Allows you to create [`Collider`]s from `Mesh`es.                                                                                | Yes                     |"
)]
#![cfg_attr(
    feature = "3d",
    doc = "| `async-collider`       | Allows you to generate [`Collider`]s from mesh handles and scenes.                                                               | Yes                     |"
)]
//! | `debug-plugin`         | Enables physics debug rendering using the [`PhysicsDebugPlugin`]. The plugin must be added separately.                           | Yes                     |
//! | `enhanced-determinism` | Enables increased determinism.                                                                                                   | No                      |
//! | `parallel`             | Enables some extra multithreading, which improves performance for larger simulations but can add some overhead for smaller ones. | Yes                     |
//! | `simd`                 | Enables [SIMD] optimizations.                                                                                                    | No                      |
//! | `serialize`            | Enables support for serialization and deserialization using Serde.                                                               | No                      |
//!
//! [SIMD]: https://en.wikipedia.org/wiki/Single_instruction,_multiple_data
//!
//! ### Install the plugin
//!
//! Bevy XPBD is designed to be very modular. It is built from several [plugins] that
//! manage different parts of the engine. These plugins can be easily initialized and configured through
//! the [`PhysicsPlugins`] plugin group.
//!
//! ```no_run
//! use bevy::prelude::*;
#![cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
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
//! use bevy::prelude::*;
#![cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
//!
//! fn setup(mut commands: Commands) {
//!     commands.spawn((RigidBody::Dynamic, Collider::ball(0.5)));
//! }
//! ```
//!
//! You can find lots of [usage examples](https://github.com/Jondolf/bevy_xpbd#more-examples)
//! in the project's [repository](https://github.com/Jondolf/bevy_xpbd).
//!
//! ## Table of contents
//!
//! Below is a structured overview of the documentation for the various
//! features of the engine.
//!
//! ### Rigid bodies
//!
//! - [Rigid body types](RigidBody#rigid-body-types)
//! - [Creating rigid bodies](RigidBody#creation)
//! - [Movement](RigidBody#movement)
//!     - [Linear](LinearVelocity) and [angular](AngularVelocity) velocity
//!     - [Forces](ExternalForce), [torque](ExternalTorque), and [linear](ExternalImpulse) and [angular](ExternalAngularImpulse) impulses
//! - [Gravity] and [gravity scale](GravityScale)
//! - [Mass properties](RigidBody#mass-properties)
//! - [Linear](LinearDamping) and [angular](AngularDamping) velocity damping
//! - [Lock translational and rotational axes](LockedAxes)
//! - [Dominance]
//! - [Automatic deactivation with sleeping](Sleeping)
//!
//! ### Collision detection
//!
//! - [Colliders](Collider)
//!     - [Creation](Collider#creation)
//!     - [Density](ColliderDensity)
//!     - [Friction] and [restitution](Restitution) (bounciness)
//!     - [Collision layers](CollisionLayers)
//!     - [Sensors](Sensor)
#![cfg_attr(
    feature = "3d",
    doc = "    - Creating colliders from meshes with [`AsyncCollider`] and [`AsyncSceneCollider`]"
)]
//! - [Get colliding entities](CollidingEntities)
//! - [Collision events](ContactReportingPlugin#collision-events)
//! - [Accessing, filtering and modifying collisions](Collisions)
//! - [Manual contact queries](contact_query)
//!
//! ### Constraints and joints
//!
//! - [Constraints](constraints) (advanced)
//! - [Joints](joints)
//!     - [Fixed joint](FixedJoint)
//!     - [Distance joint](DistanceJoint)
//!     - [Prismatic joint](PrismaticJoint)
//!     - [Revolute joint](RevoluteJoint)
//!     - [Spherical joint](SphericalJoint)
//!
//! Joint motors and articulations are not supported yet, but they will be implemented in a future release.
//!
//! ### Spatial queries
//!
//! - [Spatial query types](spatial_query)
//!     - [Raycasting](spatial_query#raycasting) and [`RayCaster`]
//!     - [Shapecasting](spatial_query#shapecasting) and [`ShapeCaster`]
//!     - [Point projection](spatial_query#point-projection)
//!     - [Intersection tests](spatial_query#intersection-tests)
//! - [Spatial query filters](SpatialQueryFilter)
//! - [The `SpatialQuery` system parameter](SpatialQuery)
//!
//! ### Configuration
//!
//! - [Gravity]
//! - [Physics timestep](Physics#usage)
//! - [Physics speed](Physics#physics-speed)
//! - [Configure simulation fidelity with substeps](SubstepCount)
//! - [Render physics objects for debugging](PhysicsDebugPlugin)
//!
//! ### Scheduling
//!
//! - [Schedules and sets](PhysicsSetupPlugin#schedules-and-sets)
//!     - [`PhysicsSet`]
//!     - [`PhysicsSchedule`] and [`PhysicsStepSet`]
//!     - [`SubstepSchedule`] and [`SubstepSet`]
//!     - [`PostProcessCollisions`] schedule
//!     - [`PrepareSet`]
//! - [Configure the schedule used for running physics](PhysicsPlugins#custom-schedule)
//! - [Pausing, resuming and stepping physics](Physics#pausing-resuming-and-stepping-physics)
//! - [Usage on servers](#can-the-engine-be-used-on-servers)
//!
//! ### Architecture
//!
//! - [List of plugins and their responsibilities](PhysicsPlugins)
//! - [What is Extended Position Based Dynamics?](#what-is-xpbd)
//! - Extending and modifying the engine
//!     - [Custom plugins](PhysicsPlugins#custom-plugins)
//!     - [Custom constraints](constraints#custom-constraints)
//!     - [Custom joints](joints#custom-joints)
//!
//! ## Frequently asked questions
//!
//! - [How does Bevy XPBD compare to Rapier and bevy_rapier?](#how-does-bevy-xpbd-compare-to-rapier-and-bevy_rapier)
//! - [Why is nothing happening?](#why-is-nothing-happening)
//! - [Why is everything moving so slowly?](#why-is-everything-moving-so-slowly)
//! - [Why did my rigid body suddenly vanish?](#why-did-my-rigid-body-suddenly-vanish)
//! - [Why is performance so bad?](#why-is-performance-so-bad)
//! - [Why does my camera following jitter?](#why-does-my-camera-following-jitter)
//! - [Is there a character controller?](#is-there-a-character-controller)
//! - [Why are there separate `Position` and `Rotation` components?](#why-are-there-separate-position-and-rotation-components)
//! - [Can the engine be used on servers?](#can-the-engine-be-used-on-servers)
//! - [Something else?](#something-else)
//!
//! ### How does Bevy XPBD compare to Rapier and bevy_rapier?
//!
//! Rapier is the biggest and most used physics engine in the Rust ecosystem, and it is currently
//! the most mature and feature-rich option.
//!
//! `bevy_rapier` is a great physics integration for Bevy, but it does have several problems:
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
//! physics integration, `bevy_rapier` is the better choice, but if you prefer an engine with less overhead
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
//! ### Why does my camera following jitter?
//!
//! When you write a system that makes the camera follow a physics entity, you might notice some jitter.
//!
//! To fix this, the system needs to:
//!
//! - Run after physics so that it has the up-to-date position of the player.
//! - Run before transform propagation so that your changes to the camera's `Transform` are written
//! to the camera's `GlobalTransform` before the end of the frame.
//!
//! The following ordering constraints should resolve the issue.
//!
//! ```
//! # use bevy::prelude::*;
//! # use bevy::transform::TransformSystem;
#![cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
//! #
//! # fn main() {
//! #     let mut app = App::new();
//! #
//! app.add_systems(
//!     PostUpdate,
//!     camera_follow_player
//!         .after(PhysicsSet::Sync)
//!         .before(TransformSystem::TransformPropagate),
//! );
//! # }
//! #
//! # fn camera_follow_player() {}
//! ```
//!
//! ### Is there a character controller?
//!
//! Bevy XPBD does not have a built-in character controller, so if you need one,
//! you will need to implement it yourself. However, third party character controllers
//! like [`bevy_tnua`](https://github.com/idanarye/bevy-tnua) support Bevy XPBD, and [`bevy_mod_wanderlust`](https://github.com/PROMETHIA-27/bevy_mod_wanderlust)
//! and others are also likely to get Bevy XPBD support soon.
//!
//! For custom character controllers, you can take a look at the
#![cfg_attr(
    feature = "2d",
    doc = "[`dynamic_character_2d`] and [`kinematic_character_2d`] examples to get started."
)]
#![cfg_attr(
    feature = "3d",
    doc = "[`dynamic_character_3d`] and [`kinematic_character_3d`] examples to get started."
)]
//!
#![cfg_attr(
    feature = "2d",
    doc = "[`dynamic_character_2d`]: https://github.com/Jondolf/bevy_xpbd/tree/main/crates/bevy_xpbd_2d/examples/dynamic_character_2d
[`kinematic_character_2d`]: https://github.com/Jondolf/bevy_xpbd/tree/main/crates/bevy_xpbd_2d/examples/kinematic_character_2d"
)]
#![cfg_attr(
    feature = "3d",
    doc = "[`dynamic_character_3d`]: https://github.com/Jondolf/bevy_xpbd/tree/main/crates/bevy_xpbd_3d/examples/dynamic_character_3d
[`kinematic_character_3d`]: https://github.com/Jondolf/bevy_xpbd/tree/main/crates/bevy_xpbd_3d/examples/kinematic_character_3d"
)]
//!
//! ### Why are there separate `Position` and `Rotation` components?
//!
//! While `Transform` can be used for the vast majority of things, Bevy XPBD internally
//! uses separate [`Position`] and [`Rotation`] components. These are automatically
//! kept in sync by the [`SyncPlugin`].
//!
//! There are several reasons why the separate components are currently used.
//!
//! - Position and rotation should be global from the physics engine's point of view.
//! - Transform scale and shearing can cause issues and rounding errors in physics.
//! - Transform hierarchies can be problematic.
//! - There is no `f64` version of `Transform`.
//! - There is no 2D version of `Transform` (yet), and having a 2D version can optimize several computations.
//! - When position and rotation are separate, we can technically have more systems running in parallel.
//! - Only rigid bodies have rotation, particles typically don't (although we don't make a distinction yet).
//!
//! In external projects however, using [`Position`] and [`Rotation`] is only necessary when you
//! need to manage positions in the [`SubstepSchedule`]. Elsewhere, you should be able to use `Transform`.
//!
//! There is also a possibility that we will revisit this if/when Bevy has a `Transform2d` component.
//! Using `Transform` feels more idiomatic and simple, so it would be nice if it could be used directly
//! as long as we can get around the drawbacks.
//!
//! ### Can the engine be used on servers?
//!
//! Yes! Networking often requires running the simulation in a specific schedule, and in Bevy XPBD you can
//! [set the schedule that runs physics](PhysicsPlugins#custom-schedule) and [configure the timestep](Physics)
//! to whatever you want.
//!
//! One configuration is to run the client in `FixedUpdate`, and to use a fixed timestep for [`Time<Physics>`](Physics)
//! on both the server and the client to make sure the physics simulation is only advanced by one step
//! each time the schedule runs.
//!
//! Note that while Bevy XPBD should be locally deterministic, it can produce slightly different results on different
//! machines.
//!
//! ### Something else?
//!
//! Physics engines are very large and Bevy XPBD is young, so stability issues and bugs are to be expected.
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
//! where `h` is the substep size, `q` is the [rotation](Rotation) as a quaternion,
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
#![warn(clippy::doc_markdown, missing_docs)]

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
    #[cfg(feature = "debug-plugin")]
    pub use crate::plugins::debug::*;
    pub use crate::{
        components::*,
        constraints::{joints::*, *},
        plugins::{
            collision::{
                broad_phase::BroadCollisionPairs,
                contact_reporting::{Collision, CollisionEnded, CollisionStarted},
                narrow_phase::NarrowPhaseConfig,
                *,
            },
            prepare::{init_transforms, update_mass_properties, PrepareConfig, PreparePlugin},
            setup::*,
            solver::solve_constraint,
            spatial_query::*,
            *,
        },
        resources::*,
        PhysicsSet, PostProcessCollisions,
    };
    pub(crate) use crate::{math::*, *};
    pub use bevy_xpbd_derive::*;
}

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

/// A schedule where you can add systems to filter or modify collisions
/// using the [`Collisions`] resource.
///
/// The schedule is empty by default and runs in [`SubstepSet::PostProcessCollisions`].
///
/// ## Example
///
/// Below is an example of how you could add a system that filters collisions.
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
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
/// 3. Report contacts (send collision events)
/// 4. Sleeping
/// 5. Spatial queries
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
    /// Responsible for sending collision events and updating [`CollidingEntities`].
    ///
    /// See [`ContactReportingPlugin`].
    ReportContacts,
    /// Responsible for controlling when bodies should be deactivated and marked as [`Sleeping`].
    ///
    /// See [`SleepingPlugin`].
    Sleeping,
    /// Responsible for spatial queries like [raycasting](`RayCaster`) and shapecasting.
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
/// 7. Store contact impulses in [`Collisions`].
/// 8. Apply [`AccumulatedTranslation`] to positions.
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
    /// Contact impulses computed by the solver are stored in contacts in [`Collisions`].
    ///
    /// See [`SolverPlugin`].
    StoreImpulses,
    /// Responsible for applying translation accumulated during the substep.
    ///
    /// See [`SolverPlugin`].
    ApplyTranslation,
}
