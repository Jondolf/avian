//! # Bevy XPBD
//!
//! **Bevy XPBD** is a 2D and 3D physics engine based on *Extended Position Based Dynamics* (XPBD) for the [Bevy game engine](https://bevyengine.org/). The *Entity Component System* (ECS) is used heavily throughout the engine to enable enhanced parallellism, configurability and familiarity, while making the engine fit better into the Bevy ecosystem.
//!
//! XPBD is an improved variant of traditional *Position Based Dynamics* (PBD).
//! It provides unconditionally stable, time step independent and physically accurate simulations that use simple constraint projection to handle things like contacts, joints, and interactions between rigid bodies, soft bodies and fluids.
//!
//! To understand the algorithm better, it's worth checking out some of the academic papers:
//!
//! - Müller M, Macklin M, Chentanez N, Jeschke S, Kim T. 2020. *[Detailed Rigid Body Simulation with Extended Position Based Dynamics](https://matthias-research.github.io/pages/publications/PBDBodies.pdf)*.
//!
//! - Macklin M, Müller M, Chentanez N. 2016. *[XPBD: Position-Based Simulation of Compliant Constrained Dynamics](http://mmacklin.com/xpbd.pdf)*.

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
