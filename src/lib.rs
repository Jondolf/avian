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
        components::*,
        constraints::{joints::*, *},
        math::*,
        plugins::*,
        resources::*,
        *,
    };
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

/// This plugin group will add the following physics plugins:
///
/// - [`PhysicsSetupPlugin`]
/// - [`PreparePlugin`]
/// - [`BroadPhasePlugin`]
/// - [`IntegratorPlugin`]
/// - [`SolverPlugin`]
/// - [`SleepingPlugin`]
/// - [`SyncPlugin`]
///
/// Note that [`PhysicsSetupPlugin`] initializes all of the schedules, sets and resources required
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
        let builder = PluginGroupBuilder::start::<Self>();

        #[cfg(feature = "debug-render-aabbs")]
        {
            builder.add(DebugLinesPlugin::default());
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

#[cfg(feature = "debug-render-aabbs")]
fn draw_aabbs(aabbs: Query<&ColliderAabb>, mut lines: ResMut<DebugLines>) {
    #[cfg(feature = "2d")]
    for aabb in aabbs.iter() {
        let v1 = Vec3::new(aabb.mins.x, aabb.mins.y, 0.0);
        let v2 = Vec3::new(aabb.maxs.x, aabb.mins.y, 0.0);
        let v3 = Vec3::new(aabb.maxs.x, aabb.maxs.y, 0.0);
        let v4 = Vec3::new(aabb.mins.x, aabb.maxs.y, 0.0);

        lines.line(v1, v2, 0.0);
        lines.line(v2, v3, 0.0);
        lines.line(v3, v4, 0.0);
        lines.line(v4, v1, 0.0);
    }

    #[cfg(feature = "3d")]
    for aabb in aabbs.iter() {
        let v1 = Vec3::new(aabb.mins.x, aabb.mins.y, aabb.mins.z);
        let v2 = Vec3::new(aabb.maxs.x, aabb.mins.y, aabb.mins.z);
        let v3 = Vec3::new(aabb.maxs.x, aabb.maxs.y, aabb.mins.z);
        let v4 = Vec3::new(aabb.mins.x, aabb.maxs.y, aabb.mins.z);
        let v5 = Vec3::new(aabb.mins.x, aabb.mins.y, aabb.maxs.z);
        let v6 = Vec3::new(aabb.maxs.x, aabb.mins.y, aabb.maxs.z);
        let v7 = Vec3::new(aabb.maxs.x, aabb.maxs.y, aabb.maxs.z);
        let v8 = Vec3::new(aabb.mins.x, aabb.maxs.y, aabb.maxs.z);

        lines.line(v1, v2, 0.0);
        lines.line(v2, v3, 0.0);
        lines.line(v3, v4, 0.0);
        lines.line(v4, v1, 0.0);
        lines.line(v5, v6, 0.0);
        lines.line(v6, v7, 0.0);
        lines.line(v7, v8, 0.0);
        lines.line(v8, v5, 0.0);
        lines.line(v1, v5, 0.0);
        lines.line(v2, v6, 0.0);
        lines.line(v3, v7, 0.0);
        lines.line(v4, v8, 0.0);
    }
}
