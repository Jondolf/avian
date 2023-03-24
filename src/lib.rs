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

pub mod bundles;
pub mod collision;
pub mod components;
pub mod constraints;
pub mod math;
pub mod resources;
pub mod steps;

pub mod prelude {
    pub use crate::{
        bundles::*,
        components::*,
        constraints::{joints::*, *},
        math::*,
        resources::*,
        steps::*,
        *,
    };
}

mod utils;

#[cfg(test)]
mod tests;

use bevy::{
    ecs::schedule::{LogLevel, ScheduleLabel},
    prelude::*,
};
use parry::math::Isometry;
use prelude::*;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
struct FixedUpdateSet;

pub struct XpbdPlugin;

/// The high-level Xpbd physics schedule, run once per physics frame
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct XpbdSchedule;

/// The substepping schedule, the number of substeps per physics step is
/// configured through the [`NumSubsteps`] resource
#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct XpbdSubstepSchedule;

impl Plugin for XpbdPlugin {
    fn build(&self, app: &mut App) {
        // Init resources and register component types
        app.init_resource::<PhysicsTimestep>()
            .init_resource::<DeltaTime>()
            .init_resource::<SubDeltaTime>()
            .init_resource::<NumSubsteps>()
            .init_resource::<NumPosIters>()
            .init_resource::<XpbdLoop>()
            .init_resource::<Gravity>()
            .register_type::<RigidBody>()
            .register_type::<Pos>()
            .register_type::<Rot>()
            .register_type::<PrevPos>()
            .register_type::<PrevRot>()
            .register_type::<LinVel>()
            .register_type::<AngVel>()
            .register_type::<PreSolveLinVel>()
            .register_type::<PreSolveAngVel>()
            .register_type::<Restitution>()
            .register_type::<Friction>()
            .register_type::<ExternalForce>()
            .register_type::<ExternalTorque>()
            .register_type::<Mass>()
            .register_type::<InvMass>()
            .register_type::<Inertia>()
            .register_type::<InvInertia>()
            .register_type::<LocalCom>();

        let mut xpbd_schedule = Schedule::default();

        xpbd_schedule.set_build_settings(bevy::ecs::schedule::ScheduleBuildSettings {
            ambiguity_detection: LogLevel::Error,
            ..default()
        });

        xpbd_schedule.configure_sets(
            (
                PhysicsSet::Prepare,
                PhysicsSet::BroadPhase,
                PhysicsSet::Substeps,
                PhysicsSet::Sync,
            )
                .chain(),
        );

        app.add_schedule(XpbdSchedule, xpbd_schedule);

        let mut substep_schedule = Schedule::default();

        substep_schedule.set_build_settings(bevy::ecs::schedule::ScheduleBuildSettings {
            ambiguity_detection: LogLevel::Error,
            ..default()
        });

        app.add_schedule(XpbdSubstepSchedule, substep_schedule);

        // Add system set for running physics schedule
        app.configure_set(
            FixedUpdateSet
                .before(CoreSet::Update)
                .in_base_set(CoreSet::PreUpdate),
        );
        app.add_system(run_physics_schedule.in_set(FixedUpdateSet));

        app.add_system(
            run_substep_schedule
                .in_set(PhysicsSet::Substeps)
                .in_schedule(XpbdSchedule),
        );

        // Add plugins for physics simulation loop
        app.add_plugin(PreparePlugin)
            .add_plugin(BroadPhasePlugin)
            .add_plugin(IntegratorPlugin)
            .add_plugin(SolverPlugin)
            .add_plugin(SyncPlugin);

        #[cfg(feature = "debug-render-aabbs")]
        {
            app.add_plugin(DebugLinesPlugin::default());
            app.add_system(draw_aabbs);
        }
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

#[derive(Resource, Debug, Default)]
pub struct XpbdLoop {
    pub(crate) accumulator: Scalar,
    pub(crate) queued_steps: u32,
    pub paused: bool,
}

impl XpbdLoop {
    pub fn step(&mut self) {
        self.queued_steps += 1;
    }
    pub fn pause(&mut self) {
        self.paused = true;
    }
    pub fn resume(&mut self) {
        self.paused = false;
    }
}

pub fn pause(mut xpbd_loop: ResMut<XpbdLoop>) {
    xpbd_loop.pause();
}

pub fn resume(mut xpbd_loop: ResMut<XpbdLoop>) {
    xpbd_loop.resume();
}

fn run_physics_schedule(world: &mut World) {
    let mut xpbd_loop = world
        .remove_resource::<XpbdLoop>()
        .expect("no xpbd loop resource");

    #[cfg(feature = "f32")]
    let delta_seconds = world.resource::<Time>().delta_seconds();
    #[cfg(feature = "f64")]
    let delta_seconds = world.resource::<Time>().delta_seconds_f64();

    let time_step = *world.resource::<PhysicsTimestep>();

    // Update `DeltaTime` according to the `PhysicsTimestep` configuration
    let dt = match time_step {
        PhysicsTimestep::Fixed(fixed_delta_seconds) => fixed_delta_seconds,
        PhysicsTimestep::Variable { max_dt } => delta_seconds.min(max_dt),
    };
    world.resource_mut::<DeltaTime>().0 = dt;

    if xpbd_loop.paused {
        xpbd_loop.accumulator += dt * xpbd_loop.queued_steps as Scalar;
        xpbd_loop.queued_steps = 0;
    } else {
        xpbd_loop.accumulator += delta_seconds;
    }

    while xpbd_loop.accumulator >= dt && dt > 0.0 {
        debug!("running physics schedule");
        world.run_schedule(XpbdSchedule);
        xpbd_loop.accumulator -= dt;
    }

    world.insert_resource(xpbd_loop);
}

fn run_substep_schedule(world: &mut World) {
    let NumSubsteps(substeps) = *world.resource::<NumSubsteps>();
    let dt = world.resource::<DeltaTime>().0;

    // Update `SubDeltaTime`
    let mut sub_delta_time = world.resource_mut::<SubDeltaTime>();
    sub_delta_time.0 = dt / substeps as Scalar;

    for i in 0..substeps {
        debug!("running substep schedule: {i}");
        world.run_schedule(XpbdSubstepSchedule);
    }
}
