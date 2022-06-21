#[cfg(feature = "2d")]
pub extern crate parry2d as parry;

#[cfg(feature = "3d")]
pub extern crate parry3d as parry;

#[macro_use]
pub extern crate cfg_if;

pub mod bundles;
pub mod components;
pub mod constraints;
pub mod resources;
pub mod systems;

mod utils;

use resources::*;
use systems::*;

use bevy::{ecs::schedule::ShouldRun, prelude::*};

#[cfg(feature = "2d")]
pub type Vector = Vec2;

#[cfg(feature = "3d")]
pub type Vector = Vec3;

pub const DELTA_TIME: f32 = 1.0 / 60.0;

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

pub struct XPBDPlugin;

#[derive(SystemLabel, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Step {
    CollectCollisionPairs,
    Integrate,
    SolvePos,
    UpdateVel,
    SolveVel,
}

impl Plugin for XPBDPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<XPBDSubsteps>()
            .init_resource::<SubDeltaTime>()
            .init_resource::<LoopState>()
            .init_resource::<Gravity>()
            .init_resource::<PenetrationConstraints>()
            .init_resource::<Contacts>()
            .add_stage_before(
                CoreStage::Update,
                FixedUpdateStage,
                SystemStage::parallel()
                    .with_run_criteria(run_criteria)
                    .with_system_set(
                        SystemSet::new()
                            .before(Step::CollectCollisionPairs)
                            .with_system(update_sub_delta_time)
                            .with_system(update_aabb)
                            .with_system(update_mass_props),
                    )
                    .with_system(
                        collect_collision_pairs
                            .label(Step::CollectCollisionPairs)
                            .with_run_criteria(first_substep)
                            .before(Step::Integrate),
                    )
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::Integrate)
                            .with_system(integrate_pos)
                            .with_system(integrate_rot),
                    )
                    .with_system(clear_contacts.before(Step::SolvePos))
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::SolvePos)
                            .after(Step::Integrate)
                            .with_system(solve_pos),
                    )
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::UpdateVel)
                            .after(Step::SolvePos)
                            .with_system(update_lin_vel)
                            .with_system(update_ang_vel),
                    )
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::SolveVel)
                            .after(Step::UpdateVel)
                            .with_system(solve_vel),
                    )
                    .with_system(
                        sync_transforms
                            .with_run_criteria(last_substep)
                            .after(Step::SolveVel),
                    ),
            );
    }
}

struct LoopState {
    has_added_time: bool,
    accumulator: f32,
    substepping: bool,
    current_substep: u32,
}

impl Default for LoopState {
    fn default() -> Self {
        Self {
            has_added_time: false,
            accumulator: 0.0,
            substepping: false,
            current_substep: 0,
        }
    }
}

fn run_criteria(
    time: Res<Time>,
    substeps: Res<XPBDSubsteps>,
    mut state: ResMut<LoopState>,
) -> ShouldRun {
    if !state.has_added_time {
        state.has_added_time = true;
        state.accumulator += time.delta_seconds();
    }

    if state.substepping {
        state.current_substep += 1;

        if state.current_substep < substeps.0 {
            return ShouldRun::YesAndCheckAgain;
        } else {
            // We finished a whole step
            state.accumulator -= DELTA_TIME;
            state.current_substep = 0;
            state.substepping = false;
        }
    }

    if state.accumulator >= DELTA_TIME {
        state.substepping = true;
        state.current_substep = 0;
        ShouldRun::YesAndCheckAgain
    } else {
        state.has_added_time = false;
        ShouldRun::No
    }
}

fn first_substep(state: Res<LoopState>) -> ShouldRun {
    if state.current_substep == 0 {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn last_substep(substeps: Res<XPBDSubsteps>, state: Res<LoopState>) -> ShouldRun {
    if state.current_substep == substeps.0 - 1 {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}
