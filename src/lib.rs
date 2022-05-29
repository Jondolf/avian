pub mod bundles;
pub mod components;
pub mod resources;
pub mod systems;

pub use parry2d;

use resources::*;
use systems::*;

use bevy::{ecs::schedule::ShouldRun, prelude::*};

pub const DELTA_TIME: f32 = 1.0 / 60.0;
pub const NUM_SUBSTEPS: u32 = 8;
pub const SUB_DT: f32 = DELTA_TIME / NUM_SUBSTEPS as f32;

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

pub struct XPBDPlugin;

#[derive(SystemLabel, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Step {
    Integrate,
    SolvePos,
    UpdateVel,
    SolveVel,
}

impl Plugin for XPBDPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LoopState>()
            .init_resource::<Gravity>()
            .init_resource::<CollisionPairs>()
            .init_resource::<DynamicContacts>()
            .init_resource::<StaticContacts>()
            .add_stage_before(
                CoreStage::Update,
                FixedUpdateStage,
                SystemStage::parallel()
                    .with_run_criteria(run_criteria)
                    .with_system(
                        collect_collision_pairs
                            .with_run_criteria(first_substep)
                            .before(integrate),
                    )
                    .with_system(integrate.label(Step::Integrate))
                    .with_system(clear_contacts.before(Step::SolvePos))
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::SolvePos)
                            .after(Step::Integrate)
                            .with_system(solve_pos_dynamics)
                            .with_system(solve_pos_statics),
                    )
                    .with_system(update_vel.label(Step::UpdateVel).after(Step::SolvePos))
                    .with_system_set(
                        SystemSet::new()
                            .label(Step::SolveVel)
                            .after(Step::UpdateVel)
                            .with_system(solve_vel_dynamics)
                            .with_system(solve_vel_statics),
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

fn run_criteria(time: Res<Time>, mut state: ResMut<LoopState>) -> ShouldRun {
    if !state.has_added_time {
        state.has_added_time = true;
        state.accumulator += time.delta_seconds();
    }

    if state.substepping {
        state.current_substep += 1;

        if state.current_substep < NUM_SUBSTEPS {
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

fn last_substep(state: Res<LoopState>) -> ShouldRun {
    if state.current_substep == NUM_SUBSTEPS - 1 {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}
