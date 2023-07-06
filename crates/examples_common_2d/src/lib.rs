pub extern crate bevy_prototype_debug_lines;

use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugLinesPlugin;
use bevy_screen_diagnostics::{ScreenDiagnosticsPlugin, ScreenFrameDiagnosticsPlugin};
use bevy_xpbd_2d::prelude::*;

#[derive(Default)]
pub struct XpbdExamplePlugin;

impl Plugin for XpbdExamplePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(PhysicsPlugins)
            .add_plugin(ScreenDiagnosticsPlugin::default())
            .add_plugin(ScreenFrameDiagnosticsPlugin)
            .add_state::<AppState>()
            .add_system(bevy_xpbd_2d::pause.in_schedule(OnEnter(AppState::Paused)))
            .add_system(bevy_xpbd_2d::resume.in_schedule(OnExit(AppState::Paused)))
            .add_system(pause_button)
            .add_system(step_button.run_if(in_state(AppState::Paused)));
        #[cfg(not(feature = "debug-plugin"))]
        {
            app.add_plugin(DebugLinesPlugin::default());
        }
    }
}

#[derive(Debug, Clone, Eq, PartialEq, Hash, States, Default)]
pub enum AppState {
    Paused,
    #[default]
    Running,
}

fn pause_button(
    current_state: ResMut<State<AppState>>,
    mut next_state: ResMut<NextState<AppState>>,
    keys: Res<Input<KeyCode>>,
) {
    if keys.just_pressed(KeyCode::P) {
        let new_state = match current_state.0 {
            AppState::Paused => AppState::Running,
            AppState::Running => AppState::Paused,
        };
        next_state.0 = Some(new_state);
    }
}

fn step_button(mut physics_loop: ResMut<PhysicsLoop>, keys: Res<Input<KeyCode>>) {
    if keys.just_pressed(KeyCode::Return) {
        physics_loop.step();
    }
}
