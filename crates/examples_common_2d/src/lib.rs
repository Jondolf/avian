use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*};
use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_xpbd_2d::prelude::*;

#[derive(Default)]
pub struct XpbdExamplePlugin;

impl Plugin for XpbdExamplePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(PhysicsPlugins)
            .add_plugin(WorldInspectorPlugin::default())
            .add_plugin(FrameTimeDiagnosticsPlugin)
            .add_state::<AppState>()
            .add_system(bevy_xpbd_2d::pause.in_schedule(OnEnter(AppState::Paused)))
            .add_system(bevy_xpbd_2d::resume.in_schedule(OnExit(AppState::Paused)))
            .add_system(pause_button)
            .add_system(step_button.run_if(in_state(AppState::Paused)));
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

fn step_button(mut xpbd_loop: ResMut<XpbdLoop>, keys: Res<Input<KeyCode>>) {
    if keys.just_pressed(KeyCode::Return) {
        xpbd_loop.step();
    }
}
