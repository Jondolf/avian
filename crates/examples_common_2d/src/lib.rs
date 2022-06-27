use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*};

use bevy_editor_pls::EditorPlugin;
use bevy_xpbd_2d::{XpbdLoop, XpbdPlugin};

#[derive(Default)]
pub struct XpbdExamplePlugin;

impl Plugin for XpbdExamplePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(XpbdPlugin)
            .add_plugin(EditorPlugin)
            .add_plugin(FrameTimeDiagnosticsPlugin)
            .add_state(AppState::Running)
            .add_system_set(SystemSet::on_enter(AppState::Paused).with_system(bevy_xpbd_2d::pause))
            .add_system_set(SystemSet::on_exit(AppState::Paused).with_system(bevy_xpbd_2d::resume))
            .add_system(pause_button)
            .add_system_set(SystemSet::on_update(AppState::Paused).with_system(step_button));
    }
}

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub enum AppState {
    Paused,
    Running,
}

fn pause_button(mut app_state: ResMut<State<AppState>>, keys: Res<Input<KeyCode>>) {
    if keys.just_pressed(KeyCode::P) {
        let new_state = match app_state.current() {
            AppState::Paused => AppState::Running,
            AppState::Running => AppState::Paused,
        };
        app_state.set(new_state).unwrap();
    }
}

fn step_button(mut xpbd_loop: ResMut<XpbdLoop>, keys: Res<Input<KeyCode>>) {
    if keys.just_pressed(KeyCode::Return) {
        xpbd_loop.step();
    }
}
