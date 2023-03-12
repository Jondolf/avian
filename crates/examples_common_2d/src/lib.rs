use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*};
use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_xpbd_2d::XpbdPlugin;

#[derive(Default)]
pub struct XpbdExamplePlugin;

impl Plugin for XpbdExamplePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(XpbdPlugin)
            .add_plugin(WorldInspectorPlugin::default())
            .add_plugin(FrameTimeDiagnosticsPlugin)
            .add_state::<AppState>()
            // .add_system_set(SystemSet::on_enter(AppState::Paused).with_system(bevy_xpbd_2d::pause))
            // .add_system_set(SystemSet::on_exit(AppState::Paused).with_system(bevy_xpbd_2d::resume))
            .add_system(pause_button);
        // .add_system_set(SystemSet::on_update(AppState::Paused).with_system(step_button));
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

// fn step_button(mut xpbd_loop: ResMut<XpbdLoop>, keys: Res<Input<KeyCode>>) {
//     if keys.just_pressed(KeyCode::Return) {
//         xpbd_loop.step();
//     }
// }
