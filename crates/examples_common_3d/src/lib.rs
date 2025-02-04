use std::time::Duration;

use avian3d::prelude::*;
use bevy::{
    diagnostic::FrameTimeDiagnosticsPlugin,
    input::common_conditions::{input_just_pressed, input_pressed},
    prelude::*,
};

pub struct ExampleCommonPlugin;

impl Plugin for ExampleCommonPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            PhysicsDiagnosticsPlugin,
            PhysicsDiagnosticsUiPlugin,
            FrameTimeDiagnosticsPlugin,
            #[cfg(feature = "use-debug-plugin")]
            PhysicsDebugPlugin::default(),
        ))
        .insert_resource(PhysicsDiagnosticsUiSettings {
            enabled: false,
            ..default()
        })
        .init_state::<AppState>()
        .add_systems(
            OnEnter(AppState::Paused),
            |mut time: ResMut<Time<Physics>>| time.pause(),
        )
        .add_systems(
            OnExit(AppState::Paused),
            |mut time: ResMut<Time<Physics>>| time.unpause(),
        )
        .add_systems(
            Update,
            (
                toggle_diagnostics_ui
                    .run_if(input_just_pressed(KeyCode::KeyP).and(input_pressed(KeyCode::AltLeft))),
                toggle_running.run_if(
                    input_just_pressed(KeyCode::KeyP).and(not(input_pressed(KeyCode::AltLeft))),
                ),
                step.run_if(in_state(AppState::Paused).and(input_just_pressed(KeyCode::Enter))),
            ),
        );
    }
}

fn toggle_diagnostics_ui(mut settings: ResMut<PhysicsDiagnosticsUiSettings>) {
    settings.enabled = !settings.enabled;
}

#[derive(Debug, Clone, Eq, PartialEq, Hash, States, Default)]
pub enum AppState {
    Paused,
    #[default]
    Running,
}

fn toggle_running(
    current_state: ResMut<State<AppState>>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    let new_state = match current_state.get() {
        AppState::Paused => AppState::Running,
        AppState::Running => AppState::Paused,
    };
    next_state.set(new_state);
}

fn step(mut time: ResMut<Time<Physics>>) {
    time.advance_by(Duration::from_secs_f64(1.0 / 60.0));
}
