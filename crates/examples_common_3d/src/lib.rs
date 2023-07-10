use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
    text::DEFAULT_FONT_HANDLE,
};
use bevy_xpbd_3d::prelude::*;

#[derive(Default)]
pub struct XpbdExamplePlugin;

impl Plugin for XpbdExamplePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((PhysicsPlugins::default(), FrameTimeDiagnosticsPlugin))
            .add_state::<AppState>()
            .add_systems(Startup, setup)
            .add_systems(OnEnter(AppState::Paused), bevy_xpbd_3d::pause)
            .add_systems(OnExit(AppState::Paused), bevy_xpbd_3d::resume)
            .add_systems(Update, update_fps_text)
            .add_systems(Update, pause_button)
            .add_systems(Update, step_button.run_if(in_state(AppState::Paused)));
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
        let new_state = match current_state.get() {
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

#[derive(Component)]
struct FpsText;

fn setup(mut commands: Commands) {
    commands.spawn((
        TextBundle::from_section(
            "FPS: ",
            TextStyle {
                font: DEFAULT_FONT_HANDLE.typed(),
                font_size: 20.0,
                color: Color::TOMATO,
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(5.0),
            left: Val::Px(5.0),
            ..default()
        }),
        FpsText,
    ));
}

fn update_fps_text(diagnostics: Res<DiagnosticsStore>, mut query: Query<&mut Text, With<FpsText>>) {
    for mut text in &mut query {
        if let Some(fps) = diagnostics.get(FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(value) = fps.smoothed() {
                // Update the value of the second section
                text.sections[0].value = format!("FPS: {value:.2}");
            }
        }
    }
}
