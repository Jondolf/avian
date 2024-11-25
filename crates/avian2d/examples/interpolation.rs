//! This example showcases how `Transform` interpolation or extrapolation can be used
//! to make movement appear smooth at fixed timesteps.
//!
//! To produce consistent, frame rate independent behavior, physics by default runs
//! in the `FixedPostUpdate` schedule with a fixed timestep, meaning that the time between
//! physics ticks remains constant. On some frames, physics can either not run at all or run
//! more than once to catch up to real time. This can lead to visible stutter for movement.
//!
//! `Transform` interpolation resolves this issue by updating `Transform` at every frame in between
//! physics ticks to smooth out the visual result. The interpolation is done from the previous position
//! to the current physics position, which keeps movement smooth, but has the downside of making movement
//! feel slightly delayed as the rendered result lags slightly behind the true positions.
//!
//! `Transform` extrapolation works similarly, but instead of using the previous positions, it predicts
//! the next positions based on velocity. This makes movement feel much more responsive, but can cause
//! jumpy results when the prediction is wrong, such as when the velocity of an object is suddenly altered.

use avian2d::{math::*, prelude::*};
use bevy::{
    color::palettes::{
        css::WHITE,
        tailwind::{CYAN_400, LIME_400, RED_400},
    },
    prelude::*,
};

const MOVEMENT_SPEED: f32 = 250.0;
const ROTATION_SPEED: f32 = 2.0;

fn main() {
    let mut app = App::new();

    // Add the `PhysicsInterpolationPlugin` to enable interpolation and extrapolation functionality.
    app.add_plugins((
        DefaultPlugins,
        PhysicsPlugins::default(),
        PhysicsInterpolationPlugin::default(),
    ));

    // Set the fixed timestep to just 5 Hz for demonstration purposes.
    app.insert_resource(Time::from_hz(5.0));

    // Flip the movement direction of objects when they reach the left or right side of the screen.
    app.add_systems(FixedUpdate, flip_movement_direction);

    // Setup the scene and UI, and update text in `Update`.
    app.add_systems(Startup, (setup, setup_text))
        .add_systems(Update, (change_timestep, update_timestep_text));

    // Run the app.
    app.run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // Spawn a camera.
    commands.spawn(Camera2d);

    let mesh = meshes.add(Rectangle::from_length(60.0));

    // This entity uses transform interpolation.
    commands.spawn((
        Name::new("Interpolation"),
        RigidBody::Kinematic,
        LinearVelocity(Vector::new(MOVEMENT_SPEED, 0.0)),
        AngularVelocity(ROTATION_SPEED),
        TransformInterpolation,
        Transform::from_xyz(-500.0, 120.0, 0.0),
        Mesh2d(mesh.clone()),
        MeshMaterial2d(materials.add(Color::from(CYAN_400)).clone()),
    ));

    // This entity uses transform extrapolation.
    commands.spawn((
        Name::new("Extrapolation"),
        RigidBody::Kinematic,
        LinearVelocity(Vector::new(MOVEMENT_SPEED, 0.0)),
        AngularVelocity(ROTATION_SPEED),
        TransformExtrapolation,
        Transform::from_xyz(-500.0, 0.0, 0.0),
        Mesh2d(mesh.clone()),
        MeshMaterial2d(materials.add(Color::from(LIME_400)).clone()),
    ));

    // This entity is simulated in `FixedUpdate` without any smoothing.
    commands.spawn((
        Name::new("No Interpolation"),
        RigidBody::Kinematic,
        LinearVelocity(Vector::new(MOVEMENT_SPEED, 0.0)),
        AngularVelocity(ROTATION_SPEED),
        Transform::from_xyz(-500.0, -120.0, 0.0),
        Mesh2d(mesh.clone()),
        MeshMaterial2d(materials.add(Color::from(RED_400)).clone()),
    ));
}

/// Flips the movement directions of objects when they reach the left or right side of the screen.
fn flip_movement_direction(mut query: Query<(&Transform, &mut LinearVelocity)>) {
    for (transform, mut lin_vel) in &mut query {
        if transform.translation.x > 500.0 && lin_vel.0.x > 0.0 {
            lin_vel.x = -MOVEMENT_SPEED;
        } else if transform.translation.x < -500.0 && lin_vel.0.x < 0.0 {
            lin_vel.x = MOVEMENT_SPEED;
        }
    }
}

/// Changes the timestep of the simulation when the up or down arrow keys are pressed.
fn change_timestep(mut time: ResMut<Time<Fixed>>, keyboard_input: Res<ButtonInput<KeyCode>>) {
    if keyboard_input.pressed(KeyCode::ArrowUp) {
        let new_timestep = (time.delta_secs_f64() * 0.9).max(1.0 / 255.0);
        time.set_timestep_seconds(new_timestep);
    }
    if keyboard_input.pressed(KeyCode::ArrowDown) {
        let new_timestep = (time.delta_secs_f64() * 1.1).min(1.0);
        time.set_timestep_seconds(new_timestep);
    }
}

#[derive(Component)]
struct TimestepText;

fn setup_text(mut commands: Commands) {
    let font = TextFont {
        font_size: 20.0,
        ..default()
    };

    commands
        .spawn((
            Text::new("Fixed Hz: "),
            TextColor::from(WHITE),
            font.clone(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(10.0),
                left: Val::Px(10.0),
                ..default()
            },
        ))
        .with_child((TimestepText, TextSpan::default()));

    commands.spawn((
        Text::new("Change Timestep With Up/Down Arrow"),
        TextColor::from(WHITE),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            right: Val::Px(10.0),
            ..default()
        },
    ));

    commands.spawn((
        Text::new("Interpolation"),
        TextColor::from(CYAN_400),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(50.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));

    commands.spawn((
        Text::new("Extrapolation"),
        TextColor::from(LIME_400),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(75.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));

    commands.spawn((
        Text::new("No Interpolation"),
        TextColor::from(RED_400),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(100.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));
}

fn update_timestep_text(
    mut text: Single<&mut TextSpan, With<TimestepText>>,
    time: Res<Time<Fixed>>,
) {
    let timestep = time.timestep().as_secs_f32().recip();
    text.0 = format!("{timestep:.2}");
}
