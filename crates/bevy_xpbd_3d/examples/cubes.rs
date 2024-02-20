#![allow(clippy::unnecessary_cast)]

use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*};
use examples_common_3d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(Msaa::Sample4)
        .add_systems(Startup, setup)
        .add_systems(Update, movement)
        .run();
}

/// The acceleration used for movement.
#[derive(Component)]
struct MovementAcceleration(Scalar);

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let cube_mesh = meshes.add(Cuboid::default());

    // Ground
    commands.spawn((
        PbrBundle {
            mesh: cube_mesh.clone(),
            material: materials.add(Color::rgb(0.7, 0.7, 0.8)),
            transform: Transform::from_xyz(0.0, -2.0, 0.0).with_scale(Vec3::new(100.0, 1.0, 100.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(1.0, 1.0, 1.0),
    ));

    let cube_size = 2.0;

    // Spawn cube stacks
    for x in -2..2 {
        for y in -2..2 {
            for z in -2..2 {
                let position = Vec3::new(x as f32, y as f32 + 5.0, z as f32) * (cube_size + 0.05);
                commands.spawn((
                    PbrBundle {
                        mesh: cube_mesh.clone(),
                        material: materials.add(Color::rgb(0.2, 0.7, 0.9)),
                        transform: Transform::from_translation(position)
                            .with_scale(Vec3::splat(cube_size as f32)),
                        ..default()
                    },
                    RigidBody::Dynamic,
                    Collider::cuboid(1.0, 1.0, 1.0),
                    MovementAcceleration(10.0),
                ));
            }
        }
    }

    // Directional light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
        ..default()
    });

    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 12.0, 40.0))
            .looking_at(Vec3::Y * 5.0, Vec3::Y),
        ..default()
    });
}

fn movement(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&MovementAcceleration, &mut LinearVelocity)>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for (movement_acceleration, mut linear_velocity) in &mut query {
        let up = keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
        let down = keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
        let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

        let horizontal = right as i8 - left as i8;
        let vertical = down as i8 - up as i8;
        let direction =
            Vector::new(horizontal as Scalar, 0.0, vertical as Scalar).normalize_or_zero();

        // Move in input direction
        if direction != Vector::ZERO {
            linear_velocity.x += direction.x * movement_acceleration.0 * delta_time;
            linear_velocity.z += direction.z * movement_acceleration.0 * delta_time;
        }
    }
}
