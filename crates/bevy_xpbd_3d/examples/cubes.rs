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

#[derive(Component)]
struct Cube;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let cube_mesh = meshes.add(Mesh::from(shape::Cube { size: 1.0 }));

    // Ground
    commands.spawn((
        PbrBundle {
            mesh: cube_mesh.clone(),
            material: materials.add(Color::rgb(0.7, 0.7, 0.8).into()),
            transform: Transform::from_scale(Vec3::new(100.0, 1.0, 100.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::NEG_Y * 2.0),
        Collider::cuboid(1.0, 1.0, 1.0),
    ));

    let cube_size = 2.0;

    // Spawn cube stacks
    for x in -2..2 {
        for y in -2..2 {
            for z in -2..2 {
                let pos = Vector::new(
                    x as Scalar * (cube_size + 0.05),
                    y as Scalar * (cube_size + 0.05),
                    z as Scalar * (cube_size + 0.05),
                );
                commands.spawn((
                    PbrBundle {
                        mesh: cube_mesh.clone(),
                        material: materials.add(Color::rgb(0.2, 0.7, 0.9).into()),
                        transform: Transform::from_scale(Vec3::splat(cube_size as f32)),
                        ..default()
                    },
                    RigidBody::Dynamic,
                    Position(pos + Vector::Y * 5.0),
                    Collider::cuboid(1.0, 1.0, 1.0),
                    Cube,
                ));
            }
        }
    }

    // Directional light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 20_000.0,
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
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<&mut LinearVelocity, With<Cube>>,
) {
    for mut lin_vel in &mut query {
        if keyboard_input.pressed(KeyCode::Up) {
            lin_vel.z -= 0.15;
        }
        if keyboard_input.pressed(KeyCode::Down) {
            lin_vel.z += 0.15;
        }
        if keyboard_input.pressed(KeyCode::Left) {
            lin_vel.x -= 0.15;
        }
        if keyboard_input.pressed(KeyCode::Right) {
            lin_vel.x += 0.15;
        }
    }
}
