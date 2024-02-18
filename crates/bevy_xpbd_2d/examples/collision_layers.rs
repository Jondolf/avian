#![allow(clippy::unnecessary_cast)]

use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_xpbd_2d::{math::*, prelude::*};
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .run();
}

// Define the collision layers
#[derive(PhysicsLayer)]
enum Layer {
    Blue,
    Red,
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2dBundle::default());

    // Spawn blue platform that belongs on the blue layer and collides with blue
    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                color: Color::rgb(0.2, 0.7, 0.9),
                custom_size: Some(Vec2::new(500.0, 25.0)),
                ..default()
            },
            transform: Transform::from_xyz(0.0, -50.0, 0.0),
            ..default()
        },
        RigidBody::Static,
        Collider::rectangle(500.0, 25.0),
        CollisionLayers::new([Layer::Blue], [Layer::Blue]),
    ));

    // Spawn red platform that belongs on the red layer and collides with red
    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                color: Color::rgb(0.9, 0.3, 0.3),
                custom_size: Some(Vec2::new(500.0, 25.0)),
                ..default()
            },
            transform: Transform::from_xyz(0.0, -200.0, 0.0),
            ..default()
        },
        RigidBody::Static,
        Collider::rectangle(500.0, 25.0),
        CollisionLayers::new([Layer::Red], [Layer::Red]),
    ));

    let marble_radius = 7.5;
    let marble_mesh = meshes.add(Circle::new(marble_radius));

    // Spawn blue marbles that belong on the blue layer and collide with blue
    let blue_material = materials.add(Color::rgb(0.2, 0.7, 0.9));
    for x in -6..6 {
        for y in 0..4 {
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh: marble_mesh.clone().into(),
                    material: blue_material.clone(),
                    transform: Transform::from_xyz(
                        x as f32 * 2.5 * marble_radius,
                        y as f32 * 2.5 * marble_radius + 200.0,
                        0.0,
                    ),
                    ..default()
                },
                RigidBody::Dynamic,
                Collider::circle(marble_radius as Scalar),
                CollisionLayers::new([Layer::Blue], [Layer::Blue]),
            ));
        }
    }

    // Spawn red marbles that belong on the red layer and collide with red
    let red_material = materials.add(Color::rgb(0.9, 0.3, 0.3));
    for x in -6..6 {
        for y in -4..0 {
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh: marble_mesh.clone().into(),
                    material: red_material.clone(),
                    transform: Transform::from_xyz(
                        x as f32 * 2.5 * marble_radius,
                        y as f32 * 2.5 * marble_radius + 200.0,
                        0.0,
                    ),
                    ..default()
                },
                RigidBody::Dynamic,
                Collider::circle(marble_radius as Scalar),
                CollisionLayers::new([Layer::Red], [Layer::Red]),
            ));
        }
    }
}
