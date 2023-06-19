#![allow(clippy::unnecessary_cast)]

use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_xpbd_2d::prelude::*;
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_startup_system(setup)
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
            ..default()
        },
        RigidBody::Static,
        Position(Vector::NEG_Y * 50.0),
        Collider::cuboid(500.0, 25.0),
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
            ..default()
        },
        RigidBody::Static,
        Position(Vector::NEG_Y * 200.0),
        Collider::cuboid(500.0, 25.0),
        CollisionLayers::new([Layer::Red], [Layer::Red]),
    ));

    let marble_radius = 7.5;
    let marble_mesh = meshes.add(shape::Circle::new(marble_radius as f32).into());

    // Spawn blue marbles that belong on the blue layer and collide with blue
    let blue_material = materials.add(ColorMaterial::from(Color::rgb(0.2, 0.7, 0.9)));
    for x in -6..6 {
        for y in 0..4 {
            let position = Vector::new(
                x as Scalar * (2.5 * marble_radius),
                (y as Scalar) * (2.5 * marble_radius),
            );
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh: marble_mesh.clone().into(),
                    material: blue_material.clone(),
                    ..default()
                },
                RigidBody::Dynamic,
                Position(position + Vector::Y * 200.0),
                Collider::ball(marble_radius),
                CollisionLayers::new([Layer::Blue], [Layer::Blue]),
            ));
        }
    }

    // Spawn red marbles that belong on the red layer and collide with red
    let red_material = materials.add(ColorMaterial::from(Color::rgb(0.9, 0.3, 0.3)));
    for x in -6..6 {
        for y in -4..0 {
            let position = Vector::new(
                x as Scalar * (2.5 * marble_radius),
                (y as Scalar) * (2.5 * marble_radius),
            );
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh: marble_mesh.clone().into(),
                    material: red_material.clone(),
                    ..default()
                },
                RigidBody::Dynamic,
                Position(position + Vector::Y * 200.0),
                Collider::ball(marble_radius),
                CollisionLayers::new([Layer::Red], [Layer::Red]),
            ));
        }
    }
}
