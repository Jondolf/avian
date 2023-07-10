#![allow(clippy::unnecessary_cast)]

use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_xpbd_2d::prelude::*;
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(6))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .add_systems(Update, movement)
        .run();
}

#[derive(Component)]
struct Marble;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2dBundle::default());

    let square_sprite = Sprite {
        color: Color::rgb(0.7, 0.7, 0.8),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    // Ceiling
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::Y * 50.0 * 6.0),
        Collider::cuboid(50.0 * 20.0, 50.0),
    ));
    // Floor
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::NEG_Y * 50.0 * 6.0),
        Collider::cuboid(50.0 * 20.0, 50.0),
    ));
    // Left wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_scale(Vec3::new(1.0, 12.5, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::NEG_X * 50.0 * 9.5),
        Collider::cuboid(50.0, 50.0 * 12.5),
    ));
    // Right wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite,
            transform: Transform::from_scale(Vec3::new(1.0, 12.5, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::X * 50.0 * 9.5),
        Collider::cuboid(50.0, 50.0 * 12.5),
    ));

    let marble_radius = 7.5;
    let marble_mesh = MaterialMesh2dBundle {
        mesh: meshes
            .add(shape::Circle::new(marble_radius as f32).into())
            .into(),
        material: materials.add(ColorMaterial::from(Color::rgb(0.2, 0.7, 0.9))),
        ..default()
    };

    // Spawn stacks of marbles
    for x in -12..12 {
        for y in -10..10 {
            let position = Vector::new(
                x as Scalar * (2.5 * marble_radius),
                y as Scalar * (2.5 * marble_radius),
            );
            commands.spawn((
                marble_mesh.clone(),
                RigidBody::Dynamic,
                Position(position),
                Collider::ball(marble_radius),
                Marble,
            ));
        }
    }
}

fn movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut marbles: Query<&mut LinearVelocity, With<Marble>>,
) {
    for mut linear_velocity in &mut marbles {
        if keyboard_input.pressed(KeyCode::Up) {
            linear_velocity.y += 50.0;
        }
        if keyboard_input.pressed(KeyCode::Down) {
            linear_velocity.y -= 10.0;
        }
        if keyboard_input.pressed(KeyCode::Left) {
            linear_velocity.x -= 10.0;
        }
        if keyboard_input.pressed(KeyCode::Right) {
            linear_velocity.x += 10.0;
        }
    }
}
