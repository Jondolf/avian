#![allow(clippy::unnecessary_cast)]

use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_xpbd_2d::{math::*, prelude::*};
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
            transform: Transform::from_xyz(0.0, 50.0 * 6.0, 0.0)
                .with_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(50.0, 50.0),
    ));
    // Floor
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_xyz(0.0, -50.0 * 6.0, 0.0)
                .with_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(50.0, 50.0),
    ));
    // Left wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_xyz(-50.0 * 9.5, 0.0, 0.0)
                .with_scale(Vec3::new(1.0, 11.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(50.0, 50.0),
    ));
    // Right wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite,
            transform: Transform::from_xyz(50.0 * 9.5, 0.0, 0.0)
                .with_scale(Vec3::new(1.0, 11.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(50.0, 50.0),
    ));

    let marble_radius = 5.0;
    let marble_mesh = meshes.add(shape::Circle::new(marble_radius).into());
    let marble_material = materials.add(ColorMaterial::from(Color::rgb(0.2, 0.7, 0.9)));

    // Spawn stacks of marbles
    for x in -21..21 {
        for y in -20..20 {
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh: marble_mesh.clone().into(),
                    material: marble_material.clone(),
                    transform: Transform::from_xyz(
                        x as f32 * 2.5 * marble_radius,
                        y as f32 * 2.5 * marble_radius,
                        0.0,
                    ),
                    ..default()
                },
                RigidBody::Dynamic,
                Collider::ball(marble_radius as Scalar),
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
