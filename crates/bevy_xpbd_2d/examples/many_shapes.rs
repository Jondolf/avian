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
struct Controllable;

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
            transform: Transform::from_scale(Vec3::new(1.0, 11.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::NEG_X * 50.0 * 9.5),
        Collider::cuboid(50.0, 50.0 * 11.0),
    ));
    // Right wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite,
            transform: Transform::from_scale(Vec3::new(1.0, 11.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::X * 50.0 * 9.5),
        Collider::cuboid(50.0, 50.0 * 11.0),
    ));

    let ball = (
        Collider::ball(7.5),
        MaterialMesh2dBundle {
            mesh: meshes.add(shape::Circle::new(7.5).into()).into(),
            material: materials.add(ColorMaterial::from(Color::rgb(0.29, 0.33, 0.64))),
            ..default()
        },
    );
    let cuboid = (
        Collider::cuboid(15.0, 15.0),
        MaterialMesh2dBundle {
            mesh: meshes.add(shape::Box::new(15.0, 15.0, 15.0).into()).into(),
            material: materials.add(ColorMaterial::from(Color::rgb(0.47, 0.58, 0.8))),
            ..default()
        },
    );
    let capsule = (
        Collider::capsule(20.0, 7.5),
        MaterialMesh2dBundle {
            mesh: meshes
                .add(
                    shape::Capsule {
                        depth: 20.0,
                        radius: 7.5,
                        ..default()
                    }
                    .into(),
                )
                .into(),
            material: materials.add(ColorMaterial::from(Color::rgb(0.63, 0.75, 0.88))),
            ..default()
        },
    );
    // Compute points of regular triangle
    let delta_rotation = Rotation::from_degrees(120.0);
    let triangle_points = vec![
        Vector::Y * 10.0,
        delta_rotation.rotate(Vector::Y * 10.0),
        delta_rotation.inverse().rotate(Vector::Y * 10.0),
    ];
    let triangle = (
        Collider::triangle(triangle_points[0], triangle_points[1], triangle_points[2]),
        MaterialMesh2dBundle {
            mesh: meshes
                .add(shape::RegularPolygon::new(10.0, 3).into())
                .into(),
            material: materials.add(ColorMaterial::from(Color::rgb(0.77, 0.87, 0.97))),
            ..default()
        },
    );
    let shapes = [ball, cuboid, capsule, triangle];

    for x in -12_i32..12 {
        for y in -8_i32..8 {
            let position = Vector::new(x as Scalar * 20.0, y as Scalar * 20.0);
            commands.spawn((
                shapes[(x + y) as usize % 4].clone(),
                RigidBody::Dynamic,
                Position(position),
                Friction::new(0.001),
                Controllable,
            ));
        }
    }
}

fn movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut marbles: Query<&mut LinearVelocity, With<Controllable>>,
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
