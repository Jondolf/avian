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

    let ball = (
        Collider::ball(7.5),
        meshes.add(shape::Circle::new(7.5).into()).into(),
        materials.add(ColorMaterial::from(Color::rgb(0.29, 0.33, 0.64))),
    );
    let cuboid = (
        Collider::cuboid(15.0, 15.0),
        meshes.add(shape::Box::new(15.0, 15.0, 15.0).into()).into(),
        materials.add(ColorMaterial::from(Color::rgb(0.47, 0.58, 0.8))),
    );
    let capsule = (
        Collider::capsule(20.0, 7.5),
        meshes
            .add(
                shape::Capsule {
                    depth: 20.0,
                    radius: 7.5,
                    ..default()
                }
                .into(),
            )
            .into(),
        materials.add(ColorMaterial::from(Color::rgb(0.63, 0.75, 0.88))),
    );
    // Compute points of regular triangle
    let delta_rotation = Rotation::from_degrees(120.0);
    let triangle_points = [
        Vector::Y * 10.0,
        delta_rotation.rotate(Vector::Y * 10.0),
        delta_rotation.inverse().rotate(Vector::Y * 10.0),
    ];
    let triangle = (
        Collider::triangle(triangle_points[0], triangle_points[1], triangle_points[2]),
        meshes
            .add(shape::RegularPolygon::new(10.0, 3).into())
            .into(),
        materials.add(ColorMaterial::from(Color::rgb(0.77, 0.87, 0.97))),
    );
    let shapes = [ball, cuboid, capsule, triangle];

    for x in -12_i32..12 {
        for y in -8_i32..8 {
            let (collider, mesh, material) = shapes[(x + y) as usize % 4].clone();
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh,
                    material,
                    transform: Transform::from_xyz(x as f32 * 20.0, y as f32 * 20.0, 0.0),
                    ..default()
                },
                collider,
                RigidBody::Dynamic,
                Friction::new(0.1),
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
