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
        Collider::rectangle(50.0, 50.0),
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
        Collider::rectangle(50.0, 50.0),
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
        Collider::rectangle(50.0, 50.0),
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
        Collider::rectangle(50.0, 50.0),
    ));

    let circle = Circle::new(7.5);
    let rectangle = Rectangle::new(15.0, 15.0);
    let capsule = Capsule2d::new(7.5, 20.0);
    let triangle = Triangle2d::new(
        Vec2::new(0.0, 10.0),
        Vec2::new(-10.0, -10.0),
        Vec2::new(10.0, -10.0),
    );

    let shapes = [
        (
            circle.collider(),
            meshes.add(circle).into(),
            materials.add(Color::rgb(0.29, 0.33, 0.64)),
        ),
        (
            rectangle.collider(),
            meshes.add(rectangle).into(),
            materials.add(Color::rgb(0.47, 0.58, 0.8)),
        ),
        (
            capsule.collider(),
            meshes.add(capsule).into(),
            materials.add(Color::rgb(0.63, 0.75, 0.88)),
        ),
        (
            triangle.collider(),
            meshes.add(triangle).into(),
            materials.add(Color::rgb(0.77, 0.87, 0.97)),
        ),
    ];

    for x in -12_i32..12 {
        for y in -8_i32..8 {
            let (collider, mesh, material) = shapes[(x + y) as usize % 4].clone();
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh,
                    material,
                    transform: Transform::from_xyz(x as f32 * 25.0, y as f32 * 25.0, 0.0),
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
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut marbles: Query<&mut LinearVelocity, With<Controllable>>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for mut linear_velocity in &mut marbles {
        if keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]) {
            // Use a higher acceleration for upwards movement to overcome gravity
            linear_velocity.y += 2500.0 * delta_time;
        }
        if keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]) {
            linear_velocity.y -= 500.0 * delta_time;
        }
        if keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]) {
            linear_velocity.x -= 500.0 * delta_time;
        }
        if keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]) {
            linear_velocity.x += 500.0 * delta_time;
        }
    }
}
