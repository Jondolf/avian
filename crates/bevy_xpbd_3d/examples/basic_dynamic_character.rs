//! A very basic implementation of a character controller for a dynamic rigid body.
//! Supports directional movement and jumping.
//!
//! Bevy XPBD does not have a built-in character controller yet, so you will have to implement
//! the logic yourself.
//!
//! For a kinematic character controller, see the `basic_kinematic_character` example.

use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*, PhysicsSchedule, PhysicsStepSet};

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, setup)
        .add_systems(PhysicsSchedule, movement.before(PhysicsStepSet::BroadPhase))
        .run();
}

#[derive(Component)]
struct Player;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Plane::from_size(8.0))),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(8.0, 0.005, 8.0),
    ));

    // Player
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Capsule {
                radius: 0.4,
                ..default()
            })),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
            transform: Transform::from_xyz(0.0, 1.0, 0.0),
            ..default()
        },
        RigidBody::Dynamic,
        Collider::capsule(1.0, 0.4),
        // Prevent the player from falling over
        LockedAxes::new().lock_rotation_x().lock_rotation_z(),
        // Cast the player shape downwards to detect when the player is grounded
        ShapeCaster::new(
            Collider::capsule(0.9, 0.35),
            Vector::NEG_Y * 0.05,
            Quaternion::default(),
            Vector::NEG_Y,
        )
        .with_max_time_of_impact(0.2)
        .with_max_hits(1),
        Restitution::new(0.0).with_combine_rule(CoefficientCombine::Min),
        GravityScale(2.0),
        Player,
    ));

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-4.0, 6.5, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

fn movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut players: Query<(&mut LinearVelocity, &ShapeHits), With<Player>>,
) {
    for (mut linear_velocity, ground_hits) in &mut players {
        // Directional movement
        if keyboard_input.pressed(KeyCode::W) || keyboard_input.pressed(KeyCode::Up) {
            linear_velocity.z -= 1.2;
        }
        if keyboard_input.pressed(KeyCode::S) || keyboard_input.pressed(KeyCode::Down) {
            linear_velocity.z += 1.2;
        }
        if keyboard_input.pressed(KeyCode::A) || keyboard_input.pressed(KeyCode::Left) {
            linear_velocity.x -= 1.2;
        }
        if keyboard_input.pressed(KeyCode::D) || keyboard_input.pressed(KeyCode::Right) {
            linear_velocity.x += 1.2;
        }

        // Jump if space pressed and the player is close enough to the ground
        if keyboard_input.just_pressed(KeyCode::Space) && !ground_hits.is_empty() {
            linear_velocity.y += 8.0;
        }

        // Slow player down on the x and y axes
        linear_velocity.x *= 0.8;
        linear_velocity.z *= 0.8;
    }
}
