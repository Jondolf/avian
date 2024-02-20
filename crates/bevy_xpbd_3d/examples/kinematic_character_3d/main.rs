//! A basic implementation of a character controller for a kinematic rigid body.
//!
//! This showcases the following:
//!
//! - Basic directional movement and jumping
//! - Support for both keyboard and gamepad input
//! - A configurable maximum slope angle
//! - Collision response for kinematic bodies
//! - Loading a platformer environment from a glTF
//!
//! The character controller logic is contained within the `plugin` module.
//!
//! For a dynamic character controller, see the `dynamic_character_3d` example.

mod plugin;

use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*};
use plugin::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            CharacterControllerPlugin,
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    assets: Res<AssetServer>,
) {
    // Player
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Capsule3d::new(0.4, 1.0)),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_xyz(0.0, 1.5, 0.0),
            ..default()
        },
        CharacterControllerBundle::new(Collider::capsule(1.0, 0.4), Vector::NEG_Y * 9.81 * 2.0)
            .with_movement(30.0, 0.92, 7.0, (30.0 as Scalar).to_radians()),
    ));

    // A cube to move around
    commands.spawn((
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 1.0, 1.0),
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_xyz(3.0, 2.0, 3.0),
            ..default()
        },
    ));

    // Environment (see `async_colliders` example for creating colliders from scenes)
    commands.spawn((
        SceneBundle {
            scene: assets.load("character_controller_demo.glb#Scene0"),
            transform: Transform::from_rotation(Quat::from_rotation_y(-std::f32::consts::PI * 0.5)),
            ..default()
        },
        AsyncSceneCollider::new(Some(ComputedCollider::ConvexHull)),
        RigidBody::Static,
    ));

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 2_000_000.0,
            range: 50.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(0.0, 15.0, 0.0),
        ..default()
    });

    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-7.0, 9.5, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}
