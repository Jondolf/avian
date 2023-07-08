#![allow(clippy::unnecessary_cast)]

use bevy::{prelude::*, sprite::MaterialMesh2dBundle, window::PrimaryWindow};
use bevy_xpbd_2d::prelude::*;
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(50))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .add_systems(Update, follow_mouse)
        .run();
}

#[derive(Component)]
struct FollowMouse;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2dBundle::default());

    let particle_count = 100;
    let particle_radius = 1.2;
    let particle_mesh = MaterialMesh2dBundle {
        mesh: meshes
            .add(shape::Circle::new(particle_radius as f32).into())
            .into(),
        material: materials.add(ColorMaterial::from(Color::rgb(0.2, 0.7, 0.9))),
        ..default()
    };

    // Spawn kinematic particle that can follow the mouse
    let mut previous_particle = commands
        .spawn((particle_mesh.clone(), RigidBody::Kinematic, FollowMouse))
        .id();

    // Spawn the rest of the particles, connecting each one to the previous one with joints
    for i in 1..particle_count {
        let current_particle = commands
            .spawn((
                particle_mesh.clone(),
                RigidBody::Dynamic,
                Position(i as Scalar * Vector::NEG_Y * (particle_radius * 2.0 + 1.0)),
                MassPropertiesBundle::new_computed(&Collider::ball(particle_radius), 1.0),
            ))
            .id();

        commands.spawn(
            RevoluteJoint::new(previous_particle, current_particle)
                .with_local_anchor_2(Vector::Y * (particle_radius * 2.0 + 1.0))
                .with_compliance(0.0000001),
        );

        previous_particle = current_particle;
    }
}

fn follow_mouse(
    buttons: Res<Input<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera: Query<(&Camera, &GlobalTransform)>,
    mut follower: Query<&mut Position, With<FollowMouse>>,
) {
    if buttons.pressed(MouseButton::Left) {
        let window = windows.single();
        let (camera, camera_transform) = camera.single();
        let mut follower_position = follower.single_mut();

        // Set position of follower to cursor position in world coordinates
        // https://bevy-cheatbook.github.io/cookbook/cursor2world.html
        if let Some(pos) = window.cursor_position() {
            let window_size = Vec2::new(window.width(), window.height());
            let ndc = (pos / window_size) * 2.0 - Vec2::ONE;
            let ndc_to_world =
                camera_transform.compute_matrix() * camera.projection_matrix().inverse();
            let world_pos = ndc_to_world.project_point3(ndc.extend(-1.0));
            follower_position.0 = world_pos.truncate().adjust_precision();
        }
    }
}
