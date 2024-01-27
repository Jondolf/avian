#![allow(clippy::unnecessary_cast)]

use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*};
use examples_common_3d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(Msaa::Sample4)
        .insert_resource(AmbientLight {
            brightness: 2.0,
            ..default()
        })
        .insert_resource(SubstepCount(80))
        .insert_resource(Gravity(Vector::NEG_Y * 9.81 * 2.0))
        .add_systems(Startup, (setup, ui))
        .add_systems(Update, movement)
        .run();
}

/// The acceleration used for movement.
#[derive(Component)]
struct MovementAcceleration(Scalar);

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let particle_count = 100;
    let particle_radius = 0.06;
    let particle_mesh = meshes.add(
        Mesh::try_from(shape::Icosphere {
            radius: particle_radius as f32,
            ..default()
        })
        .unwrap(),
    );
    let particle_material = materials.add(StandardMaterial::from(Color::rgb(0.2, 0.7, 0.9)));

    // Spawn kinematic particle that can follow the mouse
    let mut previous_particle = commands
        .spawn((
            RigidBody::Kinematic,
            MovementAcceleration(25.0),
            PbrBundle {
                mesh: particle_mesh.clone(),
                material: particle_material.clone(),
                ..default()
            },
        ))
        .id();

    // Spawn the rest of the particles, connecting each one to the previous one with joints
    for i in 1..particle_count {
        let current_particle = commands
            .spawn((
                RigidBody::Dynamic,
                MassPropertiesBundle::new_computed(&Collider::ball(particle_radius), 1.0),
                PbrBundle {
                    mesh: particle_mesh.clone(),
                    material: particle_material.clone(),
                    transform: Transform::from_xyz(
                        0.0,
                        -i as f32 * particle_radius as f32 * 2.2,
                        0.0,
                    ),
                    ..default()
                },
            ))
            .id();

        commands.spawn(
            SphericalJoint::new(previous_particle, current_particle)
                .with_local_anchor_1(Vector::NEG_Y * particle_radius * 1.1)
                .with_local_anchor_2(Vector::Y * particle_radius * 1.1)
                .with_compliance(0.00001),
        );

        previous_particle = current_particle;
    }

    commands.spawn(Camera3dBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 5.0, 20.0))
            .looking_at(Vec3::NEG_Y * 5.0, Vec3::Y),
        ..default()
    });
}

fn movement(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&MovementAcceleration, &mut LinearVelocity)>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for (movement_acceleration, mut linear_velocity) in &mut query {
        let up = keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
        let down = keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
        let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

        let horizontal = right as i8 - left as i8;
        let vertical = down as i8 - up as i8;
        let direction =
            Vector::new(horizontal as Scalar, 0.0, vertical as Scalar).normalize_or_zero();

        // Move in input direction
        linear_velocity.x += direction.x * movement_acceleration.0 * delta_time;
        linear_velocity.z += direction.z * movement_acceleration.0 * delta_time;

        // Slow down movement
        linear_velocity.0 *= 0.9;
    }
}

fn ui(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        camera: Camera {
            order: -1,
            ..default()
        },
        ..default()
    });
    commands
        .spawn(NodeBundle {
            style: Style {
                width: Val::Percent(100.),
                ..default()
            },
            background_color: Color::rgba(0.15, 0.15, 0.15, 0.0).into(),
            ..default()
        })
        .with_children(|parent| {
            // text
            parent.spawn((
                TextBundle::from_section(
                    "Use Arrow Keys or WASD to Move The Chain",
                    TextStyle {
                        font_size: 20.0,
                        color: Color::WHITE,
                        ..default()
                    },
                )
                .with_style(Style {
                    margin: UiRect {
                        left: Val::Px(5.0),
                        top: Val::Px(30.0),
                        ..default()
                    },
                    ..default()
                }),
                // Because this is a distinct label widget and
                // not button/list item text, this is necessary
                // for accessibility to treat the text accordingly.
                Label,
            ));
        });
}
