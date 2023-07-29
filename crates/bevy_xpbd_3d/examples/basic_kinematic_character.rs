//! A very basic implementation of a character controller for a kinematic rigid body.
//! Supports directional movement and jumping.
//!
//! Bevy XPBD does not have a built-in character controller yet, so you will have to implement
//! the logic yourself. For kinematic bodies, collision response has to be handled manually, as shown in
//! this example.
//!
//! Using dynamic bodies is often easier, as they handle most of the physics for you.
//! For a dynamic character controller, see the `basic_dynamic_character` example.

use bevy::prelude::*;
use bevy_xpbd_3d::{
    math::*, prelude::*, PhysicsSchedule, PhysicsStepSet, SubstepSchedule, SubstepSet,
};
use nalgebra::RealField;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, setup)
        .add_systems(PhysicsSchedule, movement.before(PhysicsStepSet::BroadPhase))
        .add_systems(
            // Run collision handling in substep schedule
            SubstepSchedule,
            kinematic_collision.in_set(SubstepSet::SolveUserConstraints),
        )
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
            ..default()
        },
        RigidBody::Kinematic,
        Position(Vector::Y * 7.0),
        Collider::capsule(1.0, 0.4),
        // Cast the player shape downwards to detect when the player is grounded
        ShapeCaster::new(
            Collider::capsule(0.9, 0.35),
            Vector::ZERO,
            Quaternion::default(),
            Vector::NEG_Y,
        )
        .with_ignore_origin_penetration(true) // Don't count player's collider
        .with_max_time_of_impact(0.11)
        .with_max_hits(1),
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
    mut players: Query<(&mut LinearVelocity, &ShapeHits, &CollidingEntities), With<Player>>,
) {
    for (mut linear_velocity, ground_hits, entities) in &mut players {
        // Reset vertical valocity if grounded, otherwise apply gravity
        if !ground_hits.is_empty() {
            linear_velocity.y = 0.0;
        } else {
            linear_velocity.y -= 0.1;
        }

        // Directional movement
        if keyboard_input.pressed(KeyCode::W) || keyboard_input.pressed(KeyCode::Up) {
            linear_velocity.z -= 1.2;
        }
        if keyboard_input.pressed(KeyCode::A) || keyboard_input.pressed(KeyCode::Left) {
            linear_velocity.x -= 1.2;
        }
        if keyboard_input.pressed(KeyCode::S) || keyboard_input.pressed(KeyCode::Down) {
            linear_velocity.z += 1.2;
        }
        if keyboard_input.pressed(KeyCode::D) || keyboard_input.pressed(KeyCode::Right) {
            linear_velocity.x += 1.2;
        }

        // Jump if space pressed and the player is close enough to the ground
        if keyboard_input.just_pressed(KeyCode::Space) && !ground_hits.is_empty() {
            linear_velocity.y += 10.0;
        }

        // Slow player down
        linear_velocity.x *= 0.8;
        linear_velocity.y *= 0.98;
        linear_velocity.z *= 0.8;
    }
}

fn kinematic_collision(
    collisions: Res<Collisions>,
    mut bodies: Query<(&RigidBody, &mut Position)>,
) {
    // Iterate through collisions and move the kinematic body to resolve penetration
    for contacts in collisions.iter() {
        // If the collision didn't happen during this substep, skip the collision
        if !contacts.during_current_substep {
            continue;
        }
        if let Ok([(rb1, mut position1), (rb2, mut position2)]) =
            bodies.get_many_mut([contacts.entity1, contacts.entity2])
        {
            for manifold in contacts.manifolds.iter() {
                for contact in manifold.contacts.iter() {
                    if contact.penetration <= Scalar::EPSILON {
                        continue;
                    }
                    if rb1.is_kinematic() && !rb2.is_kinematic() {
                        position1.0 -= contact.normal * contact.penetration;
                    } else if rb2.is_kinematic() && !rb1.is_kinematic() {
                        position2.0 += contact.normal * contact.penetration;
                    }
                }
            }
        }
    }
}
