//! A very basic implementation of a character controller for a dynamic rigid body.
//! Supports directional movement and jumping.
//!
//! Bevy XPBD does not have a built-in character controller yet, so you will have to implement
//! the logic yourself.
//!
//! For a kinematic character controller, see the `basic_kinematic_character` example.

use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*};

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, setup)
        .add_systems(Update, movement)
        .run();
}

/// The acceleration used for character movement.
#[derive(Component)]
struct MovementAcceleration(Scalar);

/// The damping factor used for slowing down movement.
#[derive(Component)]
struct MovementDampingFactor(Scalar);

/// The strength of a jump.
#[derive(Component)]
struct JumpImpulse(Scalar);

/// A bundle that contains the components needed for a basic
/// dynamic character controller.
#[derive(Bundle)]
struct CharacterControllerBundle {
    rigid_body: RigidBody,
    collider: Collider,
    ground_caster: ShapeCaster,
    locked_axes: LockedAxes,
    movement_acceleration: MovementAcceleration,
    movement_damping: MovementDampingFactor,
    jump_impulse: JumpImpulse,
}

impl CharacterControllerBundle {
    fn new(
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        collider: Collider,
    ) -> Self {
        // Create shape caster as a slightly smaller version of the collider
        let mut caster_shape = collider.clone();
        caster_shape.set_scale(Vector::ONE * 0.99, 10);

        Self {
            rigid_body: RigidBody::Dynamic,
            locked_axes: LockedAxes::ROTATION_LOCKED,
            collider,
            ground_caster: ShapeCaster::new(
                caster_shape,
                Vector::ZERO,
                Quaternion::default(),
                Vector::NEG_Y,
            )
            .with_max_time_of_impact(0.2),
            movement_acceleration: MovementAcceleration(acceleration),
            movement_damping: MovementDampingFactor(damping),
            jump_impulse: JumpImpulse(jump_impulse),
        }
    }
}

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
        CharacterControllerBundle::new(30.0, 0.9, 8.0, Collider::capsule(1.0, 0.4)),
        Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
        Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
        GravityScale(2.0),
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
    time: Res<Time>,
    keyboard_input: Res<Input<KeyCode>>,
    mut controllers: Query<(
        &MovementAcceleration,
        &MovementDampingFactor,
        &JumpImpulse,
        &ShapeHits,
        &mut LinearVelocity,
    )>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for (movement_acceleration, damping_factor, jump_impulse, ground_hits, mut linear_velocity) in
        &mut controllers
    {
        let up = keyboard_input.any_pressed([KeyCode::W, KeyCode::Up]);
        let down = keyboard_input.any_pressed([KeyCode::S, KeyCode::Down]);
        let left = keyboard_input.any_pressed([KeyCode::A, KeyCode::Left]);
        let right = keyboard_input.any_pressed([KeyCode::D, KeyCode::Right]);

        let horizontal = right as i8 - left as i8;
        let vertical = down as i8 - up as i8;
        let direction =
            Vector::new(horizontal as Scalar, 0.0, vertical as Scalar).normalize_or_zero();

        // Move in input direction
        linear_velocity.x += direction.x * movement_acceleration.0 * delta_time;
        linear_velocity.z += direction.z * movement_acceleration.0 * delta_time;

        // Apply movement damping
        linear_velocity.x *= damping_factor.0;
        linear_velocity.z *= damping_factor.0;

        // Jump if Space is pressed and the player is close enough to the ground
        if keyboard_input.just_pressed(KeyCode::Space) && !ground_hits.is_empty() {
            linear_velocity.y = jump_impulse.0;
        }
    }
}
