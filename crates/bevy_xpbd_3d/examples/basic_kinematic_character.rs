//! A basic implementation of a character controller for a kinematic rigid body.
//! Supports directional movement and jumping with both keyboard and gamepad input.
//!
//! Bevy XPBD does not have a built-in character controller yet, so you will have to implement
//! the logic yourself. For kinematic bodies, collision response has to be handled manually, as shown in
//! this example.
//!
//! Using dynamic bodies is often easier, as they handle most of the physics for you.
//! For a dynamic character controller, see the `basic_dynamic_character` example.

use bevy::{ecs::query::Has, prelude::*};
use bevy_xpbd_3d::{math::*, prelude::*, SubstepSchedule, SubstepSet};

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_event::<MovementAction>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                keyboard_input,
                gamepad_input,
                update_grounded,
                apply_deferred,
                apply_gravity,
                movement,
                apply_movement_damping,
            )
                .chain(),
        )
        .add_systems(
            // Run collision handling in substep schedule
            SubstepSchedule,
            kinematic_controller_collisions.in_set(SubstepSet::SolveUserConstraints),
        )
        .run();
}

/// An event sent for a movement input action.
#[derive(Event)]
enum MovementAction {
    Move(Vector2),
    Jump,
}

/// A marker component indicating that an entity is using a character controller.
#[derive(Component)]
struct CharacterController;

/// A marker component indicating that an entity is on the ground.
#[derive(Component)]
#[component(storage = "SparseSet")]
struct Grounded;

/// The acceleration used for character movement.
#[derive(Component)]
struct MovementAcceleration(Scalar);

/// The damping factor used for slowing down movement.
#[derive(Component)]
struct MovementDampingFactor(Scalar);

/// The strength of a jump.
#[derive(Component)]
struct JumpImpulse(Scalar);

/// The gravitational acceleration used for a character controller.
#[derive(Component)]
struct ControllerGravity(Vector);

/// A bundle that contains the components needed for a basic
/// kinematic character controller.
#[derive(Bundle)]
struct CharacterControllerBundle {
    character_controller: CharacterController,
    rigid_body: RigidBody,
    collider: Collider,
    ground_caster: ShapeCaster,
    movement_acceleration: MovementAcceleration,
    movement_damping: MovementDampingFactor,
    jump_impulse: JumpImpulse,
    gravity: ControllerGravity,
}

impl CharacterControllerBundle {
    fn new(
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        gravity: Vector,
        collider: Collider,
    ) -> Self {
        // Create shape caster as a slightly smaller version of collider
        let mut caster_shape = collider.clone();
        caster_shape.set_scale(Vector::ONE * 0.99, 10);

        Self {
            character_controller: CharacterController,
            rigid_body: RigidBody::Kinematic,
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
            gravity: ControllerGravity(gravity),
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
            transform: Transform::from_xyz(0.0, 1.5, 0.0),
            ..default()
        },
        CharacterControllerBundle::new(
            30.0,
            0.92,
            8.0,
            // Two times the normal gravity
            Vector::NEG_Y * 9.81 * 2.0,
            Collider::capsule(1.0, 0.4),
        ),
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

/// Sends [`MovementAction`] events based on keyboard input.
fn keyboard_input(
    mut movement_event_writer: EventWriter<MovementAction>,
    keyboard_input: Res<Input<KeyCode>>,
) {
    let up = keyboard_input.any_pressed([KeyCode::W, KeyCode::Up]);
    let down = keyboard_input.any_pressed([KeyCode::S, KeyCode::Down]);
    let left = keyboard_input.any_pressed([KeyCode::A, KeyCode::Left]);
    let right = keyboard_input.any_pressed([KeyCode::D, KeyCode::Right]);

    let horizontal = right as i8 - left as i8;
    let vertical = up as i8 - down as i8;
    let direction = Vector2::new(horizontal as Scalar, vertical as Scalar).clamp_length_max(1.0);

    if direction != Vector2::ZERO {
        movement_event_writer.send(MovementAction::Move(direction));
    }

    if keyboard_input.just_pressed(KeyCode::Space) {
        movement_event_writer.send(MovementAction::Jump);
    }
}

/// Sends [`MovementAction`] events based on gamepad input.
fn gamepad_input(
    mut movement_event_writer: EventWriter<MovementAction>,
    gamepads: Res<Gamepads>,
    axes: Res<Axis<GamepadAxis>>,
    buttons: Res<Input<GamepadButton>>,
) {
    for gamepad in gamepads.iter() {
        let axis_lx = GamepadAxis {
            gamepad,
            axis_type: GamepadAxisType::LeftStickX,
        };
        let axis_ly = GamepadAxis {
            gamepad,
            axis_type: GamepadAxisType::LeftStickY,
        };

        if let (Some(x), Some(y)) = (axes.get(axis_lx), axes.get(axis_ly)) {
            movement_event_writer.send(MovementAction::Move(
                Vector2::new(x as Scalar, y as Scalar).clamp_length_max(1.0),
            ));
        }

        let jump_button = GamepadButton {
            gamepad,
            button_type: GamepadButtonType::South,
        };

        if buttons.just_pressed(jump_button) {
            movement_event_writer.send(MovementAction::Jump);
        }
    }
}

/// Updates the [`Grounded`] status for character controllers.
fn update_grounded(
    mut commands: Commands,
    mut query: Query<(Entity, &ShapeHits), With<CharacterController>>,
) {
    for (entity, hits) in &mut query {
        if !hits.is_empty() {
            commands.entity(entity).insert(Grounded);
        } else {
            commands.entity(entity).remove::<Grounded>();
        }
    }
}

/// Responds to [`MovementAction`] events and moves character controllers accordingly.
fn movement(
    time: Res<Time>,
    mut movement_event_reader: EventReader<MovementAction>,
    mut controllers: Query<(
        &MovementAcceleration,
        &JumpImpulse,
        &mut LinearVelocity,
        Has<Grounded>,
    )>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for event in movement_event_reader.iter() {
        for (movement_acceleration, jump_impulse, mut linear_velocity, is_grounded) in
            &mut controllers
        {
            match event {
                MovementAction::Move(direction) => {
                    linear_velocity.x += direction.x * movement_acceleration.0 * delta_time;
                    linear_velocity.z -= direction.y * movement_acceleration.0 * delta_time;
                }
                MovementAction::Jump => {
                    if is_grounded {
                        linear_velocity.y = jump_impulse.0;
                    }
                }
            }
        }
    }
}

/// Applies [`ControllerGravity`] to character controllers.
fn apply_gravity(
    time: Res<Time>,
    mut controllers: Query<(&ControllerGravity, &mut LinearVelocity, Has<Grounded>)>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for (gravity, mut linear_velocity, is_grounded) in &mut controllers {
        // Reset vertical velocity if grounded, otherwise apply gravity
        if is_grounded {
            linear_velocity.y = 0.0;
        } else {
            linear_velocity.0 += gravity.0 * delta_time;
        }
    }
}

/// Slows down movement in the XZ plane.
fn apply_movement_damping(mut query: Query<(&MovementDampingFactor, &mut LinearVelocity)>) {
    for (damping_factor, mut linear_velocity) in &mut query {
        // We could use `LinearDamping`, but we don't want to dampen movement along the Y axis
        linear_velocity.x *= damping_factor.0;
        linear_velocity.z *= damping_factor.0;
    }
}

/// Kinematic bodies do not get pushed by collisions by default,
/// so it needs to be done manually.
///
/// This system performs very basic collision response for kinematic
/// character controllers by pushing them along their contact normals
/// by the current penetration depths.
fn kinematic_controller_collisions(
    collisions: Res<Collisions>,
    mut bodies: Query<(&RigidBody, &mut Position, &Rotation), With<CharacterController>>,
) {
    // Iterate through collisions and move the kinematic body to resolve penetration
    for contacts in collisions.iter() {
        // If the collision didn't happen during this substep, skip the collision
        if !contacts.during_current_substep {
            continue;
        }
        if let Ok([(rb1, mut position1, rotation1), (rb2, mut position2, _)]) =
            bodies.get_many_mut([contacts.entity1, contacts.entity2])
        {
            for manifold in contacts.manifolds.iter() {
                for contact in manifold.contacts.iter() {
                    if contact.penetration <= Scalar::EPSILON {
                        continue;
                    }
                    if rb1.is_kinematic() && !rb2.is_kinematic() {
                        position1.0 -= contact.global_normal1(rotation1) * contact.penetration;
                    } else if rb2.is_kinematic() && !rb1.is_kinematic() {
                        position2.0 += contact.global_normal1(rotation1) * contact.penetration;
                    }
                }
            }
        }
    }
}
