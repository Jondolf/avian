use avian3d::{math::*, prelude::*};
use bevy::{ecs::query::Has, prelude::*};
// Skin width for the character controller.
// Basically a small offset to make the collider appear slightly larger.
pub const SKIN_WIDTH: f32 = 0.01;
// Number of collision steps for the collide and slide algorithm to act upon.
pub const COLLISION_STEPS: usize = 5;
pub struct CharacterControllerPlugin;

impl Plugin for CharacterControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<MovementAction>()
            .add_systems(
                Update,
                (
                    keyboard_input,
                    gamepad_input,
                    update_grounded,
                    apply_gravity,
                    movement,
                    apply_movement_damping,
                )
                    .chain(),
            )
            .add_systems(PostProcessCollisions, collide_and_slide);
    }
}

/// An event sent for a movement input action.
#[derive(Event)]
pub enum MovementAction {
    Move(Vector2),
    Jump,
}

/// A marker component indicating that an entity is using a character controller.
#[derive(Component, Default)]
pub struct CharacterController {
    pub velocity: Vec3,
}

/// A marker component indicating that an entity is on the ground.
#[derive(Component)]
#[component(storage = "SparseSet")]
pub struct Grounded;
/// The acceleration used for character movement.
#[derive(Component)]
pub struct MovementAcceleration(Scalar);

/// The damping factor used for slowing down movement.
#[derive(Component)]
pub struct MovementDampingFactor(Scalar);

/// The strength of a jump.
#[derive(Component)]
pub struct JumpImpulse(Scalar);

/// The gravitational acceleration used for a character controller.
#[derive(Component)]
pub struct ControllerGravity(Vector);

/// The maximum angle a slope can have for a character controller
/// to be able to climb and jump. If the slope is steeper than this angle,
/// the character will slide down.
#[derive(Component)]
pub struct MaxSlopeAngle(Scalar);

/// A bundle that contains the components needed for a basic
/// kinematic character controller.
#[derive(Bundle)]
pub struct CharacterControllerBundle {
    character_controller: CharacterController,
    rigid_body: RigidBody,
    collider: Collider,
    ground_caster: ShapeCaster,
    gravity: ControllerGravity,
    movement: MovementBundle,
}

/// A bundle that contains components for character movement.
#[derive(Bundle)]
pub struct MovementBundle {
    acceleration: MovementAcceleration,
    damping: MovementDampingFactor,
    jump_impulse: JumpImpulse,
    max_slope_angle: MaxSlopeAngle,
}

impl MovementBundle {
    pub const fn new(
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        max_slope_angle: Scalar,
    ) -> Self {
        Self {
            acceleration: MovementAcceleration(acceleration),
            damping: MovementDampingFactor(damping),
            jump_impulse: JumpImpulse(jump_impulse),
            max_slope_angle: MaxSlopeAngle(max_slope_angle),
        }
    }
}

impl Default for MovementBundle {
    fn default() -> Self {
        Self::new(30.0, 0.9, 7.0, PI * 0.45)
    }
}

impl CharacterControllerBundle {
    pub fn new(collider: Collider, gravity: Vector) -> Self {
        // Create shape caster as a slightly smaller version of collider
        let mut caster_shape = collider.clone();
        caster_shape.set_scale(Vector::ONE * 0.99, 10);

        Self {
            character_controller: CharacterController::default(),
            rigid_body: RigidBody::Kinematic,
            collider,
            ground_caster: ShapeCaster::new(
                caster_shape,
                Vector::ZERO,
                Quaternion::default(),
                Dir3::NEG_Y,
            )
            .with_max_time_of_impact(0.2),
            gravity: ControllerGravity(gravity),
            movement: MovementBundle::default(),
        }
    }

    pub fn with_movement(
        mut self,
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        max_slope_angle: Scalar,
    ) -> Self {
        self.movement = MovementBundle::new(acceleration, damping, jump_impulse, max_slope_angle);
        self
    }
}

/// Sends [`MovementAction`] events based on keyboard input.
fn keyboard_input(
    mut movement_event_writer: EventWriter<MovementAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    let up = keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
    let down = keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
    let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
    let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

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
    buttons: Res<ButtonInput<GamepadButton>>,
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
    mut query: Query<
        (Entity, &ShapeHits, &Rotation, Option<&MaxSlopeAngle>),
        With<CharacterController>,
    >,
) {
    for (entity, hits, rotation, max_slope_angle) in &mut query {
        // The character is grounded if the shape caster has a hit with a normal
        // that isn't too steep.
        let is_grounded = hits.iter().any(|hit| {
            if let Some(angle) = max_slope_angle {
                (rotation * -hit.normal2).angle_between(Vector::Y).abs() <= angle.0
            } else {
                true
            }
        });

        if is_grounded {
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
        &mut CharacterController,
        Has<Grounded>,
    )>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for event in movement_event_reader.read() {
        for (movement_acceleration, jump_impulse, _, mut character_controller, is_grounded) in
            &mut controllers
        {
            match event {
                MovementAction::Move(direction) => {
                    character_controller.velocity.x +=
                        direction.x * movement_acceleration.0 * delta_time;
                    character_controller.velocity.z -=
                        direction.y * movement_acceleration.0 * delta_time;
                }
                MovementAction::Jump => {
                    if is_grounded {
                        character_controller.velocity.y = jump_impulse.0;
                    }
                }
            }
        }
    }
}

/// Applies [`ControllerGravity`] to character controllers.
fn apply_gravity(
    time: Res<Time>,
    mut controllers: Query<(&ControllerGravity, &mut CharacterController)>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for (gravity, mut character_controller) in &mut controllers {
        character_controller.velocity += gravity.0 * delta_time;
    }
}

/// Slows down movement in the XZ plane.
fn apply_movement_damping(mut query: Query<(&MovementDampingFactor, &mut CharacterController)>) {
    for (damping_factor, mut character_controller) in &mut query {
        // We could use `LinearDamping`, but we don't want to dampen movement along the Y axis
        character_controller.velocity.x *= damping_factor.0;
        character_controller.velocity.z *= damping_factor.0;
    }
}

/// This system is used to run the recursive_collide_and_slide function for our kinematic character controllers.
fn collide_and_slide(
    mut character_controllers: Query<
        (
            &mut Transform,
            Option<&MaxSlopeAngle>,
            &Collider,
            Entity,
            Has<Grounded>,
            &mut CharacterController,
        ),
        With<RigidBody>,
    >,
    mut spatial_query: SpatialQuery,
    time: Res<Time>,
) {
    let delta_seconds = time.delta_seconds_f64().adjust_precision();

    // Iterate over all character controllers and run the recursive_collide_and_slide function.
    for (mut transform, _max_slope_angle, collider, entity, grounded, mut character_controller) in
        &mut character_controllers
    {
        let velocity = character_controller.velocity * delta_seconds;

        // Filter out ourself from the spatial query.
        let mut filter = SpatialQueryFilter::default().with_excluded_entities([entity]);

        // This algorithm keeps a list of planes for the function in order to prevent a "crushing" effect
        // where tight corridors can cause the character to get stuck or otherwise forced into the ground
        let mut planes = Vec::new();

        let translation = recursive_collide_and_slide(
            &mut spatial_query,
            &mut filter,
            &collider,
            &transform,
            COLLISION_STEPS,
            velocity,
            SKIN_WIDTH,
            &mut planes,
            Vector::Y,
            grounded,
        );

        // Move us to the new position
        transform.translation += translation;

        // Update the velocity
        character_controller.velocity = translation / delta_seconds;
    }
}

/// Kinematic bodies do not get pushed by collisions by default,
/// so it needs to be done manually.
///
/// This function handles the collision response for kinematic character controllers.
/// Its based upon the collide-and-slide algorithm, which is a common approach for
/// handling collisions with kinematic bodies.
///
/// This specific implementation is based primarily on [Improved Collision detection and Response](https://www.peroxide.dk/papers/collision/collision.pdf).
/// by Kasper Fauerby.
fn recursive_collide_and_slide(
    spatial_query: &mut spatial_query::SpatialQuery,
    filter: &mut spatial_query::SpatialQueryFilter,
    collider: &Collider,
    transform: &Transform,
    max_depth: usize,
    velocity: Vec3,
    padding: f32,
    planes: &mut Vec<Vec3>,
    up_vector: Vec3,
    grounded: bool,
) -> Vec3 {
    if max_depth == 0 {
        return velocity;
    }

    if velocity.length_squared() < 0.00001 {
        return Vec3::ZERO;
    }

    if !velocity.is_finite() {
        error!(
            "Failed to run `recursive_collide_and_slide`: velocity is not finite, but `{velocity:?}`"
        );
        return Vec3::ZERO;
    }

    // Safety: we already returned if `velocity` is zero or not finite
    let (velocity_normalized, length) = Dir3::new_and_length(velocity).unwrap();

    let cast_result = if let Some(cast_result) = spatial_query.cast_shape(
        collider,
        transform.translation,
        transform.rotation,
        velocity_normalized,
        length,
        true,
        &filter.clone(),
    ) {
        cast_result
    } else {
        return velocity;
    };

    if (cast_result.time_of_impact - padding).abs() > 0.01 {
        planes.clear();
    }

    planes.push(cast_result.normal1);

    let surface_point = velocity * (cast_result.time_of_impact - padding).max(0.0);
    let remaining_velocity = velocity - surface_point;

    let mut projected_velocity =
        remaining_velocity - cast_result.normal1 * remaining_velocity.dot(cast_result.normal1);

    // Performance: This could get expensive with a lot of planes. We should optimize this later.
    if planes.len() > 1 {
        for (plane, next_plane) in planes
            .iter()
            .zip(planes.iter().cycle().skip(1))
            .take(planes.len())
        {
            let crease = plane.cross(*next_plane);
            projected_velocity = crease * crease.dot(projected_velocity);
        }
    }

    if projected_velocity.dot(velocity) <= 0.0 {
        return Vec3::ZERO;
    }

    return surface_point
        + recursive_collide_and_slide(
            spatial_query,
            filter,
            collider,
            transform,
            max_depth - 1,
            projected_velocity,
            padding,
            planes,
            up_vector,
            grounded,
        );
}
