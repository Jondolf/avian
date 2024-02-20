use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_xpbd_2d::{math::*, prelude::*};
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::ZERO))
        .add_event::<MovementAction>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                keyboard_input1,
                keyboard_input2,
                movement.run_if(has_movement), // don't mutably access the character if there is no movement
                apply_movement_damping,
                apply_pressure_plate_colour,
                update_velocity_text,
                log_events,
            )
                .chain(),
        )
        .run();
}

#[derive(Component, Default, Reflect)]
struct Character;

#[derive(Component, Default, Reflect)]
struct PressurePlate;

#[derive(Component, Default, Reflect)]
struct CharacterVelocityText;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(Capsule2d::new(12.5, 20.0)).into(),
            material: materials.add(Color::rgb(0.2, 0.7, 0.9)),
            transform: Transform::from_xyz(0.0, -100.0, 1.0),
            ..default()
        },
        Character,
        RigidBody::Dynamic,
        Collider::capsule(20.0, 12.5),
        Name::new("Character"),
    ));

    commands.spawn((
        SpriteBundle {
            transform: Transform::from_xyz(0.0, 150.0, 0.0),
            sprite: Sprite {
                color: Color::WHITE,
                custom_size: Some(Vec2::new(100.0, 100.0)),
                ..default()
            },
            ..default()
        },
        PressurePlate,
        Sensor,
        RigidBody::Static,
        Collider::rectangle(100.0, 100.0),
        Name::new("Pressure Plate"),
    ));

    commands.spawn((
        TextBundle::from_section(
            "Velocity: ",
            TextStyle {
                font_size: 16.0,
                ..default()
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            bottom: Val::Px(5.0),
            left: Val::Px(5.0),
            ..default()
        }),
        CharacterVelocityText,
        Name::new("Character Velocity Text"),
    ));

    commands.spawn(Camera2dBundle::default());
}

#[derive(Event, Debug, Reflect)]
pub enum MovementAction {
    Velocity(Vector),
    Offset(Vector),
    Stop,
}

// use velocity
fn keyboard_input1(
    mut movement_event_writer: EventWriter<MovementAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    time: Res<Time<Physics>>,
) {
    if time.is_paused() {
        return;
    }
    let space = keyboard_input.pressed(KeyCode::Space);
    if space {
        movement_event_writer.send(MovementAction::Stop);
        return;
    }

    let left = keyboard_input.any_pressed([KeyCode::KeyA]);
    let right = keyboard_input.any_pressed([KeyCode::KeyD]);
    let up = keyboard_input.any_pressed([KeyCode::KeyW]);
    let down = keyboard_input.any_pressed([KeyCode::KeyS]);
    let horizontal = right as i8 - left as i8;
    let vertical = up as i8 - down as i8;
    let direction = Vector::new(horizontal as Scalar, vertical as Scalar);
    if direction != Vector::ZERO {
        movement_event_writer.send(MovementAction::Velocity(direction));
    }
}
// use position offset
fn keyboard_input2(
    mut movement_event_writer: EventWriter<MovementAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    time: Res<Time<Physics>>,
) {
    if time.is_paused() {
        return;
    }
    let left = keyboard_input.any_pressed([KeyCode::ArrowLeft]);
    let right = keyboard_input.any_pressed([KeyCode::ArrowRight]);
    let up = keyboard_input.any_pressed([KeyCode::ArrowUp]);
    let down = keyboard_input.any_pressed([KeyCode::ArrowDown]);
    let horizontal = right as i8 - left as i8;
    let vertical = up as i8 - down as i8;
    let direction = Vector::new(horizontal as Scalar, vertical as Scalar);
    if direction != Vector::ZERO {
        movement_event_writer.send(MovementAction::Offset(direction));
    }
}

fn has_movement(mut reader: EventReader<MovementAction>) -> bool {
    reader.read().next().is_some()
}
fn movement(
    time: Res<Time>,
    mut movement_event_reader: EventReader<MovementAction>,
    mut controllers: Query<(&mut LinearVelocity, &mut Position), With<Character>>,
) {
    let delta_time = time.delta_seconds_f64().adjust_precision();
    for event in movement_event_reader.read() {
        for (mut linear_velocity, mut position) in &mut controllers {
            match event {
                MovementAction::Stop => {
                    linear_velocity.x = 0.0;
                    linear_velocity.y = 0.0;
                }
                MovementAction::Velocity(direction) => {
                    let movement_acceleration = 2000.0;
                    linear_velocity.x += direction.x * movement_acceleration * delta_time;
                    linear_velocity.y += direction.y * movement_acceleration * delta_time;
                }
                MovementAction::Offset(direction) => {
                    let speed = 100.0;
                    position.x += direction.x * speed * delta_time;
                    position.y += direction.y * speed * delta_time;
                }
            }
        }
    }
}

#[allow(clippy::type_complexity)]
fn apply_movement_damping(
    mut query: Query<
        (&mut LinearVelocity, &mut AngularVelocity),
        (With<Character>, Without<Sleeping>),
    >,
    time: Res<Time<Physics>>,
) {
    if time.is_paused() {
        return;
    }
    let damping_factor = 0.95;
    for (mut linear_velocity, mut angular_velocity) in &mut query {
        linear_velocity.x *= damping_factor;
        if linear_velocity.x.abs() < 0.001 {
            linear_velocity.x = 0.0;
        }
        linear_velocity.y *= damping_factor;
        if linear_velocity.y.abs() < 0.001 {
            linear_velocity.y = 0.0;
        }
        angular_velocity.0 *= damping_factor;
        if angular_velocity.0.abs() < 0.001 {
            angular_velocity.0 = 0.0;
        }
    }
}

fn apply_pressure_plate_colour(
    mut query: Query<(&mut Sprite, &CollidingEntities), With<PressurePlate>>,
) {
    for (mut sprite, colliding_entities) in &mut query {
        if colliding_entities.0.is_empty() {
            sprite.color = Color::rgb(0.2, 0.7, 0.9);
        } else {
            sprite.color = Color::rgb(0.9, 0.7, 0.2);
        }
    }
}

fn update_velocity_text(
    character_query: Query<(&LinearVelocity, Has<Sleeping>), With<Character>>,
    pressure_plate_query: Query<Has<Sleeping>, With<PressurePlate>>,
    mut text_query: Query<&mut Text, With<CharacterVelocityText>>,
) {
    if let (Ok((velocity, character_sleeping)), Ok(pressure_plate_sleeping)) = (
        character_query.get_single(),
        pressure_plate_query.get_single(),
    ) {
        text_query.single_mut().sections[0].value = format!(
            "Velocity: {:.4}, {:.4}\nCharacter sleeping:{}\nPressure plate sleeping: {}",
            velocity.x, velocity.y, character_sleeping, pressure_plate_sleeping
        );
    }
}

fn log_events(mut started: EventReader<CollisionStarted>, mut ended: EventReader<CollisionEnded>) {
    // print out the started and ended events
    for event in started.read() {
        println!("CollisionStarted: {:?}", event);
    }
    for event in ended.read() {
        println!("CollisionEnded: {:?}", event);
    }
}
