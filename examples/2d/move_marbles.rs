use bevy::prelude::*;
use bevy_xpbd::{bundles::*, components::*, resources::Gravity, *};

#[derive(Component)]
struct Player;

#[derive(Component, Deref, DerefMut)]
pub struct MoveAcceleration(pub f32);

#[derive(Component, Deref, DerefMut)]
pub struct MaxVelocity(pub Vec2);

pub enum MovementEvent {
    Up(Entity),
    Down(Entity),
    Left(Entity),
    Right(Entity),
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let sphere = meshes.add(Mesh::from(shape::Icosphere {
        radius: 1.0,
        subdivisions: 4,
    }));

    let white = materials.add(StandardMaterial {
        base_color: Color::rgb(0.8, 0.8, 1.0),
        unlit: true,
        ..default()
    });

    let blue = materials.add(StandardMaterial {
        base_color: Color::rgb(0.2, 0.6, 0.8),
        unlit: true,
        ..default()
    });

    let floor = commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Quad::new(Vec2::ONE))),
            material: white.clone(),
            transform: Transform::from_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        })
        .insert_bundle(StaticBodyBundle {
            pos: Pos(Vec2::new(0.0, -7.5)),
            restitution: Restitution(0.3),
        })
        .insert_bundle(ColliderBundle::with_shape(ColliderShape::cuboid(10.0, 0.5)));

    let ceiling = commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Quad::new(Vec2::ONE))),
            material: white.clone(),
            transform: Transform::from_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        })
        .insert_bundle(StaticBodyBundle {
            pos: Pos(Vec2::new(0.0, 7.5)),
            restitution: Restitution(0.3),
        })
        .insert_bundle(ColliderBundle::with_shape(ColliderShape::cuboid(10.0, 0.5)));

    let left_wall = commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Quad::new(Vec2::ONE))),
            material: white.clone(),
            transform: Transform::from_scale(Vec3::new(1.0, 15.0, 1.0)),
            ..default()
        })
        .insert_bundle(StaticBodyBundle {
            pos: Pos(Vec2::new(-9.5, 0.0)),
            restitution: Restitution(0.3),
        })
        .insert_bundle(ColliderBundle::with_shape(ColliderShape::cuboid(0.5, 10.0)));

    let right_wall = commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Quad::new(Vec2::ONE))),
            material: white.clone(),
            transform: Transform::from_scale(Vec3::new(1.0, 15.0, 1.0)),
            ..default()
        })
        .insert_bundle(StaticBodyBundle {
            pos: Pos(Vec2::new(9.5, 0.0)),
            restitution: Restitution(0.3),
        })
        .insert_bundle(ColliderBundle::with_shape(ColliderShape::cuboid(0.5, 10.0)));

    let radius = 0.15;
    let stacks = 25;
    for i in 0..15 {
        for j in 0..stacks {
            let pos = Vec2::new(
                (j as f32 - stacks as f32 * 0.5) * 2.5 * radius,
                2.0 * radius * i as f32 - 2.0,
            );
            let vel = Vec2::ZERO;
            commands
                .spawn_bundle(PbrBundle {
                    mesh: sphere.clone(),
                    material: blue.clone(),
                    transform: Transform {
                        scale: Vec3::splat(radius),
                        translation: pos.extend(0.0),
                        ..default()
                    },
                    ..default()
                })
                .insert_bundle(DynamicBodyBundle {
                    restitution: Restitution(0.3),
                    ..DynamicBodyBundle::new_with_pos_and_vel(pos, vel)
                })
                .insert_bundle(ColliderBundle::with_shape(ColliderShape::ball(radius)))
                .insert(Player)
                .insert(MoveAcceleration(0.005))
                .insert(MaxVelocity(Vec2::new(30.0, 30.0)));
        }
    }

    commands.spawn_bundle(OrthographicCameraBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 100.0)),
        orthographic_projection: OrthographicProjection {
            scale: 0.025,
            ..default()
        },
        ..OrthographicCameraBundle::new_3d()
    });
}

fn handle_input(
    keyboard_input: Res<Input<KeyCode>>,
    mut ev_movement: EventWriter<MovementEvent>,
    query: Query<Entity, With<Player>>,
) {
    for entity in query.iter() {
        if keyboard_input.pressed(KeyCode::Up) {
            ev_movement.send(MovementEvent::Up(entity));
        }
        if keyboard_input.pressed(KeyCode::Down) {
            ev_movement.send(MovementEvent::Down(entity));
        }
        if keyboard_input.pressed(KeyCode::Left) {
            ev_movement.send(MovementEvent::Left(entity));
        }
        if keyboard_input.pressed(KeyCode::Right) {
            ev_movement.send(MovementEvent::Right(entity));
        }
    }
}

fn player_movement(
    mut ev_movement: EventReader<MovementEvent>,
    mut query: Query<(&mut LinVel, &MaxVelocity, &MoveAcceleration), With<Player>>,
) {
    for ev in ev_movement.iter() {
        for (mut vel, max_vel, move_acceleration) in query.iter_mut() {
            match ev {
                MovementEvent::Up(_ent) => vel.y += move_acceleration.0,
                MovementEvent::Down(_ent) => vel.y -= move_acceleration.0,
                MovementEvent::Left(_ent) => vel.x -= move_acceleration.0,
                MovementEvent::Right(_ent) => vel.x += move_acceleration.0,
            }
            vel.0 = vel.0.clamp(-max_vel.0, max_vel.0);
        }
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(Gravity(Vec2::new(0.0, -9.81)))
        .add_plugins(DefaultPlugins)
        .add_plugin(XPBDPlugin)
        .add_event::<MovementEvent>()
        .add_startup_system(setup)
        .add_system(handle_input)
        .add_system(player_movement)
        .run();
}
