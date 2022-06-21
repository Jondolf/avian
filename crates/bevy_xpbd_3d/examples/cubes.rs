use bevy::prelude::*;
use bevy_xpbd_3d::{bundles::*, components::*, resources::Gravity, *};

#[derive(Component)]
struct Player;

#[derive(Component, Deref, DerefMut)]
pub struct MoveAcceleration(pub f32);

#[derive(Component, Deref, DerefMut)]
pub struct MaxLinearVelocity(pub Vec3);

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let cube = meshes.add(Mesh::from(shape::Cube { size: 1.0 }));

    let white = materials.add(StandardMaterial {
        base_color: Color::rgb(0.8, 0.8, 1.0),
        ..default()
    });

    let blue = materials.add(StandardMaterial {
        base_color: Color::rgb(0.2, 0.6, 0.8),
        ..default()
    });

    let floor_size = Vec3::new(30.0, 1.0, 30.0);
    let _floor = commands
        .spawn_bundle(PbrBundle {
            mesh: cube.clone(),
            material: white,
            transform: Transform::from_scale(floor_size),
            ..default()
        })
        .insert_bundle(RigidBodyBundle::new_static().with_pos(Vec3::new(0.0, -1.0, 0.0)))
        .insert(Collider::from_shape(ColliderShape::cuboid(
            floor_size.x * 0.5,
            floor_size.y * 0.5,
            floor_size.z * 0.5,
        )));

    let radius = 0.5;
    let count_x = 5;
    let count_y = 5;
    let count_z = 5;
    for y in 0..count_y {
        for x in 0..count_x {
            for z in 0..count_z {
                let pos = Vec3::new(
                    (x as f32 - count_x as f32 * 0.5) * 2.1 * radius,
                    2.1 * radius * y as f32,
                    (z as f32 - count_z as f32 * 0.5) * 2.1 * radius,
                );
                commands
                    .spawn_bundle(PbrBundle {
                        mesh: cube.clone(),
                        material: blue.clone(),
                        transform: Transform {
                            scale: Vec3::splat(radius * 2.0),
                            translation: pos,
                            ..default()
                        },
                        ..default()
                    })
                    .insert_bundle(RigidBodyBundle::new_dynamic().with_pos(pos))
                    .insert(Collider::from_shape(ColliderShape::cuboid(
                        radius, radius, radius,
                    )))
                    .insert(Player)
                    .insert(MoveAcceleration(0.1))
                    .insert(MaxLinearVelocity(Vec3::new(30.0, 30.0, 30.0)));
            }
        }
    }

    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 1.5,
    });

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 10.0, -20.0))
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

fn player_movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&mut LinVel, &MaxLinearVelocity, &MoveAcceleration), With<Player>>,
) {
    for (mut lin_vel, max_vel, move_acceleration) in query.iter_mut() {
        if keyboard_input.pressed(KeyCode::Up) {
            lin_vel.z += move_acceleration.0;
        }
        if keyboard_input.pressed(KeyCode::Down) {
            lin_vel.z -= move_acceleration.0;
        }
        if keyboard_input.pressed(KeyCode::Left) {
            lin_vel.x += move_acceleration.0;
        }
        if keyboard_input.pressed(KeyCode::Right) {
            lin_vel.x -= move_acceleration.0;
        }
        lin_vel.0 = lin_vel.0.clamp(-max_vel.0, max_vel.0);
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(Gravity::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(XPBDPlugin)
        .add_startup_system(setup)
        .add_system(player_movement)
        .run();
}
