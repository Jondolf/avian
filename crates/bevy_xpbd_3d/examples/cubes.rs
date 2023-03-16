use bevy::prelude::*;

#[cfg(feature = "f64")]
use bevy_xpbd_3d_f64::prelude::*;

#[cfg(feature = "f32")]
use bevy_xpbd_3d::prelude::*;

#[cfg(feature = "f64")]
use examples_common_3d_f64::XpbdExamplePlugin;

#[cfg(feature = "f32")]
use examples_common_3d::XpbdExamplePlugin;

#[derive(Component)]
struct Player;

#[derive(Component, Deref, DerefMut)]
pub struct MoveAcceleration(pub Scalar);

#[derive(Component, Deref, DerefMut)]
pub struct MaxLinearVelocity(pub Vector);

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

    let floor_size = Vec3::new(80.0, 1.0, 80.0);
    let _floor = commands
        .spawn(PbrBundle {
            mesh: cube.clone(),
            material: white,
            transform: Transform::from_scale(floor_size),
            ..default()
        })
        .insert(RigidBodyBundle::new_static().with_pos(Vec3::new(0.0, -1.0, 0.0)))
        .insert(ColliderBundle::new(
            &Shape::cuboid(
                floor_size.x as Scalar * 0.5,
                floor_size.y as Scalar * 0.5,
                floor_size.z as Scalar * 0.5,
            ),
            1.0,
        ));

    let radius = 1.0;
    let count_x = 4;
    let count_y = 4;
    let count_z = 4;
    for y in 0..count_y {
        for x in 0..count_x {
            for z in 0..count_z {
                let pos = Vec3::new(
                    (x as f32 - count_x as f32 * 0.5) * 2.1 * radius,
                    10.0 * radius * y as f32,
                    (z as f32 - count_z as f32 * 0.5) * 2.1 * radius,
                );
                commands
                    .spawn(PbrBundle {
                        mesh: cube.clone(),
                        material: blue.clone(),
                        transform: Transform {
                            scale: Vec3::splat(radius * 2.0),
                            ..default()
                        },
                        ..default()
                    })
                    .insert(RigidBodyBundle::new_dynamic().with_pos(pos + Vec3::Y * 5.0))
                    .insert(ColliderBundle::new(
                        &Shape::cuboid(radius as Scalar, radius as Scalar, radius as Scalar),
                        1.0,
                    ))
                    .insert(Player)
                    .insert(MoveAcceleration(0.1))
                    .insert(MaxLinearVelocity(Vector::new(30.0, 30.0, 30.0)));
            }
        }
    }

    // Directional 'sun' light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 20_000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform {
            translation: Vec3::new(0.0, 10.0, 0.0),
            rotation: Quat::from_euler(
                EulerRot::XYZ,
                std::f32::consts::PI * 1.3,
                std::f32::consts::PI * 1.85,
                0.0,
            ),
            ..default()
        },
        ..default()
    });

    commands.spawn(Camera3dBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 15.0, -50.0))
            .looking_at(Vec3::Y * 10.0, Vec3::Y),
        ..default()
    });
}

fn player_movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&mut LinVel, &MaxLinearVelocity, &MoveAcceleration), With<Player>>,
) {
    for (mut lin_vel, max_vel, move_acceleration) in &mut query {
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
        .insert_resource(Msaa::Sample4)
        .insert_resource(Gravity::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .add_startup_system(setup)
        .add_system(player_movement)
        .run();
}
