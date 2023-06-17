use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;
use examples_common_3d::XpbdExamplePlugin;

#[derive(Component)]
struct Player;

#[derive(Component, Deref, DerefMut)]
pub struct RollAcceleration(pub Vec3);

#[derive(Component, Deref, DerefMut)]
pub struct MaxAngularVelocity(pub Vec3);

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let ball = meshes.add(
        Mesh::try_from(shape::Icosphere {
            radius: 0.5,
            subdivisions: 4,
        })
        .unwrap(),
    );
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
    let _floor = commands.spawn((
        PbrBundle {
            mesh: cube,
            material: white,
            transform: Transform::from_scale(floor_size),
            ..default()
        },
        RigidBody::Static,
        Position(Vec3::NEG_Y * 5.0),
        Collider::cuboid(floor_size.x, floor_size.y, floor_size.z),
    ));

    let radius = 0.5;
    let count_x = 1;
    let count_y = 500;
    let count_z = 1;
    for y in 0..count_y {
        for x in 0..count_x {
            for z in 0..count_z {
                let pos = Vec3::new(
                    (x as f32 - count_x as f32 * 0.5) * 2.5 * radius,
                    2.5 * radius * y as f32,
                    (z as f32 - count_z as f32 * 0.5) * 2.5 * radius,
                );
                commands.spawn((
                    PbrBundle {
                        mesh: ball.clone(),
                        material: blue.clone(),
                        transform: Transform::from_scale(Vec3::splat(radius * 2.0)),
                        ..default()
                    },
                    RigidBody::Dynamic,
                    Position(pos),
                    Collider::ball(radius),
                    Player,
                    RollAcceleration(Vec3::splat(0.5)),
                    MaxAngularVelocity(Vec3::splat(30.0)),
                ));
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
    mut query: Query<(&mut AngularVelocity, &MaxAngularVelocity, &RollAcceleration), With<Player>>,
) {
    for (mut ang_vel, max_ang_vel, move_acceleration) in &mut query {
        if keyboard_input.pressed(KeyCode::Up) {
            ang_vel.x += move_acceleration.x;
        }
        if keyboard_input.pressed(KeyCode::Down) {
            ang_vel.x -= move_acceleration.x;
        }
        if keyboard_input.pressed(KeyCode::Left) {
            ang_vel.z -= move_acceleration.z;
        }
        if keyboard_input.pressed(KeyCode::Right) {
            ang_vel.z += move_acceleration.z;
        }
        ang_vel.0 = ang_vel.0.clamp(-max_ang_vel.0, max_ang_vel.0);
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(Msaa::Sample4)
        .insert_resource(SubstepCount(6))
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .add_startup_system(setup)
        .add_system(player_movement)
        .run();
}
