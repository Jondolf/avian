use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;
use examples_common_3d::XpbdExamplePlugin;

#[derive(Component)]
struct Player;

#[derive(Component, Deref, DerefMut)]
pub struct MoveSpeed(pub f32);

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

    let sphere = meshes.add(Mesh::from(shape::Icosphere {
        radius: 0.5,
        subdivisions: 4,
    }));

    let floor_size = Vec3::new(30.0, 1.0, 30.0);
    let _floor = commands
        .spawn_bundle(PbrBundle {
            mesh: cube,
            material: white,
            transform: Transform::from_scale(floor_size),
            ..default()
        })
        .insert_bundle(RigidBodyBundle::new_static().with_pos(Vec3::new(0.0, -18.0, 0.0)))
        .insert(Collider::from_shape(ColliderShape::cuboid(
            floor_size.x * 0.5,
            floor_size.y * 0.5,
            floor_size.z * 0.5,
        )));

    // Rope
    create_chain(
        &mut commands,
        sphere,
        blue,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::Y,
        100,
        0.001,
        0.075,
        0.0,
    );

    // Pendulum
    /*create_chain(
        &mut commands,
        cube,
        blue,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::Y,
        2,
        3.0,
        1.0,
        0.0,
    );*/

    // Directional 'sun' light
    let sun_half_size = 50.0;
    commands.spawn_bundle(DirectionalLightBundle {
        directional_light: DirectionalLight {
            // Configure the projection to better fit the scene
            shadow_projection: OrthographicProjection {
                left: -sun_half_size,
                right: sun_half_size,
                bottom: -sun_half_size,
                top: sun_half_size,
                near: -10.0 * sun_half_size,
                far: 10.0 * sun_half_size,
                ..default()
            },
            illuminance: 20_000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform {
            translation: Vec3::new(0.0, 10.0, 0.0),
            rotation: Quat::from_euler(
                EulerRot::XYZ,
                std::f32::consts::PI * 1.3,
                std::f32::consts::PI * 2.05,
                0.0,
            ),
            ..default()
        },
        ..default()
    });

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, -20.0))
            .looking_at(Vec3::Y * -10.0, Vec3::Y),
        ..default()
    });
}

#[allow(clippy::too_many_arguments)]
fn create_chain(
    commands: &mut Commands,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    start_pos: Vec3,
    dir: Vec3,
    node_count: usize,
    node_dist: f32,
    node_size: f32,
    compliance: f32,
) {
    let mut prev = commands
        .spawn_bundle(PbrBundle {
            mesh: mesh.clone(),
            material: material.clone(),
            transform: Transform {
                scale: Vec3::splat(node_size),
                translation: Vec3::ZERO,
                ..default()
            },
            ..default()
        })
        .insert_bundle(RigidBodyBundle::new_kinematic().with_pos(start_pos))
        .insert(Player)
        .insert(MoveSpeed(0.3))
        .id();

    for i in 1..node_count {
        let delta_pos = -dir * (node_size + node_dist);
        let curr = commands
            .spawn_bundle(PbrBundle {
                mesh: mesh.clone(),
                material: material.clone(),
                transform: Transform {
                    scale: Vec3::splat(node_size),
                    translation: Vec3::ZERO,
                    ..default()
                },
                ..default()
            })
            .insert_bundle(
                RigidBodyBundle::new_dynamic().with_pos(start_pos + delta_pos * i as f32),
            )
            .insert(ExplicitMassProperties(MassProperties::from_shape(
                &ColliderShape::ball(node_size * 0.5),
                1.0,
            )))
            .id();

        commands.spawn().insert(
            SphericalJoint::new_with_compliance(prev, curr, compliance)
                .with_local_anchor_1(0.5 * delta_pos)
                .with_local_anchor_2(-0.5 * delta_pos),
        );

        prev = curr;
    }
}

fn player_movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&mut LinVel, &MoveSpeed), With<Player>>,
) {
    for (mut vel, move_speed) in query.iter_mut() {
        vel.0 *= 0.95;
        if keyboard_input.pressed(KeyCode::Up) {
            vel.z += move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Down) {
            vel.z -= move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Left) {
            vel.x += move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Right) {
            vel.x -= move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::W) {
            vel.y += move_speed.0 * 0.75;
        }
        if keyboard_input.pressed(KeyCode::S) {
            vel.y -= move_speed.0 * 0.75;
        }
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    App::new()
        .insert_resource(ClearColor(Color::rgb(0.0, 0.0, 0.1)))
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(Gravity(Vec3::Y * -9.81))
        .insert_resource(NumSubsteps(50))
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .add_startup_system(setup)
        .add_system(player_movement)
        .run();
}
