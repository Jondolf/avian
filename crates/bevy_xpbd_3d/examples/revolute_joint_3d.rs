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

    let blue = materials.add(StandardMaterial {
        base_color: Color::rgb(0.2, 0.6, 0.8),
        ..default()
    });

    let anchor = commands
        .spawn_bundle(PbrBundle {
            mesh: cube.clone(),
            material: blue.clone(),
            transform: Transform {
                scale: Vec3::splat(1.0),
                translation: Vec3::ZERO,
                ..default()
            },
            ..default()
        })
        .insert_bundle(RigidBodyBundle::new_kinematic().with_pos(Vec3::new(0.0, 0.0, 0.0)))
        .insert(Player)
        .insert(MoveSpeed(0.3))
        .id();

    let object = commands
        .spawn_bundle(PbrBundle {
            mesh: cube,
            material: blue,
            transform: Transform {
                scale: Vec3::splat(1.0),
                translation: Vec3::ZERO,
                ..default()
            },
            ..default()
        })
        .insert_bundle(
            RigidBodyBundle::new_dynamic()
                .with_pos(Vec3::Y * -3.0)
                .with_mass_props_from_shape(&Shape::cuboid(0.5, 0.5, 0.5), 1.0),
        )
        .id();

    commands.spawn().insert(
        RevoluteJoint::new_with_compliance(anchor, object, 0.0)
            .with_local_anchor_1(Vec3::Y * -0.5)
            .with_local_anchor_2(Vec3::Y * 0.5)
            .with_aligned_axis(Vec3::Z)
            .with_angle_limits(-0.1 * std::f32::consts::PI, 0.5 * std::f32::consts::PI),
    );

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

    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, -10.0))
            .looking_at(Vec3::Y * 0.0, Vec3::Y),
        ..default()
    });
}

fn player_movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&mut LinVel, &MoveSpeed), With<Player>>,
) {
    for (mut vel, move_speed) in &mut query {
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
