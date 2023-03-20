use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;
use examples_common_3d::XpbdExamplePlugin;

#[derive(Component)]
struct Player;

#[derive(Component, Deref, DerefMut)]
pub struct MoveSpeed(pub Scalar);

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
        .spawn(PbrBundle {
            mesh: cube.clone(),
            material: blue.clone(),
            transform: Transform {
                scale: Vec3::splat(1.0),
                translation: Vec3::ZERO,
                ..default()
            },
            ..default()
        })
        .insert(RigidBodyBundle::new_kinematic().with_pos(Vec3::new(0.0, 0.0, 0.0)))
        .insert(Player)
        .insert(MoveSpeed(0.3))
        .id();

    let object = commands
        .spawn(PbrBundle {
            mesh: cube,
            material: blue,
            transform: Transform {
                scale: Vec3::splat(1.0),
                translation: Vec3::ZERO,
                ..default()
            },
            ..default()
        })
        .insert(
            RigidBodyBundle::new_dynamic()
                .with_pos(Vec3::Y * -3.0)
                .with_mass_props_from_shape(&Shape::cuboid(0.5, 0.5, 0.5), 1.0),
        )
        .id();

    commands.spawn(
        RevoluteJoint::new_with_compliance(anchor, object, 0.0)
            .with_local_anchor_1(Vector::Y * -0.5)
            .with_local_anchor_2(Vector::Y * 0.5)
            .with_aligned_axis(Vector::Z)
            .with_angle_limits(-0.1 * PI, 0.5 * PI),
    );

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
                std::f32::consts::PI * 2.05,
                0.0,
            ),
            ..default()
        },
        ..default()
    });

    commands.spawn(Camera3dBundle {
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
        .insert_resource(Msaa::Sample4)
        .insert_resource(Gravity(Vector::Y * -9.81))
        .insert_resource(NumSubsteps(50))
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .add_startup_system(setup)
        .add_system(player_movement)
        .run();
}
