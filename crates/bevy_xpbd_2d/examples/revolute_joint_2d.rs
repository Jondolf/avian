use bevy::prelude::*;
use bevy_xpbd_2d::prelude::*;
use examples_common_2d::XpbdExamplePlugin;

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
        unlit: true,
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
        .insert_bundle(RigidBodyBundle::new_kinematic().with_pos(Vec2::new(0.0, 0.0)))
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
        .insert_bundle(RigidBodyBundle::new_dynamic().with_pos(Vec2::Y * -3.0))
        .insert(ExplicitMassProperties(MassProperties::from_shape(
            &ColliderShape::cuboid(0.5, 0.5),
            1.0,
        )))
        .id();

    commands.spawn().insert(
        RevoluteJoint::new_with_compliance(anchor, object, 0.0)
            .with_local_anchor_1(Vec2::Y * -0.5)
            .with_local_anchor_2(Vec2::Y * 0.5)
            .with_angle_limits(0.0, 0.5 * std::f32::consts::PI),
    );

    commands.spawn_bundle(OrthographicCameraBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 100.0)),
        orthographic_projection: OrthographicProjection {
            scale: 0.025,
            ..default()
        },
        ..OrthographicCameraBundle::new_3d()
    });
}

fn player_movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&mut LinVel, &MoveSpeed), With<Player>>,
) {
    for (mut vel, move_speed) in query.iter_mut() {
        vel.0 *= 0.95;
        if keyboard_input.pressed(KeyCode::Up) {
            vel.y += move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Down) {
            vel.y -= move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Left) {
            vel.x -= move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Right) {
            vel.x += move_speed.0;
        }
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    App::new()
        .insert_resource(ClearColor(Color::rgb(0.0, 0.0, 0.1)))
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(Gravity(Vec2::Y * -9.81))
        .insert_resource(NumSubsteps(50))
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .add_startup_system(setup)
        .add_system(player_movement)
        .run();
}
