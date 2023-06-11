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
        .insert(RigidBodyBundle::new_kinematic().with_pos(Vec2::new(0.0, 0.0)))
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
                .with_pos(Vec2::X * 1.5)
                .with_computed_mass_props(&Shape::cuboid(0.5, 0.5), 1.0),
        )
        .id();

    commands.spawn(FixedJoint::new(anchor, object).with_local_anchor_1(Vec2::X * 1.5));

    commands.spawn(Camera3dBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 100.0)),
        projection: OrthographicProjection {
            scale: 0.025,
            ..default()
        }
        .into(),
        ..default()
    });
}

fn player_movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&mut LinVel, &mut AngVel, &MoveSpeed), With<Player>>,
) {
    for (mut lin_vel, mut ang_vel, move_speed) in &mut query {
        lin_vel.0 *= 0.95;
        ang_vel.0 *= 0.95;
        if keyboard_input.pressed(KeyCode::Up) {
            lin_vel.y += move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Down) {
            lin_vel.y -= move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Left) {
            lin_vel.x -= move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Right) {
            lin_vel.x += move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::Q) {
            ang_vel.0 += move_speed.0;
        }
        if keyboard_input.pressed(KeyCode::E) {
            ang_vel.0 -= move_speed.0;
        }
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    App::new()
        .insert_resource(ClearColor(Color::rgb(0.0, 0.0, 0.1)))
        .insert_resource(Msaa::Sample4)
        .insert_resource(Gravity(Vec2::Y * -9.81))
        .insert_resource(NumSubsteps(50))
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .add_startup_system(setup)
        .add_system(player_movement)
        .run();
}
