use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*};
use examples_common_3d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(Msaa::Sample4)
        .insert_resource(SubstepCount(50))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let cube = PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        material: materials.add(Color::rgb(0.2, 0.7, 0.9).into()),
        ..default()
    };

    // Kinematic rotating "anchor" object
    let anchor = commands
        .spawn((
            cube.clone(),
            RigidBody::Kinematic,
            AngularVelocity(Vector::Z * 1.5),
        ))
        .id();

    // Dynamic object rotating around anchor
    let object = commands
        .spawn((
            cube,
            RigidBody::Dynamic,
            Position(Vector::X * 1.5),
            MassPropertiesBundle::new_computed(&Collider::cuboid(1.0, 1.0, 1.0), 1.0),
        ))
        .id();

    // Connect anchor and dynamic object
    commands.spawn(
        PrismaticJoint::new(anchor, object)
            .with_local_anchor_1(Vector::X)
            .with_free_axis(Vector::X)
            .with_limits(0.5, 2.0),
    );

    // Directional light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 20_000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
        ..default()
    });

    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_translation(Vec3::Z * 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}
