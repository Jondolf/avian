use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_mesh = meshes.add(Mesh::from(shape::Cube { size: 1.0 }));
    let cube_material = materials.add(Color::rgb(0.8, 0.7, 0.6).into());

    // Spawn a static cube and a dynamic cube that is connected to it by a distance joint.
    let static_cube = commands
        .spawn((
            PbrBundle {
                mesh: cube_mesh.clone(),
                material: cube_material.clone(),
                ..default()
            },
            RigidBody::Static,
            Collider::cuboid(1., 1., 1.),
        ))
        .id();
    let dynamic_cube = commands
        .spawn((
            PbrBundle {
                mesh: cube_mesh,
                material: cube_material,
                transform: Transform::from_xyz(-2.0, -0.5, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(1., 1., 1.),
            MassPropertiesBundle::new_computed(&Collider::cuboid(1.0, 1.0, 1.0), 1.0),
        ))
        .id();

    // Add a distance joint to keep the cubes at a certain distance from each other.
    commands.spawn(
        DistanceJoint::new(static_cube, dynamic_cube)
            .with_local_anchor_2(0.5 * Vector::ONE)
            .with_rest_length(1.5)
            .with_compliance(1.0 / 400.0),
    );

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 0.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}
