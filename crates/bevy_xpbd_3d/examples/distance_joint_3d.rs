use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*, SubstepSchedule, SubstepSet};

fn main() {
    let mut app = App::new();

    // Add plugins and startup system
    app.add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, setup);
    // Run the app
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_mesh = PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        ..default()
    };

    // Spawn a static cube and a dynamic cube that is outside of the rest length.
    let static_cube = commands.spawn((cube_mesh.clone(),
                                      RigidBody::Static,
                                      Collider::cuboid(1., 1., 1.))).id();
    let dynamic_cube = commands
        .spawn((
            cube_mesh,
            RigidBody::Dynamic,
            Position(Vector::new(0.0, -2.5, 0.0)),
            Collider::cuboid(1., 1., 1.),
            MassPropertiesBundle::new_computed(&Collider::cuboid(1.0, 1.0, 1.0), 1.0),
        ))
        .id();

    // Add a distance joint to keep the cubes at a certain distance from each other.
    // The dynamic cube should bounce like it's on a spring.
    commands.spawn(
        DistanceJoint::new(static_cube, dynamic_cube)
            .with_local_anchor_1(0.5 * Vector::NEG_Y)
            .with_local_anchor_2(0.5 * Vector::new(1.0, 1.0, 1.0))
            .with_rest_length(1.5)
            .with_limits(0.75, 2.5)
            // .with_linear_velocity_damping(0.1)
            // .with_angular_velocity_damping(1.0)
            .with_compliance(1.0 / 100.0)
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
