//! An example showcasing how to create colliders for meshes and scenes
//! using `AsyncCollider` and `AsyncSceneCollider` respectively.

use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;
use examples_common_3d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    assets: ResMut<AssetServer>,
) {
    // Spawn ground and generate a collider for the mesh using AsyncCollider
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh().size(8.0, 8.0)),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3)),
            ..default()
        },
        AsyncCollider(ComputedCollider::TriMesh),
        RigidBody::Static,
    ));

    // Spawn Ferris the crab and generate colliders for the scene using AsyncSceneCollider
    commands.spawn((
        SceneBundle {
            // The model was made by RayMarch, licenced under CC0-1.0, and can be found here:
            // https://github.com/RayMarch/ferris3d
            scene: assets.load("ferris.glb#Scene0"),
            transform: Transform::from_xyz(0.0, 1.0, 0.0).with_scale(Vec3::splat(2.0)),
            ..default()
        },
        // Create colliders using convex decomposition.
        // This takes longer than creating a trimesh or convex hull collider,
        // but is more performant for collision detection.
        AsyncSceneCollider::new(Some(ComputedCollider::ConvexDecomposition(
            VHACDParameters::default(),
        )))
        // Make the arms heavier to make it easier to stand upright
        .with_density_for_name("armL_mesh", 3.0)
        .with_density_for_name("armR_mesh", 3.0),
        RigidBody::Dynamic,
    ));

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1_000_000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(2.0, 8.0, 2.0),
        ..default()
    });

    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-5.0, 3.5, 5.5).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}
