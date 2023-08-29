//! A very basic implementation of a character controller for a kinematic rigid body.
//! Supports directional movement and jumping.
//!
//! Bevy XPBD does not have a built-in character controller yet, so you will have to implement
//! the logic yourself. For kinematic bodies, collision response has to be handled manually, as shown in
//! this example.
//!
//! Using dynamic bodies is often easier, as they handle most of the physics for you.
//! For a dynamic character controller, see the `basic_dynamic_character` example.

use bevy::prelude::*;
use bevy::scene::SceneInstance;
use bevy_xpbd_3d::prelude::*;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                init_async_scene_colliders,
                debug_output,
                bevy::window::close_on_esc,
            ),
        )
        .run();
}

fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
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
        transform: Transform::from_xyz(-4.0, 6.5, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Test Scene
    commands.spawn((
        SceneBundle {
            scene: asset_server.load("models/test_angles.gltf#Scene0"),
            ..default()
        },
        Name::new("Level"),
        AsyncSceneCollider,
    ));
}

#[derive(Component, Debug, Default, Clone)]
pub struct AsyncSceneCollider;

pub fn init_async_scene_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    scene_spawner: Res<SceneSpawner>,
    async_colliders: Query<(Entity, &SceneInstance), With<AsyncSceneCollider>>,
    children: Query<&Children>,
    mesh_handles: Query<(&Name, &Handle<Mesh>)>,
) {
    for (scene_entity, scene_instance) in async_colliders.iter() {
        if scene_spawner.instance_is_ready(**scene_instance) {
            for child_entity in children.iter_descendants(scene_entity) {
                if let Ok((name, handle)) = mesh_handles.get(child_entity) {
                    let mesh = meshes.get(handle).expect("mesh should already be loaded");
                    match Collider::trimesh_from_bevy_mesh(mesh) {
                        Some(collider) => {
                            println!("{}", name);
                            commands
                                .entity(child_entity)
                                .insert((collider, RigidBody::Static));
                        }
                        None => error!(
                            "unable to generate collider from mesh {:?} with name {}",
                            mesh, name
                        ),
                    }
                }
            }

            commands.entity(scene_entity).remove::<AsyncSceneCollider>();
        }
    }
}

fn debug_output(query: Query<(&Name, &Transform, &Handle<Mesh>), Changed<Transform>>) {
    for (name, transform, _) in query.iter() {
        if name.as_str() == "Cube.001.0" {
            println!("{} transform: {:?}", name, transform);
        }
    }
}
