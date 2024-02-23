//! Hooks into the loading of scenes from gLTFs and loads the associated physics objects onto
//! Those entities from the OMI physics extensions.
use crate::plugins::omi_physics::parsing_structs::{OmiPhysicsBody, OmiPhysicsShape};
use crate::prelude::App;
use bevy::gltf::{GltfExtensions, GltfMesh, WeakGltf};
use bevy::prelude::{
    Assets, Commands, Component, Entity, Mesh, Plugin, Query, Res, SceneSpawner, Update, Without,
};
use bevy::scene::SceneInstance;

mod parsing_structs;

/// Hooks into the loading of scenes from gLTFs and loads the associated physics objects onto those
/// entities from the OMI physics extension.
pub struct OmiPhysicsGltfCompatability;

impl Plugin for OmiPhysicsGltfCompatability {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, hooked_scene_load);
    }
}

#[derive(Component)]
struct Hooked;

fn hooked_scene_load(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    gltf_meshes: Res<Assets<GltfMesh>>,
    unloaded_instance: Query<(Entity, &SceneInstance), Without<Hooked>>,
    scene_manager: Res<SceneSpawner>,
    extensions: Query<&GltfExtensions>,
    gltfs_query: Query<&WeakGltf>,
) {
    for (entity, instance) in unloaded_instance.iter() {
        if scene_manager.instance_is_ready(**instance) {
            commands.entity(entity).insert(Hooked);
        }
        let entities = scene_manager
            .iter_instance_entities(**instance)
            .chain(std::iter::once(entity));
        let mut gltf = None;
        for entity in entities {
            if let Ok(gltf_handle) = gltfs_query.get(entity) {
                gltf.replace(gltf_handle.clone());
            }
        }
        let Some(gltf) = gltf else { continue };
        let entities = scene_manager
            .iter_instance_entities(**instance)
            .chain(std::iter::once(entity));
        let mut colliders = vec![];
        for entity in entities {
            let Ok(extension) = extensions.get(entity) else {
                continue;
            };
            let Ok(mut map): Result<serde_json::Map<String, serde_json::Value>, _> =
                serde_json::from_str(&extension.value)
            else {
                continue;
            };
            let Some(omi_physics_shape) = map.remove("OMI_physics_shape") else {
                continue;
            };
            let Ok(omi_physics_shape): Result<OmiPhysicsShape, _> =
                serde_json::from_value(omi_physics_shape)
            else {
                continue;
            };
            colliders = omi_physics_shape.into_colliders(&gltf, &meshes, &gltf_meshes);
        }

        let entities = scene_manager
            .iter_instance_entities(**instance)
            .chain(std::iter::once(entity));
        for entity in entities {
            let Ok(extension) = extensions.get(entity) else {
                continue;
            };
            let Ok(mut map): Result<serde_json::Map<String, serde_json::Value>, _> =
                serde_json::from_str(&extension.value)
            else {
                continue;
            };
            let Some(omi_physics_body) = map.remove("OMI_physics_body") else {
                continue;
            };
            let Ok(omi_physics_body): Result<OmiPhysicsBody, _> =
                serde_json::from_value(omi_physics_body)
            else {
                continue;
            };
            omi_physics_body.try_spawn(entity, &mut commands, &colliders);
        }
    }
}
