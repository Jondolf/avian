use bevy::{platform::collections::HashMap, prelude::*};

use super::{Collider, ColliderConstructor};

/// A plugin for caching colliders created from meshes via [`ColliderConstructor`] or [`ColliderConstructorHierarchy`](super::ColliderConstructorHierarchy).
/// With this plugin enabled, colliders created from meshes through such constructors will be created only once and reused.
/// This is especially useful when performing convex decomposition, as this is a very expensive operation.
pub struct ColliderCachePlugin;

impl Plugin for ColliderCachePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ColliderCache>();
        app.add_systems(PostUpdate, garbage_collect_collider_cache);
    }
}

#[derive(Debug, Resource, Default)]
pub(crate) struct ColliderCache(HashMap<Handle<Mesh>, Collider>);

impl ColliderCache {
    pub(crate) fn get_or_insert(
        &mut self,
        mesh_handle: &Handle<Mesh>,
        mesh: &Mesh,
        constructor: ColliderConstructor,
    ) -> Option<Collider> {
        if !self.0.contains_key(mesh_handle) {
            let collider = Collider::try_from_constructor(constructor, Some(mesh))?;
            self.0.insert(mesh_handle.clone_weak(), collider);
        }
        self.0.get(mesh_handle).cloned()
    }
}

fn garbage_collect_collider_cache(
    mut collider_cache: ResMut<ColliderCache>,
    asset_server: Res<AssetServer>,
) {
    collider_cache
        .0
        // Not using `is_loaded_with_dependencies` because when dropping a handle,
        // the entire hierarchy of handles is dropped.
        .retain(|mesh_handle, _| asset_server.is_loaded(mesh_handle));
}
