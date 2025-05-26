use bevy::{platform::collections::HashMap, prelude::*};

use super::{Collider, ColliderConstructor};

/// A plugin for caching colliders created from meshes.
pub struct ColliderCachePlugin;

impl Plugin for ColliderCachePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ColliderCache>();
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
