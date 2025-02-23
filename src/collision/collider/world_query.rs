// This is at the module-level because `QueryData` generates a `ColliderQueryItem` type,
// and we cannot easily document or allow missing docs for that.
#![allow(missing_docs)]

use crate::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};

/// A [`QueryData`] struct to make code handling colliders cleaner.
///
/// This is mostly an internal type, but can also be used by users.
#[derive(QueryData)]
pub struct ColliderQuery<C: AnyCollider> {
    pub entity: Entity,
    pub parent: Option<&'static ColliderParent>,
    pub position: Ref<'static, Position>,
    pub rotation: Ref<'static, Rotation>,
    pub accumulated_translation: Option<Ref<'static, AccumulatedTranslation>>,
    pub transform: Option<&'static ColliderTransform>,
    pub aabb: Ref<'static, ColliderAabb>,
    pub collision_margin: Option<&'static CollisionMargin>,
    pub speculative_margin: Option<&'static SpeculativeMargin>,
    pub is_rb: Has<RigidBody>,
    pub is_sensor: Has<Sensor>,
    pub collision_events_enabled: Has<CollisionEventsEnabled>,
    pub friction: Option<&'static Friction>,
    pub restitution: Option<&'static Restitution>,
    pub shape: &'static C,
    pub active_hooks: Option<&'static ActiveCollisionHooks>,
}

impl<C: AnyCollider> ColliderQueryItem<'_, C> {
    /// Returns the current position of the body. This is a sum of the [`Position`] and
    /// [`AccumulatedTranslation`] components.
    pub fn current_position(&self) -> Vector {
        self.position.0
            + self
                .accumulated_translation
                .as_ref()
                .map_or_else(default, |t| t.0)
    }

    /// Returns the [`ActiveCollisionHooks`] for the collider.
    pub fn active_hooks(&self) -> ActiveCollisionHooks {
        self.active_hooks
            .map_or(ActiveCollisionHooks::empty(), |h| *h)
    }
}
