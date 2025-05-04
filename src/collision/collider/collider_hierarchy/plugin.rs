use crate::{prelude::*, sync::ancestor_marker::AncestorMarker};
use bevy::prelude::*;

/// A plugin for managing [`ColliderOf`] relationships based on the entity hierarchy.
///
/// By default, colliders are automatically attached to the nearest rigid body ancestor.
///
/// This plugin requires that colliders have the [`ColliderMarker`] component,
/// which is added automatically for colliders if the [`ColliderBackendPlugin`] is enabled.
pub struct ColliderHierarchyPlugin;

impl Plugin for ColliderHierarchyPlugin {
    fn build(&self, app: &mut App) {
        // Imsert `ColliderOf` for colliders that are added to a rigid body.
        app.add_observer(
            |trigger: Trigger<OnAdd, (RigidBody, ColliderMarker)>,
             mut commands: Commands,
             query: Query<(), With<ColliderMarker>>,
             rb_query: Query<Entity, With<RigidBody>>,
             parent_query: Query<&ChildOf>| {
                let entity = trigger.target();

                // Make sure the entity is a collider.
                if !query.contains(entity) {
                    return;
                }

                // Find the closest rigid body ancestor.
                if let Some(rigid_body) = rb_query.get(entity).ok().map_or_else(
                    || {
                        parent_query
                            .iter_ancestors(trigger.target())
                            .find(|&entity| rb_query.contains(entity))
                    },
                    Some,
                ) {
                    commands.entity(entity).insert(ColliderOf { rigid_body });
                }
            },
        );

        // Remove `ColliderOf` when the rigid body or collider is removed.
        app.add_observer(
            |trigger: Trigger<OnRemove, (RigidBody, ColliderMarker)>,
             mut commands: Commands,
             query: Query<(), With<ColliderMarker>>| {
                let entity = trigger.target();

                // Make sure the collider is on the same entity as the rigid body.
                if query.contains(entity) {
                    commands.entity(entity).try_remove::<ColliderOf>();
                }
            },
        );

        // Update `ColliderOf` components for colliders when their ancestors change.
        app.add_observer(on_collider_rigid_body_changed);

        // Remove `ColliderOf` from colliders when their rigid bodies are removed.
        app.add_observer(on_rigid_body_removed);
    }
}

/// Updates [`ColliderOf`] components for colliders when their ancestors change
/// or when a rigid body is added to the hierarchy.
fn on_collider_rigid_body_changed(
    trigger: Trigger<OnInsert, (ChildOf, RigidBody, AncestorMarker<ColliderMarker>)>,
    mut commands: Commands,
    query: Query<
        Has<ColliderMarker>,
        Or<(With<ColliderMarker>, With<AncestorMarker<ColliderMarker>>)>,
    >,
    child_query: Query<&Children>,
    parent_query: Query<&ChildOf>,
    rb_query: Query<Entity, With<RigidBody>>,
    collider_query: Query<(), With<ColliderMarker>>,
) {
    let entity = trigger.target();

    // Skip if the entity is not a collider or an ancestor of a collider.
    let Ok(is_collider) = query.get(entity) else {
        return;
    };

    // Find the closest rigid body ancestor.
    let Some(rb_entity) = rb_query.get(entity).ok().map_or_else(
        || {
            parent_query
                .iter_ancestors(trigger.target())
                .find(|&entity| rb_query.contains(entity))
        },
        Some,
    ) else {
        return;
    };

    // Insert `ColliderOf` for the new child entity.
    if is_collider {
        commands.entity(entity).insert(ColliderOf {
            rigid_body: rb_entity,
        });
    }

    // Iterate over all descendants starting from the new child entity,
    // and update the `ColliderOf` component to point to the closest rigid body ancestor.
    for child in child_query.iter_descendants(entity) {
        // Stop traversal if we reach another rigid body.
        // FIXME: This might lead to siblings of the rigid body being skipped.
        if rb_query.contains(child) {
            break;
        }

        // Update the `ColliderOf` component.
        if collider_query.contains(child) {
            commands.entity(child).insert(ColliderOf {
                rigid_body: rb_entity,
            });
        }
    }
}

/// Removes [`ColliderOf`] from colliders when their rigid bodies are removed.
fn on_rigid_body_removed(
    trigger: Trigger<OnRemove, RigidBody>,
    mut commands: Commands,
    rb_collider_query: Query<&RigidBodyColliders>,
) {
    // TODO: Here we assume that rigid bodies are not nested, so `ColliderOf` is simply removed
    //       instead of being updated to point to a new rigid body in the hierarchy.

    let rb_entity = trigger.target();

    // Remove `ColliderOf` from all colliders attached to the rigid body.
    if let Ok(colliders) = rb_collider_query.get(rb_entity) {
        for collider_entity in colliders.iter() {
            commands
                .entity(collider_entity)
                .try_remove::<(ColliderOf, ColliderTransform)>();
        }
    }
}
