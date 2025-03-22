//! [`ColliderOf`] relationships for attaching colliders to rigid bodies
//! based on the entity hierarchy.

mod plugin;

pub use plugin::ColliderHierarchyPlugin;

use crate::prelude::*;
use bevy::{
    ecs::{
        component::HookContext,
        relationship::{Relationship, RelationshipHookMode},
        world::DeferredWorld,
    },
    prelude::*,
};

/// A [`Relationship`] component that attaches a [`Collider`] to a [`RigidBody`].
///
/// Unless manually specified, the [`ColliderOf`] component is automatically initialized
/// with the nearest rigid body up the chain in the entity hierarchy.
///
/// For example, given the following entities:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// # fn setup(mut commands: Commands) {
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::capsule(0.5, 1.5),
///     children![
///         (Collider::capsule(0.5, 1.5), Transform::from_xyz(-2.0, 0.0, 0.0)),
///         (Collider::capsule(0.5, 1.5), Transform::from_xyz(2.0, 0.0, 0.0)),
///     ],
/// ));
/// # }
/// ```
///
/// all three colliders will be attached to the same rigid body.
///
/// However, it also possible to explicitly specify which rigid body a collider is attached to
/// by inserting the [`ColliderOf`] component manually.
///
/// [`Relationship`]: bevy::ecs::relationship::Relationship
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Reflect)]
#[component(immutable, on_insert = <ColliderOf as Relationship>::on_insert, on_replace = <ColliderOf as Relationship>::on_replace)]
#[require(ColliderTransform)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderOf {
    /// The [`Entity`] ID of the [`RigidBody`] that this collider is attached to.
    pub rigid_body: Entity,
}

impl FromWorld for ColliderOf {
    #[inline(always)]
    fn from_world(_world: &mut World) -> Self {
        ColliderOf {
            rigid_body: Entity::PLACEHOLDER,
        }
    }
}

// Bevy does not currently allow relationships that point to their own entity,
// so we implement the relationship manually to work around this limitation.
impl Relationship for ColliderOf {
    type RelationshipTarget = RigidBodyColliders;

    fn get(&self) -> Entity {
        self.rigid_body
    }

    fn from(entity: Entity) -> Self {
        ColliderOf { rigid_body: entity }
    }

    fn on_insert(
        mut world: DeferredWorld,
        HookContext {
            entity,
            caller,
            relationship_hook_mode,
            ..
        }: HookContext,
    ) {
        // This is largely the same as the default implementation,
        // but allows relationships to point to their own entity.

        match relationship_hook_mode {
            RelationshipHookMode::Run => {}
            RelationshipHookMode::Skip => return,
            RelationshipHookMode::RunIfNotLinked => {
                if RigidBodyColliders::LINKED_SPAWN {
                    return;
                }
            }
        }

        let collider = entity;
        let rigid_body = world
            .entity(collider)
            .get::<ColliderOf>()
            .unwrap()
            .rigid_body;

        if let Some(mut rigid_body_mut) = world
            .get_entity_mut(rigid_body)
            .ok()
            .filter(|e| e.contains::<RigidBody>())
        {
            // Attach the collider to the rigid body.
            if let Some(mut colliders) = rigid_body_mut.get_mut::<RigidBodyColliders>() {
                colliders.0.push(collider);
            } else {
                world
                    .commands()
                    .entity(rigid_body)
                    .insert(RigidBodyColliders(vec![collider]));
            }
        } else {
            warn!(
                "{}Tried to attach collider on entity {collider} to rigid body on entity {rigid_body}, but the rigid body does not exist.",
                caller.map(|location| format!("{location}: ")).unwrap_or_default(),
            );
        }
    }
}

/// A [`RelationshipTarget`] component that tracks which colliders are attached to a [`RigidBody`].
///
/// This is automatically inserted and pupulated with entities that are attached to a rigid body
/// using the [`ColliderOf`] [`Relationship`] component.
///
/// You should not modify this component directly to avoid desynchronization.
/// Instead, modify the [`ColliderOf`] components on the colliders.
///
/// [`Relationship`]: bevy::ecs::relationship::Relationship
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[relationship_target(relationship = ColliderOf, linked_spawn)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct RigidBodyColliders(Vec<Entity>);
