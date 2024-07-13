//! Handles transform propagation, scale updates, and [`ColliderParent`] updates for colliders.
//!
//! See [`ColliderHierarchyPlugin`].

use crate::{
    prelude::*,
    prepare::{match_any, PrepareSet},
    sync::ancestor_marker::{AncestorMarker, AncestorMarkerPlugin},
};
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};
use narrow_phase::NarrowPhaseSet;

/// A plugin for managing the collider hierarchy and related updates.
///
/// - Updates [`ColliderParent`].
/// - Propagates [`ColliderTransform`].
///
/// This plugin requires Bevy's `HierarchyPlugin` and that colliders have the `ColliderMarker` component,
/// which is added automatically for colliders if the [`ColliderBackendPlugin`] is enabled.
pub struct ColliderHierarchyPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl ColliderHierarchyPlugin {
    /// Creates a [`ColliderHierarchyPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for ColliderHierarchyPlugin {
    fn default() -> Self {
        Self {
            schedule: PostUpdate.intern(),
        }
    }
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct MarkColliderAncestors;

impl Plugin for ColliderHierarchyPlugin {
    fn build(&self, app: &mut App) {
        // Mark ancestors of colliders with `AncestorMarker<ColliderMarker>`.
        // This is used to speed up `ColliderTransform` propagation by skipping
        // trees that have no colliders.
        app.add_plugins(
            AncestorMarkerPlugin::<ColliderMarker>::new(self.schedule)
                .add_markers_in_set(MarkColliderAncestors),
        );

        app.configure_sets(
            self.schedule,
            MarkColliderAncestors
                .after(PrepareSet::InitColliders)
                .before(PrepareSet::PropagateTransforms),
        );

        // Update collider parents.
        app.add_systems(
            self.schedule,
            update_collider_parents
                .after(PrepareSet::InitColliders)
                .before(PrepareSet::Finalize),
        );

        // Run transform propagation if new colliders without rigid bodies have been added.
        // The `PreparePlugin` should handle transform propagation for new rigid bodies.
        app.add_systems(
            self.schedule,
            (
                crate::sync::sync_simple_transforms_physics,
                crate::sync::propagate_transforms_physics,
            )
                .chain()
                .run_if(match_any::<(Added<ColliderMarker>, Without<RigidBody>)>)
                .in_set(PrepareSet::PropagateTransforms)
                .ambiguous_with_all(),
        );

        // Propagate `ColliderTransform`s if there are new colliders.
        // Only traverses trees with `AncestorMarker<ColliderMarker>`.
        app.add_systems(
            self.schedule,
            (
                propagate_collider_transforms,
                update_child_collider_position,
            )
                .chain()
                .run_if(match_any::<Added<ColliderMarker>>)
                .after(PrepareSet::InitTransforms)
                .before(PrepareSet::Finalize),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule
            .add_systems(handle_rigid_body_removals.after(PhysicsStepSet::SpatialQuery));

        // Propagate `ColliderTransform`s before narrow phase collision detection.
        // Only traverses trees with `AncestorMarker<ColliderMarker>`.
        physics_schedule.add_systems(
            (
                propagate_collider_transforms,
                update_child_collider_position,
            )
                .chain()
                .in_set(NarrowPhaseSet::First),
        );
    }

    fn finish(&self, app: &mut App) {
        if !app.is_plugin_added::<HierarchyPlugin>() {
            warn!("`ColliderHierarchyPlugin` requires Bevy's `HierarchyPlugin` to function. If you don't need collider hierarchies, consider disabling this plugin.",);
        }
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub(crate) struct PreviousColliderTransform(pub ColliderTransform);

/// Updates [`ColliderParent`] for descendant colliders of [`RigidBody`] entities.
///
/// The [`ColliderBackendPlugin`] handles collider parents for colliders that are
/// on the same entity as the rigid body.
#[allow(clippy::type_complexity)]
fn update_collider_parents(
    mut commands: Commands,
    mut bodies: Query<Entity, (With<RigidBody>, With<AncestorMarker<ColliderMarker>>)>,
    children: Query<&Children>,
    mut child_colliders: Query<
        Option<&mut ColliderParent>,
        (With<ColliderMarker>, Without<RigidBody>),
    >,
) {
    for entity in &mut bodies {
        for child in children.iter_descendants(entity) {
            if let Ok(collider_parent) = child_colliders.get_mut(child) {
                if let Some(mut collider_parent) = collider_parent {
                    collider_parent.0 = entity;
                } else {
                    commands.entity(child).insert((
                        ColliderParent(entity),
                        // TODO: This probably causes a one frame delay. Compute real value?
                        ColliderTransform::default(),
                        PreviousColliderTransform::default(),
                    ));
                }
            }
        }
    }
}

/// Updates colliders when the rigid bodies they were attached to have been removed.
fn handle_rigid_body_removals(
    mut commands: Commands,
    colliders: Query<(Entity, &ColliderParent), Without<RigidBody>>,
    bodies: Query<(), With<RigidBody>>,
    removals: RemovedComponents<RigidBody>,
) {
    // Return if no rigid bodies have been removed
    if removals.is_empty() {
        return;
    }

    for (collider_entity, collider_parent) in &colliders {
        // If the body associated with the collider parent entity doesn't exist,
        // remove ColliderParent and ColliderTransform.
        if !bodies.contains(collider_parent.get()) {
            commands.entity(collider_entity).remove::<(
                ColliderParent,
                ColliderTransform,
                PreviousColliderTransform,
            )>();
        }
    }
}

#[allow(clippy::type_complexity)]
pub(crate) fn update_child_collider_position(
    mut colliders: Query<
        (
            &ColliderTransform,
            &mut Position,
            &mut Rotation,
            &ColliderParent,
        ),
        Without<RigidBody>,
    >,
    parents: Query<(&Position, &Rotation), (With<RigidBody>, With<Children>)>,
) {
    for (collider_transform, mut position, mut rotation, parent) in &mut colliders {
        let Ok((parent_pos, parent_rot)) = parents.get(parent.get()) else {
            continue;
        };

        position.0 = parent_pos.0 + parent_rot * collider_transform.translation;
        #[cfg(feature = "2d")]
        {
            *rotation = *parent_rot * collider_transform.rotation;
        }
        #[cfg(feature = "3d")]
        {
            *rotation = (parent_rot.0 * collider_transform.rotation.0)
                .normalize()
                .into();
        }
    }
}

// `ColliderTransform` propagation should only be continued if the child
// is a collider or is a `AncestorMarker<ColliderMarker>`.
type ShouldPropagate = Or<(With<AncestorMarker<ColliderMarker>>, With<ColliderMarker>)>;

/// Updates [`ColliderTransform`]s based on entity hierarchies. Each transform is computed by recursively
/// traversing the children of each rigid body and adding their transforms together to form
/// the total transform relative to the body.
///
/// This is largely a clone of `propagate_transforms` in `bevy_transform`.
#[allow(clippy::type_complexity)]
pub(crate) fn propagate_collider_transforms(
    mut root_query: Query<
        (Entity, Ref<Transform>, &Children),
        (Without<Parent>, With<AncestorMarker<ColliderMarker>>),
    >,
    collider_query: Query<
        (
            Ref<Transform>,
            Option<&mut ColliderTransform>,
            Option<&Children>,
        ),
        (With<Parent>, ShouldPropagate),
    >,
    parent_query: Query<(Entity, Ref<Transform>, Has<RigidBody>, Ref<Parent>), ShouldPropagate>,
) {
    root_query.par_iter_mut().for_each(
        |(entity, transform, children)| {
            for (child, child_transform, is_child_rb, parent) in parent_query.iter_many(children) {
                assert_eq!(
                    parent.get(), entity,
                    "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
                );
                let changed = transform.is_changed() || parent.is_changed();
                let parent_transform = ColliderTransform::from(*transform);
                let child_transform = ColliderTransform::from(*child_transform);
                let scale = (parent_transform.scale * child_transform.scale).max(Vector::splat(Scalar::EPSILON));

                // SAFETY:
                // - `child` must have consistent parentage, or the above assertion would panic.
                // Since `child` is parented to a root entity, the entire hierarchy leading to it is consistent.
                // - We may operate as if all descendants are consistent, since `propagate_collider_transform_recursive` will panic before 
                //   continuing to propagate if it encounters an entity with inconsistent parentage.
                // - Since each root entity is unique and the hierarchy is consistent and forest-like,
                //   other root entities' `propagate_collider_transform_recursive` calls will not conflict with this one.
                // - Since this is the only place where `collider_query` gets used, there will be no conflicting fetches elsewhere.
                unsafe {
                    propagate_collider_transforms_recursive(
                        if is_child_rb {
                            ColliderTransform {
                                scale,
                                ..default()
                            }
                        } else {
                            ColliderTransform {
                                translation: parent_transform.scale * child_transform.translation,
                                rotation: child_transform.rotation,
                                scale,
                            }
                        },
                        &collider_query,
                        &parent_query,
                        child,
                        changed,
                    );
                }
            }
        },
    );
}

/// Recursively computes the [`ColliderTransform`] for `entity` and all of its descendants
/// by propagating transforms.
///
/// This is largely a clone of `propagate_recursive` in `bevy_transform`.
///
/// # Panics
///
/// If `entity`'s descendants have a malformed hierarchy, this function will panic occur before propagating
/// the transforms of any malformed entities and their descendants.
///
/// # Safety
///
/// - While this function is running, `collider_query` must not have any fetches for `entity`,
/// nor any of its descendants.
/// - The caller must ensure that the hierarchy leading to `entity`
/// is well-formed and must remain as a tree or a forest. Each entity must have at most one parent.
#[allow(clippy::type_complexity)]
unsafe fn propagate_collider_transforms_recursive(
    transform: ColliderTransform,
    collider_query: &Query<
        (
            Ref<Transform>,
            Option<&mut ColliderTransform>,
            Option<&Children>,
        ),
        (With<Parent>, ShouldPropagate),
    >,
    parent_query: &Query<(Entity, Ref<Transform>, Has<RigidBody>, Ref<Parent>), ShouldPropagate>,
    entity: Entity,
    mut changed: bool,
) {
    let children = {
        // SAFETY: This call cannot create aliased mutable references.
        //   - The top level iteration parallelizes on the roots of the hierarchy.
        //   - The caller ensures that each child has one and only one unique parent throughout the entire
        //     hierarchy.
        //
        // For example, consider the following malformed hierarchy:
        //
        //     A
        //   /   \
        //  B     C
        //   \   /
        //     D
        //
        // D has two parents, B and C. If the propagation passes through C, but the Parent component on D points to B,
        // the above check will panic as the origin parent does match the recorded parent.
        //
        // Also consider the following case, where A and B are roots:
        //
        //  A       B
        //   \     /
        //    C   D
        //     \ /
        //      E
        //
        // Even if these A and B start two separate tasks running in parallel, one of them will panic before attempting
        // to mutably access E.
        let Ok((transform_ref, collider_transform, children)) =
            (unsafe { collider_query.get_unchecked(entity) })
        else {
            return;
        };

        changed |= transform_ref.is_changed();
        if changed {
            if let Some(mut collider_transform) = collider_transform {
                if *collider_transform != transform {
                    *collider_transform = transform;
                }
            }
        }

        children
    };

    let Some(children) = children else { return };
    for (child, child_transform, is_rb, parent) in parent_query.iter_many(children) {
        assert_eq!(
            parent.get(), entity,
            "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
        );

        let child_transform = ColliderTransform::from(*child_transform);

        // SAFETY: The caller guarantees that `collider_query` will not be fetched
        // for any descendants of `entity`, so it is safe to call `propagate_collider_transforms_recursive` for each child.
        //
        // The above assertion ensures that each child has one and only one unique parent throughout the
        // entire hierarchy.
        unsafe {
            propagate_collider_transforms_recursive(
                if is_rb {
                    ColliderTransform {
                        scale: child_transform.scale,
                        ..default()
                    }
                } else {
                    ColliderTransform {
                        translation: transform.transform_point(child_transform.translation),
                        #[cfg(feature = "2d")]
                        rotation: transform.rotation * child_transform.rotation,
                        #[cfg(feature = "3d")]
                        rotation: Rotation(transform.rotation.0 * child_transform.rotation.0),
                        scale: (transform.scale * child_transform.scale)
                            .max(Vector::splat(Scalar::EPSILON)),
                    }
                },
                collider_query,
                parent_query,
                child,
                changed || parent.is_changed(),
            );
        }
    }
}

// TODO: Add thorough tests for propagation. It's pretty error-prone and changes are risky.
