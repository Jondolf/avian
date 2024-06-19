//! Handles transform propagation, scale updates, and [`ColliderParent`] updates for colliders.
//!
//! See [`ColliderHierarchyPlugin`].

use std::marker::PhantomData;

use crate::{
    prelude::*,
    prepare::{match_any, PrepareSet},
    sync::SyncSet,
};
use bevy::{ecs::intern::Interned, prelude::*};

/// A plugin for managing the collider hierarchy and related updates.
///
/// - Updates [`ColliderParent`].
/// - Propagates [`ColliderTransform`].
/// - Updates collider scale based on `Transform` scale.
///
/// By default, [`PhysicsPlugins`] adds this plugin for the [`Collider`] component.
/// You can also use a custom collider backend by adding this plugin for any type
/// that implements the [`ScalableCollider`] trait.
///
/// This plugin should typically be used together with the [`ColliderBackendPlugin`].
pub struct ColliderHierarchyPlugin<C: ScalableCollider> {
    schedule: Interned<dyn ScheduleLabel>,
    _phantom: PhantomData<C>,
}

impl<C: ScalableCollider> ColliderHierarchyPlugin<C> {
    /// Creates a [`ColliderHierarchyPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
            _phantom: PhantomData,
        }
    }
}

impl<C: ScalableCollider> Default for ColliderHierarchyPlugin<C> {
    fn default() -> Self {
        Self {
            schedule: PostUpdate.intern(),
            _phantom: PhantomData,
        }
    }
}

impl<C: ScalableCollider> Plugin for ColliderHierarchyPlugin<C> {
    fn build(&self, app: &mut App) {
        // Mark ancestors of added colliders with the `ColliderAncestor` component.
        // This is used to speed up `ColliderTransform` propagation.
        app.add_systems(
            self.schedule,
            mark_collider_ancestors::<C>
                .after(super::backend::init_colliders::<C>)
                .in_set(PrepareSet::InitColliders),
        );

        // Remove `ColliderAncestor` markers from removed colliders and their ancestors,
        // until an ancestor that has other `ColliderAncestor` entities as children is encountered.
        #[allow(clippy::type_complexity)]
        app.observe(
            |trigger: Trigger<OnRemove, C>,
             mut commands: Commands,
             child_query: Query<&Children>,
             parent_query: Query<&Parent>,
             ancestor_query: Query<
                (Entity, Has<C>),
                Or<(With<ColliderAncestor>, With<C>)>,
            >| {
                let entity = trigger.entity();

                // Iterate over ancestors, removing `ColliderAncestor` markers until
                // an entity that has other `ColliderAncestor` children is encountered.
                let mut previous_parent = entity;
                for parent_entity in parent_query.iter_ancestors(entity) {
                    if let Ok(children) = child_query.get(parent_entity) {
                        // Keep the marker if `parent_entity` has a child that is a collider ancestor
                        // or a collider, but not the one that was removed.
                        let keep_marker =
                            ancestor_query
                                .iter_many(children)
                                .any(|(child, is_collider)| {
                                    child != previous_parent || (is_collider && child != entity)
                                });

                        if keep_marker {
                            return;
                        } else {
                            commands.entity(parent_entity).remove::<ColliderAncestor>();
                        }
                    }

                    previous_parent = parent_entity;
                }
            },
        );

        // Run transform propagation if new colliders without rigid bodies have been added.
        // The `PreparePlugin` should handle transform propagation for new rigid bodies.
        app.add_systems(
            self.schedule,
            (
                bevy::transform::systems::sync_simple_transforms,
                bevy::transform::systems::propagate_transforms,
            )
                .chain()
                .run_if(match_any::<(Added<C>, Without<RigidBody>)>)
                .in_set(PrepareSet::PropagateTransforms)
                // Allowing ambiguities is required so that it's possible
                // to have multiple collision backends at the same time.
                .ambiguous_with_all(),
        );

        // Update collider parents.
        app.add_systems(
            self.schedule,
            update_collider_parents::<C>
                // TODO: Decouple the ordering here.
                .after(super::backend::init_colliders::<C>)
                .in_set(PrepareSet::InitColliders),
        );

        // Propagate `ColliderTransform`s if there are new colliders.
        // Only traverses trees with `ColliderAncestor`.
        app.add_systems(
            self.schedule,
            (
                propagate_collider_transforms,
                update_child_collider_position,
            )
                .chain()
                .run_if(match_any::<Added<C>>)
                // TODO: Decouple the ordering here.
                .before(super::backend::update_collider_mass_properties::<C>)
                .in_set(PrepareSet::Finalize),
        );

        // Update colliders based on the scale from `ColliderTransform`.
        app.add_systems(
            self.schedule,
            update_collider_scale::<C>
                .after(SyncSet::Update)
                .before(SyncSet::Last),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule
            .add_systems(handle_rigid_body_removals.after(PhysicsStepSet::SpatialQuery));

        let substep_schedule = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");

        // Propagate `ColliderTransform`s before narrow phase collision detection.
        // Only traverses trees with `ColliderAncestor`.
        substep_schedule.add_systems(
            (
                propagate_collider_transforms,
                update_child_collider_position,
            )
                .chain()
                .after(SubstepSet::Integrate)
                .before(SubstepSet::NarrowPhase)
                .ambiguous_with_all(),
        );
    }
}

/// A marker component that marks an entity as an ancestor of an entity with a collider.
///
/// This is used to avoid unnecessary work when propagating transforms for colliders.
#[derive(Reflect, Clone, Copy, Component)]
pub struct ColliderAncestor;

// TODO: This could also be an observer, but it'd need to have the appropriate filters
//       and trigger for `Parent` changes, which doesn't seem to be possible yet.
//       Unless we perhaps added an `OnColliderParentChanged` trigger.
/// Marks ancestors of added colliders with the `ColliderAncestor` component.
/// This is used to speed up `ColliderTransform` propagation.
#[allow(clippy::type_complexity)]
fn mark_collider_ancestors<C: AnyCollider>(
    mut commands: Commands,
    collider_query: Query<Entity, (With<C>, Changed<Parent>)>,
    parent_query: Query<&Parent>,
    ancestor_query: Query<(), With<ColliderAncestor>>,
) {
    for entity in &collider_query {
        // Traverse up the tree, marking entities with `ColliderAncestor`
        // until an entity that already has it is encountered.
        for parent_entity in parent_query.iter_ancestors(entity) {
            if ancestor_query.contains(parent_entity) {
                return;
            } else {
                commands.entity(parent_entity).insert(ColliderAncestor);
            }
        }
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub(crate) struct PreviousColliderTransform(pub ColliderTransform);

#[allow(clippy::type_complexity)]
fn update_collider_parents<C: AnyCollider>(
    mut commands: Commands,
    mut bodies: Query<(Entity, Option<&mut ColliderParent>, Has<C>), With<RigidBody>>,
    children: Query<&Children>,
    mut child_colliders: Query<Option<&mut ColliderParent>, (With<C>, Without<RigidBody>)>,
) {
    for (entity, collider_parent, has_collider) in &mut bodies {
        if has_collider {
            if let Some(mut collider_parent) = collider_parent {
                collider_parent.0 = entity;
            } else {
                commands.entity(entity).try_insert((
                    ColliderParent(entity),
                    // Todo: This probably causes a one frame delay. Compute real value?
                    ColliderTransform::default(),
                    PreviousColliderTransform::default(),
                ));
            }
        }
        for child in children.iter_descendants(entity) {
            if let Ok(collider_parent) = child_colliders.get_mut(child) {
                if let Some(mut collider_parent) = collider_parent {
                    collider_parent.0 = entity;
                } else {
                    commands.entity(child).insert((
                        ColliderParent(entity),
                        // Todo: This probably causes a one frame delay. Compute real value?
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

        position.0 = parent_pos.0 + parent_rot.rotate(collider_transform.translation);
        #[cfg(feature = "2d")]
        {
            *rotation = *parent_rot + collider_transform.rotation;
        }
        #[cfg(feature = "3d")]
        {
            *rotation = (parent_rot.0 * collider_transform.rotation.0)
                .normalize()
                .into();
        }
    }
}

/// Updates the scale of colliders based on [`Transform`] scale.
#[allow(clippy::type_complexity)]
pub fn update_collider_scale<C: ScalableCollider>(
    mut colliders: ParamSet<(
        // Root bodies
        Query<(&Transform, &mut C), Without<Parent>>,
        // Child colliders
        Query<(&ColliderTransform, &mut C), With<Parent>>,
    )>,
) {
    // Update collider scale for root bodies
    for (transform, mut collider) in &mut colliders.p0() {
        #[cfg(feature = "2d")]
        let scale = transform.scale.truncate().adjust_precision();
        #[cfg(feature = "3d")]
        let scale = transform.scale.adjust_precision();
        if scale != collider.scale() {
            // TODO: Support configurable subdivision count for shapes that
            //       can't be represented without approximations after scaling.
            collider.set_scale(scale, 10);
        }
    }

    // Update collider scale for child colliders
    for (collider_transform, mut collider) in &mut colliders.p1() {
        if collider_transform.scale != collider.scale() {
            // TODO: Support configurable subdivision count for shapes that
            //       can't be represented without approximations after scaling.
            collider.set_scale(collider_transform.scale, 10);
        }
    }
}

// `ColliderTransform` propagation should only be continued if the child
// is a collider (has a `ColliderTransform`) or is a `ColliderAncestor`.
type ShouldPropagate = Or<(With<ColliderAncestor>, With<ColliderTransform>)>;

/// Updates [`ColliderTransform`]s based on entity hierarchies. Each transform is computed by recursively
/// traversing the children of each rigid body and adding their transforms together to form
/// the total transform relative to the body.
///
/// This is largely a clone of `propagate_transforms` in `bevy_transform`.
#[allow(clippy::type_complexity)]
pub(crate) fn propagate_collider_transforms(
    mut root_query: Query<
        (Entity, Ref<Transform>, &Children),
        (Without<Parent>, With<ColliderAncestor>),
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
        |(entity, transform,children)| {
            for (child, child_transform, is_child_rb, parent) in parent_query.iter_many(children) {
                assert_eq!(
                    parent.get(), entity,
                    "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
                );
                let child_transform = ColliderTransform::from(*child_transform);

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
                                scale: child_transform.scale,
                                ..default()
                            }
                        } else {
                            let transform = ColliderTransform::from(*transform);

                            ColliderTransform {
                                translation: transform.scale * child_transform.translation,
                                rotation: child_transform.rotation,
                                scale: (transform.scale * child_transform.scale).max(Vector::splat(Scalar::EPSILON)),
                            }
                        },
                        &collider_query,
                        &parent_query,
                        child,
                        transform.is_changed() || parent.is_changed()
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
                        rotation: transform.rotation + child_transform.rotation,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collider_ancestor_markers() {
        let mut app = App::new();

        app.init_schedule(PhysicsSchedule);
        app.init_schedule(SubstepSchedule);

        app.add_plugins(ColliderHierarchyPlugin::<Collider>::new(PostUpdate));

        let collider = Collider::capsule(2.0, 0.5);

        // Set up an entity tree like the following:
        //
        //     AN
        //    /  \
        //  BN    CY
        //       /  \
        //     DN    EN
        //    /  \
        //  FY    GY
        //
        // where Y means that the entity has a collider,
        // and N means that the entity does not have a collider.

        let an = app.world_mut().spawn_empty().id();

        let bn = app.world_mut().spawn_empty().set_parent(an).id();
        let cy = app.world_mut().spawn(collider.clone()).set_parent(an).id();

        let dn = app.world_mut().spawn_empty().set_parent(cy).id();
        let en = app.world_mut().spawn_empty().set_parent(cy).id();

        let fy = app.world_mut().spawn(collider.clone()).set_parent(dn).id();
        let gy = app.world_mut().spawn(collider.clone()).set_parent(dn).id();

        app.world_mut().run_schedule(PostUpdate);

        // Check that the correct entities have the `ColliderAncestor` component.
        assert!(app.world().entity(an).contains::<ColliderAncestor>());
        assert!(!app.world().entity(bn).contains::<ColliderAncestor>());
        assert!(app.world().entity(cy).contains::<ColliderAncestor>());
        assert!(app.world().entity(dn).contains::<ColliderAncestor>());
        assert!(!app.world().entity(en).contains::<ColliderAncestor>());
        assert!(!app.world().entity(fy).contains::<ColliderAncestor>());
        assert!(!app.world().entity(gy).contains::<ColliderAncestor>());

        // Remove the collider from FY. DN, CY, and AN should all keep the `ColliderAncestor` marker.
        let mut entity_mut = app.world_mut().entity_mut(fy);
        entity_mut.remove::<Collider>();

        app.world_mut().run_schedule(PostUpdate);

        assert!(app.world().entity(dn).contains::<ColliderAncestor>());
        assert!(app.world().entity(cy).contains::<ColliderAncestor>());
        assert!(app.world().entity(an).contains::<ColliderAncestor>());

        // Remove the collider from GY. The `ColliderAncestor` marker should
        // now be removed from DN and CY, but it should remain on AN.
        let mut entity_mut = app.world_mut().entity_mut(gy);
        entity_mut.remove::<Collider>();

        app.world_mut().run_schedule(PostUpdate);

        assert!(!app.world().entity(dn).contains::<ColliderAncestor>());
        assert!(!app.world().entity(cy).contains::<ColliderAncestor>());
        assert!(app.world().entity(an).contains::<ColliderAncestor>());

        // Remove the collider from CY. The `ColliderAncestor` marker should
        // now be removed from AN.
        let mut entity_mut = app.world_mut().entity_mut(cy);
        entity_mut.remove::<Collider>();

        app.world_mut().run_schedule(PostUpdate);

        assert!(!app.world().entity(an).contains::<ColliderAncestor>());
    }
}
