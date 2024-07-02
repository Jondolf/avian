//! Responsible for synchronizing physics components with other data, like keeping [`Position`]
//! and [`Rotation`] in sync with `Transform`.
//!
//! See [`SyncPlugin`].

use crate::{prelude::*, prepare::PrepareSet, utils::get_pos_translation};
use ancestor_marker::{AncestorMarker, AncestorMarkerPlugin};
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

// TODO: Where should this be?
pub mod ancestor_marker;

/// Responsible for synchronizing physics components with other data, like keeping [`Position`]
/// and [`Rotation`] in sync with `Transform`.
///
/// ## Syncing between [`Position`]/[`Rotation`] and [`Transform`]
///
/// By default, each body's `Transform` will be updated when [`Position`] or [`Rotation`]
/// change, and vice versa. This means that you can use any of these components to move
/// or position bodies, and the changes be reflected in the other components.
///
/// You can configure what data is synchronized and how it is synchronized
/// using the [`SyncConfig`] resource.
///
/// ## `Transform` hierarchies
///
/// When synchronizing changes in [`Position`] or [`Rotation`] to `Transform`,
/// the engine treats nested [rigid bodies](RigidBody) as a flat structure. This means that
/// the bodies move independently of the parents, and moving the parent will not affect the child.
///
/// If you would like a child entity to be rigidly attached to its parent, you could use a [`FixedJoint`]
/// or write your own system to handle hierarchies differently.
pub struct SyncPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl SyncPlugin {
    /// Creates a [`SyncPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for SyncPlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct MarkRigidBodyAncestors;

impl Plugin for SyncPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SyncConfig>()
            .register_type::<SyncConfig>();

        app.configure_sets(
            self.schedule,
            (
                SyncSet::First,
                SyncSet::TransformToPosition,
                SyncSet::PositionToTransform,
                SyncSet::Update,
                SyncSet::Last,
            )
                .chain()
                .in_set(PhysicsSet::Sync),
        );

        // Mark ancestors of colliders with `AncestorMarker<RigidBody>`.
        // This is used to speed up transform propagation by skipping
        // trees that have no rigid bodies.
        app.add_plugins(
            AncestorMarkerPlugin::<RigidBody>::new(self.schedule)
                .add_markers_in_set(MarkRigidBodyAncestors),
        );

        app.configure_sets(
            self.schedule,
            MarkRigidBodyAncestors.in_set(PrepareSet::PreInit),
        );

        // Initialize `PreviousGlobalTransform` and apply `Transform` changes that happened
        // between the end of the previous physics frame and the start of this physics frame.
        app.add_systems(
            self.schedule,
            (
                sync_simple_transforms_physics,
                propagate_transforms_physics,
                init_previous_global_transform,
                transform_to_position,
                // Update `PreviousGlobalTransform` for the physics step's `GlobalTransform` change detection
                update_previous_global_transforms,
            )
                .chain()
                .after(PhysicsSet::Prepare)
                .before(PhysicsSet::StepSimulation)
                .run_if(|config: Res<SyncConfig>| config.transform_to_position),
        );

        // Apply `Transform` changes to `Position` and `Rotation`.
        // TODO: Do we need this?
        app.add_systems(
            self.schedule,
            (
                sync_simple_transforms_physics,
                propagate_transforms_physics,
                transform_to_position,
            )
                .chain()
                .in_set(SyncSet::TransformToPosition)
                .run_if(|config: Res<SyncConfig>| config.transform_to_position),
        );

        // Apply `Position` and `Rotation` changes to `Transform`
        app.add_systems(
            self.schedule,
            position_to_transform
                .in_set(SyncSet::PositionToTransform)
                .run_if(|config: Res<SyncConfig>| config.position_to_transform),
        );

        // Update `PreviousGlobalTransform` for next frame's `GlobalTransform` change detection
        app.add_systems(
            self.schedule,
            (
                sync_simple_transforms_physics,
                propagate_transforms_physics,
                update_previous_global_transforms,
            )
                .chain()
                .in_set(SyncSet::Update)
                .run_if(|config: Res<SyncConfig>| config.transform_to_position),
        );
    }
}

/// Configures what physics data is synchronized by the [`SyncPlugin`] and how.
#[derive(Resource, Reflect, Clone, Debug, PartialEq, Eq)]
#[reflect(Resource)]
pub struct SyncConfig {
    /// Updates transforms based on [`Position`] and [`Rotation`] changes. Defaults to true.
    pub position_to_transform: bool,
    /// Updates [`Position`] and [`Rotation`] based on transform changes,
    /// allowing you to move bodies using `Transform`. Defaults to true.
    pub transform_to_position: bool,
}

impl Default for SyncConfig {
    fn default() -> Self {
        SyncConfig {
            position_to_transform: true,
            transform_to_position: true,
        }
    }
}

/// System sets for systems running in [`PhysicsSet::Sync`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SyncSet {
    /// Runs at the start of [`PhysicsSet::Sync`]. Empty by default.
    First,
    /// Updates [`Position`] and [`Rotation`] based on transform changes.
    TransformToPosition,
    /// Updates transforms based on [`Position`] and [`Rotation`] changes.
    PositionToTransform,
    /// Handles transform propagation and other updates after physics positions have been synced with transforms.
    Update,
    /// Runs at the end of [`PhysicsSet::Sync`]. Empty by default.
    Last,
}

/// The global transform of a body at the end of the previous frame.
/// Used for detecting if the transform was modified before the start of the physics schedule.
#[derive(Component, Reflect, Clone, Copy, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct PreviousGlobalTransform(pub GlobalTransform);

#[allow(clippy::type_complexity)]
pub(crate) fn init_previous_global_transform(
    mut commands: Commands,
    query: Query<(Entity, &GlobalTransform), Or<(Added<Position>, Added<Rotation>)>>,
) {
    for (entity, transform) in &query {
        commands
            .entity(entity)
            .try_insert(PreviousGlobalTransform(*transform));
    }
}

/// Copies `GlobalTransform` changes to [`Position`] and [`Rotation`].
/// This allows users to use transforms for moving and positioning bodies and colliders.
///
/// To account for hierarchies, transform propagation should be run before this system.
#[allow(clippy::type_complexity)]
pub fn transform_to_position(
    mut query: Query<(
        &GlobalTransform,
        &PreviousGlobalTransform,
        &mut Position,
        Option<&AccumulatedTranslation>,
        &mut Rotation,
        Option<&PreviousRotation>,
        Option<&CenterOfMass>,
    )>,
) {
    for (
        global_transform,
        previous_transform,
        mut position,
        accumulated_translation,
        mut rotation,
        previous_rotation,
        center_of_mass,
    ) in &mut query
    {
        // Skip entity if the global transform value hasn't changed
        if *global_transform == previous_transform.0 {
            continue;
        }

        let transform = global_transform.compute_transform();
        let previous_transform = previous_transform.compute_transform();
        let pos = position.0
            + accumulated_translation.map_or(Vector::ZERO, |t| {
                get_pos_translation(
                    t,
                    &previous_rotation.copied().unwrap_or_default(),
                    &rotation,
                    &center_of_mass.copied().unwrap_or_default(),
                )
            });

        #[cfg(feature = "2d")]
        {
            position.0 = (previous_transform.translation.truncate()
                + (transform.translation - previous_transform.translation).truncate())
            .adjust_precision()
                + (pos - previous_transform.translation.truncate().adjust_precision());
        }
        #[cfg(feature = "3d")]
        {
            position.0 = (previous_transform.translation
                + (transform.translation - previous_transform.translation))
                .adjust_precision()
                + (pos - previous_transform.translation.adjust_precision());
        }

        #[cfg(feature = "2d")]
        {
            let rot = Rotation::from(transform.rotation.adjust_precision());
            let prev_rot = Rotation::from(previous_transform.rotation.adjust_precision());
            *rotation = prev_rot * (rot * prev_rot.inverse()) * (*rotation * prev_rot.inverse());
        }
        #[cfg(feature = "3d")]
        {
            rotation.0 = (previous_transform.rotation
                + (transform.rotation - previous_transform.rotation)
                + (rotation.f32() - previous_transform.rotation))
                .normalize()
                .adjust_precision();
        }
    }
}

type PosToTransformComponents = (
    &'static mut Transform,
    &'static Position,
    &'static Rotation,
    Option<&'static Parent>,
);

type PosToTransformFilter = (With<RigidBody>, Or<(Changed<Position>, Changed<Rotation>)>);

type ParentComponents = (
    &'static GlobalTransform,
    Option<&'static Position>,
    Option<&'static Rotation>,
);

/// Copies [`Position`] and [`Rotation`] changes to `Transform`.
/// This allows users and the engine to use these components for moving and positioning bodies.
///
/// Nested rigid bodies move independently of each other, so the `Transform`s of child entities are updated
/// based on their own and their parent's [`Position`] and [`Rotation`].
#[cfg(feature = "2d")]
pub fn position_to_transform(
    mut query: Query<PosToTransformComponents, PosToTransformFilter>,
    parents: Query<ParentComponents, With<Children>>,
) {
    for (mut transform, pos, rot, parent) in &mut query {
        if let Some(parent) = parent {
            if let Ok((parent_transform, parent_pos, parent_rot)) = parents.get(**parent) {
                // Compute the global transform of the parent using its Position and Rotation
                let parent_transform = parent_transform.compute_transform();
                let parent_pos = parent_pos.map_or(parent_transform.translation, |pos| {
                    pos.f32().extend(parent_transform.translation.z)
                });
                let parent_rot = parent_rot.map_or(parent_transform.rotation, |rot| {
                    Quaternion::from(*rot).f32()
                });
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);

                // The new local transform of the child body,
                // computed from the its global transform and its parents global transform
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(
                        pos.f32()
                            .extend(parent_pos.z + transform.translation.z * parent_scale.z),
                    )
                    .with_rotation(Quaternion::from(*rot).f32()),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));

                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else {
            transform.translation = pos.f32().extend(transform.translation.z);
            transform.rotation = Quaternion::from(*rot).f32();
        }
    }
}

/// Copies [`Position`] and [`Rotation`] changes to `Transform`.
/// This allows users and the engine to use these components for moving and positioning bodies.
///
/// Nested rigid bodies move independently of each other, so the `Transform`s of child entities are updated
/// based on their own and their parent's [`Position`] and [`Rotation`].
#[cfg(feature = "3d")]
pub fn position_to_transform(
    mut query: Query<PosToTransformComponents, PosToTransformFilter>,
    parents: Query<ParentComponents, With<Children>>,
) {
    for (mut transform, pos, rot, parent) in &mut query {
        if let Some(parent) = parent {
            if let Ok((parent_transform, parent_pos, parent_rot)) = parents.get(**parent) {
                // Compute the global transform of the parent using its Position and Rotation
                let parent_transform = parent_transform.compute_transform();
                let parent_pos = parent_pos.map_or(parent_transform.translation, |pos| pos.f32());
                let parent_rot = parent_rot.map_or(parent_transform.rotation, |rot| rot.f32());
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);

                // The new local transform of the child body,
                // computed from the its global transform and its parents global transform
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(pos.f32()).with_rotation(rot.f32()),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));

                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else {
            transform.translation = pos.f32();
            transform.rotation = rot.f32();
        }
    }
}

/// Updates [`PreviousGlobalTransform`] by setting it to `GlobalTransform` at the very end or start of a frame.
pub fn update_previous_global_transforms(
    mut bodies: Query<(&GlobalTransform, &mut PreviousGlobalTransform)>,
) {
    for (transform, mut previous_transform) in &mut bodies {
        previous_transform.0 = *transform;
    }
}

// Below are copies of Bevy's transform propagation systems, but optimized to only traverse trees with rigid bodies.
// Propagation is unnecessary for everything else, because the physics engine should only modify the positions
// of rigid bodies and their descendants. Bevy runs its own propagation near the end of the frame.

/// Updates the [`GlobalTransform`] component of physics entities that don't have other physics entities in the hierarchy.
#[allow(clippy::type_complexity)]
pub fn sync_simple_transforms_physics(
    mut query: ParamSet<(
        Query<
            (&Transform, &mut GlobalTransform),
            (
                Or<(Changed<Transform>, Added<GlobalTransform>)>,
                Without<Parent>,
                Or<(
                    Without<AncestorMarker<RigidBody>>,
                    Without<AncestorMarker<ColliderMarker>>,
                )>,
                Or<(With<RigidBody>, With<ColliderMarker>)>,
            ),
        >,
        Query<
            (Ref<Transform>, &mut GlobalTransform),
            (
                Without<Parent>,
                Or<(
                    Without<AncestorMarker<RigidBody>>,
                    Without<AncestorMarker<ColliderMarker>>,
                )>,
                Or<(With<RigidBody>, With<ColliderMarker>)>,
            ),
        >,
    )>,
    mut orphaned: RemovedComponents<Parent>,
) {
    // Update changed entities.
    query
        .p0()
        .par_iter_mut()
        .for_each(|(transform, mut global_transform)| {
            *global_transform = GlobalTransform::from(*transform);
        });
    // Update orphaned entities.
    let mut query = query.p1();
    let mut iter = query.iter_many_mut(orphaned.read());
    while let Some((transform, mut global_transform)) = iter.fetch_next() {
        if !transform.is_changed() && !global_transform.is_added() {
            *global_transform = GlobalTransform::from(*transform);
        }
    }
}

// Below is a diagram of an example hierarchy.
//
//   A
//  / \
// N   A
//    / \
//   P   N
//  / \
// N   N
//
// P = a physics entity
// A = a physics entity ancestor
// N = not a physics entity ancestor
//
// We can stop propagation, if:
//
// 1. we encounter an N that doesn't have any P as an ancestor.
// 2. we encounter a P with no children.

// TODO: A general `PhysicsMarker` for both rigid bodies and colliders could be nice.

type TransformQueryData = (
    Ref<'static, Transform>,
    &'static mut GlobalTransform,
    Option<&'static Children>,
    Has<RigidBody>,
    Has<ColliderMarker>,
);

type ParentQueryData = (
    Entity,
    Ref<'static, Parent>,
    Has<RigidBody>,
    Has<ColliderMarker>,
);

type PhysicsObjectOrAncestorFilter = Or<(
    Or<(With<RigidBody>, With<AncestorMarker<RigidBody>>)>,
    Or<(With<ColliderMarker>, With<AncestorMarker<ColliderMarker>>)>,
)>;

/// Update [`GlobalTransform`] component of physics entities based on entity hierarchy and
/// [`Transform`] component.
#[allow(clippy::type_complexity)]
pub fn propagate_transforms_physics(
    mut root_query: Query<
        (
            Entity,
            &Children,
            Ref<Transform>,
            &mut GlobalTransform,
            Has<RigidBody>,
            Has<ColliderMarker>,
        ),
        (
            Without<Parent>,
            Or<(
                With<AncestorMarker<RigidBody>>,
                With<AncestorMarker<ColliderMarker>>,
            )>,
        ),
    >,
    mut orphaned: RemovedComponents<Parent>,
    transform_query: Query<TransformQueryData, With<Parent>>,
    // This is used if the entity has no physics entity ancestor.
    parent_query_1: Query<ParentQueryData>,
    // This is used if the entity is a physics entity with children *or* if any ancestor is a physics entity.
    parent_query_2: Query<ParentQueryData, PhysicsObjectOrAncestorFilter>,
    mut orphaned_entities: Local<Vec<Entity>>,
) {
    orphaned_entities.clear();
    orphaned_entities.extend(orphaned.read());
    orphaned_entities.sort_unstable();
    root_query.par_iter_mut().for_each(
        |(entity, children, transform, mut global_transform, is_root_rb, is_root_collider)| {
            let changed = transform.is_changed() || global_transform.is_added() || orphaned_entities.binary_search(&entity).is_ok();
            if changed {
                *global_transform = GlobalTransform::from(*transform);
            }

            let handle = |(child, actual_parent, is_parent_rb, is_parent_collider): (Entity, Ref<Parent>, bool, bool)| {
                assert_eq!(
                    actual_parent.get(), entity,
                    "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
                );
                // SAFETY:
                // - `child` must have consistent parentage, or the above assertion would panic.
                // Since `child` is parented to a root entity, the entire hierarchy leading to it is consistent.
                // - We may operate as if all descendants are consistent, since `propagate_recursive` will panic before 
                //   continuing to propagate if it encounters an entity with inconsistent parentage.
                // - Since each root entity is unique and the hierarchy is consistent and forest-like,
                //   other root entities' `propagate_recursive` calls will not conflict with this one.
                // - Since this is the only place where `transform_query` gets used, there will be no conflicting fetches elsewhere.
                #[allow(unsafe_code)]
                unsafe {
                    propagate_transforms_physics_recursive(
                        &global_transform,
                        &transform_query,
                        &parent_query_1,
                        &parent_query_2,
                        child,
                        changed || actual_parent.is_changed(),
                        is_parent_rb || is_parent_collider,
                    );
                }
            };

            if is_root_rb || is_root_collider {
                parent_query_1.iter_many(children).for_each(handle);
            } else {
                parent_query_2.iter_many(children).for_each(handle);
            }
        },
    );
}

/// Recursively propagates the transforms for `entity` and all of its descendants.
///
/// # Panics
///
/// If `entity`'s descendants have a malformed hierarchy, this function will panic occur before propagating
/// the transforms of any malformed entities and their descendants.
///
/// # Safety
///
/// - While this function is running, `transform_query` must not have any fetches for `entity`,
/// nor any of its descendants.
/// - The caller must ensure that the hierarchy leading to `entity`
/// is well-formed and must remain as a tree or a forest. Each entity must have at most one parent.
#[allow(clippy::type_complexity)]
unsafe fn propagate_transforms_physics_recursive(
    parent: &GlobalTransform,
    transform_query: &Query<TransformQueryData, With<Parent>>,
    // This is used if the entity has no physics entity ancestor.
    parent_query_1: &Query<ParentQueryData>,
    // This is used if the entity is a physics entity with children *or* if any ancestor is a physics entity.
    parent_query_2: &Query<ParentQueryData, PhysicsObjectOrAncestorFilter>,
    entity: Entity,
    mut changed: bool,
    mut any_ancestor_is_physics_entity: bool,
) {
    let (global_matrix, children) = {
        let Ok((transform, mut global_transform, children, is_rb, is_collider)) =
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
            (unsafe { transform_query.get_unchecked(entity) }) else {
                return;
            };

        if any_ancestor_is_physics_entity || is_rb || is_collider {
            any_ancestor_is_physics_entity = true;

            changed |= transform.is_changed() || global_transform.is_added();
            if changed {
                *global_transform = parent.mul_transform(*transform);
            }
        }

        (*global_transform, children)
    };

    let Some(children) = children else { return };

    // If the entity has a physics entity ancestor, propagate down regardless of the child type.
    // Otherwise, only propagate to entities that are physics entities or physics entity ancestors.
    if any_ancestor_is_physics_entity {
        for (child, actual_parent, is_parent_rb, is_parent_collider) in
            parent_query_1.iter_many(children)
        {
            assert_eq!(
                actual_parent.get(), entity,
                "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
            );
            // SAFETY: The caller guarantees that `transform_query` will not be fetched
            // for any descendants of `entity`, so it is safe to call `propagate_transforms_physics_recursive` for each child.
            //
            // The above assertion ensures that each child has one and only one unique parent throughout the
            // entire hierarchy.
            unsafe {
                propagate_transforms_physics_recursive(
                    &global_matrix,
                    transform_query,
                    parent_query_1,
                    parent_query_2,
                    child,
                    changed || actual_parent.is_changed(),
                    any_ancestor_is_physics_entity || is_parent_rb || is_parent_collider,
                );
            }
        }
    } else {
        for (child, actual_parent, is_parent_rb, is_parent_collider) in
            parent_query_2.iter_many(children)
        {
            assert_eq!(
                actual_parent.get(), entity,
                "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
            );
            // SAFETY: The caller guarantees that `transform_query` will not be fetched
            // for any descendants of `entity`, so it is safe to call `propagate_transforms_physics_recursive` for each child.
            //
            // The above assertion ensures that each child has one and only one unique parent throughout the
            // entire hierarchy.
            unsafe {
                propagate_transforms_physics_recursive(
                    &global_matrix,
                    transform_query,
                    parent_query_1,
                    parent_query_2,
                    child,
                    changed || actual_parent.is_changed(),
                    any_ancestor_is_physics_entity || is_parent_rb || is_parent_collider,
                );
            }
        }
    }
}
