//! Responsible for synchronizing physics components with other data, like keeping [`Position`]
//! and [`Rotation`] in sync with `Transform`.
//!
//! See [`SyncPlugin`].

use crate::{prelude::*, utils::get_pos_translation};
use bevy::{ecs::query::Has, prelude::*, utils::intern::Interned};

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

impl Plugin for SyncPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SyncConfig>()
            .register_type::<SyncConfig>();

        // Initialize `PreviousGlobalTransform` and apply `Transform` changes that happened
        // between the end of the previous physics frame and the start of this physics frame.
        app.add_systems(
            self.schedule,
            ((
                bevy::transform::systems::sync_simple_transforms,
                bevy::transform::systems::propagate_transforms,
                init_previous_global_transform,
                transform_to_position,
                // Update `PreviousGlobalTransform` for the physics step's `GlobalTransform` change detection
                update_previous_global_transforms,
            )
                .chain()
                .after(PhysicsSet::Prepare)
                .before(PhysicsSet::StepSimulation),)
                .chain()
                .run_if(|config: Res<SyncConfig>| config.transform_to_position),
        );

        // Apply `Transform`, `Position` and `Rotation` changes that happened during the physics frame.
        app.add_systems(
            self.schedule,
            (
                (
                    // Apply `Transform` changes to `Position` and `Rotation`
                    bevy::transform::systems::sync_simple_transforms,
                    bevy::transform::systems::propagate_transforms,
                    transform_to_position,
                )
                    .chain()
                    .run_if(|config: Res<SyncConfig>| config.transform_to_position),
                // Apply `Position` and `Rotation` changes to `Transform`
                position_to_transform
                    .run_if(|config: Res<SyncConfig>| config.position_to_transform),
                (
                    // Update `PreviousGlobalTransform` for next frame's `GlobalTransform` change detection
                    bevy::transform::systems::sync_simple_transforms,
                    bevy::transform::systems::propagate_transforms,
                    update_previous_global_transforms,
                )
                    .chain()
                    .run_if(|config: Res<SyncConfig>| config.transform_to_position),
                update_collider_scale,
            )
                .chain()
                .in_set(PhysicsSet::Sync),
        );

        // Update child colliders before narrow phase in substepping loop
        let substep_schedule = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");
        substep_schedule.add_systems(
            (
                propagate_collider_transforms,
                update_child_collider_position,
            )
                .chain()
                .after(SubstepSet::Integrate)
                .before(SubstepSet::NarrowPhase),
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

/// The global transform of a body at the end of the previous frame.
/// Used for detecting if the transform was modified before the start of the physics schedule.
#[derive(Component, Reflect, Clone, Copy, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct PreviousGlobalTransform(pub GlobalTransform);

type PhysicsObjectAddedFilter = Or<(Added<RigidBody>, Added<Collider>)>;

fn init_previous_global_transform(
    mut commands: Commands,
    query: Query<(Entity, &GlobalTransform), PhysicsObjectAddedFilter>,
) {
    for (entity, transform) in &query {
        commands
            .entity(entity)
            .insert(PreviousGlobalTransform(*transform));
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

#[allow(clippy::type_complexity)]
pub(crate) fn update_collider_scale(
    mut colliders: ParamSet<(
        // Root bodies
        Query<(&Transform, &mut Collider), Without<Parent>>,
        // Child colliders
        Query<(&ColliderTransform, &mut Collider), With<Parent>>,
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

/// Updates [`ColliderTransform`]s based on entity hierarchies. Each transform is computed by recursively
/// traversing the children of each rigid body and adding their transforms together to form
/// the total transform relative to the body.
///
/// This is largely a clone of `propagate_transforms` in `bevy_transform`.
#[allow(clippy::type_complexity)]
pub(crate) fn propagate_collider_transforms(
    mut root_query: Query<(Entity, Ref<Transform>, &Children), Without<Parent>>,
    collider_query: Query<
        (
            Ref<Transform>,
            Option<&mut ColliderTransform>,
            Option<&Children>,
        ),
        With<Parent>,
    >,
    parent_query: Query<(Entity, Ref<Transform>, Has<RigidBody>, Ref<Parent>)>,
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
                // - Since this is the only place where `transform_query` gets used, there will be no conflicting fetches elsewhere.
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
/// - While this function is running, `transform_query` must not have any fetches for `entity`,
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
        With<Parent>,
    >,
    parent_query: &Query<(Entity, Ref<Transform>, Has<RigidBody>, Ref<Parent>)>,
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

        // SAFETY: The caller guarantees that `transform_query` will not be fetched
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

/// Copies `GlobalTransform` changes to [`Position`] and [`Rotation`].
/// This allows users to use transforms for moving and positioning bodies and colliders.
///
/// To account for hierarchies, transform propagation should be run before this system.
#[allow(clippy::type_complexity)]
fn transform_to_position(
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
            *rotation = prev_rot + (rot - prev_rot) + (*rotation - prev_rot);
        }
        #[cfg(feature = "3d")]
        {
            rotation.0 = (previous_transform.rotation
                + (transform.rotation - previous_transform.rotation)
                + (rotation.as_f32() - previous_transform.rotation))
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
fn position_to_transform(
    mut query: Query<PosToTransformComponents, PosToTransformFilter>,
    parents: Query<ParentComponents, With<Children>>,
) {
    for (mut transform, pos, rot, parent) in &mut query {
        if let Some(parent) = parent {
            if let Ok((parent_transform, parent_pos, parent_rot)) = parents.get(**parent) {
                // Compute the global transform of the parent using its Position and Rotation
                let parent_transform = parent_transform.compute_transform();
                let parent_pos = parent_pos.map_or(parent_transform.translation, |pos| {
                    pos.as_f32().extend(parent_transform.translation.z)
                });
                let parent_rot = parent_rot.map_or(parent_transform.rotation, |rot| {
                    Quaternion::from(*rot).as_f32()
                });
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);

                // The new local transform of the child body,
                // computed from the its global transform and its parents global transform
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(
                        pos.as_f32()
                            .extend(parent_pos.z + transform.translation.z * parent_scale.z),
                    )
                    .with_rotation(Quaternion::from(*rot).as_f32()),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));

                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else {
            transform.translation = pos.as_f32().extend(transform.translation.z);
            transform.rotation = Quaternion::from(*rot).as_f32();
        }
    }
}

/// Copies [`Position`] and [`Rotation`] changes to `Transform`.
/// This allows users and the engine to use these components for moving and positioning bodies.
///
/// Nested rigid bodies move independently of each other, so the `Transform`s of child entities are updated
/// based on their own and their parent's [`Position`] and [`Rotation`].
#[cfg(feature = "3d")]
fn position_to_transform(
    mut query: Query<PosToTransformComponents, PosToTransformFilter>,
    parents: Query<ParentComponents, With<Children>>,
) {
    for (mut transform, pos, rot, parent) in &mut query {
        if let Some(parent) = parent {
            if let Ok((parent_transform, parent_pos, parent_rot)) = parents.get(**parent) {
                // Compute the global transform of the parent using its Position and Rotation
                let parent_transform = parent_transform.compute_transform();
                let parent_pos =
                    parent_pos.map_or(parent_transform.translation, |pos| pos.as_f32());
                let parent_rot = parent_rot.map_or(parent_transform.rotation, |rot| rot.as_f32());
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);

                // The new local transform of the child body,
                // computed from the its global transform and its parents global transform
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(pos.as_f32()).with_rotation(rot.as_f32()),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));

                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else {
            transform.translation = pos.as_f32();
            transform.rotation = rot.as_f32();
        }
    }
}

/// Updates [`PreviousGlobalTransform`] by setting it to `GlobalTransform` at the very end or start of a frame.
fn update_previous_global_transforms(
    mut bodies: Query<(&GlobalTransform, &mut PreviousGlobalTransform)>,
) {
    for (transform, mut previous_transform) in &mut bodies {
        previous_transform.0 = *transform;
    }
}
