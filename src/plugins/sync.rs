//! Responsible for synchronizing physics components with other data, like keeping [`Position`]
//! and [`Rotation`] in sync with `Transform`.
//!
//! See [`SyncPlugin`].

use crate::prelude::*;
use bevy::prelude::*;

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
    schedule: Box<dyn ScheduleLabel>,
}

impl SyncPlugin {
    /// Creates a [`SyncPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: Box::new(schedule),
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
            self.schedule.dyn_clone(),
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
            self.schedule.dyn_clone(),
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
                position_to_transform,
                (
                    // Update `PreviousGlobalTransform` for next frame's `GlobalTransform` change detection
                    bevy::transform::systems::sync_simple_transforms,
                    bevy::transform::systems::propagate_transforms,
                    update_previous_global_transforms,
                )
                    .chain()
                    .run_if(|config: Res<SyncConfig>| config.transform_to_position),
            )
                .chain()
                .in_set(PhysicsSet::Sync)
                .run_if(|config: Res<SyncConfig>| config.position_to_transform),
        );

        // Update child colliders before narrow phase in substepping loop
        let substep_schedule = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");
        substep_schedule.add_systems(
            (update_collider_offset, update_child_collider_position)
                .chain()
                .chain()
                .after(SubstepSet::Integrate)
                .before(SubstepSet::NarrowPhase),
        );

        // Update child colliders after substepping loop
        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");
        physics_schedule.add_systems(
            (update_collider_offset, update_child_collider_position)
                .chain()
                .after(PhysicsStepSet::Substeps)
                .before(PhysicsStepSet::Sleeping),
        );
    }
}

/// Configures what physics data is synchronized by the [`SyncPlugin`] and how.
#[derive(Resource, Reflect, Clone, Debug, PartialEq, Eq)]
#[reflect(Resource)]
pub struct SyncConfig {
    /// Updates transforms based on [`Position`] and [`Rotation`] changes. Defaults to true.
    position_to_transform: bool,
    /// Updates [`Position`] and [`Rotation`] based on transform changes,
    /// allowing you to move bodies using `Transform`. Defaults to true.
    transform_to_position: bool,
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
#[derive(Component, Deref, DerefMut)]
struct PreviousGlobalTransform(GlobalTransform);

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
            &Transform,
            &ColliderOffset,
            &mut Position,
            &mut Rotation,
            &ColliderParent,
        ),
        Without<RigidBody>,
    >,
    parents: Query<(&Position, &Rotation), (With<RigidBody>, With<Children>)>,
) {
    for (transform, offset, mut position, mut rotation, parent) in &mut colliders {
        let Ok((parent_pos, parent_rot)) = parents.get(parent.get()) else {
            continue;
        };

        position.0 = parent_pos.0 + parent_rot.rotate(offset.0);
        #[cfg(feature = "2d")]
        {
            *rotation = *parent_rot + Rotation::from(transform.rotation);
        }
        #[cfg(feature = "3d")]
        {
            *rotation = (parent_rot.0 * transform.rotation.adjust_precision())
                .normalize()
                .into();
        }
    }
}

#[allow(clippy::type_complexity)]
pub(crate) fn update_collider_offset(
    transformed_colliders: Query<Entity, (With<Collider>, Without<RigidBody>, Changed<Transform>)>,
    children: Query<&Children, With<Collider>>,
    bodies: Query<(&Position, &Rotation), (With<RigidBody>, With<Children>)>,
    mut child_colliders: Query<
        (
            &mut ColliderOffset,
            &Position,
            Ref<Transform>,
            &ColliderParent,
        ),
        (With<Parent>, Without<RigidBody>),
    >,
) {
    for (mut collider_offset, position, transform, collider_parent) in &mut child_colliders {
        if !transform.is_changed() {
            continue;
        }
        if let Ok((parent_pos, parent_rot)) = bodies.get(collider_parent.get()) {
            collider_offset.0 = parent_rot.inverse().rotate(position.0 - parent_pos.0);
        }
    }

    for entity in &transformed_colliders {
        for child in children.iter_descendants(entity) {
            if let Ok((mut collider_offset, position, _, collider_parent)) =
                child_colliders.get_mut(child)
            {
                if let Ok((parent_pos, parent_rot)) = bodies.get(collider_parent.get()) {
                    collider_offset.0 = parent_rot.inverse().rotate(position.0 - parent_pos.0);
                }
            }
        }
    }
}

/// Copies `GlobalTransform` changes to [`Position`] and [`Rotation`].
/// This allows users to use transforms for moving and positioning bodies and colliders.
///
/// To account for hierarchies, transform propagation should be run before this system.
fn transform_to_position(
    mut query: Query<(
        &GlobalTransform,
        &PreviousGlobalTransform,
        &mut Position,
        Option<&AccumulatedTranslation>,
        &mut Rotation,
    )>,
) {
    for (
        global_transform,
        previous_transform,
        mut position,
        accumulated_translation,
        mut rotation,
    ) in &mut query
    {
        // Skip entity if the global transform value hasn't changed
        if *global_transform == previous_transform.0 {
            continue;
        }

        let transform = global_transform.compute_transform();
        let previous_transform = previous_transform.compute_transform();
        let pos = position.0 + accumulated_translation.map_or(Vector::ZERO, |t| t.0);

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
                    Transform::from_translation(pos.as_f32().extend(0.0))
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
