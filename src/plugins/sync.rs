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
/// Note that the engine uses [`Position`] and [`Rotation`] for all of the physics internally,
/// so changes to `Transform` will only be detected if they happen before the [`PhysicsSchedule`].
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

        // Sync changes in `Transform` to `Position` and `Rotation`
        app.add_systems(
            self.schedule.dyn_clone(),
            (
                // Run transform propagation to make sure hierarchies are handled correctly
                bevy::transform::systems::sync_simple_transforms,
                bevy::transform::systems::propagate_transforms,
                transform_to_position,
            )
                .chain()
                .before(PhysicsStepSet::BroadPhase)
                .run_if(|config: Res<SyncConfig>| config.transform_to_position),
        );

        // Init `PreviousGlobalTransform` and update it at the very start of each frame.
        // This is used for detecting if the user changed transforms before the start of the `PhysicsSchedule`.
        app.add_systems(
            self.schedule.dyn_clone(),
            init_previous_global_transform
                .in_set(PhysicsSet::Prepare)
                .run_if(|config: Res<SyncConfig>| config.transform_to_position),
        )
        .add_systems(First, update_previous_global_transforms);

        // Sync changes in `Position` and `Rotation` to `Transform`
        app.add_systems(
            self.schedule.dyn_clone(),
            position_to_transform
                .in_set(PhysicsSet::Sync)
                .run_if(|config: Res<SyncConfig>| config.position_to_transform),
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

fn init_previous_global_transform(
    mut commands: Commands,
    bodies: Query<(Entity, &GlobalTransform), Added<RigidBody>>,
) {
    for (entity, transform) in &bodies {
        commands
            .entity(entity)
            .insert(PreviousGlobalTransform(*transform));
    }
}

/// Copies `GlobalTransform` changes to [`Position`] and [`Rotation`].
/// This allows users to use transforms for moving and positioning bodies.
///
/// To account for hierarchies, transform propagation should be run before this system.
fn transform_to_position(
    mut bodies: Query<(
        &GlobalTransform,
        &PreviousGlobalTransform,
        &mut Position,
        &mut Rotation,
    )>,
) {
    for (global_transform, previous_transform, mut position, mut rotation) in &mut bodies {
        if *global_transform == previous_transform.0 {
            continue;
        }

        let transform = global_transform.compute_transform();
        #[cfg(feature = "2d")]
        {
            position.0 = transform.translation.adjust_precision().truncate();
            *rotation = Rotation::from(transform.rotation.adjust_precision());
        }
        #[cfg(feature = "3d")]
        {
            position.0 = transform.translation.adjust_precision();
            rotation.0 = transform.rotation.adjust_precision();
        }
    }
}

type RbSyncQueryComponents = (
    &'static mut Transform,
    &'static Position,
    &'static Rotation,
    Option<&'static Parent>,
);

type RbSyncQueryFilter = Or<(Changed<Position>, Changed<Rotation>)>;

type RigidBodyParentComponents = (
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
    mut bodies: Query<RbSyncQueryComponents, RbSyncQueryFilter>,
    parents: Query<RigidBodyParentComponents, With<Children>>,
) {
    for (mut transform, pos, rot, parent) in &mut bodies {
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
                    Transform::from_translation(pos.as_f32().extend(transform.translation.z))
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
    mut bodies: Query<RbSyncQueryComponents, RbSyncQueryFilter>,
    parents: Query<RigidBodyParentComponents, With<Children>>,
) {
    for (mut transform, pos, rot, parent) in &mut bodies {
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
