//! Responsible for synchronizing physics components with other data, like writing [`Position`]
//! and [`Rotation`] components to `Transform`s.
//!
//! See [`SyncPlugin`].

use crate::prelude::*;
use bevy::prelude::*;

/// Responsible for synchronizing physics components with other data, like writing [`Position`]
/// and [`Rotation`] components to `Transform`s.
///
/// Currently, the transforms of nested bodies are updated to reflect their global positions.
/// This means that nested [rigid bodies](RigidBody) can behave independently regardless of the hierarchy.
///
/// The synchronization systems run in [`PhysicsSet::Sync`].
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
        app.add_systems(
            self.schedule.dyn_clone(),
            sync_transforms.in_set(PhysicsSet::Sync),
        );
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

/// Copies [`Position`] and [`Rotation`] values from the physics world to Bevy `Transform`s.
#[cfg(feature = "2d")]
fn sync_transforms(
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

/// Copies [`Position`] and [`Rotation`] values from the physics world to Bevy's `Transform`s.
///
/// Nested rigid bodies move independently of each other, so the `Transform`s of child entities are updated
/// based on their own and their parent's [`Position`] and [`Rotation`].
#[cfg(feature = "3d")]
fn sync_transforms(
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
