use super::super::{prelude::*, XpbdSchedule};
use bevy::prelude::*;

/// Synchronizes changes from the physics world to Bevy `Transform`s
pub struct SyncPlugin;

impl Plugin for SyncPlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.get_schedule_mut(XpbdSchedule)
            .expect("add xpbd schedule first")
            .add_system(sync_transforms.in_set(PhysicsSet::Sync));
    }
}

/// Copies positions from the physics world to bevy Transforms
#[cfg(feature = "2d")]
fn sync_transforms(mut bodies: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in &mut bodies {
        transform.translation = pos.extend(0.0).as_vec3_f32();
        transform.rotation = (*rot).into();
    }
}

/// Copies positions from the physics world to bevy Transforms
#[cfg(feature = "3d")]
fn sync_transforms(mut bodies: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in &mut bodies {
        transform.translation = pos.0.as_vec3_f32();
        transform.rotation = rot.0.as_quat_f32();
    }
}
