//! Synchronizes changes from the physics world to Bevy [`Transform`]s.

use crate::{prelude::*, XpbdSchedule};
use bevy::prelude::*;

/// Synchronizes changes from the physics world to Bevy [`Transform`]s.
pub struct SyncPlugin;

impl Plugin for SyncPlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.get_schedule_mut(XpbdSchedule)
            .expect("add xpbd schedule first")
            .add_system(sync_transforms.in_set(PhysicsSet::Sync))
            .add_systems(
                (
                    activate_sleeping,
                    deactivate_sleeping,
                    gravity_deactivate_sleeping,
                )
                    .chain()
                    .in_set(PhysicsSet::Sync),
            );
    }
}

/// Copies [`Pos`] and [`Rot`] values from the physics world to Bevy [`Transform`]s.
#[cfg(feature = "2d")]
fn sync_transforms(mut bodies: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in &mut bodies {
        transform.translation = pos.extend(0.0).as_vec3_f32();

        let q: Quaternion = (*rot).into();
        transform.rotation = q.as_quat_f32();
    }
}

/// Copies [`Pos`] and [`Rot`] values from the physics world to Bevy's [`Transform`]s.
#[cfg(feature = "3d")]
fn sync_transforms(mut bodies: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in &mut bodies {
        transform.translation = pos.0.as_vec3_f32();
        transform.rotation = rot.0.as_quat_f32();
    }
}

fn activate_sleeping(
    mut commands: Commands,
    mut bodies: Query<
        (
            Entity,
            &RigidBody,
            &mut LinVel,
            &mut AngVel,
            &mut TimeSleeping,
        ),
        Without<Sleeping>,
    >,
    deactivation_time: Res<DeactivationTime>,
    sleeping_threshold: Res<SleepingThreshold>,
    dt: Res<DeltaTime>,
) {
    for (entity, rb, mut lin_vel, mut ang_vel, mut time_sleeping) in &mut bodies {
        // Only dynamic bodies can sleep
        if !rb.is_dynamic() {
            continue;
        }

        #[cfg(feature = "2d")]
        let ang_vel_magnitude = ang_vel.abs();
        #[cfg(feature = "3d")]
        let ang_vel_magnitude = ang_vel.length();

        // If linear and angular velocity are below the sleeping threshold,
        // add delta time to the time sleeping, i.e. the time that the body has remained still.
        if lin_vel.length() < sleeping_threshold.0 && ang_vel_magnitude < sleeping_threshold.0 {
            time_sleeping.0 += dt.0;
        } else {
            time_sleeping.0 = 0.0;
        }

        // If the body has been still for long enough, set it to sleep and reset velocities.
        if time_sleeping.0 > deactivation_time.0 {
            commands.entity(entity).insert(Sleeping);
            *lin_vel = LinVel::ZERO;
            *ang_vel = AngVel::ZERO;
        }
    }
}

fn deactivate_sleeping(
    mut commands: Commands,
    mut bodies: Query<
        (Entity, &mut TimeSleeping),
        (
            With<Sleeping>,
            Or<(
                Changed<LinVel>,
                Changed<AngVel>,
                Changed<ExternalForce>,
                Changed<ExternalTorque>,
            )>,
        ),
    >,
) {
    for (entity, mut time_sleeping) in &mut bodies {
        commands.entity(entity).remove::<Sleeping>();
        time_sleeping.0 = 0.0;
    }
}

fn gravity_deactivate_sleeping(
    mut commands: Commands,
    mut bodies: Query<(Entity, &mut TimeSleeping), With<Sleeping>>,
    gravity: Res<Gravity>,
) {
    if gravity.is_changed() {
        for (entity, mut time_sleeping) in &mut bodies {
            commands.entity(entity).remove::<Sleeping>();
            time_sleeping.0 = 0.0;
        }
    }
}
