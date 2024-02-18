//! Controls when bodies should be deactivated and marked as [`Sleeping`] to improve performance.
//!
//! See [`SleepingPlugin`].

use crate::prelude::*;
use bevy::prelude::*;

/// Controls when bodies should be deactivated and marked as [`Sleeping`] to improve performance.
///
/// Bodies are marked as [`Sleeping`] when their linear and angular velocities are below the [`SleepingThreshold`]
/// for a duration indicated by [`DeactivationTime`].
///
/// Bodies are woken up when an active body or constraint interacts with them, or when gravity changes,
/// or when the body's position, rotation, velocity, or external forces are changed.
///
/// This plugin does *not* handle constraints waking up bodies. That is done by the [solver].
///
/// The sleeping systems run in [`PhysicsStepSet::Sleeping`].
pub struct SleepingPlugin;

impl Plugin for SleepingPlugin {
    fn build(&self, app: &mut App) {
        app.get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first")
            .add_systems(wake_on_collision_ended.in_set(PhysicsStepSet::ReportContacts))
            .add_systems(
                (
                    mark_sleeping_bodies,
                    wake_on_changed,
                    wake_all_sleeping_bodies.run_if(resource_changed::<Gravity>),
                )
                    .chain()
                    .in_set(PhysicsStepSet::Sleeping),
            );
    }
}

type SleepingQueryComponents = (
    Entity,
    &'static RigidBody,
    &'static mut LinearVelocity,
    &'static mut AngularVelocity,
    &'static mut TimeSleeping,
);

/// Adds the [`Sleeping`] component to bodies whose linear and anigular velocities have been
/// under the [`SleepingThreshold`] for a duration indicated by [`DeactivationTime`].
pub fn mark_sleeping_bodies(
    mut commands: Commands,
    mut bodies: Query<SleepingQueryComponents, (Without<Sleeping>, Without<SleepingDisabled>)>,
    deactivation_time: Res<DeactivationTime>,
    sleep_threshold: Res<SleepingThreshold>,
    dt: Res<Time>,
) {
    for (entity, rb, mut lin_vel, mut ang_vel, mut time_sleeping) in &mut bodies {
        // Only dynamic bodies can sleep.
        if !rb.is_dynamic() {
            continue;
        }

        let lin_vel_sq = lin_vel.length_squared();

        #[cfg(feature = "2d")]
        let ang_vel_sq = ang_vel.0.powi(2);
        #[cfg(feature = "3d")]
        let ang_vel_sq = ang_vel.0.dot(ang_vel.0);

        // Negative thresholds indicate that sleeping is disabled.
        let lin_sleeping_threshold_sq = sleep_threshold.linear * sleep_threshold.linear.abs();
        let ang_sleeping_threshold_sq = sleep_threshold.angular * sleep_threshold.angular.abs();

        // If linear and angular velocity are below the sleeping threshold,
        // add delta time to the time sleeping, i.e. the time that the body has remained still.
        if lin_vel_sq < lin_sleeping_threshold_sq && ang_vel_sq < ang_sleeping_threshold_sq {
            time_sleeping.0 += dt.delta_seconds_adjusted();
        } else {
            time_sleeping.0 = 0.0;
        }

        // If the body has been still for long enough, set it to sleep and reset velocities.
        if time_sleeping.0 > deactivation_time.0 {
            commands.entity(entity).insert(Sleeping);
            *lin_vel = LinearVelocity::ZERO;
            *ang_vel = AngularVelocity::ZERO;
        }
    }
}

type WokeUpFilter = Or<(
    Changed<Position>,
    Changed<Rotation>,
    Changed<LinearVelocity>,
    Changed<AngularVelocity>,
    Changed<ExternalForce>,
    Changed<ExternalTorque>,
    Changed<ExternalImpulse>,
    Changed<ExternalAngularImpulse>,
    Changed<GravityScale>,
)>;

/// Removes the [`Sleeping`] component from sleeping bodies when properties like
/// position, rotation, velocity and external forces are changed.
#[allow(clippy::type_complexity)]
pub fn wake_on_changed(
    mut commands: Commands,
    mut bodies: Query<(Entity, &mut TimeSleeping), (With<Sleeping>, WokeUpFilter)>,
) {
    for (entity, mut time_sleeping) in &mut bodies {
        commands.entity(entity).remove::<Sleeping>();
        time_sleeping.0 = 0.0;
    }
}

/// Removes the [`Sleeping`] component from all sleeping bodies.
/// Triggered automatically when [`Gravity`] is changed.
fn wake_all_sleeping_bodies(
    mut commands: Commands,
    mut bodies: Query<(Entity, &mut TimeSleeping), With<Sleeping>>,
) {
    for (entity, mut time_sleeping) in &mut bodies {
        commands.entity(entity).remove::<Sleeping>();
        time_sleeping.0 = 0.0;
    }
}

/// Wakes up bodies when they stop colliding.
fn wake_on_collision_ended(
    mut commands: Commands,
    moved_bodies: Query<(), (Changed<Position>, Without<Sleeping>)>,
    collisions: Res<Collisions>,
    mut sleeping: Query<(Entity, &mut TimeSleeping), With<Sleeping>>,
) {
    // Wake up bodies when a body they're colliding with moves.
    for (entity, mut time_sleeping) in &mut sleeping {
        // Here we could use CollidingEntities, but it'd be empty if the ContactReportingPlugin was disabled.
        let mut colliding_entities = collisions.collisions_with_entity(entity).map(|c| {
            if entity == c.entity1 {
                c.entity2
            } else {
                c.entity1
            }
        });
        if colliding_entities.any(|entity| moved_bodies.contains(entity)) {
            commands.entity(entity).remove::<Sleeping>();
            time_sleeping.0 = 0.0;
        }
    }

    // Wake up bodies when a collision ends, for example when one of the bodies is despawned.
    for contacts in collisions.get_internal().values() {
        if contacts.during_current_frame || !contacts.during_previous_frame {
            continue;
        }
        if let Ok((_, mut time_sleeping)) = sleeping.get_mut(contacts.entity1) {
            commands.entity(contacts.entity1).remove::<Sleeping>();
            time_sleeping.0 = 0.0;
        }
        if let Ok((_, mut time_sleeping)) = sleeping.get_mut(contacts.entity2) {
            commands.entity(contacts.entity2).remove::<Sleeping>();
            time_sleeping.0 = 0.0;
        }
    }
}
