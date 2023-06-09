//! Controls when bodies are active. This improves performance and helps prevent jitter. See [`Sleeping`].

use crate::prelude::*;
use bevy::prelude::*;

/// Controls when bodies are active. This improves performance and helps prevent jitter.
///
/// Bodies are marked as [`Sleeping`] when their linear and angular velocities are below the [`SleepingThreshold`] for a duration indicated by [`DeactivationTime`].
///
/// Bodies are woken up when an active body or constraint interacts with them, or when gravity changes, or when the body's position, rotation, velocity, or external forces are changed.
pub struct SleepingPlugin;

impl Plugin for SleepingPlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.get_schedule_mut(XpbdSchedule)
            .expect("add xpbd schedule first")
            .add_systems(
                (mark_sleeping_bodies, wake_up_bodies, gravity_wake_up_bodies)
                    .chain()
                    .in_set(PhysicsSet::Sleeping),
            );
    }
}

type SleepingQueryComponents = (
    Entity,
    &'static RigidBody,
    &'static mut LinVel,
    &'static mut AngVel,
    &'static mut TimeSleeping,
);

/// Adds the [`Sleeping`] component to bodies whose linear and anigular velocities have been
/// under the [`SleepingThreshold`] for a duration indicated by [`DeactivationTime`].
fn mark_sleeping_bodies(
    mut commands: Commands,
    mut bodies: Query<SleepingQueryComponents, (Without<Sleeping>, Without<SleepingDisabled>)>,
    deactivation_time: Res<DeactivationTime>,
    sleep_threshold: Res<SleepingThreshold>,
    dt: Res<DeltaTime>,
) {
    for (entity, rb, mut lin_vel, mut ang_vel, mut time_sleeping) in &mut bodies {
        // Only dynamic bodies can sleep.
        if !rb.is_dynamic() {
            continue;
        }

        let lin_vel_sq = lin_vel.length_squared();

        #[cfg(feature = "2d")]
        let ang_vel_sq = ang_vel.powi(2);
        #[cfg(feature = "3d")]
        let ang_vel_sq = ang_vel.dot(ang_vel.0);

        // Negative thresholds indicate that sleeping is disabled.
        let lin_sleeping_threshold_sq = sleep_threshold.linear * sleep_threshold.linear.abs();
        let ang_sleeping_threshold_sq = sleep_threshold.angular * sleep_threshold.angular.abs();

        // If linear and angular velocity are below the sleeping threshold,
        // add delta time to the time sleeping, i.e. the time that the body has remained still.
        if lin_vel_sq < lin_sleeping_threshold_sq && ang_vel_sq < ang_sleeping_threshold_sq {
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

type BodyWokeUpFilter = Or<(
    Changed<Pos>,
    Changed<Rot>,
    Changed<LinVel>,
    Changed<AngVel>,
    Changed<ExternalForce>,
    Changed<ExternalTorque>,
)>;

/// Removes the [`Sleeping`] component from sleeping bodies when properties like
/// position, rotation, velocity and external forces are changed.
fn wake_up_bodies(
    mut commands: Commands,
    mut bodies: Query<(Entity, &mut TimeSleeping), (With<Sleeping>, BodyWokeUpFilter)>,
) {
    for (entity, mut time_sleeping) in &mut bodies {
        commands.entity(entity).remove::<Sleeping>();
        time_sleeping.0 = 0.0;
    }
}

/// Removes the [`Sleeping`] component from sleeping bodies when [`Gravity`] is changed.
fn gravity_wake_up_bodies(
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
