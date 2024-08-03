//! Manages sleeping and waking for bodies, automatically deactivating them to save computational resources.
//!
//! See [`SleepingPlugin`].

use crate::prelude::*;
use bevy::{
    ecs::{component::Tick, system::SystemChangeTick},
    prelude::*,
};

/// Manages sleeping and waking for bodies, automatically deactivating them to save computational resources.
///
/// Bodies are marked as [`Sleeping`] when their linear and angular velocities are below the [`SleepingThreshold`]
/// for a duration indicated by [`DeactivationTime`].
///
/// Bodies are woken up when an active body or constraint interacts with them, or when gravity changes,
/// or when the body's position, rotation, velocity, or external forces are changed.
///
/// This plugin does *not* handle constraints waking up bodies. That is done by the [solver](dynamics::solver).
///
/// The sleeping systems run in [`PhysicsStepSet::Sleeping`].
pub struct SleepingPlugin;

impl Plugin for SleepingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SleepingThreshold>()
            .init_resource::<DeactivationTime>()
            .init_resource::<LastPhysicsTick>();

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        // TODO: Where exactly should this be in the schedule?
        physics_schedule.add_systems(
            (
                wake_on_changed,
                wake_all_sleeping_bodies.run_if(resource_changed::<Gravity>),
                mark_sleeping_bodies,
            )
                .chain()
                .after(PhysicsStepSet::First)
                .before(PhysicsStepSet::BroadPhase),
        );

        physics_schedule
            .add_systems(wake_on_collision_ended.in_set(PhysicsStepSet::ReportContacts));

        physics_schedule.add_systems(
            (|mut last_physics_tick: ResMut<LastPhysicsTick>,
              system_change_tick: SystemChangeTick| {
                last_physics_tick.0 = system_change_tick.this_run();
            })
            .after(PhysicsStepSet::Last),
        );
    }
}

/// A threshold that indicates the maximum linear and angular velocity allowed for a body to be deactivated.
///
/// Setting a negative sleeping threshold disables sleeping entirely.
///
/// See [`Sleeping`] for further information about sleeping.
#[derive(Reflect, Resource, Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Resource, PartialEq)]
pub struct SleepingThreshold {
    /// The maximum linear velocity allowed for a body to be marked as sleeping.
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    ///
    /// Default: `0.15`
    pub linear: Scalar,
    /// The maximum angular velocity allowed for a body to be marked as sleeping.
    ///
    /// Default: `0.15`
    pub angular: Scalar,
}

impl Default for SleepingThreshold {
    fn default() -> Self {
        Self {
            linear: 0.15,
            angular: 0.15,
        }
    }
}

/// How long in seconds the linear and angular velocity of a body need to be below
/// the [`SleepingThreshold`] before the body is deactivated. Defaults to 1 second.
///
/// See [`Sleeping`] for further information about sleeping.
///
/// Default: `0.5`
#[derive(Reflect, Resource, Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Default, PartialEq)]
pub struct DeactivationTime(pub Scalar);

impl Default for DeactivationTime {
    fn default() -> Self {
        Self(0.5)
    }
}

/// Adds the [`Sleeping`] component to bodies whose linear and anigular velocities have been
/// under the [`SleepingThreshold`] for a duration indicated by [`DeactivationTime`].
#[allow(clippy::type_complexity)]
pub fn mark_sleeping_bodies(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &RigidBody,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &mut TimeSleeping,
            &CollidingEntities,
        ),
        (Without<Sleeping>, Without<SleepingDisabled>),
    >,
    rb_query: Query<&RigidBody>,
    deactivation_time: Res<DeactivationTime>,
    sleep_threshold: Res<SleepingThreshold>,
    length_unit: Res<PhysicsLengthUnit>,
    time: Res<Time>,
) {
    let length_unit_sq = length_unit.powi(2);
    let delta_secs = time.delta_seconds_adjusted();

    for (entity, rb, mut lin_vel, mut ang_vel, mut time_sleeping, colliding_entities) in &mut query
    {
        // Only dynamic bodies can sleep, and only if they are not
        // in contact with other dynamic bodies.
        //
        // Contacts with other types of bodies will be allowed once
        // sleeping/waking is implemented with simulation islands.
        if !rb.is_dynamic()
            || rb_query
                .iter_many(colliding_entities.iter())
                .any(|rb| rb.is_dynamic())
        {
            continue;
        }

        let lin_vel_sq = lin_vel.length_squared();

        #[cfg(feature = "2d")]
        let ang_vel_sq = ang_vel.0.powi(2);
        #[cfg(feature = "3d")]
        let ang_vel_sq = ang_vel.0.dot(ang_vel.0);

        // Negative thresholds indicate that sleeping is disabled.
        let lin_sleeping_threshold_sq =
            length_unit_sq * sleep_threshold.linear * sleep_threshold.linear.abs();
        let ang_sleeping_threshold_sq = sleep_threshold.angular * sleep_threshold.angular.abs();

        // If linear and angular velocity are below the sleeping threshold,
        // add delta time to the time sleeping, i.e. the time that the body has remained still.
        if lin_vel_sq < lin_sleeping_threshold_sq && ang_vel_sq < ang_sleeping_threshold_sq {
            time_sleeping.0 += delta_secs;
        } else {
            time_sleeping.0 = 0.0;
        }

        // If the body has been still for long enough, set it to sleep and reset velocities.
        if time_sleeping.0 > deactivation_time.0 {
            commands.entity(entity).try_insert(Sleeping);
            *lin_vel = LinearVelocity::ZERO;
            *ang_vel = AngularVelocity::ZERO;
        }
    }
}

/// A [`Tick`] corresponding to the end of the previous run of the [`PhysicsSchedule`].
#[derive(Resource, Reflect, Default)]
#[reflect(Resource, Default)]
pub(crate) struct LastPhysicsTick(pub Tick);

/// Removes the [`Sleeping`] component from sleeping bodies when properties like
/// position, rotation, velocity and external forces are changed by the user.
#[allow(clippy::type_complexity)]
pub(crate) fn wake_on_changed(
    mut commands: Commands,
    mut query: ParamSet<(
        // These could've been changed by physics too.
        // We need to ignore non-user changes.
        Query<
            (
                Entity,
                Ref<Position>,
                Ref<Rotation>,
                Ref<LinearVelocity>,
                Ref<AngularVelocity>,
                &mut TimeSleeping,
            ),
            (
                With<Sleeping>,
                Or<(
                    Changed<Position>,
                    Changed<Rotation>,
                    Changed<LinearVelocity>,
                    Changed<AngularVelocity>,
                )>,
            ),
        >,
        // These are not modified by the physics engine
        // and don't need special handling.
        Query<
            (Entity, &mut TimeSleeping),
            Or<(
                Changed<ExternalForce>,
                Changed<ExternalTorque>,
                Changed<ExternalImpulse>,
                Changed<ExternalAngularImpulse>,
                Changed<GravityScale>,
            )>,
        >,
    )>,
    last_physics_tick: Res<LastPhysicsTick>,
    system_tick: SystemChangeTick,
) {
    let this_run = system_tick.this_run();

    for (entity, pos, rot, lin_vel, ang_vel, mut time_sleeping) in &mut query.p0() {
        if is_changed_after_tick(pos, last_physics_tick.0, this_run)
            || is_changed_after_tick(rot, last_physics_tick.0, this_run)
            || is_changed_after_tick(lin_vel, last_physics_tick.0, this_run)
            || is_changed_after_tick(ang_vel, last_physics_tick.0, this_run)
        {
            commands.entity(entity).remove::<Sleeping>();
            time_sleeping.0 = 0.0;
        }
    }

    for (entity, mut time_sleeping) in &mut query.p1() {
        commands.entity(entity).remove::<Sleeping>();
        time_sleeping.0 = 0.0;
    }
}

fn is_changed_after_tick<C: Component>(component_ref: Ref<C>, tick: Tick, this_run: Tick) -> bool {
    let last_changed = component_ref.last_changed();
    component_ref.is_changed() && last_changed.is_newer_than(tick, this_run)
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
#[allow(clippy::type_complexity)]
fn wake_on_collision_ended(
    mut commands: Commands,
    moved_bodies: Query<Ref<Position>, (Changed<Position>, Without<Sleeping>)>,
    colliders: Query<(&ColliderParent, Ref<ColliderTransform>)>,
    collisions: Res<Collisions>,
    mut sleeping: Query<(Entity, &mut TimeSleeping)>,
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
        if colliding_entities.any(|other_entity| {
            colliders.get(other_entity).is_ok_and(|(p, transform)| {
                transform.is_changed()
                    || moved_bodies.get(p.get()).is_ok_and(|pos| pos.is_changed())
            })
        }) {
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
