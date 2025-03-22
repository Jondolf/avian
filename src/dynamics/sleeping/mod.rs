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
        // TODO: This is only relevant for dynamic bodies. Different bodies should be distinguished with marker components.
        // Add sleep timer for all rigid bodies.
        let _ = app.try_register_required_components::<RigidBody, TimeSleeping>();

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

/// A [`Command`] that wakes up a [rigid body](RigidBody) by removing the [`Sleeping`] component
/// and resetting the [`TimeSleeping`] to zero.
pub struct WakeUpBody(pub Entity);

impl Command for WakeUpBody {
    fn apply(self, world: &mut World) {
        let Ok(mut entity_mut) = world.get_entity_mut(self.0) else {
            return;
        };

        entity_mut.remove::<Sleeping>();

        if let Some(mut time_sleeping) = entity_mut.get_mut::<TimeSleeping>() {
            time_sleeping.0 = 0.0;
        };
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
        ),
        (Without<Sleeping>, Without<SleepingDisabled>),
    >,
    collisions: Res<Collisions>,
    rb_query: Query<&RigidBody>,
    deactivation_time: Res<DeactivationTime>,
    sleep_threshold: Res<SleepingThreshold>,
    length_unit: Res<PhysicsLengthUnit>,
    time: Res<Time>,
) {
    let length_unit_sq = length_unit.squared();
    let delta_secs = time.delta_seconds_adjusted();

    for (entity, rb, mut lin_vel, mut ang_vel, mut time_sleeping) in &mut query {
        let colliding_entities = collisions.collisions_with_entity(entity).map(|c| {
            if entity == c.entity1 {
                c.entity2
            } else {
                c.entity1
            }
        });

        // Only dynamic bodies can sleep, and only if they are not
        // in contact with other dynamic bodies.
        //
        // Contacts with other types of bodies will be allowed once
        // sleeping/waking is implemented with simulation islands.
        if !rb.is_dynamic()
            || rb_query
                .iter_many(colliding_entities)
                .any(|rb| rb.is_dynamic())
        {
            continue;
        }

        let lin_vel_sq = lin_vel.length_squared();

        #[cfg(feature = "2d")]
        let ang_vel_sq = ang_vel.0.squared();
        #[cfg(feature = "3d")]
        let ang_vel_sq = ang_vel.0.dot(ang_vel.0);

        // Negative thresholds indicate that sleeping is disabled.
        let lin_sleeping_threshold_sq =
            length_unit_sq * sleep_threshold.linear * ops::abs(sleep_threshold.linear);
        let ang_sleeping_threshold_sq = sleep_threshold.angular * ops::abs(sleep_threshold.angular);

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
            Entity,
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

    for (entity, pos, rot, lin_vel, ang_vel) in &query.p0() {
        if is_changed_after_tick(pos, last_physics_tick.0, this_run)
            || is_changed_after_tick(rot, last_physics_tick.0, this_run)
            || is_changed_after_tick(lin_vel, last_physics_tick.0, this_run)
            || is_changed_after_tick(ang_vel, last_physics_tick.0, this_run)
        {
            commands.queue(WakeUpBody(entity));
        }
    }

    for entity in &query.p1() {
        commands.queue(WakeUpBody(entity));
    }
}

fn is_changed_after_tick<C: Component>(component_ref: Ref<C>, tick: Tick, this_run: Tick) -> bool {
    let last_changed = component_ref.last_changed();
    component_ref.is_changed() && last_changed.is_newer_than(tick, this_run)
}

/// Removes the [`Sleeping`] component from all sleeping bodies.
/// Triggered automatically when [`Gravity`] is changed.
fn wake_all_sleeping_bodies(mut commands: Commands, bodies: Query<Entity, With<Sleeping>>) {
    for entity in &bodies {
        commands.queue(WakeUpBody(entity));
    }
}

/// Wakes up bodies when they stop colliding.
#[allow(clippy::type_complexity)]
fn wake_on_collision_ended(
    mut commands: Commands,
    bodies: Query<
        (Ref<Position>, Has<RigidBodyDisabled>),
        (
            Or<(Added<RigidBodyDisabled>, Changed<Position>)>,
            Without<Sleeping>,
        ),
    >,
    colliders: Query<(&ColliderOf, Ref<ColliderTransform>, Has<ColliderDisabled>)>,
    collisions: Res<Collisions>,
    mut sleeping: Query<(Entity, &mut TimeSleeping, Has<Sleeping>)>,
) {
    // Wake up bodies when a body they're colliding with moves or gets disabled.
    for (entity, mut time_sleeping, is_sleeping) in &mut sleeping {
        // Skip anything that isn't currently sleeping and already has a time_sleeping of zero.
        // We can't gate the sleeping query using With<Sleeping> here because must also reset
        // non-zero time_sleeping to 0 when a colliding body moves.
        let must_check = is_sleeping || time_sleeping.0 > 0.0;
        if !must_check {
            continue;
        }

        // Here we could use CollidingEntities, but it'd be empty if the ContactReportingPlugin was disabled.
        let mut colliding_entities = collisions.collisions_with_entity(entity).map(|c| {
            if entity == c.entity1 {
                c.entity2
            } else {
                c.entity1
            }
        });
        if colliding_entities.any(|other_entity| {
            colliders.get(other_entity).is_ok_and(
                |(collider_of, transform, is_collider_disabled)| {
                    is_collider_disabled
                        || transform.is_changed()
                        || bodies
                            .get(collider_of.rigid_body)
                            .is_ok_and(|(pos, is_rb_disabled)| is_rb_disabled || pos.is_changed())
                },
            )
        }) {
            if is_sleeping {
                commands.entity(entity).remove::<Sleeping>();
            }
            time_sleeping.0 = 0.0;
        }
    }

    // Wake up bodies when a collision ends, for example when one of the bodies is despawned.
    for contacts in collisions.get_internal().values() {
        if contacts.during_current_frame || !contacts.during_previous_frame {
            continue;
        }
        if let Ok((_, mut time_sleeping, is_sleeping)) = sleeping.get_mut(contacts.entity1) {
            if is_sleeping {
                commands.entity(contacts.entity1).remove::<Sleeping>();
            }
            time_sleeping.0 = 0.0;
        }
        if let Ok((_, mut time_sleeping, is_sleeping)) = sleeping.get_mut(contacts.entity2) {
            if is_sleeping {
                commands.entity(contacts.entity2).remove::<Sleeping>();
            }
            time_sleeping.0 = 0.0;
        }
    }
}
