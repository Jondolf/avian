//! Sleeping and waking for [`PhysicsIsland`](super::PhysicsIsland)s.
//!
//! See [`PhysicsIslandSleepingPlugin`].

use bevy::{
    app::{App, Plugin},
    ecs::{
        entity::Entity,
        query::{Changed, Or, With, Without},
        resource::Resource,
        schedule::{IntoScheduleConfigs, common_conditions::resource_changed},
        system::{
            Command, Commands, Local, ParamSet, Query, Res, ResMut, SystemChangeTick, SystemState,
            lifetimeless::{SQuery, SResMut},
        },
        world::{Mut, Ref, World},
    },
    log::warn,
    prelude::{Deref, DerefMut},
    time::Time,
};

use crate::{
    data_structures::bit_vec::BitVec,
    dynamics::solver::{
        constraint_graph::ConstraintGraph,
        islands::{BodyIslandNode, PhysicsIslands},
        solver_body::SolverBody,
    },
    prelude::*,
    schedule::{LastPhysicsTick, is_changed_after_tick},
};

/// A plugin for managing sleeping and waking of [`PhysicsIsland`](super::PhysicsIsland)s.
pub struct PhysicsIslandSleepingPlugin;

impl Plugin for PhysicsIslandSleepingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AwakeIslandBitVec>();
        app.init_resource::<TimeToSleep>();

        // Insert `SleepThreshold` and `SleepTimer` for each `SolverBody`.
        app.register_required_components::<SolverBody, SleepThreshold>();
        app.register_required_components::<SolverBody, SleepTimer>();

        // Set up cached system state for sleeping and waking islands.
        let cached_system_state = CachedSleepingSystemState(SystemState::new(app.world_mut()));
        app.insert_resource(cached_system_state);

        app.add_systems(
            PhysicsSchedule,
            (
                update_sleeping_states,
                wake_islands_with_sleeping_disabled,
                wake_on_changed,
                wake_all_islands.run_if(resource_changed::<Gravity>),
                sleep_islands,
            )
                .chain()
                .in_set(PhysicsStepSet::Sleeping),
        );
    }
}

/// A bit vector that stores which islands are kept awake and which are allowed to sleep.
#[derive(Resource, Default, Deref, DerefMut)]
pub(crate) struct AwakeIslandBitVec(pub(crate) BitVec);

fn wake_islands_with_sleeping_disabled(
    mut awake_island_bit_vec: ResMut<AwakeIslandBitVec>,
    mut query: Query<(&BodyIslandNode, &mut SleepTimer), With<SleepingDisabled>>,
) {
    // Wake up all islands that have a body with `SleepingDisabled`.
    for (body_island, mut sleep_timer) in &mut query {
        awake_island_bit_vec.set_and_grow(body_island.island_id as usize);

        // Reset the sleep timer.
        sleep_timer.0 = 0.0;
    }
}

fn update_sleeping_states(
    mut awake_island_bit_vec: ResMut<AwakeIslandBitVec>,
    mut islands: ResMut<PhysicsIslands>,
    mut query: Query<
        (
            &mut SleepTimer,
            &SleepThreshold,
            &SolverBody,
            &BodyIslandNode,
        ),
        (Without<Sleeping>, Without<SleepingDisabled>),
    >,
    length_unit: Res<PhysicsLengthUnit>,
    time_to_sleep: Res<TimeToSleep>,
    time: Res<Time>,
) {
    let length_unit_squared = length_unit.0 * length_unit.0;
    let delta_secs = time.delta_secs();

    islands.split_candidate_sleep_timer = 0.0;

    // TODO: This would be nice to do in parallel.
    for (mut sleep_timer, sleep_threshold, solver_body, island_data) in query.iter_mut() {
        let lin_vel_squared = solver_body.linear_velocity.length_squared();
        #[cfg(feature = "2d")]
        let ang_vel_squared = solver_body.angular_velocity * solver_body.angular_velocity;
        #[cfg(feature = "3d")]
        let ang_vel_squared = solver_body.angular_velocity.length_squared();

        // Keep signs.
        let lin_threshold_squared = sleep_threshold.linear * sleep_threshold.linear.abs();
        let ang_threshold_squared = sleep_threshold.angular * sleep_threshold.angular.abs();

        if lin_vel_squared < length_unit_squared * lin_threshold_squared as Scalar
            && ang_vel_squared < ang_threshold_squared as Scalar
        {
            // Increment the sleep timer.
            sleep_timer.0 += delta_secs;
        } else {
            // Reset the sleep timer if the body is moving.
            sleep_timer.0 = 0.0;
        }

        if sleep_timer.0 < time_to_sleep.0 {
            // Keep the island awake.
            awake_island_bit_vec.set_and_grow(island_data.island_id as usize);
        } else if let Some(island) = islands.get(island_data.island_id)
            && island.constraints_removed > 0
        {
            // The body wants to sleep, but its island needs splitting first.
            if sleep_timer.0 > islands.split_candidate_sleep_timer {
                // This island is now the sleepiest candidate for splitting.
                islands.split_candidate = Some(island_data.island_id);
                islands.split_candidate_sleep_timer = sleep_timer.0;
            }
        }
    }
}

fn sleep_islands(
    mut awake_island_bit_vec: ResMut<AwakeIslandBitVec>,
    mut islands: ResMut<PhysicsIslands>,
    mut commands: Commands,
    mut sleep_buffer: Local<Vec<u32>>,
    mut wake_buffer: Local<Vec<u32>>,
) {
    // Clear the buffers.
    sleep_buffer.clear();
    wake_buffer.clear();

    // Sleep islands that are not in the awake bit vector.
    for island in islands.iter_mut() {
        if awake_island_bit_vec.get(island.id as usize) {
            if island.is_sleeping {
                wake_buffer.push(island.id);
            }
        } else if !island.is_sleeping {
            sleep_buffer.push(island.id);
        }
    }

    // Sleep islands.
    let sleep_buffer = sleep_buffer.clone();
    commands.queue(|world: &mut World| {
        SleepIslands(sleep_buffer).apply(world);
    });

    // Wake islands.
    let wake_buffer = wake_buffer.clone();
    commands.queue(|world: &mut World| {
        WakeIslands(wake_buffer).apply(world);
    });

    // Reset the awake island bit vector.
    awake_island_bit_vec.set_bit_count_and_clear(islands.len());
}

#[derive(Resource)]
struct CachedSleepingSystemState(
    SystemState<(
        SQuery<(&'static BodyIslandNode, &'static mut SleepTimer)>,
        SResMut<PhysicsIslands>,
        SResMut<ContactGraph>,
        SResMut<ConstraintGraph>,
    )>,
);

/// A [`Command`] that forces a [`RigidBody`] and its [`PhysicsIsland`][super::PhysicsIsland] to be [`Sleeping`].
pub struct SleepBody(pub Entity);

impl Command for SleepBody {
    fn apply(self, world: &mut World) {
        if let Some(body_island) = world.get::<BodyIslandNode>(self.0) {
            SleepIslands(vec![body_island.island_id]).apply(world);
        } else {
            warn!("Tried to sleep body {:?} that does not exist", self.0);
        }
    }
}

/// A [`Command`] that makes the [`PhysicsIsland`](super::PhysicsIsland)s with the given IDs sleep if they are not already sleeping.
pub struct SleepIslands(pub Vec<u32>);

impl Command for SleepIslands {
    fn apply(self, world: &mut World) {
        world.resource_scope(|world, mut state: Mut<CachedSleepingSystemState>| {
            let (bodies, mut islands, mut contact_graph, mut constraint_graph) =
                state.0.get_mut(world);

            let mut bodies_to_sleep = Vec::<(Entity, Sleeping)>::new();

            for island_id in self.0 {
                if let Some(island) = islands.get_mut(island_id) {
                    if island.is_sleeping {
                        // The island is already sleeping, no need to sleep it again.
                        return;
                    }

                    island.is_sleeping = true;

                    let mut body = island.head_body;

                    while let Some(entity) = body {
                        // Transfer the contact pairs to the sleeping set, and remove the body from the constraint graph.
                        contact_graph.sleep_entity_with(entity, |graph, contact_pair| {
                            // Remove touching contacts from the constraint graph.
                            if !contact_pair.is_touching() || contact_pair.is_sensor() {
                                return;
                            }
                            let contact_edge = graph
                                .get_edge_mut_by_id(contact_pair.contact_id)
                                .unwrap_or_else(|| {
                                    panic!(
                                        "Contact edge with id {:?} not found in contact graph.",
                                        contact_pair.contact_id
                                    )
                                });
                            if let (Some(body1), Some(body2)) =
                                (contact_pair.body1, contact_pair.body2)
                            {
                                for _ in 0..contact_edge.constraint_handles.len() {
                                    constraint_graph.pop_manifold(
                                        &mut graph.edges,
                                        contact_pair.contact_id,
                                        body1,
                                        body2,
                                    );
                                }
                            }
                        });

                        bodies_to_sleep.push((entity, Sleeping));
                        body = bodies.get(entity).unwrap().0.next;
                    }
                }
            }

            // Batch insert `Sleeping` to the bodies.
            world.insert_batch(bodies_to_sleep);
        });
    }
}

/// A [`Command`] that wakes up a [`RigidBody`] and its [`PhysicsIsland`](super::PhysicsIsland) if it is [`Sleeping`].
pub struct WakeBody(pub Entity);

impl Command for WakeBody {
    fn apply(self, world: &mut World) {
        if let Some(body_island) = world.get::<BodyIslandNode>(self.0) {
            WakeIslands(vec![body_island.island_id]).apply(world);
        } else {
            warn!("Tried to wake body {:?} that does not exist", self.0);
        }
    }
}

/// A deprecated alias for [`WakeBody`].
#[deprecated(since = "0.4.0", note = "Renamed to `WakeBody`.")]
pub struct WakeUpBody(pub Entity);

#[expect(deprecated)]
impl Command for WakeUpBody {
    fn apply(self, world: &mut World) {
        WakeBody(self.0).apply(world);
    }
}

/// A [`Command`] that wakes up the [`PhysicsIsland`](super::PhysicsIsland)s with the given IDs if they are sleeping.
pub struct WakeIslands(pub Vec<u32>);

impl Command for WakeIslands {
    fn apply(self, world: &mut World) {
        world.resource_scope(|world, mut state: Mut<CachedSleepingSystemState>| {
            let (mut bodies, mut islands, mut contact_graph, mut constraint_graph) =
                state.0.get_mut(world);

            let mut bodies_to_wake = Vec::<Entity>::new();

            for island_id in self.0 {
                if let Some(island) = islands.get_mut(island_id) {
                    if !island.is_sleeping {
                        // The island is not sleeping, no need to wake it up.
                        continue;
                    }

                    island.is_sleeping = false;

                    let mut body = island.head_body;

                    while let Some(entity) = body {
                        // Transfer the contact pairs to the awake set, and add touching contacts to the constraint graph.
                        contact_graph.wake_entity_with(entity, |graph, contact_pair| {
                            // Add touching contacts to the constraint graph.
                            if !contact_pair.is_touching() || contact_pair.is_sensor() {
                                return;
                            }
                            let contact_edge = graph
                                .get_edge_mut_by_id(contact_pair.contact_id)
                                .unwrap_or_else(|| {
                                    panic!(
                                        "Contact edge with id {:?} not found in contact graph.",
                                        contact_pair.contact_id
                                    )
                                });
                            for _ in contact_pair.manifolds.iter() {
                                constraint_graph.push_manifold(contact_edge, contact_pair);
                            }
                        });

                        bodies_to_wake.push(entity);

                        if let Ok((body_island, mut sleep_timer)) = bodies.get_mut(entity) {
                            body = body_island.next;
                            // Reset the sleep timer.
                            sleep_timer.0 = 0.0;
                        } else {
                            body = None;
                        }
                    }
                }
            }

            // Remove `Sleeping` from the bodies.
            bodies_to_wake.into_iter().for_each(|entity| {
                world.entity_mut(entity).remove::<Sleeping>();
            });
        });
    }
}

#[cfg(feature = "2d")]
type ConstantForceChanges = Or<(
    Changed<ConstantForce>,
    Changed<ConstantTorque>,
    Changed<ConstantLinearAcceleration>,
    Changed<ConstantAngularAcceleration>,
    Changed<ConstantLocalForce>,
    Changed<ConstantLocalLinearAcceleration>,
)>;
#[cfg(feature = "3d")]
type ConstantForceChanges = Or<(
    Changed<ConstantForce>,
    Changed<ConstantTorque>,
    Changed<ConstantLinearAcceleration>,
    Changed<ConstantAngularAcceleration>,
    Changed<ConstantLocalForce>,
    Changed<ConstantLocalTorque>,
    Changed<ConstantLocalLinearAcceleration>,
    Changed<ConstantLocalAngularAcceleration>,
)>;

/// Removes the [`Sleeping`] component from sleeping bodies when properties like
/// position, rotation, velocity and external forces are changed by the user.
fn wake_on_changed(
    mut query: ParamSet<(
        // These could've been changed by physics too.
        // We need to ignore non-user changes.
        Query<
            (
                Ref<Position>,
                Ref<Rotation>,
                Ref<LinearVelocity>,
                Ref<AngularVelocity>,
                Ref<SleepTimer>,
                &BodyIslandNode,
            ),
            (
                With<Sleeping>,
                Or<(
                    Changed<Position>,
                    Changed<Rotation>,
                    Changed<LinearVelocity>,
                    Changed<AngularVelocity>,
                    Changed<SleepTimer>,
                )>,
            ),
        >,
        // These are not modified by the physics engine
        // and don't need special handling.
        Query<&BodyIslandNode, Or<(ConstantForceChanges, Changed<GravityScale>)>>,
    )>,
    mut awake_island_bit_vec: ResMut<AwakeIslandBitVec>,
    last_physics_tick: Res<LastPhysicsTick>,
    system_tick: SystemChangeTick,
) {
    let this_run = system_tick.this_run();

    for (pos, rot, lin_vel, ang_vel, sleep_timer, body_island) in &query.p0() {
        if is_changed_after_tick(pos, last_physics_tick.0, this_run)
            || is_changed_after_tick(rot, last_physics_tick.0, this_run)
            || is_changed_after_tick(lin_vel, last_physics_tick.0, this_run)
            || is_changed_after_tick(ang_vel, last_physics_tick.0, this_run)
            || is_changed_after_tick(sleep_timer, last_physics_tick.0, this_run)
        {
            awake_island_bit_vec.set_and_grow(body_island.island_id as usize);
        }
    }

    for body_island in &query.p1() {
        awake_island_bit_vec.set_and_grow(body_island.island_id as usize);
    }
}

/// Wakes up all sleeping [`PhysicsIsland`](super::PhysicsIsland)s. Triggered automatically when [`Gravity`] is changed.
fn wake_all_islands(mut commands: Commands, islands: Res<PhysicsIslands>) {
    let sleeping_islands: Vec<u32> = islands
        .iter()
        .filter_map(|island| island.is_sleeping.then_some(island.id))
        .collect();

    if !sleeping_islands.is_empty() {
        commands.queue(WakeIslands(sleeping_islands));
    }
}
