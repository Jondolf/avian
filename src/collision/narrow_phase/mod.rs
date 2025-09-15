//! Manages contacts and generates contact constraints.
//!
//! See [`NarrowPhasePlugin`].

mod system_param;
use system_param::ContactStatusBits;
pub use system_param::NarrowPhase;
#[cfg(feature = "parallel")]
use system_param::ThreadLocalContactStatusBits;

use core::marker::PhantomData;

use crate::{
    dynamics::solver::{ContactConstraints, constraint_graph::ConstraintGraph},
    prelude::*,
};
use bevy::{
    ecs::{
        entity_disabling::Disabled,
        intern::Interned,
        schedule::ScheduleLabel,
        system::{StaticSystemParam, SystemParam, SystemParamItem, SystemState},
    },
    prelude::*,
};

use super::{CollisionDiagnostics, contact_types::ContactEdgeFlags};

/// Manages contacts and generates contact constraints.
///
/// # Overview
///
/// Before the narrow phase, the [broad phase](super::broad_phase) creates a contact pair
/// in the [`ContactGraph`] resource for each pair of intersecting [`ColliderAabb`]s.
///
/// The narrow phase then determines which contact pairs found in the [`ContactGraph`] are touching,
/// and computes updated contact points and normals in a parallel loop.
///
/// Afterwards, the narrow phase removes contact pairs whose AABBs no longer overlap,
/// and sends collision events for colliders that started or stopped touching.
/// This is done in a fast serial loop to preserve determinism.
///
/// Finally, a [`ContactConstraint`] is generated for each contact pair that is touching
/// or expected to touch during the time step. These constraints are added to the [`ContactConstraints`]
/// resource, and are later used by the [`SolverPlugin`] to solve contacts.
///
/// [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
///
/// # Collider Types
///
/// The plugin takes a collider type. This should be [`Collider`] for
/// the vast majority of applications, but for custom collision backends
/// you may use any collider that implements the [`AnyCollider`] trait.
pub struct NarrowPhasePlugin<C: AnyCollider, H: CollisionHooks = ()> {
    schedule: Interned<dyn ScheduleLabel>,
    /// If `true`, the narrow phase will generate [`ContactConstraint`]s
    /// and add them to the [`ContactConstraints`] resource.
    ///
    /// Contact constraints are used by the [`SolverPlugin`] for solving contacts.
    ///
    /// [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
    generate_constraints: bool,
    _phantom: PhantomData<(C, H)>,
}

impl<C: AnyCollider, H: CollisionHooks> NarrowPhasePlugin<C, H> {
    /// Creates a [`NarrowPhasePlugin`] with the schedule used for running its systems
    /// and whether it should generate [`ContactConstraint`]s for the [`ContactConstraints`] resource.
    ///
    /// Contact constraints are used by the [`SolverPlugin`] for solving contacts.
    ///
    /// The default schedule is [`PhysicsSchedule`].
    ///
    /// [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
    pub fn new(schedule: impl ScheduleLabel, generate_constraints: bool) -> Self {
        Self {
            schedule: schedule.intern(),
            generate_constraints,
            _phantom: PhantomData,
        }
    }
}

impl<C: AnyCollider, H: CollisionHooks> Default for NarrowPhasePlugin<C, H> {
    fn default() -> Self {
        Self::new(PhysicsSchedule, true)
    }
}

/// A resource that indicates that the narrow phase has been initialized.
///
/// This is used to ensure that some systems are only added once
/// even with multiple collider types.
#[derive(Resource, Default)]
struct NarrowPhaseInitialized;

impl<C: AnyCollider, H: CollisionHooks + 'static> Plugin for NarrowPhasePlugin<C, H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(&self, app: &mut App) {
        let already_initialized = app.world().is_resource_added::<NarrowPhaseInitialized>();

        app.init_resource::<NarrowPhaseConfig>()
            .init_resource::<ContactGraph>()
            .init_resource::<ContactStatusBits>()
            .init_resource::<DefaultFriction>()
            .init_resource::<DefaultRestitution>();

        #[cfg(feature = "parallel")]
        app.init_resource::<ThreadLocalContactStatusBits>();

        app.register_type::<(
            NarrowPhaseConfig,
            DefaultFriction,
            DefaultRestitution,
            CollisionEventsEnabled,
        )>();

        app.add_message::<CollisionStarted>()
            .add_message::<CollisionEnded>();

        if self.generate_constraints {
            app.init_resource::<ContactConstraints>();
        }

        // Set up system set scheduling.
        app.configure_sets(
            self.schedule,
            (
                NarrowPhaseSet::First,
                NarrowPhaseSet::Update,
                NarrowPhaseSet::Last,
            )
                .chain()
                .in_set(PhysicsStepSet::NarrowPhase),
        );
        app.configure_sets(
            self.schedule,
            CollisionEventSystems.in_set(PhysicsStepSet::Finalize),
        );

        // Perform narrow phase collision detection.
        app.add_systems(
            self.schedule,
            update_narrow_phase::<C, H>
                .in_set(NarrowPhaseSet::Update)
                // Allowing ambiguities is required so that it's possible
                // to have multiple collision backends at the same time.
                .ambiguous_with_all(),
        );

        if !already_initialized {
            // Remove collision pairs when colliders are disabled or removed.
            app.add_observer(remove_collider_on::<Add, (Disabled, ColliderDisabled)>);
            app.add_observer(remove_collider_on::<Remove, ColliderMarker>);

            // Add colliders to the constraint graph when `Sensor` is removed,
            // and remove them when `Sensor` is added.
            // TODO: If we separate sensors from normal colliders, this won't be needed.
            app.add_observer(add_to_constraint_graph_on::<Remove, Sensor>);
            app.add_observer(remove_from_constraint_graph_on::<Add, Sensor>);

            // Trigger collision events for colliders that started or stopped touching.
            app.add_systems(
                PhysicsSchedule,
                trigger_collision_events
                    .in_set(CollisionEventSystems)
                    // TODO: Ideally we don't need to make this ambiguous, but currently it is
                    //       to avoid conflicts since the system has exclusive world access.
                    .ambiguous_with(PhysicsStepSet::Finalize),
            );
        }

        app.init_resource::<NarrowPhaseInitialized>();
    }

    fn finish(&self, app: &mut App) {
        // Register timer and counter diagnostics for collision detection.
        app.register_physics_diagnostics::<CollisionDiagnostics>();
    }
}

/// A system set for triggering the [`CollisionStart`] and [`CollisionEnd`] events.
///
/// Runs in [`PhysicsStepSet::Finalize`], after the solver has run and contact impulses
/// have been computed and applied.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CollisionEventSystems;

/// A resource for configuring the [narrow phase](NarrowPhasePlugin).
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Resource, PartialEq)]
pub struct NarrowPhaseConfig {
    /// The default maximum [speculative margin](SpeculativeMargin) used for
    /// [speculative collisions](dynamics::ccd#speculative-collision). This can be overridden
    /// for individual entities with the [`SpeculativeMargin`] component.
    ///
    /// By default, the maximum speculative margin is unbounded, so contacts can be predicted
    /// from any distance, provided that the bodies are moving fast enough. As the prediction distance
    /// grows, the contact data becomes more and more approximate, and in rare cases, it can even cause
    /// [issues](dynamics::ccd#caveats-of-speculative-collision) such as ghost collisions.
    ///
    /// By limiting the maximum speculative margin, these issues can be mitigated, at the cost
    /// of an increased risk of tunneling. Setting it to `0.0` disables speculative collision
    /// altogether for entities without [`SpeculativeMargin`].
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    ///
    /// Default: `MAX` (unbounded)
    pub default_speculative_margin: Scalar,

    /// A contact tolerance that acts as a minimum bound for the [speculative margin](dynamics::ccd#speculative-collision).
    ///
    /// A small, positive contact tolerance helps ensure that contacts are not missed
    /// due to numerical issues or solver jitter for objects that are in continuous
    /// contact, such as pushing against each other.
    ///
    /// Making the contact tolerance too large will have a negative impact on performance,
    /// as contacts will be computed even for objects that are not in close proximity.
    ///
    /// This is implicitly scaled by the [`PhysicsLengthUnit`].
    ///
    /// Default: `0.005`
    pub contact_tolerance: Scalar,

    /// If `true`, the current contacts will be matched with the previous contacts
    /// based on feature IDs or contact positions, and the contact impulses from
    /// the previous frame will be copied over for the new contacts.
    ///
    /// Using these impulses as the initial guess is referred to as *warm starting*,
    /// and it can help the contact solver resolve overlap and stabilize much faster.
    ///
    /// Default: `true`
    pub match_contacts: bool,
}

impl Default for NarrowPhaseConfig {
    fn default() -> Self {
        Self {
            default_speculative_margin: Scalar::MAX,
            contact_tolerance: 0.005,
            match_contacts: true,
        }
    }
}

/// System sets for systems running in [`PhysicsStepSet::NarrowPhase`].
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum NarrowPhaseSet {
    /// Runs at the start of the narrow phase. Empty by default.
    First,
    /// Updates contacts in the [`ContactGraph`] and processes contact state changes.
    Update,
    /// Runs at the end of the narrow phase. Empty by default.
    Last,
}

fn update_narrow_phase<C: AnyCollider, H: CollisionHooks + 'static>(
    mut narrow_phase: NarrowPhase<C>,
    mut collision_started_writer: MessageWriter<CollisionStarted>,
    mut collision_ended_writer: MessageWriter<CollisionEnded>,
    time: Res<Time>,
    hooks: StaticSystemParam<H>,
    context: StaticSystemParam<C::Context>,
    mut commands: ParallelCommands,
    mut diagnostics: ResMut<CollisionDiagnostics>,
) where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    let start = crate::utils::Instant::now();

    narrow_phase.update::<H>(
        &mut collision_started_writer,
        &mut collision_ended_writer,
        time.delta_seconds_adjusted(),
        &hooks,
        &context,
        &mut commands,
    );

    diagnostics.narrow_phase = start.elapsed();
    diagnostics.contact_count = narrow_phase.contact_graph.edges.edge_count() as u32;
}

#[derive(SystemParam)]
struct TriggerCollisionEventsContext<'w, 's> {
    query: Query<'w, 's, (Option<&'static ColliderOf>, Has<CollisionEventsEnabled>)>,
    started: MessageReader<'w, 's, CollisionStarted>,
    ended: MessageReader<'w, 's, CollisionEnded>,
}

/// Triggers [`CollisionStart`] and [`CollisionEnd`] events for colliders
/// that started or stopped touching and have the [`CollisionEventsEnabled`] component.
fn trigger_collision_events(
    // We use exclusive access here to avoid queuing a new command for each event.
    world: &mut World,
    state: &mut SystemState<TriggerCollisionEventsContext>,
    // Cache pairs in buffers to avoid reallocating every time.
    mut started: Local<Vec<CollisionStart>>,
    mut ended: Local<Vec<CollisionEnd>>,
) {
    let mut state = state.get_mut(world);

    // Collect `CollisionStart` events.
    for event in state.started.read() {
        let Ok(
            [
                (collider_of1, events_enabled1),
                (collider_of2, events_enabled2),
            ],
        ) = state.query.get_many([event.0, event.1])
        else {
            continue;
        };
        let body1 = collider_of1.map(|c| c.body);
        let body2 = collider_of2.map(|c| c.body);

        if events_enabled1 {
            started.push(CollisionStart {
                collider1: event.0,
                collider2: event.1,
                body1,
                body2,
            });
        }
        if events_enabled2 {
            started.push(CollisionStart {
                collider1: event.1,
                collider2: event.0,
                body1: body2,
                body2: body1,
            });
        }
    }

    // Collect `CollisionEnd` events.
    for event in state.ended.read() {
        let Ok(
            [
                (collider_of1, events_enabled1),
                (collider_of2, events_enabled2),
            ],
        ) = state.query.get_many([event.0, event.1])
        else {
            continue;
        };
        let body1 = collider_of1.map(|c| c.body);
        let body2 = collider_of2.map(|c| c.body);

        if events_enabled1 {
            ended.push(CollisionEnd {
                collider1: event.0,
                collider2: event.1,
                body1,
                body2,
            });
        }
        if events_enabled2 {
            ended.push(CollisionEnd {
                collider1: event.1,
                collider2: event.0,
                body1: body2,
                body2: body1,
            });
        }
    }

    // Trigger the events, draining the buffers in the process.
    started.drain(..).for_each(|event| {
        world.trigger(event);
    });
    ended.drain(..).for_each(|event| {
        world.trigger(event);
    });
}

/// Removes colliders from the [`ContactGraph`] when the given trigger is activated.
///
/// Also removes the collider from the [`CollidingEntities`] of the other entity,
/// wakes up the other body, and sends a [`CollisionEnded`] event.
fn remove_collider_on<E: EntityEvent, B: Bundle>(
    trigger: On<E, B>,
    mut contact_graph: ResMut<ContactGraph>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    // TODO: Change this hack to include disabled entities with `Allows<T>` for 0.17
    mut query: Query<&mut CollidingEntities, Or<(With<Disabled>, Without<Disabled>)>>,
    collider_of: Query<&ColliderOf, Or<(With<Disabled>, Without<Disabled>)>>,
    mut writer: MessageWriter<CollisionEnded>,
    mut commands: Commands,
) {
    let entity = trigger.event_target();

    let body1 = collider_of
        .get(entity)
        .map(|&ColliderOf { body }| body)
        .ok();

    // Remove the collider from the contact graph.
    contact_graph.remove_collider_with(entity, |contact_graph, contact_id| {
        // Get the contact edge.
        let contact_edge = contact_graph.edge_weight(contact_id.into()).unwrap();

        // If the contact pair was not touching, we don't need to do anything.
        if !contact_edge.flags.contains(ContactEdgeFlags::TOUCHING) {
            return;
        }

        // Send a collision ended event.
        if contact_edge
            .flags
            .contains(ContactEdgeFlags::CONTACT_EVENTS)
        {
            writer.write(CollisionEnded(
                contact_edge.collider1,
                contact_edge.collider2,
            ));
        }

        // Remove the entity from the `CollidingEntities` of the other entity.
        let other_entity = if contact_edge.collider1 == entity {
            contact_edge.collider2
        } else {
            contact_edge.collider1
        };
        if let Ok(mut colliding_entities) = query.get_mut(other_entity) {
            colliding_entities.remove(&entity);
        }

        // Wake up the other body.
        let body2 = collider_of
            .get(other_entity)
            .map(|&ColliderOf { body }| body)
            .ok();
        if let Some(body2) = body2 {
            commands.queue(WakeUpBody(body2));
        }

        // Remove the contact edge from the constraint graph.
        if let (Some(body1), Some(body2)) = (body1, body2) {
            for _ in 0..contact_edge.constraint_handles.len() {
                constraint_graph.pop_manifold(contact_graph, contact_id, body1, body2);
            }
        }
    });
}

// TODO: These are currently used just for sensors. It wouldn't be needed if sensor logic
//       was separate from normal colliders and didn't compute contact manifolds.

/// Adds all touching contact edges associated with an entity to the [`ConstraintGraph`].
fn add_to_constraint_graph_on<E: EntityEvent, B: Bundle>(
    trigger: On<E, B>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    mut contact_graph: ResMut<ContactGraph>,
) {
    let entity = trigger.event_target();

    // Get the node index of the entity in the contact graph.
    let Some(node) = contact_graph.entity_to_node(entity) else {
        return;
    };

    // TODO: It'd be nice to have some APIs on the contact graph to do this more cleanly.
    let ContactGraph {
        edges,
        active_pairs,
        ..
    } = &mut *contact_graph;

    // Add all contact edges associated with the sensor to the constraint graph.
    for contact_edge in edges.edge_weights_mut(node) {
        if !contact_edge.is_touching() {
            continue;
        }
        let pair = &active_pairs[contact_edge.pair_index];
        constraint_graph.push_manifold(contact_edge, pair);
    }
}

/// Removes all contact edges associated with an entity from the [`ConstraintGraph`].
fn remove_from_constraint_graph_on<E: EntityEvent, B: Bundle>(
    trigger: On<E, B>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    mut contact_graph: ResMut<ContactGraph>,
) {
    let entity = trigger.event_target();

    // Get all touching contact pairs.
    let contact_pairs = contact_graph
        .contact_pairs_with(entity)
        .filter_map(|contact_pair| {
            if contact_pair.is_touching()
                && let Some(body1) = contact_pair.body1
                && let Some(body2) = contact_pair.body2
            {
                Some((contact_pair.contact_id, body1, body2))
            } else {
                None
            }
        })
        .collect::<Vec<_>>();

    // Remove all contact edges associated with the sensor from the constraint graph.
    for (contact_id, body1, body2) in contact_pairs {
        constraint_graph.pop_manifold(&mut contact_graph.edges, contact_id, body1, body2);
    }
}
