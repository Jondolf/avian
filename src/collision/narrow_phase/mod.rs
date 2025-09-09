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
    dynamics::solver::{
        ContactConstraints,
        constraint_graph::ConstraintGraph,
        islands::{BodyIslandNode, PhysicsIslands},
        joint_graph::JointGraph,
    },
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

        app.add_event::<CollisionStarted>()
            .add_event::<CollisionEnded>();

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
            app.add_observer(remove_collider_on::<OnAdd, (Disabled, ColliderDisabled)>);
            app.add_observer(remove_collider_on::<OnRemove, ColliderMarker>);

            // Add colliders to the constraint graph when `Sensor` is removed,
            // and remove them when `Sensor` is added.
            // TODO: If we separate sensors from normal colliders, this won't be needed.
            app.add_observer(on_add_sensor);
            app.add_observer(on_remove_sensor);

            // Add contacts to the constraint graph when a body is enabled,
            // and remove them when a body is disabled.
            app.add_observer(on_body_remove_rigid_body_disabled);
            app.add_observer(on_disable_body);

            // Remove contacts when the body body is disabled or `RigidBody` is replaced or removed.
            app.add_observer(remove_body_on::<OnInsert, RigidBody>);
            app.add_observer(remove_body_on::<OnRemove, RigidBody>);

            // Trigger collision events for colliders that started or stopped touching.
            app.add_systems(
                self.schedule,
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

/// A system set for triggering the [`OnCollisionStart`] and [`OnCollisionEnd`] events.
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
    mut collision_started_event_writer: EventWriter<CollisionStarted>,
    mut collision_ended_event_writer: EventWriter<CollisionEnded>,
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
        &mut collision_started_event_writer,
        &mut collision_ended_event_writer,
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
    started: EventReader<'w, 's, CollisionStarted>,
    ended: EventReader<'w, 's, CollisionEnded>,
}

/// Triggers [`OnCollisionStart`] and [`OnCollisionEnd`] events for colliders
/// that started or stopped touching and have the [`CollisionEventsEnabled`] component.
fn trigger_collision_events(
    // We use exclusive access here to avoid queuing a new command for each event.
    world: &mut World,
    state: &mut SystemState<TriggerCollisionEventsContext>,
    // Cache pairs in buffers to avoid reallocating every time.
    mut started_pairs: Local<Vec<(Entity, OnCollisionStart)>>,
    mut ended_pairs: Local<Vec<(Entity, OnCollisionEnd)>>,
) {
    let mut state = state.get_mut(world);

    // Collect `OnCollisionStart` and `OnCollisionEnd` events
    // for entities that have events enabled.
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
        if events_enabled1 {
            let collider = event.1;
            let body = collider_of2.map(|c| c.body);
            started_pairs.push((event.0, OnCollisionStart { collider, body }));
        }
        if events_enabled2 {
            let collider = event.0;
            let body = collider_of1.map(|c| c.body);
            started_pairs.push((event.1, OnCollisionStart { collider, body }));
        }
    }
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
        if events_enabled1 {
            let collider = event.1;
            let body = collider_of2.map(|c| c.body);
            ended_pairs.push((event.0, OnCollisionEnd { collider, body }));
        }
        if events_enabled2 {
            let collider = event.0;
            let body = collider_of1.map(|c| c.body);
            ended_pairs.push((event.1, OnCollisionEnd { collider, body }));
        }
    }

    // Trigger the events, draining the buffers in the process.
    started_pairs.drain(..).for_each(|(entity, event)| {
        world.trigger_targets(event, entity);
    });
    ended_pairs.drain(..).for_each(|(entity, event)| {
        world.trigger_targets(event, entity);
    });
}

// ===============================================================
// The rest of this module contains observers and helper functions
// for updating the contact graph, constraint graph, and islands
// when bodies or colliders are added/removed or enabled/disabled.
// ===============================================================

// Cases to consider:
// - Collider is removed -> remove all contacts
// - Collider is disabled -> remove all contacts
// - Collider becomes a sensor -> remove all touching contacts from constraint graph and islands
// - Collider stops being a sensor -> add all touching contacts to constraint graph and islands
// - Body is removed -> remove all touching contacts from constraint graph and islands
// - Body is disabled -> remove all touching contacts from constraint graph and islands
// - Body is enabled -> add all touching contacts to constraint graph and islands
// - Body becomes static -> remove all static-static contacts

/// Removes a collider from the [`ContactGraph`].
///
/// Also removes the collider from the [`CollidingEntities`] of the other entity,
/// wakes up the other body, and sends a [`CollisionEnded`] event.
fn remove_collider(
    entity: Entity,
    contact_graph: &mut ContactGraph,
    joint_graph: &JointGraph,
    constraint_graph: &mut ConstraintGraph,
    islands: &mut PhysicsIslands,
    body_islands: &mut Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    colliding_entities_query: &mut Query<
        &mut CollidingEntities,
        Or<(With<Disabled>, Without<Disabled>)>,
    >,
    event_writer: &mut EventWriter<CollisionEnded>,
) {
    // TODO: Wake up the island of the other bodies.
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
            event_writer.write(CollisionEnded(
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
        if let Ok(mut colliding_entities) = colliding_entities_query.get_mut(other_entity) {
            colliding_entities.remove(&entity);
        }

        let has_island = contact_edge.island.is_some();

        // Remove the contact edge from the constraint graph.
        if let (Some(body1), Some(body2)) = (contact_edge.body1, contact_edge.body2) {
            for _ in 0..contact_edge.constraint_handles.len() {
                constraint_graph.pop_manifold(contact_graph, contact_id, body1, body2);
            }
        }

        // Unlink the contact pair from its island.
        if has_island {
            islands.remove_contact(contact_id, body_islands, contact_graph, joint_graph);
        }
    });
}

/// Removes contacts from the [`ConstraintGraph`], [`ContactGraph`], and [`PhysicsIslands`]
/// when both bodies in a contact pair become static.
fn remove_body_on<E: Event, B: Bundle>(
    trigger: Trigger<E, B>,
    body_collider_query: Query<&RigidBodyColliders>,
    mut colliding_entities_query: Query<
        &mut CollidingEntities,
        Or<(With<Disabled>, Without<Disabled>)>,
    >,
    mut event_writer: EventWriter<CollisionEnded>,
    mut body_islands: Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    mut islands: ResMut<PhysicsIslands>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: ResMut<JointGraph>,
    mut commands: Commands,
) {
    let Ok(colliders) = body_collider_query.get(trigger.target()) else {
        return;
    };

    // Wake up the body's island.
    if let Ok(body_island) = body_islands.get_mut(trigger.target()) {
        commands.queue(WakeIslands(vec![body_island.island_id]));
    }

    // TODO: Only remove static-static contacts and unlink from islands.
    for collider in colliders {
        remove_collider(
            collider,
            &mut contact_graph,
            &joint_graph,
            &mut constraint_graph,
            &mut islands,
            &mut body_islands,
            &mut colliding_entities_query,
            &mut event_writer,
        );
    }
}

/// Removes colliders from the [`ContactGraph`] when the given trigger is activated.
///
/// Also removes the collider from the [`CollidingEntities`] of the other entity,
/// wakes up the other body, and sends a [`CollisionEnded`] event.
fn remove_collider_on<E: Event, B: Bundle>(
    trigger: Trigger<E, B>,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: ResMut<JointGraph>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    mut islands: ResMut<PhysicsIslands>,
    mut body_islands: Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    // TODO: Change this hack to include disabled entities with `Allows<T>` for 0.17
    mut query: Query<&mut CollidingEntities, Or<(With<Disabled>, Without<Disabled>)>>,
    collider_of: Query<&ColliderOf, Or<(With<Disabled>, Without<Disabled>)>>,
    mut event_writer: EventWriter<CollisionEnded>,
    mut commands: Commands,
) {
    let entity = trigger.target();

    let body1 = collider_of
        .get(entity)
        .map(|&ColliderOf { body }| body)
        .ok();

    // If the collider was attached to a rigid body, wake its island.
    if let Some(body) = body1
        && let Ok(body_island) = body_islands.get_mut(body)
    {
        commands.queue(WakeIslands(vec![body_island.island_id]));
    }

    // Remove the collider from the contact graph.
    remove_collider(
        entity,
        &mut contact_graph,
        &joint_graph,
        &mut constraint_graph,
        &mut islands,
        &mut body_islands,
        &mut query,
        &mut event_writer,
    );
}

/// Adds the touching contacts of a body to the [`ConstraintGraph`] and [`PhysicsIslands`]
/// when the body is enabled by removing [`RigidBodyDisabled`].
fn on_body_remove_rigid_body_disabled(
    trigger: Trigger<OnAdd, BodyIslandNode>,
    body_collider_query: Query<&RigidBodyColliders>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: ResMut<JointGraph>,
    mut islands: ResMut<PhysicsIslands>,
    mut body_islands: Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    mut colliding_entities_query: Query<
        &mut CollidingEntities,
        Or<(With<Disabled>, Without<Disabled>)>,
    >,
    mut event_writer: EventWriter<CollisionEnded>,
) {
    let Ok(colliders) = body_collider_query.get(trigger.target()) else {
        return;
    };

    for collider in colliders {
        remove_collider(
            collider,
            &mut contact_graph,
            &joint_graph,
            &mut constraint_graph,
            &mut islands,
            &mut body_islands,
            &mut colliding_entities_query,
            &mut event_writer,
        );
    }
}

/// Removes the touching contacts of a body from the [`ConstraintGraph`] and [`PhysicsIslands`]
/// when the body is disabled with [`Disabled`] or [`RigidBodyDisabled`].
fn on_disable_body(
    trigger: Trigger<OnAdd, (Disabled, RigidBodyDisabled)>,
    body_collider_query: Query<&RigidBodyColliders, Or<(With<Disabled>, Without<Disabled>)>>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: Res<JointGraph>,
    mut islands: ResMut<PhysicsIslands>,
    mut body_islands: Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    mut colliding_entities_query: Query<
        &mut CollidingEntities,
        Or<(With<Disabled>, Without<Disabled>)>,
    >,
    mut event_writer: EventWriter<CollisionEnded>,
) {
    let Ok(colliders) = body_collider_query.get(trigger.target()) else {
        return;
    };

    for collider in colliders {
        remove_collider(
            collider,
            &mut contact_graph,
            &joint_graph,
            &mut constraint_graph,
            &mut islands,
            &mut body_islands,
            &mut colliding_entities_query,
            &mut event_writer,
        );
    }
}

// TODO: These are currently used just for sensors. It wouldn't be needed if sensor logic
//       was separate from normal colliders and didn't compute contact manifolds.

/// Removes the touching contacts of a collider from the [`ConstraintGraph`] and [`PhysicsIslands`]
/// when a collider becomes a [`Sensor`].
fn on_add_sensor(
    trigger: Trigger<OnAdd, Sensor>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: Res<JointGraph>,
    mut islands: ResMut<PhysicsIslands>,
    mut body_islands: Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    mut colliding_entities_query: Query<
        &mut CollidingEntities,
        Or<(With<Disabled>, Without<Disabled>)>,
    >,
    mut event_writer: EventWriter<CollisionEnded>,
) {
    remove_collider(
        trigger.target(),
        &mut contact_graph,
        &joint_graph,
        &mut constraint_graph,
        &mut islands,
        &mut body_islands,
        &mut colliding_entities_query,
        &mut event_writer,
    );
}

/// Adds the touching contacts of a collider to the [`ConstraintGraph`] and [`PhysicsIslands`]
/// when a collider stops being a [`Sensor`].
fn on_remove_sensor(
    trigger: Trigger<OnRemove, Sensor>,
    mut constraint_graph: ResMut<ConstraintGraph>,
    mut contact_graph: ResMut<ContactGraph>,
    joint_graph: ResMut<JointGraph>,
    mut islands: ResMut<PhysicsIslands>,
    mut body_islands: Query<&mut BodyIslandNode, Or<(With<Disabled>, Without<Disabled>)>>,
    mut colliding_entities_query: Query<
        &mut CollidingEntities,
        Or<(With<Disabled>, Without<Disabled>)>,
    >,
    mut event_writer: EventWriter<CollisionEnded>,
) {
    remove_collider(
        trigger.target(),
        &mut contact_graph,
        &joint_graph,
        &mut constraint_graph,
        &mut islands,
        &mut body_islands,
        &mut colliding_entities_query,
        &mut event_writer,
    );
}
