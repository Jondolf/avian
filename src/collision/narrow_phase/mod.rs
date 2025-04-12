//! Manages contacts and generates contact constraints.
//!
//! See [`NarrowPhasePlugin`].

mod system_param;
use system_param::ContactStatusBits;
#[cfg(feature = "parallel")]
use system_param::ContactStatusBitsThreadLocal;
pub use system_param::NarrowPhase;

use core::marker::PhantomData;

use crate::{
    dynamics::solver::{ContactConstraints, ContactSoftnessCoefficients},
    prelude::*,
};
use bevy::{
    ecs::{
        entity_disabling::Disabled,
        intern::Interned,
        schedule::ScheduleLabel,
        system::{StaticSystemParam, SystemParamItem},
    },
    prelude::*,
};
use dynamics::solver::SolverDiagnostics;

/// Manages contacts and generates contact constraints.
///
/// # Overview
///
/// Before the narrow phase, the [broad phase](broad_phase) creates a contact pair in the [`ContactGraph`]
/// resource for each pair of intersecting [`ColliderAabb`]s.
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

impl<C: AnyCollider, H: CollisionHooks + 'static> Plugin for NarrowPhasePlugin<C, H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(&self, app: &mut App) {
        app.init_resource::<NarrowPhaseConfig>()
            .init_resource::<ContactGraph>()
            .init_resource::<ContactStatusBits>()
            .init_resource::<DefaultFriction>()
            .init_resource::<DefaultRestitution>();

        #[cfg(feature = "parallel")]
        app.init_resource::<ContactStatusBitsThreadLocal>();

        app.register_type::<(NarrowPhaseConfig, DefaultFriction, DefaultRestitution)>();

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
                NarrowPhaseSet::GenerateConstraints,
                NarrowPhaseSet::Last,
            )
                .chain()
                .in_set(PhysicsStepSet::NarrowPhase),
        );

        // Remove collision pairs when colliders are disabled or removed.
        app.add_observer(remove_collider_on::<OnAdd, Disabled>);
        app.add_observer(remove_collider_on::<OnAdd, ColliderDisabled>);
        app.add_observer(remove_collider_on::<OnRemove, Collider>);

        // Perform narrow phase collision detection.
        app.add_systems(
            self.schedule,
            update_narrow_phase::<C, H>
                .in_set(NarrowPhaseSet::Update)
                // Allowing ambiguities is required so that it's possible
                // to have multiple collision backends at the same time.
                .ambiguous_with_all(),
        );

        if self.generate_constraints {
            // Generate contact constraints.
            app.add_systems(
                self.schedule,
                generate_constraints::<C>
                    .in_set(NarrowPhaseSet::GenerateConstraints)
                    // Allowing ambiguities is required so that it's possible
                    // to have multiple collision backends at the same time.
                    .ambiguous_with_all(),
            );
        }
    }

    fn finish(&self, app: &mut App) {
        // Register timer and counter diagnostics for collision detection.
        app.register_physics_diagnostics::<CollisionDiagnostics>();
    }
}

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
    /// Generates [`ContactConstraint`]s and adds them to [`ContactConstraints`].
    ///
    /// [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
    GenerateConstraints,
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
    diagnostics.contact_count = narrow_phase.contact_graph.internal.edge_count() as u32;
}

fn generate_constraints<C: AnyCollider>(
    narrow_phase: NarrowPhase<C>,
    mut constraints: ResMut<ContactConstraints>,
    contact_softness: Res<ContactSoftnessCoefficients>,
    time: Res<Time>,
    mut collision_diagnostics: ResMut<CollisionDiagnostics>,
    solver_diagnostics: Option<ResMut<SolverDiagnostics>>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_seconds_adjusted();

    constraints.clear();

    // TODO: Parallelize.
    for (i, contacts) in narrow_phase.contact_graph.iter().enumerate() {
        let Ok([collider1, collider2]) = narrow_phase
            .collider_query
            .get_many([contacts.entity1, contacts.entity2])
        else {
            continue;
        };

        let body1_bundle = collider1
            .rigid_body
            .and_then(|&ColliderOf { rigid_body }| narrow_phase.body_query.get(rigid_body).ok());
        let body2_bundle = collider2
            .rigid_body
            .and_then(|&ColliderOf { rigid_body }| narrow_phase.body_query.get(rigid_body).ok());
        if let (Some((body1, rb_collision_margin1)), Some((body2, rb_collision_margin2))) = (
            body1_bundle.map(|(body, rb_collision_margin1, _)| (body, rb_collision_margin1)),
            body2_bundle.map(|(body, rb_collision_margin2, _)| (body, rb_collision_margin2)),
        ) {
            // At least one of the bodies must be dynamic for contact constraints
            // to be generated.
            if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                continue;
            }

            // Use the collider's own collision margin if specified, and fall back to the body's
            // collision margin.
            //
            // The collision margin adds artificial thickness to colliders for performance
            // and stability. See the `CollisionMargin` documentation for more details.
            let collision_margin1 = collider1
                .collision_margin
                .or(rb_collision_margin1)
                .map_or(0.0, |margin| margin.0);
            let collision_margin2 = collider2
                .collision_margin
                .or(rb_collision_margin2)
                .map_or(0.0, |margin| margin.0);
            let collision_margin_sum = collision_margin1 + collision_margin2;

            // Generate contact constraints for the computed contacts
            // and add them to `constraints`.
            narrow_phase.generate_constraints(
                i,
                contacts,
                &mut constraints,
                &body1,
                &body2,
                &collider1,
                &collider2,
                collision_margin_sum,
                *contact_softness,
                delta_secs,
            );
        }
    }

    collision_diagnostics.generate_constraints = start.elapsed();

    if let Some(mut solver_diagnostics) = solver_diagnostics {
        solver_diagnostics.contact_constraint_count = constraints.len() as u32;
    }
}

fn remove_collider_on<E: Event, C: Component>(
    trigger: Trigger<E, C>,
    mut contact_graph: ResMut<ContactGraph>,
    mut query: Query<&mut CollidingEntities>,
    mut event_writer: EventWriter<CollisionEnded>,
    mut commands: Commands,
) {
    let entity = trigger.target();

    // Remove the collider from the contact graph.
    contact_graph.remove_collider_with(entity, |contact_pair| {
        // If the contact pair was not touching, we don't need to do anything.
        if !contact_pair.flags.contains(ContactPairFlags::TOUCHING) {
            return;
        }

        // Send a collision ended event.
        if contact_pair
            .flags
            .contains(ContactPairFlags::CONTACT_EVENTS)
        {
            event_writer.write(CollisionEnded(contact_pair.entity1, contact_pair.entity2));
        }

        // Remove the entity from the `CollidingEntities` of the other entity.
        let other_entity = if contact_pair.entity1 == entity {
            contact_pair.entity2
        } else {
            contact_pair.entity1
        };
        if let Ok(mut colliding_entities) = query.get_mut(other_entity) {
            colliding_entities.remove(&entity);
        }

        // Wake up the other body.
        commands.queue(WakeUpBody(other_entity));
    });
}
