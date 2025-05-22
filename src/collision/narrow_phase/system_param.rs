#[cfg(feature = "parallel")]
use core::cell::RefCell;

use crate::{
    collision::{
        collider::ColliderQuery,
        contact_types::{ContactEdgeFlags, ContactId},
    },
    data_structures::{bit_vec::BitVec, pair_key::PairKey},
    dynamics::solver::{constraint_graph::ConstraintGraph, ContactSoftnessCoefficients},
    prelude::*,
};
use bevy::{
    ecs::system::{SystemParam, SystemParamItem},
    prelude::*,
};
#[cfg(feature = "parallel")]
use thread_local::ThreadLocal;

/// A system parameter for managing the narrow phase.
///
/// Responsibilities:
///
/// - Updates contacts for each contact pair in the [`ContactGraph`].
/// - Sends collision events when colliders start or stop touching.
/// - Removes contact pairs from the [`ContactGraph`] when AABBs stop overlapping.
/// - Generates contact constraints for each contact pair that is touching or expected to start touching.
#[derive(SystemParam)]
#[expect(missing_docs)]
pub struct NarrowPhase<'w, 's, C: AnyCollider> {
    pub collider_query: Query<'w, 's, ColliderQuery<C>, Without<ColliderDisabled>>,
    pub colliding_entities_query: Query<'w, 's, &'static mut CollidingEntities>,
    pub body_query: Query<
        'w,
        's,
        (
            // TODO: Only query for linear velocity
            RigidBodyQueryReadOnly,
            Option<&'static CollisionMargin>,
            Option<&'static SpeculativeMargin>,
        ),
        Without<RigidBodyDisabled>,
    >,
    pub contact_graph: ResMut<'w, ContactGraph>,
    pub constraint_graph: ResMut<'w, ConstraintGraph>,
    contact_status_bits: ResMut<'w, ContactStatusBits>,
    #[cfg(feature = "parallel")]
    thread_local_contact_status_bits: ResMut<'w, ThreadLocalContactStatusBits>,
    pub config: Res<'w, NarrowPhaseConfig>,
    default_friction: Res<'w, DefaultFriction>,
    default_restitution: Res<'w, DefaultRestitution>,
    contact_softness: Res<'w, ContactSoftnessCoefficients>,
    length_unit: Res<'w, PhysicsLengthUnit>,
    // These are scaled by the length unit.
    default_speculative_margin: Local<'s, Scalar>,
    contact_tolerance: Local<'s, Scalar>,
}

/// A bit vector for tracking contact status changes.
/// Set bits correspond to contact pairs that were either added or removed.
#[derive(Resource, Default, Deref, DerefMut)]
pub(super) struct ContactStatusBits(pub BitVec);

// TODO: We could just combine all the bit vectors into the first one
//       instead of having a separate `ContactStatusBits` resource.
/// A thread-local bit vector for tracking contact status changes.
/// Set bits correspond to contact pairs that were either added or removed.
///
/// The thread-local bit vectors are combined with the global [`ContactStatusBits`].
#[cfg(feature = "parallel")]
#[derive(Resource, Default, Deref, DerefMut)]
pub(super) struct ThreadLocalContactStatusBits(pub ThreadLocal<RefCell<BitVec>>);

impl<C: AnyCollider> NarrowPhase<'_, '_, C> {
    /// Updates the narrow phase.
    ///
    /// - Updates contacts for each contact pair in the [`ContactGraph`].
    /// - Sends collision events when colliders start or stop touching.
    /// - Removes pairs from the [`ContactGraph`] when AABBs stop overlapping.
    pub fn update<H: CollisionHooks>(
        &mut self,
        collision_started_event_writer: &mut EventWriter<CollisionStarted>,
        collision_ended_event_writer: &mut EventWriter<CollisionEnded>,
        delta_secs: Scalar,
        hooks: &SystemParamItem<H>,
        context: &SystemParamItem<C::Context>,
        commands: &mut ParallelCommands,
    ) where
        for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
    {
        // Cache default margins scaled by the length unit.
        if self.config.is_changed() {
            *self.default_speculative_margin =
                self.length_unit.0 * self.config.default_speculative_margin;
            *self.contact_tolerance = self.length_unit.0 * self.config.contact_tolerance;
        }

        // Update contacts for all contact pairs.
        self.update_contacts::<H>(delta_secs, hooks, context, commands);

        // Process contact status changes, iterating over set bits serially to maintain determinism.
        //
        // Iterating over set bits is done efficiently with the "count trailing zeros" method:
        // https://lemire.me/blog/2018/02/21/iterating-over-set-bits-quickly/
        for (i, mut bits) in self.contact_status_bits.blocks().enumerate() {
            while bits != 0 {
                let trailing_zeros = bits.trailing_zeros();
                let contact_id = ContactId(i as u32 * 64 + trailing_zeros);

                let contact_edge = self
                    .contact_graph
                    .get_mut_by_id(contact_id)
                    .unwrap_or_else(|| panic!("Contact edge not found for {contact_id:?}"));

                let contact_pair = self
                    .constraint_graph
                    .get_contact_pair_mut(contact_edge.color_index, contact_edge.local_index)
                    .unwrap_or_else(|| panic!("Contact pair not found for {contact_id:?}"));

                // Three options:
                // 1. The AABBs are no longer overlapping, and the contact pair should be removed.
                // 2. The colliders started touching, and a collision started event should be sent.
                // 3. The colliders stopped touching, and a collision ended event should be sent.
                if contact_pair.aabbs_disjoint() {
                    // Send a collision ended event if the contact pair was touching.
                    let send_event = contact_edge
                        .flags
                        .contains(ContactEdgeFlags::TOUCHING | ContactEdgeFlags::CONTACT_EVENTS);
                    if send_event {
                        collision_ended_event_writer.write(CollisionEnded(
                            contact_pair.collider1,
                            contact_pair.collider2,
                        ));
                    }

                    // Remove from `CollidingEntities`.
                    Self::remove_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.collider1,
                        contact_pair.collider2,
                    );

                    // Wake up the bodies.
                    // TODO: When we have simulation islands, this will be more efficient.
                    commands.command_scope(|mut commands| {
                        commands.queue(WakeUpBody(
                            contact_pair.body1.unwrap_or(contact_pair.collider1),
                        ));
                        commands.queue(WakeUpBody(
                            contact_pair.body2.unwrap_or(contact_pair.collider2),
                        ));
                    });

                    // Remove the contact pair from the contact graph.
                    let pair_key = PairKey::new(
                        contact_pair.collider1.index(),
                        contact_pair.collider2.index(),
                    );
                    if contact_edge.color_index != usize::MAX {
                        // The contact is an active constraint.
                        // Remove the contact from the constraint graph.
                        if let (Some(body1), Some(body2)) = (contact_pair.body1, contact_pair.body2)
                        {
                            let color_index = contact_edge.color_index;
                            let local_index = contact_edge.local_index;
                            self.constraint_graph.remove_touching_contact(
                                &mut self.contact_graph,
                                body1,
                                body2,
                                color_index,
                                local_index,
                            );
                        }
                    } else {
                        // The contact is non-touching or sleeping.
                        // Remove the contact from the constraint graph.
                        // TODO: debug_assert here
                        let local_index = contact_edge.local_index;
                        self.constraint_graph
                            .remove_non_touching_contact(&mut self.contact_graph, local_index);
                    }
                    self.contact_graph.remove_pair_by_id(&pair_key, contact_id);
                } else if contact_pair.collision_started() {
                    // Send collision started event.
                    if contact_edge.events_enabled() {
                        collision_started_event_writer.write(CollisionStarted(
                            contact_pair.collider1,
                            contact_pair.collider2,
                        ));
                    }

                    // Add to `CollidingEntities`.
                    Self::add_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.collider1,
                        contact_pair.collider2,
                    );

                    debug_assert!(
                        !contact_pair.manifolds.is_empty(),
                        "Manifolds should not be empty when colliders start touching"
                    );

                    contact_edge.flags.set(ContactEdgeFlags::TOUCHING, true);
                    contact_pair
                        .flags
                        .set(ContactPairFlags::STARTED_TOUCHING, false);

                    // TODO: Can we avoid cloning and just take the non-touching contact?
                    let contact_pair = contact_pair.clone();
                    let local_index = contact_edge.local_index;
                    self.constraint_graph
                        .add_touching_contact(contact_edge, contact_pair);
                    self.constraint_graph
                        .remove_non_touching_contact(&mut self.contact_graph, local_index);
                } else if contact_pair
                    .flags
                    .contains(ContactPairFlags::STOPPED_TOUCHING)
                {
                    // Send collision ended event.
                    if contact_edge.events_enabled() {
                        collision_ended_event_writer.write(CollisionEnded(
                            contact_pair.collider1,
                            contact_pair.collider2,
                        ));
                    }

                    // Remove from `CollidingEntities`.
                    Self::remove_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.collider1,
                        contact_pair.collider2,
                    );

                    // Wake up the bodies.
                    // TODO: When we have simulation islands, this will be more efficient.
                    commands.command_scope(|mut commands| {
                        commands.queue(WakeUpBody(
                            contact_pair.body1.unwrap_or(contact_pair.collider1),
                        ));
                        commands.queue(WakeUpBody(
                            contact_pair.body2.unwrap_or(contact_pair.collider2),
                        ));
                    });

                    debug_assert!(
                        contact_pair.manifolds.is_empty(),
                        "Manifolds should be empty when colliders stopped touching"
                    );

                    contact_edge.flags.set(ContactEdgeFlags::TOUCHING, false);
                    contact_pair
                        .flags
                        .set(ContactPairFlags::STOPPED_TOUCHING, false);

                    if let (Some(body1), Some(body2)) = (contact_pair.body1, contact_pair.body2) {
                        // TODO: Can we avoid cloning and just take the touching contact?
                        let contact_pair = contact_pair.clone();
                        let color_index = contact_edge.color_index;
                        let local_index = contact_edge.local_index;
                        self.constraint_graph
                            .add_non_touching_contact(contact_edge, contact_pair);
                        self.constraint_graph.remove_touching_contact(
                            &mut self.contact_graph,
                            body1,
                            body2,
                            color_index,
                            local_index,
                        );
                    }
                }

                // Clear the least significant set bit.
                bits &= bits - 1;
            }
        }
    }

    /// Adds the colliding entities to their respective [`CollidingEntities`] components.
    fn add_colliding_entities(
        query: &mut Query<&mut CollidingEntities>,
        entity1: Entity,
        entity2: Entity,
    ) {
        if let Ok(mut colliding_entities1) = query.get_mut(entity1) {
            colliding_entities1.insert(entity2);
        }
        if let Ok(mut colliding_entities2) = query.get_mut(entity2) {
            colliding_entities2.insert(entity1);
        }
    }

    /// Removes the colliding entities from their respective [`CollidingEntities`] components.
    fn remove_colliding_entities(
        query: &mut Query<&mut CollidingEntities>,
        entity1: Entity,
        entity2: Entity,
    ) {
        if let Ok(mut colliding_entities1) = query.get_mut(entity1) {
            colliding_entities1.remove(&entity2);
        }
        if let Ok(mut colliding_entities2) = query.get_mut(entity2) {
            colliding_entities2.remove(&entity1);
        }
    }

    /// Updates contacts for all contact pairs in the [`ContactGraph`].
    ///
    /// Also updates the [`ContactStatusBits`] resource to track status changes for each contact pair.
    /// Set bits correspond to contact pairs that were either added or removed,
    /// while unset bits correspond to contact pairs that were persisted.
    ///
    /// The order of contact pairs is preserved.
    fn update_contacts<H: CollisionHooks>(
        &mut self,
        delta_secs: Scalar,
        hooks: &SystemParamItem<H>,
        collider_context: &SystemParamItem<C::Context>,
        par_commands: &mut ParallelCommands,
    ) where
        for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
    {
        // Contact bit vecs must be sized based on the full contact capacity,
        // not the number of active contact pairs, because pair indices
        // are unstable and can be invalidated when pairs are removed.
        let bit_count = self.contact_graph.edges.raw_edges().len();

        // Clear the bit vector used to track status changes for each contact pair.
        self.contact_status_bits.set_bit_count_and_clear(bit_count);

        #[cfg(feature = "parallel")]
        self.thread_local_contact_status_bits
            .iter_mut()
            .for_each(|context| {
                let bit_vec_mut = &mut context.borrow_mut();
                bit_vec_mut.set_bit_count_and_clear(bit_count);
            });

        // Collect mutable references to contacts into a single vector for easier parallel-for.
        // TODO: Don't allocate this vector from scratch every time.
        let mut contacts =
            Vec::<&mut ContactPair>::with_capacity(self.contact_graph.edges.edge_count());

        let ConstraintGraph {
            colors: graph_colors,
            non_touching_contacts,
            ..
        } = &mut *self.constraint_graph;

        // Add contacts from each color in the constraint graph.
        for color in graph_colors.iter_mut() {
            contacts.extend(color.contacts.iter_mut());
        }

        // Add non-touching contacts from the constraint graph.
        // TODO: Keep active and disabled contacts separate?
        contacts.extend(non_touching_contacts.iter_mut());

        // Compute contacts for all contact pairs in parallel or serially
        // based on the `parallel` feature.
        //
        // Constraints are also generated for each contact pair that is touching
        // or expected to start touching.
        //
        // If parallelism is used, status changes are written to thread-local bit vectors,
        // where set bits correspond to contact pairs that were added or removed.
        // At the end, the bit vectors are combined using bit-wise OR.
        //
        // If parallelism is not used, status changes are instead written
        // directly to the global bit vector.
        //
        // TODO: An alternative to thread-local bit vectors could be to have one larger bit vector
        //       and to chunk it into smaller bit vectors for each thread. Might not be any faster though.
        crate::utils::par_for_each!(contacts, |_i, contacts| {
            let contact_id = contacts.contact_id.0 as usize;

            #[cfg(not(feature = "parallel"))]
            let status_change_bits = &mut self.contact_status_bits;

            // TODO: Move this out of the chunk iteration? Requires refactoring `par_for_each!`.
            #[cfg(feature = "parallel")]
            // Get the thread-local narrow phase context.
            let mut thread_context = self
                .thread_local_contact_status_bits
                .get_or(|| {
                    // No thread-local bit vector exists for this thread yet.
                    // Create a new one with the same capacity as the global bit vector.
                    let mut contact_status_bits = BitVec::new(bit_count);
                    contact_status_bits.set_bit_count_and_clear(bit_count);
                    RefCell::new(contact_status_bits)
                })
                .borrow_mut();
            #[cfg(feature = "parallel")]
            let status_change_bits = &mut *thread_context;

            // Get the colliders for the contact pair.
            let Ok([collider1, collider2]) = self
                .collider_query
                .get_many([contacts.collider1, contacts.collider2])
            else {
                return;
            };

            // Check if the AABBs of the colliders still overlap and the contact pair is valid.
            let overlap = collider1.aabb.intersects(&collider2.aabb);

            // Also check if the collision layers are still compatible and the contact pair is valid.
            // TODO: Ideally, we would have fine-grained change detection for `CollisionLayers`
            //       rather than checking it for every pair here.
            if !overlap || !collider1.layers.interacts_with(*collider2.layers) {
                // The AABBs no longer overlap. The contact pair should be removed.
                contacts.flags.set(ContactPairFlags::DISJOINT_AABB, true);
                status_change_bits.set(contact_id);
            } else {
                // The AABBs overlap. Compute contacts.

                let body1_bundle = collider1
                    .body()
                    .and_then(|body| self.body_query.get(body).ok());
                let body2_bundle = collider2
                    .body()
                    .and_then(|body| self.body_query.get(body).ok());

                // The rigid body's friction, restitution, collision margin, and speculative margin
                // will be used if the collider doesn't have them specified.
                let (
                    is_static1,
                    mut lin_vel1,
                    rb_friction1,
                    rb_collision_margin1,
                    rb_speculative_margin1,
                ) = body1_bundle
                    .as_ref()
                    .map(|(body, collision_margin, speculative_margin)| {
                        (
                            body.rb.is_static(),
                            body.linear_velocity.0,
                            body.friction,
                            *collision_margin,
                            *speculative_margin,
                        )
                    })
                    .unwrap_or_default();
                let (
                    is_static2,
                    mut lin_vel2,
                    rb_friction2,
                    rb_collision_margin2,
                    rb_speculative_margin2,
                ) = body2_bundle
                    .as_ref()
                    .map(|(body, collision_margin, speculative_margin)| {
                        (
                            body.rb.is_static(),
                            body.linear_velocity.0,
                            body.friction,
                            *collision_margin,
                            *speculative_margin,
                        )
                    })
                    .unwrap_or_default();

                // Store these to avoid having to query for the bodies
                // when processing status changes for the constraint grapn.
                contacts.flags.set(ContactPairFlags::STATIC1, is_static1);
                contacts.flags.set(ContactPairFlags::STATIC2, is_static2);

                // Get combined friction and restitution coefficients of the colliders
                // or the bodies they are attached to. Fall back to the global defaults.
                let friction = collider1
                    .friction
                    .or(rb_friction1)
                    .copied()
                    .unwrap_or(self.default_friction.0)
                    .combine(
                        collider2
                            .friction
                            .or(rb_friction2)
                            .copied()
                            .unwrap_or(self.default_friction.0),
                    )
                    .dynamic_coefficient;
                let restitution = collider1
                    .restitution
                    .copied()
                    .unwrap_or(self.default_restitution.0)
                    .combine(
                        collider2
                            .restitution
                            .copied()
                            .unwrap_or(self.default_restitution.0),
                    )
                    .coefficient;

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

                // Use the collider's own speculative margin if specified, and fall back to the body's
                // speculative margin.
                //
                // The speculative margin is used to predict contacts that might happen during the frame.
                // This is used for speculative collision. See the CCD and `SpeculativeMargin` documentation
                // for more details.
                let speculative_margin1 = collider1
                    .speculative_margin
                    .map_or(rb_speculative_margin1.map(|margin| margin.0), |margin| {
                        Some(margin.0)
                    });
                let speculative_margin2 = collider2
                    .speculative_margin
                    .map_or(rb_speculative_margin2.map(|margin| margin.0), |margin| {
                        Some(margin.0)
                    });

                let relative_linear_velocity: Vector;

                // Compute the effective speculative margin, clamping it based on velocities and the maximum bound.
                let effective_speculative_margin = {
                    let speculative_margin1 =
                        speculative_margin1.unwrap_or(*self.default_speculative_margin);
                    let speculative_margin2 =
                        speculative_margin2.unwrap_or(*self.default_speculative_margin);
                    let inv_delta_secs = delta_secs.recip();

                    // Clamp velocities to the maximum speculative margins.
                    if speculative_margin1 < Scalar::MAX {
                        lin_vel1 = lin_vel1.clamp_length_max(speculative_margin1 * inv_delta_secs);
                    }
                    if speculative_margin2 < Scalar::MAX {
                        lin_vel2 = lin_vel2.clamp_length_max(speculative_margin2 * inv_delta_secs);
                    }

                    // Compute the effective margin based on how much the bodies
                    // are expected to move relative to each other.
                    relative_linear_velocity = lin_vel2 - lin_vel1;
                    delta_secs * relative_linear_velocity.length()
                };

                // The maximum distance at which contacts are detected.
                // At least as large as the contact tolerance.
                let max_contact_distance = effective_speculative_margin
                    .max(*self.contact_tolerance)
                    + collision_margin_sum;

                let was_touching = contacts.flags.contains(ContactPairFlags::TOUCHING);

                // Save the old manifolds for warm starting.
                let old_manifolds = contacts.manifolds.clone();

                // TODO: It'd be good to persist the manifolds and let Parry match contacts.
                //       This isn't currently done because it requires using Parry's contact manifold type.
                // Compute the contact manifolds using the effective speculative margin.
                let context = ContactManifoldContext::new(
                    collider1.entity,
                    collider2.entity,
                    collider_context,
                );
                collider1.shape.contact_manifolds_with_context(
                    collider2.shape,
                    collider1.position.0,
                    *collider1.rotation,
                    collider2.position.0,
                    *collider2.rotation,
                    max_contact_distance,
                    &mut contacts.manifolds,
                    context,
                );

                // Set the initial surface properties.
                // TODO: This could be done in `contact_manifolds` to avoid the extra iteration.
                contacts.manifolds.iter_mut().for_each(|manifold| {
                    manifold.friction = friction;
                    manifold.restitution = restitution;
                    #[cfg(feature = "2d")]
                    {
                        manifold.tangent_speed = 0.0;
                    }
                    #[cfg(feature = "3d")]
                    {
                        manifold.tangent_velocity = Vector::ZERO;
                    }
                });

                // Check if the colliders are now touching.
                let mut touching = !contacts.manifolds.is_empty();

                if touching && contacts.flags.contains(ContactPairFlags::MODIFY_CONTACTS) {
                    par_commands.command_scope(|mut commands| {
                        touching = hooks.modify_contacts(contacts, &mut commands);
                    });
                    if !touching {
                        contacts.manifolds.clear();
                    }
                }

                contacts.flags.set(ContactPairFlags::TOUCHING, touching);

                // TODO: This condition is pretty arbitrary, mainly to skip dense trimeshes.
                //       If we let Parry handle contact matching, this wouldn't be needed.
                if contacts.manifolds.len() <= 4 && self.config.match_contacts {
                    // TODO: Cache this?
                    let distance_threshold = 0.1 * self.length_unit.0;

                    for manifold in contacts.manifolds.iter_mut() {
                        for previous_manifold in old_manifolds.iter() {
                            manifold.match_contacts(&previous_manifold.points, distance_threshold);
                        }
                    }
                }

                // TODO: For unmatched contacts, apply any leftover impulses from the previous frame.

                if touching && !was_touching {
                    // The colliders started touching.
                    contacts.flags.set(ContactPairFlags::STARTED_TOUCHING, true);
                    status_change_bits.set(contact_id);
                } else if !touching && was_touching {
                    // The colliders stopped touching.
                    contacts.flags.set(ContactPairFlags::STOPPED_TOUCHING, true);
                    status_change_bits.set(contact_id);
                }
            };
        });

        #[cfg(feature = "parallel")]
        {
            // Combine the thread-local bit vectors serially using bit-wise OR.
            self.thread_local_contact_status_bits
                .iter_mut()
                .for_each(|context| {
                    let contact_status_bits = context.borrow();
                    self.contact_status_bits.or(&contact_status_bits);
                });
        }
    }
}
