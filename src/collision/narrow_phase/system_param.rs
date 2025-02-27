use crate::{
    data_structures::{graph::EdgeIndex, pair_key::PairKey},
    dynamics::solver::{contact::ContactConstraint, ContactSoftnessCoefficients},
    prelude::*,
};
use bevy::{
    ecs::system::{SystemParam, SystemParamItem},
    prelude::*,
};
use bit_vec::BitVec;
use rayon::iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};

/// A system parameter for managing the narrow phase.
///
/// Responsibilities:
///
/// - Updates contacts for each contact pair in [`Collisions`].
/// - Sends collision events when colliders start or stop touching.
/// - Removes contact pairs from the [`Collisions`] resource when AABBs stop overlapping.
/// - Generates contact constraints for each contact pair that is touching or expected to start touching.
#[derive(SystemParam)]
#[expect(missing_docs)]
pub struct NarrowPhase<'w, 's, C: AnyCollider> {
    parallel_commands: ParallelCommands<'w, 's>,
    pub collider_query: Query<'w, 's, ColliderQuery<C>, Without<ColliderDisabled>>,
    pub colliding_entities_query: Query<'w, 's, &'static mut CollidingEntities>,
    pub body_query: Query<
        'w,
        's,
        (
            RigidBodyQueryReadOnly,
            Option<&'static CollisionMargin>,
            Option<&'static SpeculativeMargin>,
        ),
        Without<RigidBodyDisabled>,
    >,
    pub collisions: ResMut<'w, Collisions>,
    pub config: Res<'w, NarrowPhaseConfig>,
    length_unit: Res<'w, PhysicsLengthUnit>,
    // These are scaled by the length unit.
    default_speculative_margin: Local<'s, Scalar>,
    contact_tolerance: Local<'s, Scalar>,
}

impl<C: AnyCollider> NarrowPhase<'_, '_, C> {
    /// Updates the narrow phase.
    ///
    /// - Updates contacts for each contact pair in [`Collisions`].
    /// - Sends collision events when colliders start or stop touching.
    /// - Removes pairs from [`Collisions`] when AABBs stop overlapping.
    pub fn update<H: CollisionHooks>(
        &mut self,
        collision_started_event_writer: &mut EventWriter<CollisionStarted>,
        collision_ended_event_writer: &mut EventWriter<CollisionEnded>,
        delta_secs: Scalar,
        hooks: &mut H::Item<'_, '_>,
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

        // Update contacts for all contact pairs, returning a bit vector for contact state changes.
        // Set bits correspond to contact pairs that were either added or removed.
        let bit_vec = self.update_contacts::<H>(delta_secs, hooks, commands);

        // Contact pairs that should be removed.
        // TODO: This is needed because removing pairs while iterating over the bit vec can invalidate indices.
        //       With a stable mapping between contact pair indices and bits, we could remove this.
        // TODO: Pre-allocate this with some reasonable capacity?
        let mut pairs_to_remove = Vec::<EdgeIndex>::new();

        // Process contact state changes, iterating over set bits serially to maintain determinism.
        //
        // Iterating over set bits is done efficiently with the "count trailing zeros" method:
        // https://lemire.me/blog/2018/02/21/iterating-over-set-bits-quickly/
        for (i, mut bits) in bit_vec.blocks().enumerate() {
            while bits != 0 {
                let trailing_zeros = bits.trailing_zeros();
                let pair_index = EdgeIndex(i as u32 * 32 + trailing_zeros);

                let contact_pair = self
                    .collisions
                    .graph
                    .edge_weight_mut(pair_index)
                    .unwrap_or_else(|| panic!("Contact pair not found for {:?}", pair_index));

                // Three options:
                // 1. The AABBs are no longer overlapping, and the contact pair should be removed.
                // 2. The colliders started touching, and a collision started event should be sent.
                // 3. The colliders stopped touching, and a collision ended event should be sent.
                if contact_pair.aabbs_disjoint() {
                    // Send a collision ended event if the contact pair was touching.
                    let send_event = contact_pair
                        .flags
                        .contains(ContactPairFlags::TOUCHING | ContactPairFlags::CONTACT_EVENTS);
                    if send_event {
                        collision_ended_event_writer
                            .send(CollisionEnded(contact_pair.entity1, contact_pair.entity2));
                    }

                    // Remove from `CollidingEntities`.
                    Self::remove_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.entity1,
                        contact_pair.entity2,
                    );

                    // Wake up the bodies.
                    // TODO: When we have simulation islands, this will be more efficient.
                    commands.command_scope(|mut commands| {
                        commands.queue(WakeUpBody(
                            contact_pair.body_entity1.unwrap_or(contact_pair.entity1),
                        ));
                        commands.queue(WakeUpBody(
                            contact_pair.body_entity2.unwrap_or(contact_pair.entity2),
                        ));
                    });

                    // Remove the contact pair from the pair set.
                    // This is normally done by `Collisions::remove_pair`,
                    // but since we're removing edges manually, we need to do it here.
                    let pair_key =
                        PairKey::new(contact_pair.entity1.index(), contact_pair.entity2.index());
                    self.collisions.pair_set.remove(&pair_key);

                    // Queue the contact pair for removal.
                    pairs_to_remove.push(pair_index);
                } else if contact_pair.collision_started() {
                    // Send collision started event.
                    if contact_pair.events_enabled() {
                        collision_started_event_writer
                            .send(CollisionStarted(contact_pair.entity1, contact_pair.entity2));
                    }

                    // Add to `CollidingEntities`.
                    Self::add_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.entity1,
                        contact_pair.entity2,
                    );

                    debug_assert!(
                        !contact_pair.manifolds.is_empty(),
                        "Manifolds should not be empty when colliders start touching"
                    );

                    contact_pair
                        .flags
                        .set(ContactPairFlags::STARTED_TOUCHING, false);
                } else if contact_pair
                    .flags
                    .contains(ContactPairFlags::STOPPED_TOUCHING)
                {
                    // Send collision ended event.
                    if contact_pair.events_enabled() {
                        collision_ended_event_writer
                            .send(CollisionEnded(contact_pair.entity1, contact_pair.entity2));
                    }

                    // Remove from `CollidingEntities`.
                    Self::remove_colliding_entities(
                        &mut self.colliding_entities_query,
                        contact_pair.entity1,
                        contact_pair.entity2,
                    );

                    // Wake up the bodies.
                    // TODO: When we have simulation islands, this will be more efficient.
                    commands.command_scope(|mut commands| {
                        commands.queue(WakeUpBody(
                            contact_pair.body_entity1.unwrap_or(contact_pair.entity1),
                        ));
                        commands.queue(WakeUpBody(
                            contact_pair.body_entity2.unwrap_or(contact_pair.entity2),
                        ));
                    });

                    debug_assert!(
                        contact_pair.manifolds.is_empty(),
                        "Manifolds should be empty when colliders stopped touching"
                    );

                    contact_pair
                        .flags
                        .set(ContactPairFlags::STOPPED_TOUCHING, false);
                }

                // Clear the least significant set bit.
                bits &= bits - 1;
            }
        }

        // Remove the contact pairs that were marked for removal.
        for pair_index in pairs_to_remove.drain(..) {
            self.collisions.graph.remove_edge(pair_index);
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

    /// Updates contacts for all contact pairs in [`Collisions`].
    ///
    /// Returns a bit vector indicating state changes for each contact pair.
    /// Set bits correspond to contact pairs that were either added or removed,
    /// while unset bits correspond to contact pairs that were persisted.
    ///
    /// The order of contact pairs is preserved.
    fn update_contacts<H: CollisionHooks>(
        &mut self,
        delta_secs: Scalar,
        hooks: &H::Item<'_, '_>,
        par_commands: &mut ParallelCommands,
    ) -> BitVec
    where
        for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
    {
        // Initialize a bit vector to track state changes for each contact pair.
        // Set bits correspond to contact pairs that were either added or removed.
        let initial = BitVec::from_elem(self.collisions.graph.edge_count(), false);

        // Compute contacts for all contact pairs in parallel.
        // Each thread writes state changes to its own local bit vector.
        // At the end, the results are combined using bit-wise OR.
        // TODO: Store the per-thread bit vectors in some storage instead of reallocating every time.?
        let state_change_bits = self
            .collisions
            .graph
            .raw_edges_mut()
            .par_iter_mut()
            .enumerate()
            .map_with(
                initial.clone(),
                |state_change_bits, (contact_id, contacts)| {
                    let contacts = &mut contacts.weight;

                    let Ok([collider1, collider2]) = self
                        .collider_query
                        .get_many([contacts.entity1, contacts.entity2])
                    else {
                        return state_change_bits.clone();
                    };

                    // Check if the AABBs of the colliders still overlap and the contact pair is valid.
                    let overlap = collider1.aabb.intersects(&collider2.aabb);

                    if !overlap {
                        // The AABBs no longer overlap. The contact pair should be removed.
                        contacts.flags.set(ContactPairFlags::DISJOINT_AABB, true);
                        state_change_bits.set(contact_id, true);
                    } else {
                        // The AABBs overlap. Compute contacts.

                        let body1_bundle = collider1
                            .parent
                            .and_then(|p| self.body_query.get(p.get()).ok());
                        let body2_bundle = collider2
                            .parent
                            .and_then(|p| self.body_query.get(p.get()).ok());

                        // The rigid body's collision margin and speculative margin will be used
                        // if the collider doesn't have them specified.
                        let (mut lin_vel1, rb_collision_margin1, rb_speculative_margin1) =
                            body1_bundle
                                .as_ref()
                                .map(|(body, collision_margin, speculative_margin)| {
                                    (
                                        body.linear_velocity.0,
                                        *collision_margin,
                                        *speculative_margin,
                                    )
                                })
                                .unwrap_or_default();
                        let (mut lin_vel2, rb_collision_margin2, rb_speculative_margin2) =
                            body2_bundle
                                .as_ref()
                                .map(|(body, collision_margin, speculative_margin)| {
                                    (
                                        body.linear_velocity.0,
                                        *collision_margin,
                                        *speculative_margin,
                                    )
                                })
                                .unwrap_or_default();

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

                        // Compute the effective speculative margin, clamping it based on velocities and the maximum bound.
                        let effective_speculative_margin = {
                            let speculative_margin1 =
                                speculative_margin1.unwrap_or(*self.default_speculative_margin);
                            let speculative_margin2 =
                                speculative_margin2.unwrap_or(*self.default_speculative_margin);
                            let inv_delta_secs = delta_secs.recip();

                            // Clamp velocities to the maximum speculative margins.
                            if speculative_margin1 < Scalar::MAX {
                                lin_vel1 =
                                    lin_vel1.clamp_length_max(speculative_margin1 * inv_delta_secs);
                            }
                            if speculative_margin2 < Scalar::MAX {
                                lin_vel2 =
                                    lin_vel2.clamp_length_max(speculative_margin2 * inv_delta_secs);
                            }

                            // TODO: Check if AABBs intersect?

                            // Compute the effective margin based on how much the bodies
                            // are expected to move relative to each other.
                            delta_secs * (lin_vel1 - lin_vel2).length()
                        };

                        // The maximum distance at which contacts are detected.
                        // At least as large as the contact tolerance.
                        let max_contact_distance = effective_speculative_margin
                            .max(*self.contact_tolerance)
                            + collision_margin_sum;

                        let position1 = collider1.current_position();
                        let position2 = collider2.current_position();

                        let was_touching = contacts.flags.contains(ContactPairFlags::TOUCHING);

                        // Save the old manifolds for warm starting.
                        let old_manifolds = contacts.manifolds.clone();

                        // TODO: It'd be good to persist the manifolds and let Parry match contacts.
                        //       This isn't currently done because it requires using Parry's contact manifold type.
                        // Compute the contact manifolds using the effective speculative margin.
                        collider1.shape.contact_manifolds(
                            collider2.shape,
                            position1,
                            *collider1.rotation,
                            position2,
                            *collider2.rotation,
                            max_contact_distance,
                            &mut contacts.manifolds,
                        );

                        // Check if the colliders are now touching.
                        let mut touching = contacts.manifolds.iter().any(|m| !m.points.is_empty());

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
                                    manifold.match_contacts(
                                        &previous_manifold.points,
                                        distance_threshold,
                                    );
                                }
                            }
                        }

                        // TODO: For unmatched contacts, apply any leftover impulses from the previous frame.

                        if touching && !was_touching {
                            // The colliders started touching.
                            contacts.flags.set(ContactPairFlags::STARTED_TOUCHING, true);
                            state_change_bits.set(contact_id, true);
                        } else if !touching && was_touching {
                            // The colliders stopped touching.
                            contacts.flags.set(ContactPairFlags::STOPPED_TOUCHING, true);
                            state_change_bits.set(contact_id, true);
                        }
                    };

                    state_change_bits.clone()
                },
            )
            .reduce(
                || initial.clone(),
                |mut a, b| {
                    // Combine the state changes into a global bit vector using bit-wise OR.
                    a.or(&b);
                    a
                },
            );

        state_change_bits
    }

    /// Generates [`ContactConstraint`]s for the given bodies and their corresponding colliders
    /// based on the given `contacts`. The constraints are added to the `constraints` vector.
    ///
    /// The `collision_margin` can be used to add artificial thickness to the colliders,
    /// which can improve performance and stability in some cases. See [`CollisionMargin`]
    /// for more details.
    ///
    /// The `contact_softness` is used to tune the damping and stiffness of the contact constraints.
    #[allow(clippy::too_many_arguments)]
    pub fn generate_constraints(
        &self,
        contact_pair_index: usize,
        contacts: &Contacts,
        constraints: &mut Vec<ContactConstraint>,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        collider1: &ColliderQueryItem<C>,
        collider2: &ColliderQueryItem<C>,
        friction: Friction,
        restitution: Restitution,
        collision_margin: impl Into<CollisionMargin> + Copy,
        contact_softness: ContactSoftnessCoefficients,
        delta_secs: Scalar,
    ) {
        let inactive1 = body1.rb.is_static() || body1.is_sleeping;
        let inactive2 = body2.rb.is_static() || body2.is_sleeping;

        // No collision response if both bodies are static or sleeping
        // or if either of the colliders is a sensor collider.
        if (inactive1 && inactive2)
            || (collider1.is_sensor || body1.is_sensor)
            || (collider2.is_sensor || body2.is_sensor)
        {
            return;
        }

        // When an active body collides with a sleeping body, wake up the sleeping body.
        self.parallel_commands.command_scope(|mut commands| {
            if body1.is_sleeping {
                commands.queue(WakeUpBody(body1.entity));
            } else if body2.is_sleeping {
                commands.queue(WakeUpBody(body2.entity));
            }
        });

        let contact_softness = if !body1.rb.is_dynamic() || !body2.rb.is_dynamic() {
            contact_softness.non_dynamic
        } else {
            contact_softness.dynamic
        };

        // Generate contact constraints for each contact.
        for (i, contact_manifold) in contacts.manifolds.iter().enumerate() {
            let constraint = ContactConstraint::generate(
                contact_pair_index,
                i,
                contact_manifold,
                body1,
                body2,
                collider1.entity,
                collider2.entity,
                collider1.transform.copied(),
                collider2.transform.copied(),
                collision_margin,
                // TODO: Shouldn't this be the effective speculative margin?
                *self.default_speculative_margin,
                friction.dynamic_coefficient,
                restitution.coefficient,
                #[cfg(feature = "2d")]
                contact_manifold.tangent_speed,
                #[cfg(feature = "3d")]
                contact_manifold.tangent_velocity,
                contact_softness,
                self.config.match_contacts,
                delta_secs,
            );

            if !constraint.points.is_empty() {
                constraints.push(constraint);
            }
        }
    }
}
