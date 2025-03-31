//! Constraints and other types used for solving contacts.

mod normal_part;
mod tangent_part;

pub use normal_part::ContactNormalPart;
pub use tangent_part::ContactTangentPart;

use crate::{dynamics::solver::softness_parameters::SoftnessCoefficients, prelude::*};
use bevy::{
    ecs::entity::{Entity, EntityMapper, MapEntities},
    reflect::Reflect,
    utils::default,
};

// TODO: One-body constraint version
/// Data and logic for solving a single contact point for a [`ContactConstraint`].
#[derive(Clone, Debug, PartialEq, Reflect)]
pub struct ContactConstraintPoint {
    /// The normal part of the contact constraint.
    pub normal_part: ContactNormalPart,

    /// The tangential friction part of the contact constraint.
    ///
    /// `None` if the coefficient of friction is zero.
    pub tangent_part: Option<ContactTangentPart>,

    // TODO: This could probably just be a boolean?
    /// The largest incremental contact impulse magnitude along the contact normal during this frame.
    ///
    /// This is used for determining whether restitution should be applied.
    pub max_normal_impulse: Scalar,

    // TODO: If a rotation delta was used for bodies, these local anchors could be removed.
    /// The local contact point relative to the center of mass of the first body.
    pub local_anchor1: Vector,

    /// The local contact point relative to the center of mass of the second body.
    pub local_anchor2: Vector,

    /// The world-space contact point relative to the center of mass of the first body.
    pub anchor1: Vector,

    /// The world-space contact point relative to the center of mass of the second body.
    pub anchor2: Vector,

    /// The relative velocity of the bodies along the normal at the contact point.
    pub normal_speed: Scalar,

    /// The pre-solve separation distance between the bodies.
    ///
    /// A negative separation indicates penetration.
    pub initial_separation: Scalar,
}

/// A contact constraint used for resolving inter-penetration between two bodies.
///
/// Each constraint corresponds to a [`ContactManifold`] indicated by the `manifold_index`.
/// The contact points are stored in `points`, and they all share the same `normal`.
#[derive(Clone, Debug, PartialEq, Reflect)]
pub struct ContactConstraint {
    /// The first entity in the contact.
    pub entity1: Entity,
    /// The second entity in the contact.
    pub entity2: Entity,
    /// The combined coefficient of dynamic [friction](Friction) of the bodies.
    pub friction: Scalar,
    /// The combined coefficient of [restitution](Restitution) of the bodies.
    pub restitution: Scalar,
    /// The desired relative linear speed of the bodies along the surface,
    /// expressed in world space as `tangent_speed2 - tangent_speed1`.
    ///
    /// Defaults to zero. If set to a non-zero value, this can be used to simulate effects
    /// such as conveyor belts.
    #[cfg(feature = "2d")]
    pub tangent_speed: Scalar,
    /// The desired relative linear velocity of the bodies along the surface,
    /// expressed in world space as `tangent_velocity2 - tangent_velocity1`.
    ///
    /// Defaults to zero. If set to a non-zero value, this can be used to simulate effects
    /// such as conveyor belts.
    #[cfg(feature = "3d")]
    pub tangent_velocity: Vector,
    /// The world-space contact normal shared by all points in the contact manifold.
    pub normal: Vector,
    /// The contact points in the manifold. Each point shares the same `normal`.
    pub points: Vec<ContactConstraintPoint>,
    /// The index of the contact pair in the [`ContactGraph`].
    pub contact_pair_index: usize,
    /// The index of the [`ContactManifold`] in the [`ContactPair`] stored for the two bodies.
    pub manifold_index: usize,
}

impl ContactConstraint {
    /// Generates a new [`ContactConstraint`] for the given bodies based on a [`ContactManifold`].
    #[allow(clippy::too_many_arguments)]
    pub fn generate(
        contact_pair_index: usize,
        manifold_index: usize,
        manifold: &ContactManifold,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        collider_transform1: Option<ColliderTransform>,
        collider_transform2: Option<ColliderTransform>,
        collision_margin: impl Into<CollisionMargin>,
        speculative_margin: impl Into<SpeculativeMargin>,
        friction: Scalar,
        restitution: Scalar,
        #[cfg(feature = "2d")] tangent_speed: Scalar,
        #[cfg(feature = "3d")] tangent_velocity: Vector,
        softness: SoftnessCoefficients,
        warm_start: bool,
        delta_secs: Scalar,
    ) -> Self {
        let collision_margin: Scalar = collision_margin.into().0;
        let speculative_margin: Scalar = speculative_margin.into().0;

        // TODO: Cache these?
        // TODO: How should we properly take the locked axes into account for the mass here?
        let inverse_mass_sum = body1.mass().inverse() + body2.mass().inverse();
        let i1 = body1.effective_global_angular_inertia();
        let i2 = body2.effective_global_angular_inertia();

        let mut constraint = Self {
            entity1: body1.entity,
            entity2: body2.entity,
            friction,
            restitution,
            #[cfg(feature = "2d")]
            tangent_speed,
            #[cfg(feature = "3d")]
            tangent_velocity,
            normal: manifold.normal,
            points: Vec::with_capacity(manifold.points.len()),
            contact_pair_index,
            manifold_index,
        };

        let tangents =
            constraint.tangent_directions(body1.linear_velocity.0, body2.linear_velocity.0);

        for mut contact in manifold.points.iter().copied() {
            // Transform contact points from collider-space to body-space.
            if let Some(transform) = collider_transform1 {
                contact.local_point1 =
                    transform.rotation * contact.local_point1 + transform.translation;
            }
            if let Some(transform) = collider_transform2 {
                contact.local_point2 =
                    transform.rotation * contact.local_point2 + transform.translation;
            }

            contact.penetration += collision_margin;

            let effective_distance = -contact.penetration;

            let local_anchor1 = contact.local_point1 - body1.center_of_mass.0;
            let local_anchor2 = contact.local_point2 - body2.center_of_mass.0;

            // Store fixed world-space anchors.
            // This improves rolling behavior for shapes like balls and capsules.
            let r1 = *body1.rotation * local_anchor1;
            let r2 = *body2.rotation * local_anchor2;

            // Relative velocity at the contact point.
            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);

            // Keep the contact if (1) the separation distance is below the required threshold,
            // or if (2) the bodies are expected to come into contact within the next frame.
            let normal_speed = relative_velocity.dot(constraint.normal);
            let keep_contact = effective_distance < speculative_margin || {
                let delta_distance = normal_speed * delta_secs;
                effective_distance + delta_distance < speculative_margin
            };

            if !keep_contact {
                continue;
            }

            let point = ContactConstraintPoint {
                // TODO: Apply warm starting scale here instead of in `warm_start`?
                normal_part: ContactNormalPart::generate(
                    inverse_mass_sum,
                    i1,
                    i2,
                    r1,
                    r2,
                    constraint.normal,
                    warm_start.then_some(contact.normal_impulse),
                    softness,
                ),
                // There should only be a friction part if the coefficient of friction is non-negative.
                tangent_part: (friction > 0.0).then_some(ContactTangentPart::generate(
                    inverse_mass_sum,
                    i1,
                    i2,
                    r1,
                    r2,
                    tangents,
                    warm_start.then_some(contact.tangent_impulse),
                )),
                max_normal_impulse: 0.0,
                local_anchor1,
                local_anchor2,
                anchor1: r1,
                anchor2: r2,
                normal_speed,
                initial_separation: -contact.penetration - (r2 - r1).dot(constraint.normal),
            };

            constraint.points.push(point);
        }

        constraint
    }

    /// Warm starts the contact constraint by applying the impulses from the previous frame or substep.
    pub fn warm_start(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        normal: Vector,
        tangent_directions: [Vector; DIM - 1],
        warm_start_coefficient: Scalar,
    ) {
        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_global_angular_inertia().inverse();
        let inv_inertia2 = body2.effective_global_angular_inertia().inverse();

        for point in self.points.iter() {
            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            let tangent_impulse = point
                .tangent_part
                .as_ref()
                .map_or(default(), |part| part.impulse);

            #[cfg(feature = "2d")]
            let p = warm_start_coefficient
                * (point.normal_part.impulse * normal + tangent_impulse * tangent_directions[0]);
            #[cfg(feature = "3d")]
            let p = warm_start_coefficient
                * (point.normal_part.impulse * normal
                    + tangent_impulse.x * tangent_directions[0]
                    + tangent_impulse.y * tangent_directions[1]);

            if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
                body1.linear_velocity.0 -= p * inv_mass1;
                body1.angular_velocity.0 -= inv_inertia1 * cross(r1, p);
            }
            if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
                body2.linear_velocity.0 += p * inv_mass2;
                body2.angular_velocity.0 += inv_inertia2 * cross(r2, p);
            }
        }
    }

    /// Solves the [`ContactConstraint`], applying an impulse to the given bodies.
    pub fn solve(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_secs: Scalar,
        use_bias: bool,
        max_overlap_solve_speed: Scalar,
    ) {
        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_global_angular_inertia().inverse();
        let inv_inertia2 = body2.effective_global_angular_inertia().inverse();

        let delta_translation = body2.accumulated_translation.0 - body1.accumulated_translation.0;

        // Normal impulses
        for point in self.points.iter_mut() {
            let r1 = *body1.rotation * point.local_anchor1;
            let r2 = *body2.rotation * point.local_anchor2;

            // TODO: Consider rotation delta for anchors
            let delta_separation = delta_translation + (r2 - r1);
            let separation = delta_separation.dot(self.normal) + point.initial_separation;

            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            // Relative velocity at contact point
            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);

            // Compute the incremental impulse. The clamping and impulse accumulation is handled by the method.
            let impulse_magnitude = point.normal_part.solve_impulse(
                separation,
                relative_velocity,
                self.normal,
                use_bias,
                max_overlap_solve_speed,
                delta_secs,
            );

            // Store the maximum impulse for restitution.
            point.max_normal_impulse = impulse_magnitude.max(point.max_normal_impulse);

            if impulse_magnitude == 0.0 {
                continue;
            }

            let impulse = impulse_magnitude * self.normal;

            // Apply the impulse.
            if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
                body1.linear_velocity.0 -= impulse * inv_mass1;
                body1.angular_velocity.0 -= inv_inertia1 * cross(r1, impulse);
            }
            if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
                body2.linear_velocity.0 += impulse * inv_mass2;
                body2.angular_velocity.0 += inv_inertia2 * cross(r2, impulse);
            }
        }

        let tangent_directions =
            self.tangent_directions(body1.linear_velocity.0, body2.linear_velocity.0);

        // Friction
        for point in self.points.iter_mut() {
            let Some(ref mut friction_part) = point.tangent_part else {
                continue;
            };

            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            // Relative velocity at contact point
            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);

            // Compute the incremental impulse. The clamping and impulse accumulation is handled by the method.
            let impulse = friction_part.solve_impulse(
                tangent_directions,
                relative_velocity,
                #[cfg(feature = "2d")]
                self.tangent_speed,
                #[cfg(feature = "3d")]
                self.tangent_velocity,
                self.friction,
                point.normal_part.impulse,
            );

            // Apply the impulse.
            if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
                body1.linear_velocity.0 -= impulse * inv_mass1;
                body1.angular_velocity.0 -= inv_inertia1 * cross(r1, impulse);
            }
            if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
                body2.linear_velocity.0 += impulse * inv_mass2;
                body2.angular_velocity.0 += inv_inertia2 * cross(r2, impulse);
            }
        }
    }

    /// Applies [restitution](`Restitution`) for the given bodies if the relative speed
    /// along the contact normal exceeds the given `threshold`.
    pub fn apply_restitution(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        threshold: Scalar,
    ) {
        for point in self.points.iter_mut() {
            // Skip restitution for speeds below the threshold.
            // We also skip contacts that don't apply an impulse to account for speculative contacts.
            if point.normal_speed > -threshold || point.max_normal_impulse == 0.0 {
                continue;
            }

            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            let inv_mass1 = body1.effective_inverse_mass();
            let inv_mass2 = body2.effective_inverse_mass();
            let inv_inertia1 = body1.effective_global_angular_inertia().inverse();
            let inv_inertia2 = body2.effective_global_angular_inertia().inverse();

            // Relative velocity at contact point
            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);
            let normal_speed = relative_velocity.dot(self.normal);

            // Compute the incremental normal impulse to account for restitution.
            let mut impulse = -point.normal_part.effective_mass
                * (normal_speed + self.restitution * point.normal_speed);

            // Clamp the accumulated impulse.
            let new_impulse = (point.normal_part.impulse + impulse).max(0.0);
            impulse = new_impulse - point.normal_part.impulse;
            point.normal_part.impulse = new_impulse;
            point.max_normal_impulse = impulse.max(point.max_normal_impulse);

            // Apply the impulse.
            let impulse = impulse * self.normal;

            if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
                body1.linear_velocity.0 -= impulse * inv_mass1;
                body1.angular_velocity.0 -= inv_inertia1 * cross(r1, impulse);
            }
            if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
                body2.linear_velocity.0 += impulse * inv_mass2;
                body2.angular_velocity.0 += inv_inertia2 * cross(r2, impulse);
            }
        }
    }

    /// Computes `DIM - 1` tangent directions.
    #[allow(unused_variables)]
    pub fn tangent_directions(&self, velocity1: Vector, velocity2: Vector) -> [Vector; DIM - 1] {
        #[cfg(feature = "2d")]
        {
            [Vector::new(self.normal.y, -self.normal.x)]
        }
        #[cfg(feature = "3d")]
        {
            let force_direction = -self.normal;
            let relative_velocity = velocity1 - velocity2;
            let tangent_velocity =
                relative_velocity - force_direction * force_direction.dot(relative_velocity);

            let tangent = tangent_velocity
                .try_normalize()
                .unwrap_or(force_direction.any_orthonormal_vector());
            let bitangent = force_direction.cross(tangent);
            [tangent, bitangent]
        }
    }
}

impl MapEntities for ContactConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}
