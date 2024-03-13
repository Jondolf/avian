//! Penetration constraint.

use crate::prelude::*;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};

/// A constraint between two bodies that prevents overlap with a given compliance.
///
/// A compliance of 0.0 resembles a constraint with infinite stiffness, so the bodies should not have any overlap.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct PenetrationConstraint {
    /// First entity in the constraint.
    pub entity1: Entity,
    /// Second entity in the constraint.
    pub entity2: Entity,
    /// The entity of the collider of the first body.
    pub collider_entity1: Entity,
    /// The entity of the collider of the second body.
    pub collider_entity2: Entity,
    /// Data associated with the contact.
    pub contact: ContactData,
    /// The index of the contact in the manifold.
    pub manifold_index: usize,
    /// Vector from the first entity's center of mass to the contact point in local coordinates.
    pub r1: Vector,
    /// Vector from the second entity's center of mass to the contact point in local coordinates.
    pub r2: Vector,
    /// Lagrange multiplier for the normal force.
    pub normal_lagrange: Scalar,
    /// Lagrange multiplier for the tangential force.
    pub tangent_lagrange: Scalar,
    /// The constraint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// The effective [friction](Friction) of the contact.
    pub friction: Friction,
    /// The effective [restitution](Restitution) of the contact.
    pub restitution: Restitution,
}

impl XpbdConstraint<2> for PenetrationConstraint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

    fn clear_lagrange_multipliers(&mut self) {
        self.normal_lagrange = 0.0;
        self.tangent_lagrange = 0.0;
    }

    /// Solves overlap between two bodies.
    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) {
        let [body1, body2] = bodies;

        let p1 = body1.current_position() + body1.rotation.rotate(self.contact.point1);
        let p2 = body2.current_position() + body2.rotation.rotate(self.contact.point2);
        self.contact.penetration = (p1 - p2).dot(self.contact.global_normal1(&body1.rotation));

        // If penetration depth is under 0, skip the collision
        if self.contact.penetration <= Scalar::EPSILON {
            return;
        }

        self.solve_contact(body1, body2, dt);
        self.solve_friction(body1, body2, dt);
    }
}

impl PenetrationConstraint {
    /// Creates a new [`PenetrationConstraint`] with the given bodies and contact data.
    ///
    /// The `manifold_index` is the index of the contact in a [`ContactManifold`].
    pub fn new(
        body1: &RigidBodyQueryItem,
        body2: &RigidBodyQueryItem,
        collider_entity1: Entity,
        collider_entity2: Entity,
        contact: ContactData,
        manifold_index: usize,
    ) -> Self {
        let r1 = contact.point1 - body1.center_of_mass.0;
        let r2 = contact.point2 - body2.center_of_mass.0;

        Self {
            entity1: body1.entity,
            entity2: body2.entity,
            collider_entity1,
            collider_entity2,
            contact,
            manifold_index,
            r1,
            r2,
            normal_lagrange: 0.0,
            tangent_lagrange: 0.0,
            compliance: 0.0,
            friction: body1.friction.combine(*body2.friction),
            restitution: body1.restitution.combine(*body2.restitution),
        }
    }

    /// Solves a non-penetration constraint between two bodies.
    fn solve_contact(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) {
        // Shorter aliases
        let compliance = self.compliance;
        let lagrange = self.normal_lagrange;
        let penetration = self.contact.penetration;
        let normal = self.contact.global_normal1(&body1.rotation);
        let r1 = body1.rotation.rotate(self.r1);
        let r2 = body2.rotation.rotate(self.r2);

        // Compute generalized inverse masses
        let w1 = self.compute_generalized_inverse_mass(body1, r1, normal);
        let w2 = self.compute_generalized_inverse_mass(body2, r2, normal);

        // Constraint gradients and inverse masses
        let gradients = [normal, -normal];
        let w = [w1, w2];

        // Compute Lagrange multiplier update
        let delta_lagrange =
            self.compute_lagrange_update(lagrange, penetration, &gradients, &w, compliance, dt);
        self.normal_lagrange += delta_lagrange;

        // Apply positional correction to solve overlap
        self.apply_positional_correction(body1, body2, delta_lagrange, normal, r1, r2);

        // Update normal impulse.
        // f = lambda / h^2
        // i = f * h = lambda / h
        self.contact.normal_impulse += self.normal_lagrange / dt;
    }

    fn solve_friction(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) {
        // Shorter aliases
        let compliance = self.compliance;
        let lagrange = self.tangent_lagrange;
        let penetration = self.contact.penetration;
        let normal = self.contact.global_normal1(&body1.rotation);
        let r1 = body1.rotation.rotate(self.r1);
        let r2 = body2.rotation.rotate(self.r2);

        // Compute relative motion of the contact points and get the tangential component
        let delta_p1 = body1.current_position() - body1.previous_position.0
            + body1.rotation.rotate(self.contact.point1)
            - body1.previous_rotation.rotate(self.contact.point1);
        let delta_p2 = body2.current_position() - body2.previous_position.0
            + body2.rotation.rotate(self.contact.point2)
            - body2.previous_rotation.rotate(self.contact.point2);
        let delta_p = delta_p1 - delta_p2;
        let delta_p_tangent = delta_p - delta_p.dot(normal) * normal;

        // Compute magnitude of relative tangential movement and get normalized tangent vector
        let sliding_len = delta_p_tangent.length();
        if sliding_len <= Scalar::EPSILON {
            return;
        }
        let tangent = delta_p_tangent / sliding_len;

        // Compute generalized inverse masses
        let w1 = self.compute_generalized_inverse_mass(body1, r1, tangent);
        let w2 = self.compute_generalized_inverse_mass(body2, r2, tangent);

        // Constraint gradients and inverse masses
        let gradients = [tangent, -tangent];
        let w = [w1, w2];

        // Apply static friction if |delta_x_perp| < mu_s * d
        if sliding_len < self.friction.static_coefficient * penetration {
            // Compute Lagrange multiplier update for static friction
            let delta_lagrange =
                self.compute_lagrange_update(lagrange, sliding_len, &gradients, &w, compliance, dt);
            self.tangent_lagrange += delta_lagrange;

            // Apply positional correction to handle static friction
            self.apply_positional_correction(body1, body2, delta_lagrange, tangent, r1, r2);

            // Update static friction impulse.
            // f = lambda / h^2
            // i = f * h = lambda / h
            self.contact.tangent_impulse += self.tangent_lagrange / dt;
        }
    }
}

impl PositionConstraint for PenetrationConstraint {}

impl MapEntities for PenetrationConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}
