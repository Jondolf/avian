//! Penetration constraint.

use crate::prelude::*;
use bevy::prelude::*;

/// A constraint between two bodies that prevents overlap with a given compliance.
///
/// A compliance of 0.0 resembles a constraint with infinite stiffness, so the bodies should not have any overlap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PenetrationConstraint {
    /// First entity in the constraint.
    pub entity1: Entity,
    /// Second entity in the constraint.
    pub entity2: Entity,
    /// Data associated with the contact.
    pub contact: Contact,
    /// Vector from the first entity's center of mass to the contact point in local coordinates.
    pub local_r1: Vector,
    /// Vector from the second entity's center of mass to the contact point in local coordinates.
    pub local_r2: Vector,
    /// Vector from the first entity's center of mass to the contact position in world coordinates.
    pub world_r1: Vector,
    /// Vector from the second entity's center of mass to the contact position in world coordinates.
    pub world_r2: Vector,
    /// Lagrange multiplier for the normal force.
    pub normal_lagrange: Scalar,
    /// Lagrange multiplier for the tangential force.
    pub tangent_lagrange: Scalar,
    /// The constraint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// Normal force acting along the constraint.
    pub normal_force: Vector,
    /// Static friction force acting along this constraint.
    pub static_friction_force: Vector,
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
        self.solve_contact(body1, body2, dt);
        self.solve_friction(body1, body2, dt);
    }
}

impl PenetrationConstraint {
    /// Creates a new [`PenetrationConstraint`] with the given bodies and contact data.
    pub fn new(body1: &RigidBodyQueryItem, body2: &RigidBodyQueryItem, contact: Contact) -> Self {
        let world_r1 = contact.point1 - body1.position.0 + body1.center_of_mass.0;
        let world_r2 = contact.point2 - body2.position.0 + body2.center_of_mass.0;
        let local_r1 = body1.rotation.inverse().rotate(world_r1);
        let local_r2 = body2.rotation.inverse().rotate(world_r2);

        Self {
            entity1: body1.entity,
            entity2: body2.entity,
            contact,
            local_r1,
            local_r2,
            world_r1,
            world_r2,
            normal_lagrange: 0.0,
            tangent_lagrange: 0.0,
            compliance: 0.0,
            normal_force: Vector::ZERO,
            static_friction_force: Vector::ZERO,
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
        let penetration = self.contact.penetration;
        let normal = self.contact.normal;
        let compliance = self.compliance;
        let lagrange = self.normal_lagrange;
        let r1 = self.world_r1;
        let r2 = self.world_r2;

        // If penetration depth is under 0, skip the collision
        if penetration <= 0.0 {
            return;
        }

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

        // Update normal force using the equation f = lambda * n / h^2
        self.normal_force = self.normal_lagrange * normal / dt.powi(2);
    }

    fn solve_friction(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) {
        // Shorter aliases
        let penetration = self.contact.penetration;
        let normal = self.contact.normal;
        let compliance = self.compliance;
        let lagrange = self.tangent_lagrange;
        let r1 = self.world_r1;
        let r2 = self.world_r2;

        // Compute contact positions at the current state and before substep integration
        let p1 = body1.position.0 + body1.rotation.rotate(self.local_r1);
        let p2 = body2.position.0 + body2.rotation.rotate(self.local_r2);
        let prev_p1 = body1.previous_position.0 + body1.previous_rotation.rotate(self.local_r1);
        let prev_p2 = body2.previous_position.0 + body2.previous_rotation.rotate(self.local_r2);

        // Compute relative motion of the contact points and get the tangential component
        let delta_p = (p1 - prev_p1) - (p2 - prev_p2);
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

        // Compute combined friction coefficients
        let static_coefficient = body1.friction.combine(*body2.friction).static_coefficient;

        // Apply static friction if |delta_x_perp| < mu_s * d
        if sliding_len < static_coefficient * penetration {
            // Compute Lagrange multiplier update for static friction
            let delta_lagrange =
                self.compute_lagrange_update(lagrange, sliding_len, &gradients, &w, compliance, dt);
            self.tangent_lagrange += delta_lagrange;

            // Apply positional correction to handle static friction
            self.apply_positional_correction(body1, body2, delta_lagrange, tangent, r1, r2);

            // Update static friction force using the equation f = lambda * n / h^2
            self.static_friction_force = self.tangent_lagrange * tangent / dt.powi(2);
        }
    }
}

impl PositionConstraint for PenetrationConstraint {}
