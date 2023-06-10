//! Penetration constraint.

use crate::prelude::{collision::Collision, *};

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
    /// Data associated with the collision.
    pub collision_data: Collision,
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

impl PenetrationConstraint {
    pub fn new(entity1: Entity, entity2: Entity, collision_data: Collision) -> Self {
        Self {
            entity1,
            entity2,
            collision_data,
            normal_lagrange: 0.0,
            tangent_lagrange: 0.0,
            compliance: 0.0,
            normal_force: Vector::ZERO,
            static_friction_force: Vector::ZERO,
        }
    }

    /// Solves overlap between two bodies according to their masses
    #[allow(clippy::too_many_arguments)]
    pub fn solve(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) {
        self.solve_contact(body1, body2, dt);
        self.solve_friction(body1, body2, dt);
    }

    /// Solves a non-penetration constraint between two bodies.
    fn solve_contact(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) {
        // Shorter aliases
        let normal = self.collision_data.normal;
        let compliance = self.compliance;
        let lagrange = self.normal_lagrange;
        let r1 = self.collision_data.world_r1;
        let r2 = self.collision_data.world_r2;

        // Compute contact positions at the current state
        let p1 = body1.pos.0 - body1.local_com.0 + body1.rot.rotate(self.collision_data.local_r1);
        let p2 = body2.pos.0 - body2.local_com.0 + body2.rot.rotate(self.collision_data.local_r2);

        // Compute penetration depth
        let penetration = (p1 - p2).dot(normal);

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
        let penetration = self.collision_data.penetration;
        let normal = self.collision_data.normal;
        let compliance = self.compliance;
        let lagrange = self.tangent_lagrange;
        let r1 = self.collision_data.world_r1;
        let r2 = self.collision_data.world_r2;

        // Compute contact positions at the current state and before substep integration
        let p1 = body1.pos.0 - body1.local_com.0 + body1.rot.rotate(self.collision_data.local_r1);
        let p2 = body2.pos.0 - body2.local_com.0 + body2.rot.rotate(self.collision_data.local_r2);
        let prev_p1 = body1.prev_pos.0 - body1.local_com.0
            + body1.prev_rot.rotate(self.collision_data.local_r1);
        let prev_p2 = body2.prev_pos.0 - body2.local_com.0
            + body2.prev_rot.rotate(self.collision_data.local_r2);

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

impl XpbdConstraint for PenetrationConstraint {
    fn clear_lagrange_multipliers(&mut self) {
        self.normal_lagrange = 0.0;
        self.tangent_lagrange = 0.0;
    }
}

impl PositionConstraint for PenetrationConstraint {}
