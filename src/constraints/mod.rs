//! General constraint logic and different types of constraints, such as joints and penetration constraints.

pub mod joints;
pub mod penetration;

mod angular_constraint;
mod position_constraint;

pub use angular_constraint::AngularConstraint;
pub use position_constraint::PositionConstraint;

use crate::prelude::*;

pub trait XpbdConstraint<const ENTITY_COUNT: usize> {
    /// The entities participating in the constraint.
    fn entities(&self) -> [Entity; ENTITY_COUNT];

    /// Solves the constraint.
    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; ENTITY_COUNT], dt: Scalar);

    /// Computes how much a constraint's Lagrange multiplier changes when projecting the constraint for all participating particles.
    ///
    /// `c` is a scalar value returned by the constraint function. When it is zero, the constraint is satisfied.
    ///
    /// Each particle should have a corresponding gradient in `gradients`. A gradient is a vector that refers to the direction in which `c` increases the most.
    fn compute_lagrange_update(
        &self,
        lagrange: Scalar,
        c: Scalar,
        gradients: &[Vector],
        inverse_masses: &[Scalar],
        compliance: Scalar,
        dt: Scalar,
    ) -> Scalar {
        // Compute the sum of all inverse masses multiplied by the squared lengths of the corresponding gradients.
        let w_sum = inverse_masses
            .iter()
            .enumerate()
            .fold(0.0, |acc, (i, w)| acc + *w * gradients[i].length_squared());

        // Avoid division by zero
        if w_sum <= Scalar::EPSILON {
            return 0.0;
        }

        // tilde_a = a/h^2
        let tilde_compliance = compliance / dt.powi(2);

        (-c - tilde_compliance * lagrange) / (w_sum + tilde_compliance)
    }

    /// Sets the constraint's Lagrange multipliers to 0.
    fn clear_lagrange_multipliers(&mut self);
}
