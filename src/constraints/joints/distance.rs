//! [`DistanceJoint`] component.

use crate::prelude::*;
use bevy::prelude::*;

/// A distance joint keeps the attached bodies at a certain distance from each other while while allowing rotation around all axes.
///
/// Distance joints can be useful for things like springs, muscles, and mass-spring networks.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct DistanceJoint {
    /// The distance the attached bodies will be kept relative to each other.
    pub rest_length: Scalar,
    /// The extents of the allowed relative translation between the attached bodies.
    pub length_limits: Option<DistanceLimit>,
    /// Lagrange multiplier for the positional correction.
    pub lagrange: Scalar,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// The force exerted by the joint.
    pub force: Vector,
}

impl Default for DistanceJoint {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl XpbdConstraint<2> for DistanceJoint {
    type SolveInput = JointAnchors;

    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar, anchors: JointAnchors) {
        self.force = self.constrain_length(bodies, dt, anchors);
    }
}

impl Joint for DistanceJoint {
    fn with_compliance(self, compliance: Scalar) -> Self {
        Self { compliance, ..self }
    }
}

impl DistanceJoint {
    /// The default joint configuration.
    pub const DEFAULT: Self = Self {
        rest_length: 0.0,
        length_limits: None,
        lagrange: 0.0,
        compliance: 0.0,
        force: Vector::ZERO,
    };

    /// Creates a new [`DistanceJoint`] with the given `rest_length`.
    pub const fn new(rest_length: Scalar) -> Self {
        Self {
            rest_length,
            ..Self::DEFAULT
        }
    }

    /// Sets the minimum and maximum distances between the attached bodies.
    pub const fn with_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.length_limits = Some(DistanceLimit::new(min, max));
        self
    }

    /// Constrains the distance the bodies with no constraint on their rotation.
    ///
    /// Returns the force exerted by this constraint.
    fn constrain_length(
        &mut self,
        bodies: [&mut RigidBodyQueryItem; 2],
        dt: Scalar,
        anchors: JointAnchors,
    ) -> Vector {
        let [body1, body2] = bodies;
        let world_r1 = body1.rotation.rotate(anchors.first);
        let world_r2 = body2.rotation.rotate(anchors.second);

        // // Compute the positional difference
        let mut delta_x =
            (body1.current_position() + world_r1) - (body2.current_position() + world_r2);

        // The current separation distance
        let mut length = delta_x.length();

        if let Some(limits) = self.length_limits {
            if length < Scalar::EPSILON {
                return Vector::ZERO;
            }
            delta_x += limits.compute_correction(
                body1.current_position() + world_r1,
                body2.current_position() + world_r2,
            );
            length = delta_x.length();
        }

        // The value of the constraint function. When this is zero, the
        // constraint is satisfied, and the distance between the bodies is the
        // rest length.
        let c = length - self.rest_length;

        // Avoid division by zero and unnecessary computation.
        if length < Scalar::EPSILON || c.abs() < Scalar::EPSILON {
            return Vector::ZERO;
        }

        // Normalized delta_x
        let n = delta_x / length;

        // Compute generalized inverse masses (method from PositionConstraint)
        let w1 = PositionConstraint::compute_generalized_inverse_mass(self, body1, world_r1, n);
        let w2 = PositionConstraint::compute_generalized_inverse_mass(self, body2, world_r2, n);
        let w = [w1, w2];

        // Constraint gradients, i.e. how the bodies should be moved
        // relative to each other in order to satisfy the constraint
        let gradients = [n, -n];

        // Compute Lagrange multiplier update, essentially the signed magnitude of the correction
        let delta_lagrange =
            self.compute_lagrange_update(self.lagrange, c, &gradients, &w, self.compliance, dt);
        self.lagrange += delta_lagrange;

        // Apply positional correction (method from PositionConstraint)
        self.apply_positional_correction(body1, body2, delta_lagrange, n, world_r1, world_r2);

        // Return constraint force
        self.compute_force(self.lagrange, n, dt)
    }
}

impl PositionConstraint for DistanceJoint {}

impl AngularConstraint for DistanceJoint {}
