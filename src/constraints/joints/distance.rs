//! [`DistanceJoint`] component.

use crate::prelude::*;
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};

/// A distance joint keeps the attached bodies at a certain distance from each other while while allowing rotation around all axes.
///
/// Distance joints can be useful for things like springs, muscles, and mass-spring networks.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(MapEntities)]
pub struct DistanceJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// The distance the attached bodies will be kept relative to each other.
    pub rest_length: Scalar,
    /// The extents of the allowed relative translation between the attached bodies.
    pub length_limits: Option<DistanceLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub lagrange: Scalar,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// The force exerted by the joint.
    pub force: Vector,
}

impl XpbdConstraint<2> for DistanceJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) {
        self.force = self.constrain_length(bodies, dt);
    }
}

impl Joint for DistanceJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            rest_length: 0.0,
            length_limits: None,
            damping_linear: 0.0,
            damping_angular: 0.0,
            lagrange: 0.0,
            compliance: 0.0,
            force: Vector::ZERO,
        }
    }

    fn with_compliance(self, compliance: Scalar) -> Self {
        Self { compliance, ..self }
    }

    fn with_local_anchor_1(self, anchor: Vector) -> Self {
        Self {
            local_anchor1: anchor,
            ..self
        }
    }

    fn with_local_anchor_2(self, anchor: Vector) -> Self {
        Self {
            local_anchor2: anchor,
            ..self
        }
    }

    fn with_linear_velocity_damping(self, damping: Scalar) -> Self {
        Self {
            damping_linear: damping,
            ..self
        }
    }

    fn with_angular_velocity_damping(self, damping: Scalar) -> Self {
        Self {
            damping_angular: damping,
            ..self
        }
    }

    fn local_anchor_1(&self) -> Vector {
        self.local_anchor1
    }

    fn local_anchor_2(&self) -> Vector {
        self.local_anchor2
    }

    fn damping_linear(&self) -> Scalar {
        self.damping_linear
    }

    fn damping_angular(&self) -> Scalar {
        self.damping_angular
    }
}

impl DistanceJoint {
    /// Constrains the distance the bodies with no constraint on their rotation.
    ///
    /// Returns the force exerted by this constraint.
    fn constrain_length(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) -> Vector {
        let [body1, body2] = bodies;
        let world_r1 = body1.rotation.rotate(self.local_anchor1);
        let world_r2 = body2.rotation.rotate(self.local_anchor2);

        // If min and max limits aren't specified, use rest length
        // TODO: Remove rest length, just use min/max limits.
        let limits = self
            .length_limits
            .unwrap_or(DistanceLimit::new(self.rest_length, self.rest_length));

        // Compute the direction and magnitude of the positional correction required
        // to keep the bodies within a certain distance from each other.
        let (dir, distance) = limits.compute_correction(
            body1.current_position() + world_r1,
            body2.current_position() + world_r2,
        );

        // Avoid division by zero and unnecessary computation
        if distance.abs() < Scalar::EPSILON {
            return Vector::ZERO;
        }

        // Compute generalized inverse masses (method from PositionConstraint)
        let w1 = PositionConstraint::compute_generalized_inverse_mass(self, body1, world_r1, dir);
        let w2 = PositionConstraint::compute_generalized_inverse_mass(self, body2, world_r2, dir);
        let w = [w1, w2];

        // Constraint gradients, i.e. how the bodies should be moved
        // relative to each other in order to satisfy the constraint
        let gradients = [dir, -dir];

        // Compute Lagrange multiplier update, essentially the signed magnitude of the correction
        let delta_lagrange = self.compute_lagrange_update(
            self.lagrange,
            distance,
            &gradients,
            &w,
            self.compliance,
            dt,
        );
        self.lagrange += delta_lagrange;

        // Apply positional correction (method from PositionConstraint)
        self.apply_positional_correction(body1, body2, delta_lagrange, dir, world_r1, world_r2);

        // Return constraint force
        self.compute_force(self.lagrange, dir, dt)
    }

    /// Sets the minimum and maximum distances between the attached bodies.
    pub fn with_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            length_limits: Some(DistanceLimit::new(min, max)),
            ..self
        }
    }

    /// Sets the joint's rest length, or distance the bodies will be kept at.
    pub fn with_rest_length(self, rest_length: Scalar) -> Self {
        Self {
            rest_length,
            ..self
        }
    }
}

impl PositionConstraint for DistanceJoint {}

impl AngularConstraint for DistanceJoint {}

impl MapEntities for DistanceJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}
