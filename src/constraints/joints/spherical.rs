//! [`SphericalJoint`] component.

use crate::prelude::*;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};

/// A spherical joint prevents relative translation of the attached bodies while allowing rotation around all axes.
///
/// Spherical joints can be useful for things like pendula, chains, ragdolls etc.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct SphericalJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// An axis that the attached bodies can swing around. This is normally the x-axis.
    pub swing_axis: Vector3,
    /// An axis that the attached bodies can twist around. This is normally the y-axis.
    pub twist_axis: Vector3,
    /// The extents of the allowed relative rotation of the bodies around the `swing_axis`.
    pub swing_limit: Option<AngleLimit>,
    /// The extents of the allowed relative rotation of the bodies around the `twist_axis`.
    pub twist_limit: Option<AngleLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub position_lagrange: Scalar,
    /// Lagrange multiplier for the angular correction caused by the swing limits.
    pub swing_lagrange: Scalar,
    /// Lagrange multiplier for the angular correction caused by the twist limits.
    pub twist_lagrange: Scalar,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// The force exerted by the joint.
    pub force: Vector,
    /// The torque exerted by the joint when limiting the relative rotation of the bodies around the `swing_axis`.
    pub swing_torque: Torque,
    /// The torque exerted by the joint when limiting the relative rotation of the bodies around the `twist_axis`.
    pub twist_torque: Torque,
}

impl XpbdConstraint<2> for SphericalJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

    fn clear_lagrange_multipliers(&mut self) {
        self.position_lagrange = 0.0;
        self.swing_lagrange = 0.0;
        self.twist_lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) {
        let [body1, body2] = bodies;
        let compliance = self.compliance;

        // Align positions
        let mut lagrange = self.position_lagrange;
        self.force = self.align_position(
            body1,
            body2,
            self.local_anchor1,
            self.local_anchor2,
            &mut lagrange,
            compliance,
            dt,
        );
        self.position_lagrange = lagrange;

        // Apply swing limits
        self.swing_torque = self.apply_swing_limits(body1, body2, dt);

        // Apply twist limits
        self.twist_torque = self.apply_twist_limits(body1, body2, dt);
    }
}

impl Joint for SphericalJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            swing_axis: Vector3::X,
            twist_axis: Vector3::Y,
            swing_limit: None,
            twist_limit: None,
            damping_linear: 1.0,
            damping_angular: 1.0,
            position_lagrange: 0.0,
            swing_lagrange: 0.0,
            twist_lagrange: 0.0,
            compliance: 0.0,
            force: Vector::ZERO,
            #[cfg(feature = "2d")]
            swing_torque: 0.0,
            #[cfg(feature = "3d")]
            swing_torque: Vector::ZERO,
            #[cfg(feature = "2d")]
            twist_torque: 0.0,
            #[cfg(feature = "3d")]
            twist_torque: Vector::ZERO,
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

impl SphericalJoint {
    /// Sets the limits of the allowed relative rotation around the `swing_axis`.
    pub fn with_swing_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            swing_limit: Some(AngleLimit::new(min, max)),
            ..self
        }
    }

    /// Sets the limits of the allowed relative rotation around the `twist_axis`.
    #[cfg(feature = "3d")]
    pub fn with_twist_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            twist_limit: Some(AngleLimit::new(min, max)),
            ..self
        }
    }

    /// Applies angle limits to limit the relative rotation of the bodies around the `swing_axis`.
    fn apply_swing_limits(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) -> Torque {
        if let Some(joint_limit) = self.swing_limit {
            let a1 = body1.rotation.rotate_vec3(self.swing_axis);
            let a2 = body2.rotation.rotate_vec3(self.swing_axis);

            let n = a1.cross(a2);
            let n_magnitude = n.length();

            if n_magnitude <= Scalar::EPSILON {
                return Torque::ZERO;
            }

            let n = n / n_magnitude;

            if let Some(dq) = joint_limit.compute_correction(n, a1, a2, PI) {
                let mut lagrange = self.swing_lagrange;
                let torque =
                    self.align_orientation(body1, body2, dq, &mut lagrange, self.compliance, dt);
                self.swing_lagrange = lagrange;
                return torque;
            }
        }
        Torque::ZERO
    }

    /// Applies angle limits to limit the relative rotation of the bodies around the `twist_axis`.
    fn apply_twist_limits(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) -> Torque {
        if let Some(joint_limit) = self.twist_limit {
            let a1 = body1.rotation.rotate_vec3(self.swing_axis);
            let a2 = body2.rotation.rotate_vec3(self.swing_axis);

            let b1 = body1.rotation.rotate_vec3(self.twist_axis);
            let b2 = body2.rotation.rotate_vec3(self.twist_axis);

            let n = a1 + a2;
            let n_magnitude = n.length();

            if n_magnitude <= Scalar::EPSILON {
                return Torque::ZERO;
            }

            let n = n / n_magnitude;

            let n1 = b1 - n.dot(b1) * n;
            let n2 = b2 - n.dot(b2) * n;
            let n1_magnitude = n1.length();
            let n2_magnitude = n2.length();

            if n1_magnitude <= Scalar::EPSILON || n2_magnitude <= Scalar::EPSILON {
                return Torque::ZERO;
            }

            let n1 = n1 / n1_magnitude;
            let n2 = n2 / n2_magnitude;

            let max_correction = if a1.dot(a2) > -0.5 { 2.0 * PI } else { dt };

            if let Some(dq) = joint_limit.compute_correction(n, n1, n2, max_correction) {
                let mut lagrange = self.twist_lagrange;
                let torque =
                    self.align_orientation(body1, body2, dq, &mut lagrange, self.compliance, dt);
                self.twist_lagrange = lagrange;
                return torque;
            }
        }
        Torque::ZERO
    }
}

impl PositionConstraint for SphericalJoint {}

impl AngularConstraint for SphericalJoint {}

impl MapEntities for SphericalJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}
