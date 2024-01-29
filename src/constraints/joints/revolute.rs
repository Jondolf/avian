//! [`RevoluteJoint`] component.

use crate::prelude::*;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};

/// A revolute joint prevents relative movement of the attached bodies, except for rotation around one `aligned_axis`.
///
/// Revolute joints can be useful for things like wheels, fans, revolving doors etc.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct RevoluteJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// A unit vector that controls which axis should be aligned for both entities.
    ///
    /// In 2D this should always be the Z axis.
    #[cfg(feature = "2d")]
    pub(crate) aligned_axis: Vector3,
    /// A unit vector that controls which axis should be aligned for both bodies.
    #[cfg(feature = "3d")]
    pub aligned_axis: Vector,
    /// The extents of the allowed relative rotation of the bodies around the `aligned_axis`.
    pub angle_limit: Option<AngleLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub position_lagrange: Scalar,
    /// Lagrange multiplier for the angular correction caused by the alignment of the bodies.
    pub align_lagrange: Scalar,
    /// Lagrange multiplier for the angular correction caused by the angle limits.
    pub angle_limit_lagrange: Scalar,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// The force exerted by the joint.
    pub force: Vector,
    /// The torque exerted by the joint when aligning the bodies.
    pub align_torque: Torque,
    /// The torque exerted by the joint when limiting the relative rotation of the bodies around the `aligned_axis`.
    pub angle_limit_torque: Torque,
}

impl XpbdConstraint<2> for RevoluteJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

    fn clear_lagrange_multipliers(&mut self) {
        self.position_lagrange = 0.0;
        self.align_lagrange = 0.0;
        self.angle_limit_lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) {
        let [body1, body2] = bodies;
        let compliance = self.compliance;

        // Constrain the relative rotation of the bodies, only allowing rotation around one free axis
        let dq = self.get_delta_q(&body1.rotation, &body2.rotation);
        let mut lagrange = self.align_lagrange;
        self.align_torque = self.align_orientation(body1, body2, dq, &mut lagrange, compliance, dt);
        self.align_lagrange = lagrange;

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

        // Apply angle limits when rotating around the free axis
        self.angle_limit_torque = self.apply_angle_limits(body1, body2, dt);
    }
}

impl Joint for RevoluteJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            aligned_axis: Vector3::Z,
            angle_limit: None,
            damping_linear: 1.0,
            damping_angular: 1.0,
            position_lagrange: 0.0,
            align_lagrange: 0.0,
            angle_limit_lagrange: 0.0,
            compliance: 0.0,
            force: Vector::ZERO,
            #[cfg(feature = "2d")]
            align_torque: 0.0,
            #[cfg(feature = "3d")]
            align_torque: Vector::ZERO,
            #[cfg(feature = "2d")]
            angle_limit_torque: 0.0,
            #[cfg(feature = "3d")]
            angle_limit_torque: Vector::ZERO,
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

impl RevoluteJoint {
    /// Sets the axis that the bodies should be aligned on.
    #[cfg(feature = "3d")]
    pub fn with_aligned_axis(self, axis: Vector) -> Self {
        Self {
            aligned_axis: axis,
            ..self
        }
    }

    /// Sets the limits of the allowed relative rotation around the `aligned_axis`.
    pub fn with_angle_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            angle_limit: Some(AngleLimit::new(min, max)),
            ..self
        }
    }

    fn get_delta_q(&self, rot1: &Rotation, rot2: &Rotation) -> Vector3 {
        let a1 = rot1.rotate_vec3(self.aligned_axis);
        let a2 = rot2.rotate_vec3(self.aligned_axis);
        a1.cross(a2)
    }

    /// Applies angle limits to limit the relative rotation of the bodies around the `aligned_axis`.
    #[allow(clippy::too_many_arguments)]
    fn apply_angle_limits(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) -> Torque {
        if let Some(angle_limit) = self.angle_limit {
            let limit_axis = Vector3::new(
                self.aligned_axis.z,
                self.aligned_axis.x,
                self.aligned_axis.y,
            );
            let a1 = body1.rotation.rotate_vec3(limit_axis);
            let a2 = body2.rotation.rotate_vec3(limit_axis);
            let n = a1.cross(a2).normalize();

            if let Some(dq) = angle_limit.compute_correction(n, a1, a2, PI) {
                let mut lagrange = self.angle_limit_lagrange;
                let torque =
                    self.align_orientation(body1, body2, dq, &mut lagrange, self.compliance, dt);
                self.angle_limit_lagrange = lagrange;
                return torque;
            }
        }
        Torque::ZERO
    }
}

impl PositionConstraint for RevoluteJoint {}

impl AngularConstraint for RevoluteJoint {}

impl MapEntities for RevoluteJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}
