//! [`FixedJoint`] component.

use crate::prelude::*;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};

/// A fixed joint prevents any relative movement of the attached bodies.
///
/// You should generally prefer using a single body instead of multiple bodies fixed together,
/// but fixed joints can be useful for things like rigid structures where a force can dynamically break the joints connecting individual bodies.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct FixedJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub position_lagrange: Scalar,
    /// Lagrange multiplier for the angular correction caused by the alignment of the bodies.
    pub align_lagrange: Scalar,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// The force exerted by the joint.
    pub force: Vector,
    /// The torque exerted by the joint when aligning the bodies.
    pub align_torque: Torque,
}

impl XpbdConstraint<2> for FixedJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

    fn clear_lagrange_multipliers(&mut self) {
        self.position_lagrange = 0.0;
        self.align_lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) {
        let [body1, body2] = bodies;
        let compliance = self.compliance;

        // Align orientation
        let dq = self.get_delta_q(&body1.rotation, &body2.rotation);
        let mut lagrange = self.align_lagrange;
        self.align_torque = self.align_orientation(body1, body2, dq, &mut lagrange, compliance, dt);
        self.align_lagrange = lagrange;

        // Align position of local attachment points
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
    }
}

impl Joint for FixedJoint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            damping_linear: 1.0,
            damping_angular: 1.0,
            position_lagrange: 0.0,
            align_lagrange: 0.0,
            compliance: 0.0,
            force: Vector::ZERO,
            #[cfg(feature = "2d")]
            align_torque: 0.0,
            #[cfg(feature = "3d")]
            align_torque: Vector::ZERO,
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

impl FixedJoint {
    #[cfg(feature = "2d")]
    fn get_delta_q(&self, rot1: &Rotation, rot2: &Rotation) -> Vector3 {
        (*rot2 - *rot1).as_radians() * Vector3::Z
    }

    #[cfg(feature = "3d")]
    fn get_delta_q(&self, rot1: &Rotation, rot2: &Rotation) -> Vector {
        2.0 * (rot1.0 * rot2.inverse().0).xyz()
    }
}

impl PositionConstraint for FixedJoint {}

impl AngularConstraint for FixedJoint {}

impl MapEntities for FixedJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}
