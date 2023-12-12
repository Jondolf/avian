//! [`FixedJoint`] component.

use crate::prelude::*;
use bevy::prelude::*;

/// A fixed joint prevents any relative movement of the attached bodies.
///
/// You should generally prefer using a single body instead of multiple bodies fixed together,
/// but fixed joints can be useful for things like rigid structures where a force can dynamically break the joints connecting individual bodies.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct FixedJoint {
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

impl Default for FixedJoint {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl XpbdConstraint<2> for FixedJoint {
    type SolveInput = JointAnchors;

    fn clear_lagrange_multipliers(&mut self) {
        self.position_lagrange = 0.0;
        self.align_lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar, anchors: JointAnchors) {
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
            anchors.first,
            anchors.second,
            &mut lagrange,
            compliance,
            dt,
        );
        self.position_lagrange = lagrange;
    }
}

impl Joint for FixedJoint {
    fn with_compliance(self, compliance: Scalar) -> Self {
        Self { compliance, ..self }
    }
}

impl FixedJoint {
    /// The default joint configuration.
    pub const DEFAULT: Self = Self {
        position_lagrange: 0.0,
        align_lagrange: 0.0,
        compliance: 0.0,
        force: Vector::ZERO,
        #[cfg(feature = "2d")]
        align_torque: 0.0,
        #[cfg(feature = "3d")]
        align_torque: Vector::ZERO,
    };

    /// Creates a new [`FixedJoint`].
    pub const fn new() -> Self {
        Self::DEFAULT
    }

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
