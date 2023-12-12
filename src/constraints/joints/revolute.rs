//! [`RevoluteJoint`] component.

use crate::prelude::*;
use bevy::prelude::*;

/// A revolute joint prevents relative movement of the attached bodies, except for rotation around one `aligned_axis`.
///
/// Revolute joints can be useful for things like wheels, fans, revolving doors etc.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[reflect(from_reflect = false)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct RevoluteJoint {
    /// A unit vector that controls which axis should be aligned for both bodies.
    #[cfg(feature = "3d")]
    pub aligned_axis: Vector,
    /// The extents of the allowed relative rotation of the bodies around the `aligned_axis`.
    pub angle_limit: Option<AngleLimit>,
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

impl Default for RevoluteJoint {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl XpbdConstraint<2> for RevoluteJoint {
    type SolveInput = JointAnchors;

    fn clear_lagrange_multipliers(&mut self) {
        self.position_lagrange = 0.0;
        self.align_lagrange = 0.0;
        self.angle_limit_lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar, anchors: JointAnchors) {
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
            anchors.anchor1,
            anchors.anchor2,
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
    fn with_compliance(self, compliance: Scalar) -> Self {
        Self { compliance, ..self }
    }
}

impl RevoluteJoint {
    /// The default joint configuration.
    pub const DEFAULT: Self = Self {
        #[cfg(feature = "3d")]
        aligned_axis: Vector3::Z,
        angle_limit: None,
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
    };

    /// Creates a new [`RevoluteJoint`].
    #[cfg(feature = "2d")]
    pub const fn new() -> Self {
        Self::DEFAULT
    }

    /// Creates a new [`RevoluteJoint`] with the given aligned axis.
    #[cfg(feature = "3d")]
    pub const fn new(aligned_axis: Vector) -> Self {
        Self {
            aligned_axis,
            ..Self::DEFAULT
        }
    }

    /// Sets the limits of the allowed relative rotation around the `aligned_axis`.
    pub const fn with_angle_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.angle_limit = Some(AngleLimit::new(min, max));
        self
    }

    const fn aligned_axis(&self) -> Vector3 {
        #[cfg(feature = "2d")]
        {
            Vector3::Z
        }
        #[cfg(feature = "3d")]
        {
            self.aligned_axis
        }
    }

    fn get_delta_q(&self, rot1: &Rotation, rot2: &Rotation) -> Vector3 {
        let a1 = rot1.rotate_vec3(self.aligned_axis());
        let a2 = rot2.rotate_vec3(self.aligned_axis());
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
                self.aligned_axis().z,
                self.aligned_axis().x,
                self.aligned_axis().y,
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
