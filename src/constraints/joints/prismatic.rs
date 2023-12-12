//! [`PrismaticJoint`] component.

use crate::prelude::*;
use bevy::prelude::*;

/// A prismatic joint prevents relative movement of the attached bodies, except for translation along one `free_axis`.
///
/// Prismatic joints can be useful for things like elevators, pistons, sliding doors and moving platforms.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct PrismaticJoint {
    /// A free axis that the attached bodies can translate along relative to each other.
    pub free_axis: Vector,
    /// The extents of the allowed relative translation along the free axis.
    pub free_axis_limits: Option<DistanceLimit>,
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

impl Default for PrismaticJoint {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl XpbdConstraint<2> for PrismaticJoint {
    type SolveInput = JointAnchors;

    fn clear_lagrange_multipliers(&mut self) {
        self.position_lagrange = 0.0;
        self.align_lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar, anchors: JointAnchors) {
        let [body1, body2] = bodies;
        let compliance = self.compliance;

        // Align orientations
        let dq = self.get_delta_q(&body1.rotation, &body2.rotation);
        let mut lagrange = self.align_lagrange;
        self.align_torque = self.align_orientation(body1, body2, dq, &mut lagrange, compliance, dt);
        self.align_lagrange = lagrange;

        // Constrain the relative positions of the bodies, only allowing translation along one free axis
        self.force = self.constrain_positions(body1, body2, dt, anchors);
    }
}

impl Joint for PrismaticJoint {
    fn with_compliance(self, compliance: Scalar) -> Self {
        Self { compliance, ..self }
    }
}

impl PrismaticJoint {
    /// The default joint configuration.
    pub const DEFAULT: Self = Self {
        free_axis: Vector::X,
        free_axis_limits: None,
        position_lagrange: 0.0,
        align_lagrange: 0.0,
        compliance: 0.0,
        force: Vector::ZERO,
        #[cfg(feature = "2d")]
        align_torque: 0.0,
        #[cfg(feature = "3d")]
        align_torque: Vector::ZERO,
    };

    /// Creates a new [`PrismaticJoint`] with the given free axis.
    /// Relative translations are allowed along this axis.
    pub const fn new(free_axis: Vector) -> Self {
        Self {
            free_axis,
            ..Self::DEFAULT
        }
    }

    /// Sets the translational limits along the joint's free axis.
    pub const fn with_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.free_axis_limits = Some(DistanceLimit::new(min, max));
        self
    }

    /// Constrains the relative positions of the bodies, only allowing translation along one free axis.
    ///
    /// Returns the force exerted by this constraint.
    fn constrain_positions(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
        anchors: JointAnchors,
    ) -> Vector {
        let world_r1 = body1.rotation.rotate(anchors.first);
        let world_r2 = body2.rotation.rotate(anchors.second);

        let mut delta_x = Vector::ZERO;

        let axis1 = body1.rotation.rotate(self.free_axis);
        if let Some(limits) = self.free_axis_limits {
            delta_x += limits.compute_correction_along_axis(
                body1.current_position() + world_r1,
                body2.current_position() + world_r2,
                axis1,
            );
        }

        let zero_distance_limit = DistanceLimit::ZERO;

        #[cfg(feature = "2d")]
        {
            let axis2 = Vector::new(axis1.y, -axis1.x);
            delta_x += zero_distance_limit.compute_correction_along_axis(
                body1.current_position() + world_r1,
                body2.current_position() + world_r2,
                axis2,
            );
        }
        #[cfg(feature = "3d")]
        {
            let axis2 = axis1.any_orthogonal_vector();
            let axis3 = axis1.cross(axis2);

            delta_x += zero_distance_limit.compute_correction_along_axis(
                body1.current_position() + world_r1,
                body2.current_position() + world_r2,
                axis2,
            );
            delta_x += zero_distance_limit.compute_correction_along_axis(
                body1.current_position() + world_r1,
                body2.current_position() + world_r2,
                axis3,
            );
        }

        let magnitude = delta_x.length();

        if magnitude <= Scalar::EPSILON {
            return Vector::ZERO;
        }

        let dir = delta_x / magnitude;

        // Compute generalized inverse masses
        let w1 = PositionConstraint::compute_generalized_inverse_mass(self, body1, world_r1, dir);
        let w2 = PositionConstraint::compute_generalized_inverse_mass(self, body2, world_r2, dir);

        // Constraint gradients and inverse masses
        let gradients = [dir, -dir];
        let w = [w1, w2];

        // Compute Lagrange multiplier update
        let delta_lagrange = self.compute_lagrange_update(
            self.position_lagrange,
            magnitude,
            &gradients,
            &w,
            self.compliance,
            dt,
        );
        self.position_lagrange += delta_lagrange;

        // Apply positional correction to align the positions of the bodies
        self.apply_positional_correction(body1, body2, delta_lagrange, dir, world_r1, world_r2);

        // Return constraint force
        self.compute_force(self.position_lagrange, dir, dt)
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

impl PositionConstraint for PrismaticJoint {}

impl AngularConstraint for PrismaticJoint {}
