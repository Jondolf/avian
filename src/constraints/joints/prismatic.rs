//! [`PrismaticJoint`] component.

use crate::prelude::*;
use bevy::prelude::*;

/// A prismatic joint prevents relative movement of the attached bodies, except for translation along one `free_axis`.
///
/// Prismatic joints can be useful for things like elevators, pistons, sliding doors and moving platforms.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PrismaticJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// A free axis that the attached bodies can translate along relative to each other.
    pub free_axis: Vector,
    /// The extents of the allowed relative translation along the free axis.
    pub free_axis_limits: Option<JointLimit>,
    /// Linear damping applied by the joint.
    pub damping_lin: Scalar,
    /// Angular damping applied by the joint.
    pub damping_ang: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub pos_lagrange: Scalar,
    /// Lagrange multiplier for the angular correction caused by the alignment of the bodies.
    pub align_lagrange: Scalar,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// The force exerted by the joint.
    pub force: Vector,
    /// The torque exerted by the joint when aligning the bodies.
    pub align_torque: Torque,
}

impl Joint for PrismaticJoint {
    fn new_with_compliance(entity1: Entity, entity2: Entity, compliance: Scalar) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            free_axis: Vector::X,
            free_axis_limits: None,
            damping_lin: 1.0,
            damping_ang: 1.0,
            pos_lagrange: 0.0,
            align_lagrange: 0.0,
            compliance,
            force: Vector::ZERO,
            #[cfg(feature = "2d")]
            align_torque: 0.0,
            #[cfg(feature = "3d")]
            align_torque: Vector::ZERO,
        }
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

    fn with_lin_vel_damping(self, damping: Scalar) -> Self {
        Self {
            damping_lin: damping,
            ..self
        }
    }

    fn with_ang_vel_damping(self, damping: Scalar) -> Self {
        Self {
            damping_ang: damping,
            ..self
        }
    }

    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

    fn damping_lin(&self) -> Scalar {
        self.damping_lin
    }

    fn damping_ang(&self) -> Scalar {
        self.damping_ang
    }

    fn solve(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) {
        let compliance = self.compliance;

        // Align orientations
        let dq = self.get_delta_q(&body1.rot, &body2.rot);
        let mut lagrange = self.align_lagrange;
        self.align_torque = self.align_orientation(body1, body2, dq, &mut lagrange, compliance, dt);
        self.align_lagrange = lagrange;

        // Constrain the relative positions of the bodies, only allowing translation along one free axis
        self.force = self.constrain_positions(body1, body2, dt);
    }
}

impl PrismaticJoint {
    /// Constrains the relative positions of the bodies, only allowing translation along one free axis.
    ///
    /// Returns the force exerted by this constraint.
    fn constrain_positions(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        dt: Scalar,
    ) -> Vector {
        let world_r1 = body1.rot.rotate(self.local_anchor1);
        let world_r2 = body2.rot.rotate(self.local_anchor2);

        let mut delta_x = Vector::ZERO;

        let axis1 = body1.rot.rotate(self.free_axis);
        if let Some(limits) = self.free_axis_limits {
            delta_x += self.limit_distance_along_axis(
                limits.min, limits.max, axis1, world_r1, world_r2, &body1.pos, &body2.pos,
            );
        }

        #[cfg(feature = "2d")]
        {
            let axis2 = Vector::new(axis1.y, -axis1.x);
            delta_x += self.limit_distance_along_axis(
                0.0, 0.0, axis2, world_r1, world_r2, &body1.pos, &body2.pos,
            );
        }
        #[cfg(feature = "3d")]
        {
            let axis2 = axis1.cross(Vector::Y);
            let axis3 = axis1.cross(axis2);

            delta_x += self.limit_distance_along_axis(
                0.0, 0.0, axis2, world_r1, world_r2, &body1.pos, &body2.pos,
            );
            delta_x += self.limit_distance_along_axis(
                0.0, 0.0, axis3, world_r1, world_r2, &body1.pos, &body2.pos,
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
            self.pos_lagrange,
            magnitude,
            &gradients,
            &w,
            self.compliance,
            dt,
        );
        self.pos_lagrange += delta_lagrange;

        // Apply positional correction to align the positions of the bodies
        self.apply_positional_correction(body1, body2, delta_lagrange, dir, world_r1, world_r2);

        // Return constraint force
        self.compute_force(self.pos_lagrange, dir, dt)
    }

    /// Sets the joint's free axis. Relative translations are allowed along this free axis.
    pub fn with_free_axis(self, axis: Vector) -> Self {
        Self {
            free_axis: axis,
            ..self
        }
    }

    /// Sets the translational limits along the joint's free axis.
    pub fn with_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            free_axis_limits: Some(JointLimit::new(min, max)),
            ..self
        }
    }

    #[cfg(feature = "2d")]
    fn get_delta_q(&self, rot1: &Rot, rot2: &Rot) -> Vector3 {
        rot1.mul(rot2.inv()).as_radians() * Vector3::Z
    }

    #[cfg(feature = "3d")]
    fn get_delta_q(&self, rot1: &Rot, rot2: &Rot) -> Vector {
        2.0 * (rot1.0 * rot2.inverse()).xyz()
    }
}

impl XpbdConstraint for PrismaticJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.pos_lagrange = 0.0;
        self.align_lagrange = 0.0;
    }
}

impl PositionConstraint for PrismaticJoint {}

impl AngularConstraint for PrismaticJoint {}
