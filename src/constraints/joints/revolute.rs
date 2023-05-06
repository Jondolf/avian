//! [`RevoluteJoint`] component.

use crate::prelude::*;
use bevy::prelude::*;

/// A revolute joint prevents relative movement of the attached bodies, except for rotation around one `aligned_axis`.
///
/// Revolute joints can be useful for things like wheels, fans, revolving doors etc.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
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
    pub angle_limit: Option<JointLimit>,
    /// Linear damping applied by the joint.
    pub damping_lin: Scalar,
    /// Angular damping applied by the joint.
    pub damping_ang: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub pos_lagrange: Scalar,
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

impl Joint for RevoluteJoint {
    fn new_with_compliance(entity1: Entity, entity2: Entity, compliance: Scalar) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            aligned_axis: Vector3::Z,
            angle_limit: None,
            damping_lin: 1.0,
            damping_ang: 1.0,
            pos_lagrange: 0.0,
            align_lagrange: 0.0,
            angle_limit_lagrange: 0.0,
            compliance,
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

    #[allow(clippy::too_many_arguments)]
    fn constrain(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        sub_dt: Scalar,
    ) {
        let delta_q = self.get_delta_q(&body1.rot, &body2.rot);
        let angle = delta_q.length();

        if angle > Scalar::EPSILON {
            let axis = delta_q / angle;

            let inv_inertia1 = body1.world_inv_inertia();
            let inv_inertia2 = body2.world_inv_inertia();

            let delta_ang_lagrange = Self::get_delta_ang_lagrange(
                &body1.rb,
                &body2.rb,
                inv_inertia1,
                inv_inertia2,
                self.align_lagrange,
                axis,
                angle,
                self.compliance,
                sub_dt,
            );

            self.align_lagrange += delta_ang_lagrange;

            Self::apply_ang_constraint(
                body1,
                body2,
                inv_inertia1,
                inv_inertia2,
                delta_ang_lagrange,
                axis,
            );

            self.update_align_torque(axis, sub_dt);
        }

        let world_r1 = body1.rot.rotate(self.local_anchor1);
        let world_r2 = body2.rot.rotate(self.local_anchor2);

        let delta_x = self.limit_distance(0.0, 0.0, world_r1, world_r2, &body1.pos, &body2.pos);
        let magnitude = delta_x.length();

        if magnitude > Scalar::EPSILON {
            let dir = delta_x / magnitude;

            let inv_inertia1 = body1.world_inv_inertia();
            let inv_inertia2 = body2.world_inv_inertia();

            let delta_lagrange = Self::get_delta_pos_lagrange(
                body1,
                body2,
                inv_inertia1,
                inv_inertia2,
                self.pos_lagrange,
                dir,
                magnitude,
                world_r1,
                world_r2,
                self.compliance,
                sub_dt,
            );

            self.pos_lagrange += delta_lagrange;

            Self::apply_pos_constraint(
                body1,
                body2,
                inv_inertia1,
                inv_inertia2,
                delta_lagrange,
                dir,
                world_r1,
                world_r2,
            );

            self.update_force(dir, sub_dt);
        }

        self.apply_angle_limits(body1, body2, sub_dt);
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
            angle_limit: Some(JointLimit::new(min, max)),
            ..self
        }
    }

    fn get_delta_q(&self, rot1: &Rot, rot2: &Rot) -> Vector3 {
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
        sub_dt: Scalar,
    ) {
        if let Some(angle_limit) = self.angle_limit {
            let limit_axis = Vector3::new(
                self.aligned_axis.z,
                self.aligned_axis.x,
                self.aligned_axis.y,
            );
            let a1 = body1.rot.rotate_vec3(limit_axis);
            let a2 = body2.rot.rotate_vec3(limit_axis);
            let n = a1.cross(a2).normalize();

            if let Some(delta_q) =
                Self::limit_angle(n, a1, a2, angle_limit.min, angle_limit.max, PI)
            {
                let angle = delta_q.length();

                if angle > Scalar::EPSILON {
                    let axis = delta_q / angle;

                    let inv_inertia1 = body1.world_inv_inertia();
                    let inv_inertia2 = body2.world_inv_inertia();

                    let delta_ang_lagrange = Self::get_delta_ang_lagrange(
                        &body1.rb,
                        &body2.rb,
                        inv_inertia1,
                        inv_inertia2,
                        self.angle_limit_lagrange,
                        axis,
                        angle,
                        self.compliance,
                        sub_dt,
                    );

                    self.angle_limit_lagrange += delta_ang_lagrange;

                    Self::apply_ang_constraint(
                        body1,
                        body2,
                        inv_inertia1,
                        inv_inertia2,
                        delta_ang_lagrange,
                        axis,
                    );

                    self.update_angle_limit_torque(axis, sub_dt);
                }
            }
        }
    }

    fn update_force(&mut self, dir: Vector, sub_dt: Scalar) {
        // Eq (10)
        self.force = self.pos_lagrange * dir / sub_dt.powi(2);
    }

    fn update_align_torque(&mut self, axis: Vector3, sub_dt: Scalar) {
        // Eq (17)
        #[cfg(feature = "2d")]
        {
            self.align_torque = self.align_lagrange * axis.z / sub_dt.powi(2);
        }
        #[cfg(feature = "3d")]
        {
            self.align_torque = self.align_lagrange * axis / sub_dt.powi(2);
        }
    }

    fn update_angle_limit_torque(&mut self, axis: Vector3, sub_dt: Scalar) {
        // Eq (17)
        #[cfg(feature = "2d")]
        {
            self.angle_limit_torque = self.angle_limit_lagrange * axis.z / sub_dt.powi(2);
        }
        #[cfg(feature = "3d")]
        {
            self.angle_limit_torque = self.angle_limit_lagrange * axis / sub_dt.powi(2);
        }
    }
}

impl Constraint for RevoluteJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.pos_lagrange = 0.0;
        self.align_lagrange = 0.0;
        self.angle_limit_lagrange = 0.0;
    }
}

impl PositionConstraint for RevoluteJoint {}

impl AngularConstraint for RevoluteJoint {}
