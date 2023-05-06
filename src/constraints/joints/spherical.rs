//! [`SphericalJoint`] component.

use crate::prelude::*;
use bevy::prelude::*;

/// A spherical joint prevents relative translation of the attached bodies while allowing rotation around all axes.
///
/// Spherical joints can be useful for things like pendula, chains, ragdolls etc.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
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
    pub swing_limit: Option<JointLimit>,
    /// The extents of the allowed relative rotation of the bodies around the `twist_axis`.
    pub twist_limit: Option<JointLimit>,
    /// Linear damping applied by the joint.
    pub damping_lin: Scalar,
    /// Angular damping applied by the joint.
    pub damping_ang: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub pos_lagrange: Scalar,
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

impl Joint for SphericalJoint {
    fn new_with_compliance(entity1: Entity, entity2: Entity, compliance: Scalar) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            swing_axis: Vector3::X,
            twist_axis: Vector3::Y,
            swing_limit: None,
            twist_limit: None,
            damping_lin: 1.0,
            damping_ang: 1.0,
            pos_lagrange: 0.0,
            swing_lagrange: 0.0,
            twist_lagrange: 0.0,
            compliance,
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
        let world_r1 = body1.rot.rotate(self.local_anchor1);
        let world_r2 = body2.rot.rotate(self.local_anchor2);

        let delta_x = self.limit_distance(0.0, 0.0, world_r1, world_r2, &body1.pos, &body2.pos);
        let magnitude = delta_x.length();

        if magnitude > Scalar::EPSILON {
            let dir = delta_x / magnitude;

            let inv_inertia1 = body1.inv_inertia.rotated(&body1.rot);
            let inv_inertia2 = body2.inv_inertia.rotated(&body2.rot);

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

impl SphericalJoint {
    /// Sets the limits of the allowed relative rotation around the `swing_axis`.
    pub fn with_swing_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            swing_limit: Some(JointLimit::new(min, max)),
            ..self
        }
    }

    /// Sets the limits of the allowed relative rotation around the `twist_axis`.
    #[cfg(feature = "3d")]
    pub fn with_twist_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            twist_limit: Some(JointLimit::new(min, max)),
            ..self
        }
    }

    /// Applies angle limits to limit the relative rotation of the bodies around the `swing_axis` and `twist_axis`.
    fn apply_angle_limits(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        sub_dt: Scalar,
    ) {
        if self.swing_limit.is_none() && self.twist_limit.is_none() {
            return;
        }

        if let Some(swing_limit) = self.swing_limit {
            let a1 = body1.rot.rotate_vec3(self.swing_axis);
            let a2 = body2.rot.rotate_vec3(self.swing_axis);

            let n = a1.cross(a2);
            let n_magnitude = n.length();

            if n_magnitude > Scalar::EPSILON {
                let n = n / n_magnitude;

                if let Some(delta_q) =
                    Self::limit_angle(n, a1, a2, swing_limit.min, swing_limit.max, PI)
                {
                    let angle = delta_q.length();

                    if angle > Scalar::EPSILON {
                        let axis = delta_q / angle;

                        let inv_inertia1 = body1.inv_inertia.rotated(&body1.rot);
                        let inv_inertia2 = body2.inv_inertia.rotated(&body2.rot);

                        let delta_ang_lagrange = Self::get_delta_ang_lagrange(
                            &body1.rb,
                            &body2.rb,
                            inv_inertia1,
                            inv_inertia2,
                            self.swing_lagrange,
                            axis,
                            angle,
                            self.compliance,
                            sub_dt,
                        );

                        self.swing_lagrange += delta_ang_lagrange;

                        Self::apply_ang_constraint(
                            body1,
                            body2,
                            inv_inertia1,
                            inv_inertia2,
                            delta_ang_lagrange,
                            axis,
                        );

                        self.update_swing_torque(axis, sub_dt);
                    }
                }
            }
        }

        if let Some(twist_limit) = self.twist_limit {
            let a1 = body1.rot.rotate_vec3(self.swing_axis);
            let a2 = body2.rot.rotate_vec3(self.swing_axis);

            let b1 = body1.rot.rotate_vec3(self.twist_axis);
            let b2 = body2.rot.rotate_vec3(self.twist_axis);

            let n = a1 + a2;
            let n_magnitude = n.length();

            if n_magnitude > Scalar::EPSILON {
                let n = n / n_magnitude;

                let n1 = b1 - n.dot(b1) * n;
                let n2 = b2 - n.dot(b2) * n;
                let n1_magnitude = n1.length();
                let n2_magnitude = n2.length();

                if n1_magnitude > Scalar::EPSILON && n2_magnitude > Scalar::EPSILON {
                    let n1 = n1 / n1_magnitude;
                    let n2 = n2 / n2_magnitude;

                    let max_correction = if a1.dot(a2) > -0.5 { 2.0 * PI } else { sub_dt };

                    if let Some(delta_q) = Self::limit_angle(
                        n,
                        n1,
                        n2,
                        twist_limit.min,
                        twist_limit.max,
                        max_correction,
                    ) {
                        let angle = delta_q.length();

                        if angle > Scalar::EPSILON {
                            let axis = delta_q / angle;

                            let inv_inertia1 = body1.inv_inertia.rotated(&body1.rot);
                            let inv_inertia2 = body2.inv_inertia.rotated(&body2.rot);

                            let delta_ang_lagrange = Self::get_delta_ang_lagrange(
                                &body1.rb,
                                &body2.rb,
                                inv_inertia1,
                                inv_inertia2,
                                self.twist_lagrange,
                                axis,
                                angle,
                                self.compliance,
                                sub_dt,
                            );

                            self.twist_lagrange += delta_ang_lagrange;

                            Self::apply_ang_constraint(
                                body1,
                                body2,
                                inv_inertia1,
                                inv_inertia2,
                                delta_ang_lagrange,
                                axis,
                            );

                            self.update_twist_torque(axis, sub_dt);
                        }
                    }
                }
            }
        }
    }

    fn update_force(&mut self, dir: Vector, sub_dt: Scalar) {
        // Eq (10)
        self.force = self.pos_lagrange * dir / sub_dt.powi(2);
    }

    fn update_swing_torque(&mut self, axis: Vector3, sub_dt: Scalar) {
        // Eq (17)
        #[cfg(feature = "2d")]
        {
            self.swing_torque = self.swing_lagrange * axis.z / sub_dt.powi(2);
        }
        #[cfg(feature = "3d")]
        {
            self.swing_torque = self.swing_lagrange * axis / sub_dt.powi(2);
        }
    }

    fn update_twist_torque(&mut self, axis: Vector3, sub_dt: Scalar) {
        // Eq (17)
        #[cfg(feature = "2d")]
        {
            self.twist_torque = self.twist_lagrange * axis.z / sub_dt.powi(2);
        }
        #[cfg(feature = "3d")]
        {
            self.twist_torque = self.twist_lagrange * axis / sub_dt.powi(2);
        }
    }
}

impl Constraint for SphericalJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.pos_lagrange = 0.0;
        self.swing_lagrange = 0.0;
        self.twist_lagrange = 0.0;
    }
}

impl PositionConstraint for SphericalJoint {}

impl AngularConstraint for SphericalJoint {}
