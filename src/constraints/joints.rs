use super::{AngularConstraint, Constraint, PositionConstraint};
use crate::{components::ConstraintBodyQueryItem, Vector};

use bevy::prelude::*;

/*
#[cfg(feature = "3d")]
use crate::utils::{get_quat_axis_1, get_quat_axis_2};
#[cfg(feature = "3d")]
use std::f32::consts::PI;
*/

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimit {
    pub min: f32,
    pub max: f32,
}

impl JointLimit {
    pub fn new(min: f32, max: f32) -> Self {
        Self { min, max }
    }
}

#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct SphericalJoint {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub local_anchor_a: Vector,
    pub local_anchor_b: Vector,
    //pub swing_limit: Option<JointLimit>,
    //pub twist_limit: Option<JointLimit>,
    pub damping_lin: f32,
    pub damping_ang: f32,
    pub pos_lagrange: f32,
    pub swing_lagrange: f32,
    pub twist_lagrange: f32,
    pub compliance: f32,
    pub force: Vector,
    pub torque: f32,
}

impl SphericalJoint {
    pub fn new_with_compliance(entity_a: Entity, entity_b: Entity, compliance: f32) -> Self {
        Self {
            entity_a,
            entity_b,
            local_anchor_a: Vector::ZERO,
            local_anchor_b: Vector::ZERO,
            //swing_limit: None,
            //twist_limit: None,
            damping_lin: 5.0,
            damping_ang: 5.0,
            pos_lagrange: 0.0,
            swing_lagrange: 0.0,
            twist_lagrange: 0.0,
            compliance,
            force: Vector::ZERO,
            torque: 0.0,
        }
    }

    pub fn with_local_anchor_1(self, anchor: Vector) -> Self {
        Self {
            local_anchor_a: anchor,
            ..self
        }
    }

    pub fn with_local_anchor_2(self, anchor: Vector) -> Self {
        Self {
            local_anchor_b: anchor,
            ..self
        }
    }

    pub fn with_lin_vel_damping(self, damping: f32) -> Self {
        Self {
            damping_lin: damping,
            ..self
        }
    }

    pub fn with_ang_vel_damping(self, damping: f32) -> Self {
        Self {
            damping_ang: damping,
            ..self
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn constrain(
        &mut self,
        body1: &mut ConstraintBodyQueryItem,
        body2: &mut ConstraintBodyQueryItem,
        sub_dt: f32,
    ) {
        let world_r_a = body1.rot.rotate(self.local_anchor_a);
        let world_r_b = body2.rot.rotate(self.local_anchor_b);

        let delta_x = self.limit_distance(0.0, 0.0, world_r_a, world_r_b, &body1.pos, &body2.pos);
        let magnitude = delta_x.length();

        if magnitude > f32::EPSILON {
            let dir = delta_x / magnitude;

            let delta_lagrange = Self::get_delta_pos_lagrange(
                body1,
                body2,
                self.pos_lagrange,
                dir,
                magnitude,
                world_r_a,
                world_r_b,
                self.compliance,
                sub_dt,
            );

            self.pos_lagrange += delta_lagrange;

            Self::apply_pos_constraint(body1, body2, delta_lagrange, dir, world_r_a, world_r_b);

            self.update_force(dir, sub_dt);
        }

        /*
        cfg_if! {
            if #[cfg(feature = "3d")] {
                self.apply_angle_limits(
                    rb_a,
                    rb_b,
                    rot_a,
                    rot_b,
                    mass_props_a,
                    mass_props_b,
                    sub_dt,
                );
            }
        }
        */
    }

    /*
    #[cfg(feature = "3d")]
    #[allow(clippy::too_many_arguments)]
    fn apply_angle_limits(
        &mut self,
        rb_a: &RigidBody,
        rb_b: &RigidBody,
        rot_a: &mut Rot,
        rot_b: &mut Rot,
        mass_props_a: &MassProperties,
        mass_props_b: &MassProperties,
        sub_dt: f32,
    ) {
        if let Some(swing_limit) = self.swing_limit {
            let a1 = get_quat_axis_1(rot_a.0);
            let a2 = get_quat_axis_1(rot_b.0);
            let n = a1.cross(a2).normalize();

            if let Some(delta_q) =
                Self::limit_angle(n, a1, a2, swing_limit.min, swing_limit.max, PI)
            {
                Self::_constrain_angle(
                    delta_q,
                    rb_a,
                    rb_b,
                    rot_a,
                    rot_b,
                    &mass_props_a.with_rotation(rot_a),
                    &mass_props_b.with_rotation(rot_b),
                    &mut self.swing_lagrange,
                    self.compliance,
                    sub_dt,
                );
            }
        }

        if let Some(twist_limit) = self.twist_limit {
            let a1 = get_quat_axis_1(rot_a.0);
            let a2 = get_quat_axis_1(rot_b.0);

            let b1 = get_quat_axis_2(rot_a.0);
            let b2 = get_quat_axis_2(rot_b.0);

            let n = a1 + a2;
            let n_magnitude = n.length();
            if n_magnitude > f32::EPSILON {
                let n = n / n_magnitude;

                let n1 = b1 - n.dot(b1) * n;
                let n2 = b2 - n.dot(b2) * n;
                let n1_magnitude = n1.length();
                let n2_magnitude = n2.length();

                if n1_magnitude > f32::EPSILON && n2_magnitude > f32::EPSILON {
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
                        Self::_constrain_angle(
                            delta_q,
                            rb_a,
                            rb_b,
                            rot_a,
                            rot_b,
                            &mass_props_a.with_rotation(rot_a),
                            &mass_props_b.with_rotation(rot_b),
                            &mut self.twist_lagrange,
                            self.compliance,
                            sub_dt,
                        );
                    }
                }
            }
        }
    }
    */

    fn update_force(&mut self, dir: Vector, sub_dt: f32) {
        // Equation 10
        self.force = self.pos_lagrange * dir / sub_dt.powi(2);
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
