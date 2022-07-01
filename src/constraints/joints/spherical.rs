use crate::prelude::*;
use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct SphericalJoint {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub local_anchor_a: Vector,
    pub local_anchor_b: Vector,
    pub swing_axis: Vec3,
    pub twist_axis: Vec3,
    pub swing_limit: Option<JointLimit>,
    pub twist_limit: Option<JointLimit>,
    pub damping_lin: f32,
    pub damping_ang: f32,
    pub pos_lagrange: f32,
    pub swing_lagrange: f32,
    pub twist_lagrange: f32,
    pub compliance: f32,
    pub force: Vector,
    pub swing_torque: Torque,
    pub twist_torque: Torque,
}

impl Joint for SphericalJoint {
    fn new_with_compliance(entity_a: Entity, entity_b: Entity, compliance: f32) -> Self {
        Self {
            entity_a,
            entity_b,
            local_anchor_a: Vector::ZERO,
            local_anchor_b: Vector::ZERO,
            swing_axis: Vec3::X,
            twist_axis: Vec3::Y,
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
            swing_torque: Vec3::ZERO,
            #[cfg(feature = "2d")]
            twist_torque: 0.0,
            #[cfg(feature = "3d")]
            twist_torque: Vec3::ZERO,
        }
    }

    fn with_local_anchor_1(self, anchor: Vector) -> Self {
        Self {
            local_anchor_a: anchor,
            ..self
        }
    }

    fn with_local_anchor_2(self, anchor: Vector) -> Self {
        Self {
            local_anchor_b: anchor,
            ..self
        }
    }

    fn with_lin_vel_damping(self, damping: f32) -> Self {
        Self {
            damping_lin: damping,
            ..self
        }
    }

    fn with_ang_vel_damping(self, damping: f32) -> Self {
        Self {
            damping_ang: damping,
            ..self
        }
    }

    fn entities(&self) -> [Entity; 2] {
        [self.entity_a, self.entity_b]
    }

    fn damping_lin(&self) -> f32 {
        self.damping_lin
    }

    fn damping_ang(&self) -> f32 {
        self.damping_ang
    }

    #[allow(clippy::too_many_arguments)]
    fn constrain(
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

            let inv_inertia1 = body1.mass_props.world_inv_inertia(&body1.rot);
            let inv_inertia2 = body2.mass_props.world_inv_inertia(&body2.rot);

            let delta_lagrange = Self::get_delta_pos_lagrange(
                body1,
                body2,
                inv_inertia1,
                inv_inertia2,
                self.pos_lagrange,
                dir,
                magnitude,
                world_r_a,
                world_r_b,
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
                world_r_a,
                world_r_b,
            );

            self.update_force(dir, sub_dt);
        }

        self.apply_angle_limits(body1, body2, sub_dt);
    }
}

impl SphericalJoint {
    pub fn with_swing_limits(self, min: f32, max: f32) -> Self {
        Self {
            swing_limit: Some(JointLimit::new(min, max)),
            ..self
        }
    }

    #[cfg(feature = "3d")]
    pub fn with_twist_limits(self, min: f32, max: f32) -> Self {
        Self {
            twist_limit: Some(JointLimit::new(min, max)),
            ..self
        }
    }

    fn apply_angle_limits(
        &mut self,
        body1: &mut ConstraintBodyQueryItem,
        body2: &mut ConstraintBodyQueryItem,
        sub_dt: f32,
    ) {
        if self.swing_limit.is_none() && self.twist_limit.is_none() {
            return;
        }

        if let Some(swing_limit) = self.swing_limit {
            let a1 = body1.rot.rotate_vec3(self.swing_axis);
            let a2 = body2.rot.rotate_vec3(self.swing_axis);

            let n = a1.cross(a2);
            let n_magnitude = n.length();

            if n_magnitude > f32::EPSILON {
                let n = n / n_magnitude;

                if let Some(delta_q) = Self::limit_angle(
                    n,
                    a1,
                    a2,
                    swing_limit.min,
                    swing_limit.max,
                    std::f32::consts::PI,
                ) {
                    let angle = delta_q.length();

                    if angle > f32::EPSILON {
                        let axis = delta_q / angle;

                        let inv_inertia1 = body1.mass_props.world_inv_inertia(&body1.rot);
                        let inv_inertia2 = body2.mass_props.world_inv_inertia(&body2.rot);

                        let delta_ang_lagrange = Self::get_delta_ang_lagrange(
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

            if n_magnitude > f32::EPSILON {
                let n = n / n_magnitude;

                let n1 = b1 - n.dot(b1) * n;
                let n2 = b2 - n.dot(b2) * n;
                let n1_magnitude = n1.length();
                let n2_magnitude = n2.length();

                if n1_magnitude > f32::EPSILON && n2_magnitude > f32::EPSILON {
                    let n1 = n1 / n1_magnitude;
                    let n2 = n2 / n2_magnitude;

                    let max_correction = if a1.dot(a2) > -0.5 {
                        2.0 * std::f32::consts::PI
                    } else {
                        sub_dt
                    };

                    if let Some(delta_q) = Self::limit_angle(
                        n,
                        n1,
                        n2,
                        twist_limit.min,
                        twist_limit.max,
                        max_correction,
                    ) {
                        let angle = delta_q.length();

                        if angle > f32::EPSILON {
                            let axis = delta_q / angle;

                            let inv_inertia1 = body1.mass_props.world_inv_inertia(&body1.rot);
                            let inv_inertia2 = body2.mass_props.world_inv_inertia(&body2.rot);

                            let delta_ang_lagrange = Self::get_delta_ang_lagrange(
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

    fn update_force(&mut self, dir: Vector, sub_dt: f32) {
        // Eq (10)
        self.force = self.pos_lagrange * dir / sub_dt.powi(2);
    }

    fn update_swing_torque(&mut self, axis: Vec3, sub_dt: f32) {
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

    fn update_twist_torque(&mut self, axis: Vec3, sub_dt: f32) {
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
