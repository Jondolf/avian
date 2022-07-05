use crate::prelude::*;
use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct RevoluteJoint {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub local_anchor_a: Vector,
    pub local_anchor_b: Vector,
    #[cfg(feature = "2d")]
    /// A unit vector that controls which axis is aligned for both entities.
    ///
    /// In 2D this should always be the Z axis.
    pub(crate) aligned_axis: Vec3,
    #[cfg(feature = "3d")]
    /// A unit vector that controls which axis is aligned for both entities.
    pub aligned_axis: Vec3,
    pub angle_limit: Option<JointLimit>,
    pub damping_lin: f32,
    pub damping_ang: f32,
    pub pos_lagrange: f32,
    pub align_lagrange: f32,
    pub angle_limit_lagrange: f32,
    pub compliance: f32,
    pub force: Vector,
    pub align_torque: Torque,
    pub angle_limit_torque: Torque,
}

impl Joint for RevoluteJoint {
    fn new_with_compliance(entity_a: Entity, entity_b: Entity, compliance: f32) -> Self {
        Self {
            entity_a,
            entity_b,
            local_anchor_a: Vector::ZERO,
            local_anchor_b: Vector::ZERO,
            aligned_axis: Vec3::Z,
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
            align_torque: Vec3::ZERO,
            #[cfg(feature = "2d")]
            angle_limit_torque: 0.0,
            #[cfg(feature = "3d")]
            angle_limit_torque: Vec3::ZERO,
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
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        sub_dt: f32,
    ) {
        let delta_q = self.get_delta_q(&body1.rot, &body2.rot);
        let angle = delta_q.length();

        if angle > f32::EPSILON {
            let axis = delta_q / angle;

            let inv_inertia1 = body1.inv_inertia.rotated(&body1.rot);
            let inv_inertia2 = body2.inv_inertia.rotated(&body2.rot);

            let delta_ang_lagrange = Self::get_delta_ang_lagrange(
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

        let world_r_a = body1.rot.rotate(self.local_anchor_a);
        let world_r_b = body2.rot.rotate(self.local_anchor_b);

        let delta_x = self.limit_distance(0.0, 0.0, world_r_a, world_r_b, &body1.pos, &body2.pos);
        let magnitude = delta_x.length();

        if magnitude > f32::EPSILON {
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

impl RevoluteJoint {
    #[cfg(feature = "3d")]
    pub fn with_aligned_axis(self, axis: Vec3) -> Self {
        Self {
            aligned_axis: axis,
            ..self
        }
    }

    pub fn with_angle_limits(self, min: f32, max: f32) -> Self {
        Self {
            angle_limit: Some(JointLimit::new(min, max)),
            ..self
        }
    }

    fn get_delta_q(&self, rot_a: &Rot, rot_b: &Rot) -> Vec3 {
        let a1 = rot_a.rotate_vec3(self.aligned_axis);
        let a2 = rot_b.rotate_vec3(self.aligned_axis);
        a1.cross(a2)
    }

    #[allow(clippy::too_many_arguments)]
    fn apply_angle_limits(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        sub_dt: f32,
    ) {
        if let Some(angle_limit) = self.angle_limit {
            let limit_axis = Vec3::new(
                self.aligned_axis.z,
                self.aligned_axis.x,
                self.aligned_axis.y,
            );
            let a1 = body1.rot.rotate_vec3(limit_axis);
            let a2 = body2.rot.rotate_vec3(limit_axis);
            let n = a1.cross(a2).normalize();

            if let Some(delta_q) = Self::limit_angle(
                n,
                a1,
                a2,
                angle_limit.min,
                angle_limit.max,
                std::f32::consts::PI,
            ) {
                let angle = delta_q.length();

                if angle > f32::EPSILON {
                    let axis = delta_q / angle;

                    let inv_inertia1 = body1.inv_inertia.rotated(&body1.rot);
                    let inv_inertia2 = body2.inv_inertia.rotated(&body2.rot);

                    let delta_ang_lagrange = Self::get_delta_ang_lagrange(
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

    fn update_force(&mut self, dir: Vector, sub_dt: f32) {
        // Eq (10)
        self.force = self.pos_lagrange * dir / sub_dt.powi(2);
    }

    fn update_align_torque(&mut self, axis: Vec3, sub_dt: f32) {
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

    fn update_angle_limit_torque(&mut self, axis: Vec3, sub_dt: f32) {
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
