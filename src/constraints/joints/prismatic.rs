use crate::prelude::*;
use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PrismaticJoint {
    pub entity1: Entity,
    pub entity2: Entity,
    pub local_anchor1: Vector,
    pub local_anchor2: Vector,
    pub free_axis: Vector,
    pub free_axis_limits: Option<JointLimit>,
    pub damping_lin: Scalar,
    pub damping_ang: Scalar,
    pub pos_lagrange: Scalar,
    pub align_lagrange: Scalar,
    pub compliance: Scalar,
    pub force: Vector,
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

            let inv_inertia1 = body1.inv_inertia.rotated(&body1.rot);
            let inv_inertia2 = body2.inv_inertia.rotated(&body2.rot);

            let delta_ang_lagrange = Self::get_delta_ang_lagrange(
                body1.rb,
                body2.rb,
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
                -axis,
            );

            self.update_align_torque(axis, sub_dt);
        }

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
    }
}

impl PrismaticJoint {
    pub fn with_free_axis(self, axis: Vector) -> Self {
        Self {
            free_axis: axis,
            ..self
        }
    }

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
}

impl Constraint for PrismaticJoint {
    fn clear_lagrange_multipliers(&mut self) {
        self.pos_lagrange = 0.0;
        self.align_lagrange = 0.0;
    }
}

impl PositionConstraint for PrismaticJoint {}

impl AngularConstraint for PrismaticJoint {}
