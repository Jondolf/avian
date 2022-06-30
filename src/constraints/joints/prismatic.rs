use crate::prelude::*;
use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PrismaticJoint {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub local_anchor_a: Vector,
    pub local_anchor_b: Vector,
    pub free_axis: Vector,
    pub free_axis_limits: Option<JointLimit>,
    pub damping_lin: f32,
    pub damping_ang: f32,
    pub pos_lagrange: f32,
    pub align_lagrange: f32,
    pub compliance: f32,
    pub force: Vector,
    pub torque: f32,
}

impl Joint for PrismaticJoint {
    fn new_with_compliance(entity_a: Entity, entity_b: Entity, compliance: f32) -> Self {
        Self {
            entity_a,
            entity_b,
            local_anchor_a: Vector::ZERO,
            local_anchor_b: Vector::ZERO,
            free_axis: Vector::X,
            free_axis_limits: None,
            damping_lin: 1.0,
            damping_ang: 1.0,
            pos_lagrange: 0.0,
            align_lagrange: 0.0,
            compliance,
            force: Vector::ZERO,
            torque: 0.0,
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

    #[allow(clippy::too_many_arguments)]
    fn constrain(
        &mut self,
        body1: &mut ConstraintBodyQueryItem,
        body2: &mut ConstraintBodyQueryItem,
        sub_dt: f32,
    ) {
        let delta_q = self.get_delta_q(&body1.rot, &body2.rot);
        let angle = delta_q.length();

        if angle > f32::EPSILON {
            let axis = delta_q / angle;

            let inv_inertia1 = body1.mass_props.world_inv_inertia(&body1.rot);
            let inv_inertia2 = body2.mass_props.world_inv_inertia(&body2.rot);

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
                -axis,
            );
        }

        let world_r_a = body1.rot.rotate(self.local_anchor_a);
        let world_r_b = body2.rot.rotate(self.local_anchor_b);

        let mut delta_x = Vector::ZERO;

        let axis1 = self.free_axis;
        if let Some(limits) = self.free_axis_limits {
            delta_x += self.limit_distance_along_axis(
                limits.min, limits.max, axis1, world_r_a, world_r_b, &body1.pos, &body2.pos,
            );
        }

        #[cfg(feature = "2d")]
        {
            let axis2 = Vec2::new(axis1.y, -axis1.x);
            delta_x += self.limit_distance_along_axis(
                0.0, 0.0, axis2, world_r_a, world_r_b, &body1.pos, &body2.pos,
            );
        }
        #[cfg(feature = "3d")]
        {
            let axis2 = axis1.cross(Vec3::Y);
            let axis3 = axis1.cross(axis2);

            delta_x += self.limit_distance_along_axis(
                0.0, 0.0, axis2, world_r_a, world_r_b, &body1.pos, &body2.pos,
            );
            delta_x += self.limit_distance_along_axis(
                0.0, 0.0, axis3, world_r_a, world_r_b, &body1.pos, &body2.pos,
            );
        }

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
    }
}

impl PrismaticJoint {
    pub fn with_free_axis(self, axis: Vector) -> Self {
        Self {
            free_axis: axis,
            ..self
        }
    }

    pub fn with_limits(self, min: f32, max: f32) -> Self {
        Self {
            free_axis_limits: Some(JointLimit::new(min, max)),
            ..self
        }
    }

    #[cfg(feature = "2d")]
    fn get_delta_q(&self, rot_a: &Rot, rot_b: &Rot) -> Vec3 {
        rot_a.mul(rot_b.inv()).as_radians() * Vec3::Z
    }

    #[cfg(feature = "3d")]
    fn get_delta_q(&self, rot_a: &Rot, rot_b: &Rot) -> Vec3 {
        2.0 * (rot_a.0 * rot_b.inverse()).xyz()
    }

    fn update_force(&mut self, dir: Vector, sub_dt: f32) {
        // Equation 10
        self.force = self.pos_lagrange * dir / sub_dt.powi(2);
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
