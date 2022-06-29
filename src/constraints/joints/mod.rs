mod revolute;
mod spherical;

pub use revolute::*;
pub use spherical::*;

use crate::prelude::*;
use bevy::prelude::*;
use std::f32::consts::PI;

pub trait Joint: Component + PositionConstraint + AngularConstraint {
    fn new_with_compliance(entity_a: Entity, entity_b: Entity, compliance: f32) -> Self;

    fn with_local_anchor_1(self, anchor: Vector) -> Self;

    fn with_local_anchor_2(self, anchor: Vector) -> Self;

    fn with_lin_vel_damping(self, damping: f32) -> Self;

    fn with_ang_vel_damping(self, damping: f32) -> Self;

    fn entities(&self) -> [Entity; 2];

    fn constrain(
        &mut self,
        body1: &mut ConstraintBodyQueryItem,
        body2: &mut ConstraintBodyQueryItem,
        sub_dt: f32,
    );

    fn limit_angle(
        n: Vec3,
        n1: Vec3,
        n2: Vec3,
        alpha: f32,
        beta: f32,
        max_correction: f32,
    ) -> Option<Vec3> {
        let mut phi = n1.cross(n2).dot(n).asin();

        if n1.dot(n2) < 0.0 {
            phi = PI - phi;
        }

        if phi > PI {
            phi -= 2.0 * PI;
        }

        if phi < -PI {
            phi += 2.0 * PI;
        }

        if phi < alpha || phi > beta {
            phi = phi.clamp(alpha, beta);

            let rot = Quat::from_axis_angle(n, phi);
            let mut omega = rot.mul_vec3(n1).cross(n2);

            phi = omega.length();

            if phi > max_correction {
                omega *= max_correction / phi;
            }

            return Some(omega);
        }

        None
    }
}

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
