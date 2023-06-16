//! General joint logic and different types of built-in joints.

mod fixed;
mod prismatic;
mod revolute;
mod spherical;

pub use fixed::*;
pub use prismatic::*;
pub use revolute::*;
pub use spherical::*;

use crate::prelude::*;
use bevy::prelude::*;

/// Joints are constraints that attach pairs of bodies and restrict their relative positional and rotational degrees of freedom.
pub trait Joint: Component + PositionConstraint + AngularConstraint {
    /// Creates a new joint between two entities.
    fn new(entity1: Entity, entity2: Entity) -> Self;

    /// Sets the joint's compliance (inverse of stiffness, meters / Newton).
    fn with_compliance(self, compliance: Scalar) -> Self;

    /// Sets the attachment point on the first body.
    fn with_local_anchor_1(self, anchor: Vector) -> Self;

    /// Sets the attachment point on the second body.
    fn with_local_anchor_2(self, anchor: Vector) -> Self;

    /// Sets the linear velocity damping caused by the joint.
    fn with_lin_vel_damping(self, damping: Scalar) -> Self;

    /// Sets the angular velocity damping caused by the joint.
    fn with_ang_vel_damping(self, damping: Scalar) -> Self;

    /// Returns the linear velocity damping of the joint.
    fn damping_lin(&self) -> Scalar;

    /// Returns the angular velocity damping of the joint.
    fn damping_ang(&self) -> Scalar;

    /// Applies a positional correction that aligns the positions of the local attachment points `r1` and `r2`.
    ///
    /// Returns the force exerted by the alignment.
    #[allow(clippy::too_many_arguments)]
    fn align_position(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        r1: Vector,
        r2: Vector,
        lagrange: &mut Scalar,
        compliance: Scalar,
        dt: Scalar,
    ) -> Vector {
        let world_r1 = body1.rot.rotate(r1);
        let world_r2 = body2.rot.rotate(r2);

        let delta_x = DistanceLimit::new(0.0, 0.0)
            .compute_correction(body1.pos.0 + world_r1, body2.pos.0 + world_r2);
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
        let delta_lagrange =
            self.compute_lagrange_update(*lagrange, magnitude, &gradients, &w, compliance, dt);
        *lagrange += delta_lagrange;

        // Apply positional correction to align the positions of the bodies
        self.apply_positional_correction(body1, body2, delta_lagrange, dir, world_r1, world_r2);

        // Return constraint force
        self.compute_force(*lagrange, dir, dt)
    }

    /// Applies an angular correction that aligns the orientation of the bodies.
    ///
    /// Returns the torque exerted by the alignment.
    fn align_orientation(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_q: Vector3,
        lagrange: &mut Scalar,
        compliance: Scalar,
        dt: Scalar,
    ) -> Torque {
        let angle = delta_q.length();

        if angle <= Scalar::EPSILON {
            return Torque::ZERO;
        }

        let axis = delta_q / angle;

        // Compute generalized inverse masses
        let w1 = AngularConstraint::compute_generalized_inverse_mass(self, body1, axis);
        let w2 = AngularConstraint::compute_generalized_inverse_mass(self, body2, axis);

        // Constraint gradients and inverse masses
        let gradients = {
            #[cfg(feature = "2d")]
            {
                let axis_vec2 = axis.truncate();
                [axis_vec2, -axis_vec2]
            }
            #[cfg(feature = "3d")]
            {
                [axis, -axis]
            }
        };
        let w = [w1, w2];

        // Compute Lagrange multiplier update
        let delta_lagrange =
            self.compute_lagrange_update(*lagrange, angle, &gradients, &w, compliance, dt);
        *lagrange += delta_lagrange;

        // Apply angular correction to aling the bodies
        self.apply_angular_correction(body1, body2, delta_lagrange, axis);

        // Return constraint torque
        self.compute_torque(*lagrange, axis, dt)
    }
}

/// A limit that indicates that the distance between two points should be between `min` and `max`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistanceLimit {
    pub min: Scalar,
    pub max: Scalar,
}

impl DistanceLimit {
    /// A `DistanceLimit` with `min` and `max` set to zero.
    pub const ZERO: Self = Self { min: 0.0, max: 0.0 };

    /// Creates a new `DistanceLimit`.
    pub fn new(min: Scalar, max: Scalar) -> Self {
        Self { min, max }
    }

    /// Returns the positional correction required to limit the distance between `p1` and `p2` to be
    /// to be inside the distance limit.
    pub fn compute_correction(&self, p1: Vector, p2: Vector) -> Vector {
        let pos_offset = p2 - p1;
        let distance = pos_offset.length();

        if distance <= Scalar::EPSILON {
            return Vector::ZERO;
        }

        // Equation 25
        if distance < self.min {
            // Separation distance lower limit
            -pos_offset / distance * (distance - self.min)
        } else if distance > self.max {
            // Separation distance upper limit
            -pos_offset / distance * (distance - self.max)
        } else {
            Vector::ZERO
        }
    }

    /// Returns the positional correction required to limit the distance between `p1` and `p2`
    /// to be inside the distance limit along a given `axis`.
    fn compute_correction_along_axis(&self, p1: Vector, p2: Vector, axis: Vector) -> Vector {
        let pos_offset = p2 - p1;
        let a = pos_offset.dot(axis);

        // Equation 25
        if a < self.min {
            // Separation distance lower limit
            -axis * (a - self.min)
        } else if a > self.max {
            // Separation distance upper limit
            -axis * (a - self.max)
        } else {
            Vector::ZERO
        }
    }
}

/// A limit that indicates that angles should be between `alpha` and `beta`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngleLimit {
    pub alpha: Scalar,
    pub beta: Scalar,
}

impl AngleLimit {
    /// An `AngleLimit` with `alpha` and `beta` set to zero.
    pub const ZERO: Self = Self {
        alpha: 0.0,
        beta: 0.0,
    };

    /// Creates a new `AngleLimit`.
    pub fn new(alpha: Scalar, beta: Scalar) -> Self {
        Self { alpha, beta }
    }

    /// Returns the angular correction required to limit the angle between the axes `n1` and `n2`
    /// to be inside the angle limits.
    fn compute_correction(
        &self,
        n: Vector3,
        n1: Vector3,
        n2: Vector3,
        max_correction: Scalar,
    ) -> Option<Vector3> {
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

        if phi < self.alpha || phi > self.beta {
            phi = phi.clamp(self.alpha, self.beta);

            let rot = Quaternion::from_axis_angle(n, phi);
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
