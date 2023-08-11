use crate::prelude::*;

/// Stores data related to a joint motor.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct JointMotor {
    /// The target angle. During each substep, this is increased by `sub_dt * target_velocity`.
    /// This way, the motor will drive the joint according to the target velocity.
    pub target_angle: Scalar,
    /// The target velocity of the joint motor in radians per second.
    pub target_velocity: Scalar,
    /// The joint motor's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// Lagrange multiplier for the angular corrections caused by joint motor.
    pub lagrange: Scalar,
    pub torque: Torque,
    pub max_torque: Scalar,
    pub axis1: Vector3,
}

impl XpbdConstraint<2> for JointMotor {
    fn entities(&self) -> [Entity; 2] {
        [Entity::PLACEHOLDER, Entity::PLACEHOLDER]
    }

    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) {
        let [body1, body2] = bodies;

        // Transform the first axis into world space.
        let a1 = body1.rotation.rotate_vec3(self.axis1);

        // Compute the unit axes b1 and b2, which are perpendicular to a1.
        // Here we assume local a1 to be the Y or Z axis for now.
        let b1 = body1.rotation.rotate_vec3(Vector3::X);
        let b2 = body2.rotation.rotate_vec3(Vector3::X);

        // Compute b_target by rotating b1 around a1 by the current target angle.
        let b_target = Quaternion::from_axis_angle(a1, self.target_angle) * b1;

        // Compute the current angle.
        // If -pi < angle < 0.0 or 0 < pi < angle, we use the opposite number
        // to make the motor able to perform a full rotation.
        let mut current_angle = body1.rotation.angle_between(body2.rotation.0);
        if (self.target_angle > -PI && self.target_angle < 0.0) || (self.target_angle > PI) {
            current_angle *= -1.0;
        }

        // Update the target angle using the target velocity.
        self.target_angle = (current_angle) + dt * self.target_velocity;

        // Compute the rotation update required to reach the target angle.
        let delta_rotation = b_target.cross(b2);
        let axis = delta_rotation.normalize();

        // Compute the Lagrange multiplier update.
        let delta_lagrange = self.compute_motor_lagrange_update(body1, body2, delta_rotation, dt);
        self.lagrange += delta_lagrange;

        // Apply angular correction caused by the motor.
        self.apply_angular_correction(body1, body2, delta_lagrange, axis);
    }
}

impl JointMotor {
    /// Creates a new [`JointMotor`].
    pub const fn new(target_velocity: Scalar) -> Self {
        Self {
            target_angle: 0.0,
            target_velocity,
            compliance: 0.0,
            lagrange: 0.0,
            torque: Torque::ZERO,
            max_torque: 10000.0,
            axis1: Vector3::Z,
        }
    }

    /// Sets the joint motor's compliance (inverse of stiffness, meters / Newton).
    pub fn with_compliance(self, compliance: Scalar) -> Self {
        Self { compliance, ..self }
    }

    /// Applies an angular correction that aligns the orientation of the bodies.
    ///
    /// Returns the torque exerted by the alignment.
    fn compute_motor_lagrange_update(
        &mut self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        delta_q: Vector3,
        dt: Scalar,
    ) -> Scalar {
        let angle = delta_q.length();

        if angle <= Scalar::EPSILON {
            return 0.0;
        }

        let axis = delta_q / angle;

        // Compute generalized inverse masses
        let w1 = AngularConstraint::compute_generalized_inverse_mass(self, body1, axis);
        let w2 = AngularConstraint::compute_generalized_inverse_mass(self, body2, axis);

        // Constraint gradients and inverse masses
        let gradients = {
            #[cfg(feature = "2d")]
            {
                // `axis.z` controls if the body should rotate counterclockwise or clockwise.
                // The gradient has to be a 2D vector, so we use the y axis instead.
                // This should work similarly, as `axis.x` and `axis.y` are normally 0 in 2D.
                [Vector::Y * axis.z, Vector::NEG_Y * axis.z]
            }
            #[cfg(feature = "3d")]
            {
                [axis, -axis]
            }
        };
        let w = [w1, w2];

        // Return Lagrange multiplier update
        let mut delta_lagrange =
            self.compute_lagrange_update(self.lagrange, angle, &gradients, &w, self.compliance, dt);
        let mut torque = self.compute_torque(self.lagrange + delta_lagrange, axis, dt);

        #[cfg(feature = "2d")]
        if torque > self.max_torque {
            let compliance = 1.0 / self.max_torque * angle;
            delta_lagrange =
                self.compute_lagrange_update(self.lagrange, angle, &gradients, &w, compliance, dt);
            torque = self.max_torque;
            self.torque = torque;
        } else {
            self.torque = self.compute_torque(self.lagrange, axis, dt);
        }

        #[cfg(feature = "3d")]
        if torque.length() * 0.8 > self.max_torque {
            let compliance = 1.0 / self.max_torque * angle;
            delta_lagrange =
                self.compute_lagrange_update(self.lagrange, angle, &gradients, &w, compliance, dt);

            torque = axis * self.max_torque;
            self.torque = torque;
        } else {
            self.torque = self.compute_torque(self.lagrange, axis, dt);
        }

        delta_lagrange
    }
}

impl PositionConstraint for JointMotor {}

impl AngularConstraint for JointMotor {}
