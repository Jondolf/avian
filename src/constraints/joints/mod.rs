//! **Joints** are a way to connect entities in a way that restricts their movement relative to each other.
//! They act as [constraints] that restrict different *Degrees Of Freedom* depending on the joint type.
//!
//! ## Degrees Of Freedom (DOF)
//!
//! In 3D, entities can normally translate and rotate along the `X`, `Y` and `Z` axes.
//! Therefore, they have 3 translational DOF and 3 rotational DOF, which is a total of 6 DOF.
//!
//! Joints reduce the number of DOF that entities have. For example, [revolute joints](RevoluteJoint)
//! only allow rotation around one axis.
//!
//! Below is a table containing the joints that are currently implemented.
//!
//! | Joint              | Allowed 2D DOF            | Allowed 3D DOF              |
//! | ------------------ | ------------------------- | --------------------------- |
//! | [`FixedJoint`]     | None                      | None                        |
//! | [`DistanceJoint`]  | 1 Translation, 1 Rotation | 2 Translations, 3 Rotations |
//! | [`PrismaticJoint`] | 1 Translation             | 1 Translation               |
//! | [`RevoluteJoint`]  | 1 Rotation                | 1 Rotation                  |
//! | [`SphericalJoint`] | 1 Rotation                | 3 Rotations                 |
//!
//! ## Using joints
//!
//! In Bevy XPBD, joints are modeled as components. You can create a joint by simply spawning
//! an entity and adding the joint component you want, giving the connected entities as arguments
//! to the `new` method.
//!
//! ```
//! use bevy::prelude::*;
#![cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]

//! fn setup(mut commands: Commands) {
//!     let entity1 = commands.spawn(RigidBody::Dynamic).id();
//!     let entity2 = commands.spawn(RigidBody::Dynamic).id();
//!     
//!     // Connect the bodies with a fixed joint
//!     commands.spawn(FixedJoint::new(entity1, entity2));
//! }
//! ```
//!
//! ### Stiffness
//!
//! You can control the stiffness of a joint with the `with_compliance` method.
//! *Compliance* refers to the inverse of stiffness, so using a compliance of 0 corresponds to
//! infinite stiffness.
//!
//! ### Attachment positions
//!
//! By default, joints are connected to the centers of entities, but attachment positions can be used to change this.
//!
//! You can use `with_local_anchor_1` and `with_local_anchor_2` to set the attachment positions on the first
//! and second entity respectively.
//!
//! ### Damping
//!
//! You can configure the linear and angular damping caused by joints using the `with_linear_velocity_damping` and
//! `with_angular_velocity_damping` methods. Increasing the damping values will cause the velocities
//! of the connected entities to decrease faster.
//!
//! ### Other configuration
//!
//! Different joints may have different configuration options. Many joints allow you to change the axis of allowed
//! translation or rotation, and they may have distance or angle limits along these axes.
//!
//! Take a look at the documentation and methods of each joint to see all of the configuration options.
//!
//! ## Custom joints
//!
//! Joints are [constraints] that implement [`Joint`] and [`XpbdConstraint`].
//!
//! The process of creating a joint is essentially the same as [creating a constraint](constraints#custom-constraints),
//! except you should also implement the [`Joint`] trait's methods. The trait has some useful helper methods
//! like `align_position` and `align_orientation` to reduce some common boilerplate.
//!
//! Many joints also have joint limits. You can use [`DistanceLimit`] and [`AngleLimit`] to help store these limits
//! and to compute the current distance from the specified limits.
//!
//! [See the code implementations](https://github.com/Jondolf/bevy_xpbd/tree/main/src/constraints/joints)
//! of the implemented joints to get a better idea of how to create joints.

mod distance;
mod fixed;
mod prismatic;
mod revolute;
mod spherical;

pub use distance::*;
pub use fixed::*;
pub use prismatic::*;
pub use revolute::*;
pub use spherical::*;

use crate::prelude::*;
use bevy::prelude::*;

/// A trait for [joints].
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
    fn with_linear_velocity_damping(self, damping: Scalar) -> Self;

    /// Sets the angular velocity damping caused by the joint.
    fn with_angular_velocity_damping(self, damping: Scalar) -> Self;

    /// Returns the local attachment point on the first body.
    fn local_anchor_1(&self) -> Vector;

    /// Returns the local attachment point on the second body.
    fn local_anchor_2(&self) -> Vector;

    /// Returns the linear velocity damping of the joint.
    fn damping_linear(&self) -> Scalar;

    /// Returns the angular velocity damping of the joint.
    fn damping_angular(&self) -> Scalar;

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
        let world_r1 = body1.rotation.rotate(r1);
        let world_r2 = body2.rotation.rotate(r2);

        let (dir, magnitude) = DistanceLimit::new(0.0, 0.0).compute_correction(
            body1.current_position() + world_r1,
            body2.current_position() + world_r2,
        );

        if magnitude <= Scalar::EPSILON {
            return Vector::ZERO;
        }

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
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct DistanceLimit {
    /// The minimum distance between two points.
    pub min: Scalar,
    /// The maximum distance between two points.
    pub max: Scalar,
}

impl DistanceLimit {
    /// A `DistanceLimit` with `min` and `max` set to zero.
    pub const ZERO: Self = Self { min: 0.0, max: 0.0 };

    /// Creates a new `DistanceLimit`.
    pub fn new(min: Scalar, max: Scalar) -> Self {
        Self { min, max }
    }

    /// Returns the direction and magnitude of the positional correction required
    /// to limit the distance between `p1` and `p2` to be within the distance limit.
    pub fn compute_correction(&self, p1: Vector, p2: Vector) -> (Vector, Scalar) {
        let pos_offset = p2 - p1;
        let distance = pos_offset.length();

        if distance <= Scalar::EPSILON {
            return (Vector::ZERO, 0.0);
        }

        // Equation 25
        if distance < self.min {
            // Separation distance lower limit
            (-pos_offset / distance, (distance - self.min))
        } else if distance > self.max {
            // Separation distance upper limit
            (-pos_offset / distance, (distance - self.max))
        } else {
            (Vector::ZERO, 0.0)
        }
    }

    /// Returns the positional correction required to limit the distance between `p1` and `p2`
    /// to be within the distance limit along a given `axis`.
    fn compute_correction_along_axis(&self, p1: Vector, p2: Vector, axis: Vector) -> Vector {
        let pos_offset = p2 - p1;
        let a = pos_offset.dot(axis);

        // Equation 25
        if a < self.min {
            // Separation distance lower limit
            axis * (self.min - a)
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
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct AngleLimit {
    /// The minimum angle.
    pub alpha: Scalar,
    /// The maximum angle.
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
    /// to be within the angle limits.
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
