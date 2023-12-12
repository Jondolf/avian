//! **Joints** are a way to connect entities in a way that restricts their movement relative to each other.
//! They act as [constraints] that restrict different *Degrees Of Freedom* depending on the joint type.
//!
//! ## Degrees Of Freedom (DOF)
//!
//! In 2D, entities can translate along the `X` and `Y` axes and rotate around the `Z` axis.
//! Therefore, they have 2 translational DOF and 1 rotational DOF, a total of 3 DOF.
//!
//! Similarly in 3D, entities can translate and rotate along the `X`, `Y`, and `Z` axes.
//! In other words, they have 3 translational DOF and 3 rotational DOF, which is a total of 6 DOF.
//!
//! Joints reduce the number of degrees of freedom that entities have. For example,
//! [revolute joints](RevoluteJoint) only allow rotation around one axis.
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
//! In Bevy XPBD, joints are modeled as components. Each joint type has its own component
//! like [`FixedJoint`] or [`PrismaticJoint`] that contains its configuration options.
//!
//! To make the joint actually connect two bodies, you need to add the [`ConstraintEntities`]
//! component with the corresponding `Entity` IDs.
//!
//! Creating a [`FixedJoint`] might look like this:
//!
//! ```
//! use bevy::prelude::*;
#![cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
//!
//! fn setup(mut commands: Commands) {
//!     let entity1 = commands.spawn(RigidBody::Dynamic).id();
//!     let entity2 = commands.spawn(RigidBody::Dynamic).id();
//!     
//!     // Connect the bodies with a fixed joint
//!     commands.spawn((
//!         FixedJoint::new(),
//!         ConstraintEntities([entity1, entity2]),
//!     ));
//! }
//! ```
//!
//! Joints can be configured further with several components:
//!
//! - [`JointAnchors`]: Configures the joint attachment positions on the bodies.
//! - [`JointDamping`]: Applies linear and angular velocity damping.
//!
//! To make joints more convenient to create, there is a [`JointBundle`] that contains these components:
//!
//! ```
//! # use bevy::prelude::*;
#![cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
//! #
//! # fn setup(mut commands: Commands) {
//! #    let entity1 = commands.spawn(RigidBody::Dynamic).id();
//! #    let entity2 = commands.spawn(RigidBody::Dynamic).id();
//! #
//! // Connect the bodies with a prismatic joint that allows
//! // relative translation along the Y axis.
//! commands.spawn(JointBundle {
//!     entities: [entity1, entity2].into(),
#![cfg_attr(feature = "2d", doc = "    joint: PrismaticJoint::new(Vec2::Y),")]
#![cfg_attr(feature = "3d", doc = "    joint: PrismaticJoint::new(Vec3::Y),")]
#![cfg_attr(
    feature = "2d",
    doc = "    anchors: JointAnchors::from_second(Vec2::Y * 0.5),"
)]
#![cfg_attr(
    feature = "3d",
    doc = "    anchors: JointAnchors::from_second(Vec3::Y * 0.5),"
)]
//!     damping: JointDamping {
//!         linear: 0.5,
//!         angular: 0.75,
//!     },
//! });
//! # }
//! ```
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

/// A bundle that contains components used by [joints].
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     let entity1 = commands.spawn(RigidBody::Dynamic).id();
///     let entity2 = commands.spawn(RigidBody::Dynamic).id();
///     
///     // Connect the bodies with a prismatic joint that allows
///     // relative translation along the Y axis.
///     commands.spawn(JointBundle {
///         entities: [entity1, entity2].into(),
#[cfg_attr(feature = "2d", doc = "        joint: PrismaticJoint::new(Vec2::Y),")]
#[cfg_attr(feature = "3d", doc = "        joint: PrismaticJoint::new(Vec3::Y),")]
#[cfg_attr(
    feature = "2d",
    doc = "        anchors: JointAnchors::from_second(Vec2::Y * 0.5),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        anchors: JointAnchors::from_second(Vec3::Y * 0.5),"
)]
///         damping: JointDamping {
///             linear: 0.5,
///             angular: 0.75,
///         },
///     });
/// }
/// ```
#[derive(Bundle, Clone, Copy, Debug, Default, PartialEq)]
pub struct JointBundle<JointType: Joint + Default> {
    /// The `Entity` IDs of the bodies connected by the joint.
    pub entities: ConstraintEntities<2>,
    /// The type of the joint.
    ///
    /// See [`joints`] for a list of built-in joints.
    pub joint: JointType,
    /// Local attachment points for bodies connected by the joint.
    pub anchors: JointAnchors,
    /// Linear and angular velocity damping applied by the joint.
    pub damping: JointDamping,
}

impl<JointType: Joint + Default> JointBundle<JointType> {
    /// Creates a new [`JointBundle`] with the given joint and the entities it connects.
    pub const fn new(joint: JointType, entity1: Entity, entity2: Entity) -> Self {
        Self {
            joint,
            anchors: JointAnchors::DEFAULT,
            damping: JointDamping::DEFAULT,
            entities: ConstraintEntities([entity1, entity2]),
        }
    }

    /// Sets the local attachment point on the first body.
    pub const fn local_anchor_1(mut self, anchor: Vector) -> Self {
        self.anchors.first = anchor;
        self
    }

    /// Sets the local attachment point on the second body.
    pub const fn local_anchor_2(mut self, anchor: Vector) -> Self {
        self.anchors.second = anchor;
        self
    }

    /// Sets the linear velocity damping caused by the joint.
    pub const fn linear_damping(mut self, damping: Scalar) -> Self {
        self.damping.linear = damping;
        self
    }

    /// Sets the angular velocity damping caused by the joint.
    pub const fn angular_damping(mut self, damping: Scalar) -> Self {
        self.damping.angular = damping;
        self
    }
}

/// Linear and angular velocity damping applied by the joint.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     let entity1 = commands.spawn(RigidBody::Dynamic).id();
///     let entity2 = commands.spawn(RigidBody::Dynamic).id();
///     
///     // Spawn a revolute joint with configured attachment positions
///     commands.spawn((
#[cfg_attr(feature = "2d", doc = "        RevoluteJoint::new(),")]
#[cfg_attr(feature = "3d", doc = "        RevoluteJoint::new(Vec3::Z),")]
///         ConstraintEntities([entity1, entity2]),
#[cfg_attr(
    feature = "2d",
    doc = r#"        JointDamping {
            linear: 0.5,
            angular: 0.8,
        }"#
)]
#[cfg_attr(
    feature = "3d",
    doc = r#"        JointDamping {
            linear: 0.5,
            angular: 0.8,
        }"#
)]
///     });
/// }
/// ```
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
pub struct JointDamping {
    /// Linear velocity damping applied by the joint.
    ///
    /// Default: `1.0`
    pub linear: f32,
    /// Angular velocity damping applied by the joint.
    ///
    /// Default: `1.0`
    pub angular: f32,
}

impl Default for JointDamping {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl JointDamping {
    /// The default joint damping values.
    pub const DEFAULT: Self = Self {
        linear: 1.0,
        angular: 1.0,
    };
}

/// Local attachment points for bodies connected by a joint.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     let entity1 = commands.spawn(RigidBody::Dynamic).id();
///     let entity2 = commands.spawn(RigidBody::Dynamic).id();
///     
///     // Spawn a prismatic joint with configured attachment positions
///     commands.spawn((
#[cfg_attr(feature = "2d", doc = "        PrismaticJoint::new(Vec2::X),")]
#[cfg_attr(feature = "3d", doc = "        PrismaticJoint::new(Vec3::X),")]
///         ConstraintEntities([entity1, entity2]),
#[cfg_attr(
    feature = "2d",
    doc = r#"        JointAnchors {
            first: Vec2::NEG_Y * 0.5,
            second: Vec2::Y * 0.5,
        }"#
)]
#[cfg_attr(
    feature = "3d",
    doc = r#"        JointAnchors {
            first: Vec3::NEG_Y * 0.5,
            second: Vec3::Y * 0.5,
        }"#
)]
///     });
/// }
/// ```
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
pub struct JointAnchors {
    /// A local attachment point on the first body.
    pub first: Vector,
    /// A local attachment point on the second body.
    pub second: Vector,
}

impl JointAnchors {
    /// The default joint anchor positions.
    pub const DEFAULT: Self = Self {
        first: Vector::ZERO,
        second: Vector::ZERO,
    };

    /// Creates a new [`JointAnchors`] configuration from the given local attachment points.
    pub const fn new(first: Vector, second: Vector) -> Self {
        Self { first, second }
    }

    /// Creates a new [`JointAnchors`] configuration from the local attachment point on the first body.
    pub const fn from_first(anchor: Vector) -> Self {
        Self {
            first: anchor,
            ..Self::DEFAULT
        }
    }

    /// Creates a new [`JointAnchors`] configuration from the local attachment point on the second body.
    pub const fn from_second(anchor: Vector) -> Self {
        Self {
            second: anchor,
            ..Self::DEFAULT
        }
    }
}

/// A trait for [joints].
pub trait Joint: Component + PositionConstraint + AngularConstraint {
    /// Sets the joint's compliance (inverse of stiffness, meters / Newton).
    fn with_compliance(self, compliance: Scalar) -> Self;

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

        let delta_x = DistanceLimit::new(0.0, 0.0).compute_correction(
            body1.current_position() + world_r1,
            body2.current_position() + world_r2,
        );
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
    pub const fn new(min: Scalar, max: Scalar) -> Self {
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
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
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
    pub const fn new(alpha: Scalar, beta: Scalar) -> Self {
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
