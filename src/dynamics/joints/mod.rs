//! **Joints** are a way to connect entities in a way that restricts their movement relative to each other.
//! They act as [constraints](dynamics::solver::xpbd#constraints) that restrict different *Degrees Of Freedom*
//! depending on the joint type.
//!
//! # Degrees of Freedom (DOF)
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
#![cfg_attr(
    feature = "3d",
    doc = "| [`SphericalJoint`] | 1 Rotation                | 3 Rotations                 |"
)]
//!
//! # Using Joints
//!
//! In Avian, joints are modeled as components. You can create a joint by simply spawning
//! an entity and adding the joint component you want, giving the connected entities as arguments
//! to the `new` method.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn setup(mut commands: Commands) {
//!     let entity1 = commands.spawn(RigidBody::Dynamic).id();
//!     let entity2 = commands.spawn(RigidBody::Dynamic).id();
//!     
//!     // Connect the bodies with a fixed joint
//!     commands.spawn(FixedJoint::new(entity1, entity2));
//! }
//! ```
//!
//! By default, the attached bodies can still collide with each other.
//! This behavior can be disabled with the [`JointCollisionDisabled`] component.
//!
//! ## Stiffness
//!
//! You can control the stiffness of a joint with the `with_compliance` method.
//! *Compliance* refers to the inverse of stiffness, so using a compliance of 0 corresponds to
//! infinite stiffness.
//!
//! ## Attachment Positions
//!
//! By default, joints are connected to the centers of entities, but attachment positions can be used to change this.
//!
//! You can use `with_local_anchor1` and `with_local_anchor2` to set the attachment positions on the first
//! and second entity respectively.
//!
//! ## Damping
//!
//! You can configure the linear and angular damping caused by joints using the `with_linear_velocity_damping` and
//! `with_angular_velocity_damping` methods. Increasing the damping values will cause the velocities
//! of the connected entities to decrease faster.
//!
//! ## Other Configuration
//!
//! Different joints may have different configuration options. Many joints allow you to change the axis of allowed
//! translation or rotation, and they may have distance or angle limits along these axes.
//!
//! Take a look at the documentation and methods of each joint to see all of the configuration options.
//!
//! # Custom Joints
//!
//! Joints are [constraints](dynamics::solver::xpbd#constraints) that implement [`Joint`] and [`XpbdConstraint`].
//!
//! The process of creating a joint is essentially the same as [creating a constraint](dynamics::solver::xpbd#custom-constraints),
//! except you should also implement the [`Joint`] trait's methods. The trait has some useful helper methods
//! like `align_position` and `align_orientation` to reduce some common boilerplate.
//!
//! Many joints also have joint limits. You can use [`DistanceLimit`] and [`AngleLimit`] to help store these limits
//! and to compute the current distance from the specified limits.
//!
//! [See the code implementations](https://github.com/Jondolf/avian/tree/main/src/constraints/joints)
//! of the implemented joints to get a better idea of how to create joints.

mod distance;
mod fixed;
mod prismatic;
mod revolute;
#[cfg(feature = "3d")]
mod spherical;

pub use distance::DistanceJoint;
pub use fixed::FixedJoint;
pub use prismatic::PrismaticJoint;
pub use revolute::RevoluteJoint;
#[cfg(feature = "3d")]
pub use spherical::SphericalJoint;

use crate::{dynamics::solver::joint_graph::JointGraph, prelude::*};
use bevy::{
    ecs::{component::HookContext, entity::MapEntities, world::DeferredWorld},
    prelude::*,
};

/// A plugin for managing and initializing [joints](self).
///
/// Note that this does *not* include the actual joint constraint solver.
/// For a built-in joint solver, enable the `xpbd_joints` feature, and use the [`XpbdSolverPlugin`].
pub struct JointPlugin;

impl Plugin for JointPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            fixed::plugin,
            distance::plugin,
            prismatic::plugin,
            revolute::plugin,
            #[cfg(feature = "3d")]
            spherical::plugin,
        ));

        app.register_type::<(
            JointDisabled,
            JointCollisionDisabled,
            JointDamping,
            JointForces,
        )>();

        app.configure_sets(
            PhysicsSchedule,
            JointSet::PrepareAnchors
                .after(SolverSet::PrepareSolverBodies)
                .before(SolverSet::PrepareJoints),
        );
    }
}

/// System sets for joints.
#[derive(SystemSet, Clone, Debug, Hash, PartialEq, Eq)]
pub enum JointSet {
    /// A system set for preparing joint anchors.
    PrepareAnchors,
}

/// A trait for constraints between entities.
pub trait EntityConstraint<const ENTITY_COUNT: usize>: MapEntities {
    /// The entities participating in the constraint.
    fn entities(&self) -> [Entity; ENTITY_COUNT];
}

/// A limit that indicates that the distance between two points should be between `min` and `max`.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
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

    /// Returns the direction and magnitude of the positional correction required
    /// to limit the given `separation` to be within the distance limit.
    pub fn compute_correction(&self, separation: Vector) -> (Vector, Scalar) {
        let distance_squared = separation.length_squared();

        if distance_squared <= Scalar::EPSILON {
            return (Vector::ZERO, 0.0);
        }

        let distance = distance_squared.sqrt();

        // Equation 25
        if distance < self.min {
            // Separation distance lower limit
            (separation / distance, (self.min - distance))
        } else if distance > self.max {
            // Separation distance upper limit
            (-separation / distance, (distance - self.max))
        } else {
            (Vector::ZERO, 0.0)
        }
    }

    /// Returns the positional correction required to limit the given `separation`
    /// to be within the distance limit along a given `axis`.
    pub fn compute_correction_along_axis(&self, separation: Vector, axis: Vector) -> Vector {
        let a = separation.dot(axis);

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
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct AngleLimit {
    /// The minimum angle.
    pub min: Scalar,
    /// The maximum angle.
    pub max: Scalar,
}

impl AngleLimit {
    /// An `AngleLimit` with `alpha` and `beta` set to zero.
    pub const ZERO: Self = Self { min: 0.0, max: 0.0 };

    /// Creates a new `AngleLimit`.
    pub const fn new(min: Scalar, max: Scalar) -> Self {
        Self { min, max }
    }

    /// Returns the angular correction required to limit the `angle_difference`
    /// to be within the angle limits.
    #[cfg(feature = "2d")]
    pub fn compute_correction(
        &self,
        angle_difference: Scalar,
        max_correction: Scalar,
    ) -> Option<Scalar> {
        let correction = if angle_difference < self.min {
            angle_difference - self.min
        } else if angle_difference > self.max {
            angle_difference - self.max
        } else {
            return None;
        };

        Some(correction.min(max_correction))
    }

    /// Returns the angular correction required to limit the angle between `axis1` and `axis2`
    /// to be within the angle limits with respect to the `limit_axis`.
    #[cfg(feature = "3d")]
    pub fn compute_correction(
        &self,
        limit_axis: Vector,
        axis1: Vector,
        axis2: Vector,
        max_correction: Scalar,
    ) -> Option<Vector> {
        // [limit_axis, axis1, axis2] = [n, n1, n2] in XPBD rigid body paper.

        // Angle between axis1 and axis2 with respect to limit_axis.
        let mut phi = axis1.cross(axis2).dot(limit_axis).asin();

        // `asin` returns the angle in the [-pi/2, pi/2] range.
        // This is correct if the angle between n1 and n2 is acute,
        // but obtuse angles must be accounted for.
        if axis1.dot(axis2) < 0.0 {
            phi = PI - phi;
        }

        // Map the angle to the [-pi, pi] range.
        if phi > PI {
            phi -= TAU;
        }

        // The XPBD rigid body paper has this, but the angle
        // should already be in the correct range.
        //
        // if phi < -PI {
        //     phi += TAU;
        // }

        // Only apply a correction if the limit is violated.
        if phi < self.min || phi > self.max {
            // phi now represents the angle between axis1 and axis2.

            // Clamp phi to get the target angle.
            phi = phi.clamp(self.min, self.max);

            // Create a quaternion that represents the rotation.
            let rot = Quaternion::from_axis_angle(limit_axis, phi);

            // Rotate axis1 by the target angle and compute the correction.
            return Some((rot * axis1).cross(axis2).clamp_length_max(max_correction));
        }

        None
    }
}

/// A marker component that indicates that a [joint](self) is disabled
/// and should not constrain the bodies it is attached to.
/// Must be on the same entity as the joint.
///
/// This is useful for temporarily disabling a joint without removing it from the world.
/// To re-enable the joint, simply remove this component.
///
/// Note that when re-enabling the joint, the bodies may snap back violently
/// if they have moved significantly from the constrained positions while the joint was disabled.
///
/// # Related Components
///
/// - [`RigidBodyDisabled`]: Disables a rigid body.
/// - [`ColliderDisabled`]: Disables a collider.
#[derive(Reflect, Clone, Copy, Component, Debug, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default)]
pub struct JointDisabled;

/// A marker component that disables collision for [rigid bodies](RigidBody)
/// connected by a [joint](self). Must be on the same entity as the joint.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     let entity1 = commands.spawn(RigidBody::Dynamic).id();
///     let entity2 = commands.spawn(RigidBody::Dynamic).id();
///     
///     // Connect the bodies with a fixed joint.
///     // Disables collision between the two bodies.
///     commands.spawn((
///         FixedJoint::new(entity1, entity2),
///         JointCollisionDisabled,
///     ));
/// }
/// ```
#[derive(Component, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
#[component(on_add = JointCollisionDisabled::on_add, on_remove = JointCollisionDisabled::on_remove)]
pub struct JointCollisionDisabled;

// TODO: Turn these into observers.
impl JointCollisionDisabled {
    fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;

        // Update the `contacts_enabled` property of the joint edge.
        // Note: The `JointGraphPlugin` handles the removal of contacts between the bodies.
        let mut joint_graph = world.resource_mut::<JointGraph>();
        if let Some(joint_edge) = joint_graph.get_joint_mut(entity) {
            joint_edge.collision_disabled = true;
        }
    }

    fn on_remove(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;

        // Update the `contacts_enabled` property of the joint edge.
        let mut joint_graph = world.resource_mut::<JointGraph>();
        if let Some(joint_edge) = joint_graph.get_joint_mut(entity) {
            joint_edge.collision_disabled = false;
        }
    }
}

/// A component for the linear and angular damping applied by a [joint](self).
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct JointDamping {
    /// Linear damping applied by the joint.
    pub linear: Scalar,
    /// Angular damping applied by the joint.
    pub angular: Scalar,
}

/// A component for reading the force and torque applied by a [joint](self).
///
/// This is not inserted automatically for joints, and must be added manually.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct JointForces {
    force: Vector,
    torque: AngularVector,
}

impl JointForces {
    /// Creates a new [`JointForces`] for reading the forces applied by a joint.
    #[inline]
    pub const fn new() -> Self {
        Self {
            force: Vector::ZERO,
            torque: AngularVector::ZERO,
        }
    }

    /// Returns the force applied by the joint.
    #[inline]
    pub const fn force(&self) -> Vector {
        self.force
    }

    /// Returns the torque applied by the joint.
    #[inline]
    pub const fn torque(&self) -> AngularVector {
        self.torque
    }

    /// Sets the force applied by the joint.
    ///
    /// This should be done automatically by the joint solver,
    #[inline]
    pub const fn set_force(&mut self, force: Vector) {
        self.force = force;
    }

    /// Sets the torque applied by the joint.
    ///
    /// This should be done automatically by the joint solver,
    #[inline]
    pub const fn set_torque(&mut self, torque: AngularVector) {
        self.torque = torque;
    }
}

/// The [reference frame] of a body that is being constrained by a [joint](self).
///
/// Each joint defines a relationship between the translation and rotation of two reference frames.
/// For example, a [`RevoluteJoint`] aims to make the positions of the two frames coincide in world space,
/// while allowing the frames to rotate freely around a common axis.
///
/// Reference frames for joints are expressed as local isometries relative to the transforms of the bodies
/// they are attached to. The translation defines the local anchor point where the bodies are attached,
/// and the rotation defines the local orientation of the frame relative to the body transform.
///
/// TODO: SVG
///
/// Storing the frames in local space allows the initial configuration to be preserved even when the bodies are moved.
/// The frames can also be specified in global coordinates using [`JointFrame::global`], but they are automatically converted
/// to local frames during the next simulation step.
///
/// [reference frame]: https://en.wikipedia.org/wiki/Frame_of_reference
///
/// # Example
///
/// TODO
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct JointFrame {
    /// The translation of the joint frame relative to the body transform.
    ///
    /// This defines the anchor point where the bodies are attached to each other.
    pub translation: JointTranslation,
    /// The rotation of the joint frame relative to the body transform.
    ///
    /// This defines the orientation of the joint frame relative to the body transform.
    pub rotation: JointRotation,
}

impl JointFrame {
    /// The identity reference frame, with the local translation and rotation set to zero.
    ///
    /// This represents a reference frame that aligns with the body transform.
    pub const IDENTITY: Self = Self {
        translation: JointTranslation::IDENTITY,
        rotation: JointRotation::IDENTITY,
    };

    /// Creates a [`JointFrame`] with the given local translation and rotation.
    #[inline]
    pub fn local(isometry: impl Into<Isometry>) -> Self {
        let isometry: Isometry = isometry.into();
        #[cfg(feature = "2d")]
        let translation = isometry.translation.adjust_precision();
        #[cfg(feature = "3d")]
        let translation = Vec3::from(isometry.translation).adjust_precision();
        Self {
            translation: JointTranslation::Local(translation),
            #[cfg(feature = "2d")]
            rotation: JointRotation::Local(Rotation::from_sin_cos(
                isometry.rotation.sin as Scalar,
                isometry.rotation.cos as Scalar,
            )),
            #[cfg(feature = "3d")]
            rotation: JointRotation::Local(isometry.rotation.adjust_precision()),
        }
    }

    /// Creates a [`JointFrame`] with the given global translation and rotation.
    ///
    /// The global frame will be converted to a local frame relative to the body transform
    /// during the next simulation step.
    #[inline]
    pub fn global(isometry: impl Into<Isometry>) -> Self {
        let isometry: Isometry = isometry.into();
        #[cfg(feature = "2d")]
        let translation = isometry.translation.adjust_precision();
        #[cfg(feature = "3d")]
        let translation = Vec3::from(isometry.translation).adjust_precision();
        Self {
            translation: JointTranslation::FromGlobal(translation),
            #[cfg(feature = "2d")]
            rotation: JointRotation::FromGlobal(Rotation::from_sin_cos(
                isometry.rotation.sin as Scalar,
                isometry.rotation.cos as Scalar,
            )),
            #[cfg(feature = "3d")]
            rotation: JointRotation::FromGlobal(isometry.rotation.adjust_precision()),
        }
    }

    /// Computes a local frames for the given [`JointFrame`]s
    /// corresponding to the transforms of two bodies constrained by a joint.
    ///
    /// This is used to initialize local frames when a joint is inserted.
    pub fn compute_local(
        frame1: Self,
        frame2: Self,
        pos1: Vector,
        pos2: Vector,
        rot1: &Rotation,
        rot2: &Rotation,
    ) -> [JointFrame; 2] {
        let [local_translation1, local_translation2] = JointTranslation::compute_local(
            frame1.translation,
            frame2.translation,
            pos1,
            pos2,
            rot1,
            rot2,
        );
        let [local_rotation1, local_rotation2] =
            JointRotation::compute_local(frame1.rotation, frame2.rotation, rot1, rot2);

        [
            JointFrame {
                translation: local_translation1,
                rotation: local_rotation1,
            },
            JointFrame {
                translation: local_translation2,
                rotation: local_rotation2,
            },
        ]
    }
}

/// The translation of a [`JointFrame`], defining the anchor point where the bodies are attached to each other.
///
/// Reference translations are stored in local space relative to the transforms of the bodies they are attached to.
/// This way, the initial configuration of the joint is preserved even when the bodies are moved.
///
/// The initial translation can also be specified in world space using [`JointTranslation::FromGlobal`],
/// but it is automatically converted to [`JointTranslation::Local`] during the next simulation step.
///
/// By default, a local identity translation is used, and the reference translation matches the body transform.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
#[doc(alias = "JointAnchor")]
pub enum JointTranslation {
    /// The anchor point is specified in local coordinates relative to the body transform.
    Local(Vector),
    /// The anchor point is specified in global coordinates.
    FromGlobal(Vector),
}

impl Default for JointTranslation {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl JointTranslation {
    /// The identity reference translation.
    ///
    /// This represents a reference translation that aligns with the body transform.
    pub const IDENTITY: Self = Self::Local(Vector::ZERO);

    /// Computes [`JointTranslation::Local`]s for the given [`JointTranslation`]s
    /// corresponding to the transforms of two bodies constrained by a joint.
    pub fn compute_local(
        anchor1: Self,
        anchor2: Self,
        pos1: Vector,
        pos2: Vector,
        rot1: &Rotation,
        rot2: &Rotation,
    ) -> [Self; 2] {
        let [local_anchor1, local_anchor2] = match [anchor1, anchor2] {
            [
                JointTranslation::Local(anchor1),
                JointTranslation::Local(anchor2),
            ] => [anchor1, anchor2],
            [
                JointTranslation::FromGlobal(anchor1),
                JointTranslation::FromGlobal(anchor2),
            ] => [
                rot1.inverse() * (anchor1 - pos1),
                rot2.inverse() * (anchor2 - pos2),
            ],
            [
                JointTranslation::Local(anchor1),
                JointTranslation::FromGlobal(anchor2),
            ] => [anchor1, rot2.inverse() * (anchor2 - pos2)],
            [
                JointTranslation::FromGlobal(anchor1),
                JointTranslation::Local(anchor2),
            ] => [rot1.inverse() * (anchor1 - pos1), anchor2],
        };

        [
            JointTranslation::Local(local_anchor1),
            JointTranslation::Local(local_anchor2),
        ]
    }
}

impl From<JointTranslation> for JointFrame {
    fn from(translation: JointTranslation) -> Self {
        Self {
            translation,
            rotation: JointRotation::IDENTITY,
        }
    }
}

/// The rotation of a [`JointFrame`], defining the reference orientation of a body relative to its transform.
///
/// Reference rotations are stored in local space relative to the transforms of the bodies they are attached to.
/// This way, the initial configuration of the joint is preserved even when the bodies are moved.
///
/// The initial rotation can also be specified in world space using [`JointRotation::FromGlobal`],
/// but it is automatically converted to [`JointRotation::Local`] during the next simulation step.
///
/// By default, a local identity rotation is used, and the reference rotation matches the body transform.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum JointRotation {
    /// The reference rotation is specified in local space relative to the body transform.
    Local(Rot),
    /// The reference rotation is specified in world space.
    FromGlobal(Rot),
}

impl Default for JointRotation {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl JointRotation {
    /// The identity reference rotation.
    ///
    /// This represents a reference rotation that aligns with the body transform.
    pub const IDENTITY: Self = Self::Local(Rot::IDENTITY);

    /// Computes a [`JointRotation::Local`] for the given [`JointRotation`]s
    /// corresponding to the transforms of two bodies constrained by a joint.
    pub fn compute_local(rotation1: Self, rotation2: Self, rot1: &Rot, rot2: &Rot) -> [Self; 2] {
        let [local_rotation1, local_rotation2] = match [rotation1, rotation2] {
            [
                JointRotation::Local(rotation1),
                JointRotation::Local(rotation2),
            ] => [rotation1, rotation2],
            [
                JointRotation::FromGlobal(rotation1),
                JointRotation::FromGlobal(rotation2),
            ] => [rotation1 * rot1.inverse(), rotation2 * rot2.inverse()],
            [
                JointRotation::Local(rotation1),
                JointRotation::FromGlobal(rotation2),
            ] => [rotation1, rotation2 * rot2.inverse()],
            [
                JointRotation::FromGlobal(rotation1),
                JointRotation::Local(rotation2),
            ] => [rotation1 * rot1.inverse(), rotation2],
        };

        [
            JointRotation::Local(local_rotation1),
            JointRotation::Local(local_rotation2),
        ]
    }
}

impl From<JointRotation> for JointFrame {
    fn from(rotation: JointRotation) -> Self {
        Self {
            translation: JointTranslation::IDENTITY,
            rotation,
        }
    }
}
