use crate::{
    dynamics::joints::{EntityConstraint, JointSet, impl_joint_frame_helpers},
    prelude::*,
};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};

/// A spherical joint prevents relative translation of the attached bodies while allowing rotation around all axes.
///
/// Spherical joints can be useful for things like pendula, chains, ragdolls etc.
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct SphericalJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// The reference frame of the first body, defining the joint anchor and basis
    /// relative to the body transform.
    pub frame1: JointFrame,
    /// The reference frame of the second body, defining the joint anchor and basis
    /// relative to the body transform.
    pub frame2: JointFrame,
    /// A local axis that the bodies can swing around. Perpendicular to the [`twist_axis`](SphericalJoint::twist_axis).
    ///
    /// By default, this is the x-axis.
    pub swing_axis: Vector,
    /// The local axis that the bodies can twist around. Perpendicular to the [`swing_axis`](SphericalJoint::swing_axis).
    ///
    /// By default, this is the y-axis.
    pub twist_axis: Vector,
    /// The extents of the allowed relative rotation of the bodies around a swing axis perpendicular to the [`twist_axis`](SphericalJoint::twist_axis).
    pub swing_limit: Option<AngleLimit>,
    /// The extents of the allowed relative rotation of the bodies around the [`twist_axis`](SphericalJoint::twist_axis).
    pub twist_limit: Option<AngleLimit>,
    /// The compliance of the point-to-point constraint (inverse of stiffness, m / N).
    pub point_compliance: Scalar,
    /// The compliance for swing (inverse of stiffness, N * m / rad).
    pub swing_compliance: Scalar,
    /// The compliance for twist (inverse of stiffness, N * m / rad).
    pub twist_compliance: Scalar,
}

impl EntityConstraint<2> for SphericalJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl SphericalJoint {
    /// The default [`swing_axis`](SphericalJoint::swing_axis) for a spherical joint.
    pub const DEFAULT_SWING_AXIS: Vector = Vector::X;

    /// The default [`twist_axis`](SphericalJoint::twist_axis) for a spherical joint.
    pub const DEFAULT_TWIST_AXIS: Vector = Vector::Y;

    /// Creates a new [`SphericalJoint`] between two entities.
    #[inline]
    pub const fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            swing_axis: Self::DEFAULT_SWING_AXIS,
            twist_axis: Self::DEFAULT_TWIST_AXIS,
            swing_limit: None,
            twist_limit: None,
            point_compliance: 0.0,
            swing_compliance: 0.0,
            twist_compliance: 0.0,
        }
    }

    /// Sets the [`swing_axis`](SphericalJoint::swing_axis) of the joint.
    ///
    /// The axis should be a unit vector perpendicular to the [`twist_axis`](SphericalJoint::twist_axis).
    /// By default, this is the x-axis.
    #[inline]
    pub const fn with_swing_axis(mut self, axis: Vector) -> Self {
        self.swing_axis = axis;
        self
    }

    /// Sets the [`twist_axis`](SphericalJoint::twist_axis) of the joint.
    ///
    /// The axis should be a unit vector perpendicular to the [`swing_axis`](SphericalJoint::swing_axis).
    /// By default, this is the y-axis.
    #[inline]
    pub const fn with_twist_axis(mut self, axis: Vector) -> Self {
        self.twist_axis = axis;
        self
    }
}

impl_joint_frame_helpers!(SphericalJoint);

impl SphericalJoint {
    /// Returns the local [`swing_axis`](SphericalJoint::swing_axis) of the first body.
    ///
    /// This is equivalent to rotating the [`swing_axis`](SphericalJoint::swing_axis)
    /// by the local basis of [`frame1`](SphericalJoint::frame1).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_swing_axis1(&self) -> Option<Vector> {
        match self.frame1.basis {
            JointBasis::Local(rotation) => Some(rotation * self.swing_axis),
            _ => None,
        }
    }

    /// Returns the local [`swing_axis`](SphericalJoint::swing_axis) of the second body.
    ///
    /// This is equivalent to rotating the [`swing_axis`](SphericalJoint::swing_axis)
    /// by the local basis of [`frame2`](SphericalJoint::frame2).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_swing_axis2(&self) -> Option<Vector> {
        match self.frame2.basis {
            JointBasis::Local(rotation) => Some(rotation * self.swing_axis),
            _ => None,
        }
    }

    /// Returns the local [`twist_axis`](SphericalJoint::twist_axis) of the first body.
    ///
    /// This is equivalent to rotating the [`twist_axis`](SphericalJoint::twist_axis)
    /// by the local basis of [`frame1`](SphericalJoint::frame1).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_twist_axis1(&self) -> Option<Vector> {
        match self.frame1.basis {
            JointBasis::Local(rotation) => Some(rotation * self.twist_axis),
            _ => None,
        }
    }

    /// Returns the local [`twist_axis`](SphericalJoint::twist_axis) of the second body.
    ///
    /// This is equivalent to rotating the [`twist_axis`](SphericalJoint::twist_axis)
    /// by the local basis of [`frame2`](SphericalJoint::frame2).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_twist_axis2(&self) -> Option<Vector> {
        match self.frame2.basis {
            JointBasis::Local(rotation) => Some(rotation * self.twist_axis),
            _ => None,
        }
    }

    /// Sets the limits of the allowed relative rotation around the `swing_axis`.
    #[inline]
    pub const fn with_swing_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.swing_limit = Some(AngleLimit::new(min, max));
        self
    }

    /// Sets the limits of the allowed relative rotation around the `twist_axis`.
    #[inline]
    pub const fn with_twist_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.twist_limit = Some(AngleLimit::new(min, max));
        self
    }

    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_point_compliance`, `with_swing_compliance`, and `with_twist_compliance` instead."
    )]
    pub const fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        self.swing_compliance = compliance;
        self.twist_compliance = compliance;
        self
    }

    /// Sets the compliance of the axis alignment constraint (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_point_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        self
    }

    /// Sets the compliance of the swing limit (inverse of stiffness, N * m / rad).
    #[inline]
    pub const fn with_swing_compliance(mut self, compliance: Scalar) -> Self {
        self.swing_compliance = compliance;
        self
    }

    /// Sets the compliance of the twist limit (inverse of stiffness, N * m / rad).
    #[inline]
    pub const fn with_twist_compliance(mut self, compliance: Scalar) -> Self {
        self.twist_compliance = compliance;
        self
    }
}

impl MapEntities for SphericalJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}

pub(super) fn plugin(app: &mut App) {
    app.register_type::<SphericalJoint>();
    app.add_systems(
        PhysicsSchedule,
        update_local_frames.in_set(JointSet::PrepareAnchors),
    );
}

fn update_local_frames(
    mut joints: Query<&mut SphericalJoint, Changed<SphericalJoint>>,
    bodies: Query<(&Position, &Rotation)>,
) {
    for mut joint in &mut joints {
        if matches!(joint.frame1.anchor, JointAnchor::Local(_))
            && matches!(joint.frame2.anchor, JointAnchor::Local(_))
            && matches!(joint.frame1.basis, JointBasis::Local(_))
            && matches!(joint.frame2.basis, JointBasis::Local(_))
        {
            continue;
        }

        let Ok([(pos1, rot1), (pos2, rot2)]) = bodies.get_many(joint.entities()) else {
            continue;
        };

        let [frame1, frame2] =
            JointFrame::compute_local(joint.frame1, joint.frame2, pos1.0, pos2.0, rot1, rot2);
        joint.frame1 = frame1;
        joint.frame2 = frame2;
    }
}

impl DebugRenderConstraint<2> for SphericalJoint {
    type Context = ();

    fn debug_render(
        &self,
        positions: [Vector; 2],
        rotations: [Rotation; 2],
        _context: &mut Self::Context,
        gizmos: &mut Gizmos<PhysicsGizmos>,
        config: &PhysicsGizmos,
    ) {
        let [pos1, pos2] = positions;
        let [rot1, rot2] = rotations;

        let Some(local_anchor1) = self.local_anchor1() else {
            return;
        };
        let Some(local_anchor2) = self.local_anchor2() else {
            return;
        };

        let anchor1 = pos1 + rot1 * local_anchor1;
        let anchor2 = pos2 + rot2 * local_anchor2;

        if let Some(anchor_color) = config.joint_anchor_color {
            gizmos.draw_line(pos1, anchor1, anchor_color);
            gizmos.draw_line(pos2, anchor2, anchor_color);
        }

        if let Some(color) = config.joint_separation_color {
            gizmos.draw_line(anchor1, anchor2, color);
        }
    }
}
