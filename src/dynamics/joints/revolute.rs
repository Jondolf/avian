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

/// A revolute joint prevents relative movement of the attached bodies, except for rotation around one [`hinge_axis`](RevoluteJoint::hinge_axis).
///
/// Revolute joints can be useful for things like wheels, fans, revolving doors etc.
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct RevoluteJoint {
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
    /// The local axis that the bodies can rotate around.
    ///
    /// By default, this is the z-axis.
    #[cfg(feature = "3d")]
    pub hinge_axis: Vector,
    /// The extents of the allowed relative rotation of the bodies around the [`hinge_axis`](RevoluteJoint::hinge_axis).
    pub angle_limit: Option<AngleLimit>,
    /// The compliance of the point-to-point constraint (inverse of stiffness, m / N).
    pub point_compliance: Scalar,
    /// The compliance used for aligning the bodies along the [`hinge_axis`](RevoluteJoint::hinge_axis) (inverse of stiffness, N * m / rad).
    #[cfg(feature = "3d")]
    pub align_compliance: Scalar,
    /// The compliance of the angle limit (inverse of stiffness, N * m / rad).
    pub limit_compliance: Scalar,
}

impl EntityConstraint<2> for RevoluteJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl RevoluteJoint {
    #[cfg(feature = "3d")]
    /// The default [`hinge_axis`](RevoluteJoint::hinge_axis) for a revolute joint.
    pub const DEFAULT_HINGE_AXIS: Vector = Vector::Z;

    /// Creates a new [`RevoluteJoint`] between two entities.
    #[cfg_attr(
        feature = "3d",
        doc = "\nThe default [`hinge_axis`](RevoluteJoint::hinge_axis) that relative rotation is allowed around is the z-axis. This can be changed using [`with_hinge_axis`](Self::with_hinge_axis)."
    )]
    #[inline]
    pub const fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            #[cfg(feature = "3d")]
            hinge_axis: Self::DEFAULT_HINGE_AXIS,
            angle_limit: None,
            point_compliance: 0.0,
            #[cfg(feature = "3d")]
            align_compliance: 0.0,
            limit_compliance: 0.0,
        }
    }

    /// Sets the [`hinge_axis`](RevoluteJoint::hinge_axis) of the joint.
    ///
    /// The axis should be a unit vector. By default, this is the z-axis.
    #[inline]
    #[cfg(feature = "3d")]
    pub const fn with_hinge_axis(mut self, axis: Vector) -> Self {
        self.hinge_axis = axis;
        self
    }
}

impl_joint_frame_helpers!(RevoluteJoint);

impl RevoluteJoint {
    /// Returns the local [`hinge_axis`](RevoluteJoint::hinge_axis) of the first body.
    ///
    /// This is equivalent to rotating the [`hinge_axis`](RevoluteJoint::hinge_axis)
    /// by the local basis of [`frame1`](RevoluteJoint::frame1).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn local_hinge_axis1(&self) -> Option<Vector> {
        match self.frame1.basis {
            JointBasis::Local(rotation) => Some(rotation * self.hinge_axis),
            _ => None,
        }
    }

    /// Returns the local [`hinge_axis`](RevoluteJoint::hinge_axis) of the second body.
    ///
    /// This is equivalent to rotating the [`hinge_axis`](RevoluteJoint::hinge_axis)
    /// by the local basis of [`frame2`](RevoluteJoint::frame2).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn local_hinge_axis2(&self) -> Option<Vector> {
        match self.frame2.basis {
            JointBasis::Local(rotation) => Some(rotation * self.hinge_axis),
            _ => None,
        }
    }

    /// Sets the limits of the allowed relative rotation around the [`hinge_axis`](RevoluteJoint::hinge_axis).
    #[inline]
    pub const fn with_angle_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.angle_limit = Some(AngleLimit::new(min, max));
        self
    }

    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_point_compliance`, `with_align_compliance`, and `with_limit_compliance` instead."
    )]
    pub const fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        #[cfg(feature = "3d")]
        {
            self.align_compliance = compliance;
        }
        self.limit_compliance = compliance;
        self
    }

    /// Sets the compliance of the point-to-point constraint (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_point_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        self
    }

    /// Sets the compliance of the axis alignment constraint (inverse of stiffness, N * m / rad).
    #[inline]
    #[cfg(feature = "3d")]
    pub const fn with_align_compliance(mut self, compliance: Scalar) -> Self {
        self.align_compliance = compliance;
        self
    }

    /// Sets the compliance of the angle limit (inverse of stiffness, N * m / rad).
    #[inline]
    pub const fn with_limit_compliance(mut self, compliance: Scalar) -> Self {
        self.limit_compliance = compliance;
        self
    }
}

impl MapEntities for RevoluteJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}

pub(super) fn plugin(app: &mut App) {
    app.register_type::<RevoluteJoint>();
    app.add_systems(
        PhysicsSchedule,
        update_local_frames.in_set(JointSet::PrepareAnchors),
    );
}

fn update_local_frames(
    mut joints: Query<&mut RevoluteJoint, Changed<RevoluteJoint>>,
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

impl DebugRenderConstraint<2> for RevoluteJoint {
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
