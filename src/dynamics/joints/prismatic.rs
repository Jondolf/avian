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

/// A prismatic joint prevents relative movement of the attached bodies, except for translation along the [`slider_axis`](PrismaticJoint::slider_axis).
///
/// Prismatic joints can be useful for things like elevators, pistons, sliding doors and moving platforms.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct PrismaticJoint {
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
    /// The local axis that the bodies can translate along.
    ///
    /// By default, this is the x-axis.
    pub slider_axis: Vector,
    /// The extents of the allowed relative translation along the [`slider_axis`](PrismaticJoint::slider_axis).
    pub limits: Option<DistanceLimit>,
    /// The compliance used for aligning the positions of the bodies to the principal axis (inverse of stiffness, m / N).
    pub axis_compliance: Scalar,
    /// The compliance of the angular constraint (inverse of stiffness, N * m / rad).
    pub angle_compliance: Scalar,
    /// The compliance of the distance limit (inverse of stiffness, m / N).
    pub limit_compliance: Scalar,
}

impl EntityConstraint<2> for PrismaticJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl PrismaticJoint {
    /// The default [`slider_axis`](PrismaticJoint::slider_axis) for a prismatic joint.
    pub const DEFAULT_SLIDER_AXIS: Vector = Vector::X;

    /// Creates a new [`PrismaticJoint`] between two entities.
    ///
    /// The default [`slider_axis`](PrismaticJoint::slider_axis) that relative translation is allowed along
    /// is the x-axis. This can be changed using [`with_slider_axis`](Self::with_slider_axis).
    #[inline]
    pub const fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            slider_axis: Self::DEFAULT_SLIDER_AXIS,
            limits: None,
            axis_compliance: 0.0,
            angle_compliance: 0.0,
            limit_compliance: 0.0,
        }
    }

    /// Sets the [`slider_axis`](PrismaticJoint::slider_axis) of the joint.
    ///
    /// The axis should be a unit vector. By default, this is the x-axis.
    #[inline]
    pub const fn with_slider_axis(mut self, axis: Vector) -> Self {
        self.slider_axis = axis;
        self
    }
}

impl_joint_frame_helpers!(PrismaticJoint);

impl PrismaticJoint {
    /// Returns the local [`slider axis`](PrismaticJoint::slider_axis) of the first body.
    ///
    /// This is equivalent to rotating the [`slider_axis`](PrismaticJoint::slider_axis)
    /// by the local basis of [`frame1`](PrismaticJoint::frame1).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_slider_axis1(&self) -> Option<Vector> {
        match self.frame1.basis {
            JointBasis::Local(rotation) => Some(rotation * self.slider_axis),
            _ => None,
        }
    }

    /// Returns the local [`slider axis`](PrismaticJoint::slider_axis) of the second body.
    ///
    /// This is equivalent to rotating the [`slider_axis`](PrismaticJoint::slider_axis)
    /// by the local basis of [`frame2`](PrismaticJoint::frame2).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_slider_axis2(&self) -> Option<Vector> {
        match self.frame2.basis {
            JointBasis::Local(rotation) => Some(rotation * self.slider_axis),
            _ => None,
        }
    }

    /// Sets the translational limits along the [`slider_axis`](PrismaticJoint::slider_axis).
    #[inline]
    pub const fn with_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.limits = Some(DistanceLimit::new(min, max));
        self
    }

    /// Sets the joint's compliance (inverse of stiffness).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_axis_compliance`, `with_limit_compliance`, and `with_angle_compliance` instead."
    )]
    pub const fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.axis_compliance = compliance;
        self.angle_compliance = compliance;
        self.limit_compliance = compliance;
        self
    }

    /// Sets the compliance of the axis alignment constraint (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_axis_compliance(mut self, compliance: Scalar) -> Self {
        self.axis_compliance = compliance;
        self
    }

    /// Sets the compliance of the angular constraint (inverse of stiffness, N * m / rad).
    #[inline]
    pub const fn with_angle_compliance(mut self, compliance: Scalar) -> Self {
        self.angle_compliance = compliance;
        self
    }

    /// Sets the compliance of the distance limit (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_limit_compliance(mut self, compliance: Scalar) -> Self {
        self.limit_compliance = compliance;
        self
    }
}

impl MapEntities for PrismaticJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}

pub(super) fn plugin(app: &mut App) {
    app.register_type::<PrismaticJoint>();
    app.add_systems(
        PhysicsSchedule,
        update_local_frames.in_set(JointSet::PrepareAnchors),
    );
}

fn update_local_frames(
    mut joints: Query<&mut PrismaticJoint, Changed<PrismaticJoint>>,
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

impl DebugRenderConstraint<2> for PrismaticJoint {
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
