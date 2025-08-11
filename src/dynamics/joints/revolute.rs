use crate::{
    dynamics::joints::{EntityConstraint, JointSet},
    prelude::*,
};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};

/// A revolute joint prevents relative movement of the attached bodies, except for rotation around one [`HINGE_AXIS`](Self::HINGE_AXIS).
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
    /// The extents of the allowed relative rotation of the bodies around the [`HINGE_AXIS`](Self::HINGE_AXIS).
    pub angle_limit: Option<AngleLimit>,
    /// The compliance of the point-to-point constraint (inverse of stiffness, m / N).
    pub point_compliance: Scalar,
    /// The compliance used for aligning the bodies along the [`HINGE_AXIS`](Self::HINGE_AXIS) (inverse of stiffness, N * m / rad).
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
    /// The axis along which the bodies can rotate relative to each other.
    ///
    /// This is the z-axis of the [`JointBasis`].
    #[cfg(feature = "3d")]
    pub const HINGE_AXIS: Vector = Vector::Z;

    /// A normal axis that is perpendicular to the [`HINGE_AXIS`](Self::HINGE_AXIS).
    ///
    /// This is the y-axis of the [`JointBasis`] and determines the rotation about the [`HINGE_AXIS`](Self::HINGE_AXIS).
    #[cfg(feature = "3d")]
    pub const NORMAL_AXIS: Vector = Vector::Y;

    /// Creates a new [`RevoluteJoint`] between two entities.
    #[inline]
    pub const fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            angle_limit: None,
            point_compliance: 0.0,
            #[cfg(feature = "3d")]
            align_compliance: 0.0,
            limit_compliance: 0.0,
        }
    }

    /// Sets the local [`JointFrame`] of the first body, configuring both the [`JointAnchor`] and [`JointBasis`].
    #[inline]
    pub fn with_local_frame1(mut self, frame: impl Into<Isometry>) -> Self {
        self.frame1 = JointFrame::local(frame);
        self
    }

    /// Sets the local [`JointFrame`] of the second body, configuring both the [`JointAnchor`] and [`JointBasis`].
    #[inline]
    pub fn with_local_frame2(mut self, frame: impl Into<Isometry>) -> Self {
        self.frame2 = JointFrame::local(frame);
        self
    }

    /// Sets the global anchor point on both bodies.
    ///
    /// This configures the [`JointAnchor`] of each [`JointFrame`].
    #[inline]
    pub const fn with_anchor(mut self, anchor: Vector) -> Self {
        self.frame1.anchor = JointAnchor::FromGlobal(anchor);
        self.frame2.anchor = JointAnchor::FromGlobal(anchor);
        self
    }

    /// Sets the local anchor point on the first body.
    ///
    /// This configures the [`JointAnchor`] of the first [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor1(mut self, anchor: Vector) -> Self {
        self.frame1.anchor = JointAnchor::Local(anchor);
        self
    }

    /// Sets the local anchor point on the second body.
    ///
    /// This configures the [`JointAnchor`] of the second [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor2(mut self, anchor: Vector) -> Self {
        self.frame2.anchor = JointAnchor::Local(anchor);
        self
    }

    /// Sets the global basis for both bodies.
    ///
    /// This configures the [`JointBasis`] of each [`JointFrame`].
    #[inline]
    pub fn with_basis(mut self, basis: impl Into<Rot>) -> Self {
        let basis = basis.into();
        self.frame1.basis = JointBasis::FromGlobal(basis);
        self.frame2.basis = JointBasis::FromGlobal(basis);
        self
    }

    /// Sets the local basis for the first body.
    ///
    /// This configures the [`JointBasis`] of the first [`JointFrame`].
    #[inline]
    pub fn with_local_basis1(mut self, basis: impl Into<Rot>) -> Self {
        self.frame1.basis = JointBasis::Local(basis.into());
        self
    }

    /// Sets the local basis for the second body.
    ///
    /// This configures the [`JointBasis`] of the second [`JointFrame`].
    #[inline]
    pub fn with_local_basis2(mut self, basis: impl Into<Rot>) -> Self {
        self.frame2.basis = JointBasis::Local(basis.into());
        self
    }

    /// Orients the [`JointBasis`] of [`frame1`](Self::frame1) and [`frame2`](Self::frame2) such that
    /// the [`HINGE_AXIS`](Self::HINGE_AXIS) (local z) and [`NORMAL_AXIS`](Self::NORMAL_AXIS) (local y)
    /// align with the given `hinge_axis` and `normal_axis` in world space.
    ///
    /// The remaining axis (local x) is computed as the cross product of the `normal_axis` and `hinge_axis`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn with_axes(mut self, hinge_axis: Vector, normal_axis: Vector) -> Self {
        let basis = JointBasis::from_global_yz(normal_axis, hinge_axis);
        self.frame1.basis = basis;
        self.frame2.basis = basis;
        self
    }

    /// Orients the [`JointBasis`] of [`frame1`](Self::frame1) such that
    /// the [`HINGE_AXIS`](Self::HINGE_AXIS) (local z) and [`NORMAL_AXIS`](Self::NORMAL_AXIS) (local y)
    /// align with the given `hinge_axis` and `normal_axis` in local space.
    ///
    /// The remaining axis (local x) is computed as the cross product of the `normal_axis` and `hinge_axis`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn with_local_axes1(mut self, hinge_axis: Vector, normal_axis: Vector) -> Self {
        self.frame1.basis = JointBasis::from_local_yz(normal_axis, hinge_axis);
        self
    }

    /// Orients the [`JointBasis`] of [`frame2`](Self::frame2) such that
    /// the [`HINGE_AXIS`](Self::HINGE_AXIS) (local z) and [`NORMAL_AXIS`](Self::NORMAL_AXIS) (local y)
    /// align with the given `hinge_axis` and `normal_axis` in local space.
    ///
    /// The remaining axis (local x) is computed as the cross product of the `normal_axis` and `hinge_axis`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn with_local_axes2(mut self, hinge_axis: Vector, normal_axis: Vector) -> Self {
        self.frame2.basis = JointBasis::from_local_yz(normal_axis, hinge_axis);
        self
    }

    /// Returns the local [`JointFrame`] of the first body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, or the [`JointBasis`] is set to
    /// [`FromGlobal`](JointBasis::FromGlobal), and the local basis has not yet
    /// been computed, this will return `None`.
    #[inline]
    pub fn local_frame1(&self) -> Option<Isometry> {
        self.frame1.get_local_isometry()
    }

    /// Returns the local [`JointFrame`] of the second body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, or the [`JointBasis`] is set to
    /// [`FromGlobal`](JointBasis::FromGlobal), and the local basis has not yet
    /// been computed, this will return `None`.
    #[inline]
    pub fn local_frame2(&self) -> Option<Isometry> {
        self.frame2.get_local_isometry()
    }

    /// Returns the local anchor point on the first body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor1(&self) -> Option<Vector> {
        match self.frame1.anchor {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local anchor point on the second body.
    ///
    /// If the [`JointAnchor`] is set to [`FromGlobal`](JointAnchor::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor2(&self) -> Option<Vector> {
        match self.frame2.anchor {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local basis of the first body.
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_basis1(&self) -> Option<Rot> {
        match self.frame1.basis {
            JointBasis::Local(basis) => Some(basis),
            _ => None,
        }
    }

    /// Returns the local basis of the second body.
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    pub fn local_basis2(&self) -> Option<Rot> {
        match self.frame2.basis {
            JointBasis::Local(basis) => Some(basis),
            _ => None,
        }
    }

    /// Returns the local hinge axis of the first body.
    ///
    /// This is equivalent to rotating the [`HINGE_AXIS`](Self::HINGE_AXIS)
    /// by the local basis of [`frame1`](Self::frame1).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn local_hinge_axis1(&self) -> Option<Vector> {
        match self.frame1.basis {
            JointBasis::Local(basis) => Some(basis * Self::HINGE_AXIS),
            _ => None,
        }
    }

    /// Returns the local hinge axis of the second body.
    ///
    /// This is equivalent to rotating the [`HINGE_AXIS`](Self::HINGE_AXIS)
    /// by the local basis of [`frame2`](Self::frame2).
    ///
    /// If the [`JointBasis`] is set to [`FromGlobal`](JointBasis::FromGlobal),
    /// and the local basis has not yet been computed, this will return `None`.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn local_hinge_axis2(&self) -> Option<Vector> {
        match self.frame2.basis {
            JointBasis::Local(basis) => Some(basis * Self::HINGE_AXIS),
            _ => None,
        }
    }

    /// Sets the limits of the allowed relative rotation.
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
