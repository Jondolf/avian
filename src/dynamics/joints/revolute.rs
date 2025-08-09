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

/// A revolute joint prevents relative movement of the attached bodies, except for rotation around one `aligned_axis`.
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
    /// The reference frame of the first body, defining the anchor point and reference rotation
    /// relative to the body transform.
    pub frame1: JointFrame,
    /// The reference frame of the second body, defining the anchor point and reference rotation
    /// relative to the body transform.
    pub frame2: JointFrame,
    /// A unit vector that controls which axis should be aligned for both bodies.
    #[cfg(feature = "3d")]
    pub aligned_axis: Vector,
    /// The extents of the allowed relative rotation of the bodies around the `aligned_axis`.
    pub angle_limit: Option<AngleLimit>,
    /// The compliance of the point-to-point constraint (inverse of stiffness, m / N).
    pub point_compliance: Scalar,
    /// The compliance used for aligning the bodies along the `aligned_axis` (inverse of stiffness, N * m / rad).
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
    /// Creates a new [`RevoluteJoint`] between two entities.
    #[inline]
    pub const fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            #[cfg(feature = "3d")]
            aligned_axis: Vector::Z,
            angle_limit: None,
            point_compliance: 0.0,
            align_compliance: 0.0,
            limit_compliance: 0.0,
        }
    }

    /// Sets the global anchor point on both bodies.
    ///
    /// This configures the [`JointTranslation`] of each [`JointFrame`].
    #[inline]
    pub const fn with_global_anchor(mut self, anchor: Vector) -> Self {
        self.frame1.translation = JointTranslation::FromGlobal(anchor);
        self.frame2.translation = JointTranslation::FromGlobal(anchor);
        self
    }

    /// Sets the local anchor point on the first body.
    ///
    /// This configures the [`JointTranslation`] of the first [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor1(mut self, anchor: Vector) -> Self {
        self.frame1.translation = JointTranslation::Local(anchor);
        self
    }

    /// Sets the local anchor point on the second body.
    ///
    /// This configures the [`JointTranslation`] of the second [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor2(mut self, anchor: Vector) -> Self {
        self.frame2.translation = JointTranslation::Local(anchor);
        self
    }

    /// Sets the global reference rotation of both bodies.
    ///
    /// This configures the [`JointRotation`] of each [`JointFrame`].
    #[inline]
    pub fn with_global_rotation(mut self, rotation: impl Into<Rot>) -> Self {
        let rotation = rotation.into();
        self.frame1.rotation = JointRotation::FromGlobal(rotation);
        self.frame2.rotation = JointRotation::FromGlobal(rotation);
        self
    }

    /// Sets the local reference rotation of the first body.
    ///
    /// This configures the [`JointRotation`] of the first [`JointFrame`].
    #[inline]
    pub fn with_local_rotation1(mut self, rotation: impl Into<Rot>) -> Self {
        self.frame1.rotation = JointRotation::Local(rotation.into());
        self
    }

    /// Sets the local reference rotation of the second body.
    ///
    /// This configures the [`JointRotation`] of the second [`JointFrame`].
    #[inline]
    pub fn with_local_rotation2(mut self, rotation: impl Into<Rot>) -> Self {
        self.frame2.rotation = JointRotation::Local(rotation.into());
        self
    }

    /// Returns the local anchor point on the first body.
    ///
    /// If the [`JointTranslation`] is set to [`FromGlobal`](JointTranslation::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor1(&self) -> Option<Vector> {
        match self.frame1.translation {
            JointTranslation::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local anchor point on the second body.
    ///
    /// If the [`JointTranslation`] is set to [`FromGlobal`](JointTranslation::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor2(&self) -> Option<Vector> {
        match self.frame2.translation {
            JointTranslation::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local reference rotation of the first body.
    ///
    /// If the [`JointRotation`] is set to [`FromGlobal`](JointRotation::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_rotation1(&self) -> Option<Rot> {
        match self.frame1.rotation {
            JointRotation::Local(rotation) => Some(rotation),
            _ => None,
        }
    }

    /// Returns the local reference rotation of the second body.
    ///
    /// If the [`JointRotation`] is set to [`FromGlobal`](JointRotation::FromGlobal),
    /// and the local rotation has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_rotation2(&self) -> Option<Rot> {
        match self.frame2.rotation {
            JointRotation::Local(rotation) => Some(rotation),
            _ => None,
        }
    }

    /// Sets the axis that the bodies should be aligned on.
    #[inline]
    #[cfg(feature = "3d")]
    pub const fn with_aligned_axis(mut self, axis: Vector) -> Self {
        self.aligned_axis = axis;
        self
    }

    /// Sets the limits of the allowed relative rotation around the `aligned_axis`.
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
        self.align_compliance = compliance;
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
        if matches!(joint.frame1.translation, JointTranslation::Local(_))
            && matches!(joint.frame2.translation, JointTranslation::Local(_))
            && matches!(joint.frame1.rotation, JointRotation::Local(_))
            && matches!(joint.frame2.rotation, JointRotation::Local(_))
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
