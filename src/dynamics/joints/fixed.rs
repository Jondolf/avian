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

/// A fixed joint prevents any relative movement of the attached bodies.
///
/// You should generally prefer using a single body instead of multiple bodies fixed together,
/// but fixed joints can be useful for things like rigid structures where a force can dynamically break the joints connecting individual bodies.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct FixedJoint {
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
    /// The compliance of the point-to-point constraint (inverse of stiffness, m / N).
    pub point_compliance: Scalar,
    /// The compliance of the angular constraint (inverse of stiffness, N * m / rad).
    pub angle_compliance: Scalar,
}

impl EntityConstraint<2> for FixedJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl FixedJoint {
    /// Creates a new [`FixedJoint`] between two entities.
    #[inline]
    pub const fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            point_compliance: 0.0,
            angle_compliance: 0.0,
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

    /// Sets the joint's compliance (inverse of stiffness).
    #[inline]
    #[deprecated(
        since = "0.4.0",
        note = "Use `with_point_compliance` and `with_angle_compliance` instead."
    )]
    pub const fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        self.angle_compliance = compliance;
        self
    }

    /// Sets the compliance of the point-to-point compliance (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_point_compliance(mut self, compliance: Scalar) -> Self {
        self.point_compliance = compliance;
        self
    }

    /// Sets the compliance of the angular constraint (inverse of stiffness, (N * m / rad).
    #[inline]
    pub const fn with_angle_compliance(mut self, compliance: Scalar) -> Self {
        self.angle_compliance = compliance;
        self
    }
}

impl MapEntities for FixedJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}

pub(super) fn plugin(app: &mut App) {
    app.register_type::<FixedJoint>();
    app.add_systems(
        PhysicsSchedule,
        update_local_frames.in_set(JointSet::PrepareAnchors),
    );
}

fn update_local_frames(
    mut joints: Query<&mut FixedJoint, Changed<FixedJoint>>,
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

        // TODO: Use weighted COM average for the anchors of dynamic Auto-Auto pairs.
        let [frame1, frame2] =
            JointFrame::compute_local(joint.frame1, joint.frame2, pos1.0, pos2.0, rot1, rot2);
        joint.frame1 = frame1;
        joint.frame2 = frame2;
    }
}

impl DebugRenderConstraint<2> for FixedJoint {
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
