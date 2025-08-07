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
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct RevoluteJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// The joint anchor point on the first body.
    pub anchor1: JointAnchor,
    /// The joint anchor point on the second body.
    pub anchor2: JointAnchor,
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
            anchor1: JointAnchor::Auto,
            anchor2: JointAnchor::Auto,
            #[cfg(feature = "3d")]
            aligned_axis: Vector::Z,
            angle_limit: None,
            point_compliance: 0.0,
            align_compliance: 0.0,
            limit_compliance: 0.0,
        }
    }

    /// Sets the local [`JointAnchor`] on the first body.
    #[inline]
    pub const fn with_local_anchor_1(mut self, anchor: Vector) -> Self {
        self.anchor1 = JointAnchor::Local(anchor);
        self
    }

    /// Sets the local [`JointAnchor`] on the second body.
    #[inline]
    pub const fn with_local_anchor_2(mut self, anchor: Vector) -> Self {
        self.anchor2 = JointAnchor::Local(anchor);
        self
    }

    /// Sets the global [`JointAnchor`] on the first body.
    #[inline]
    pub const fn with_global_anchor_1(mut self, anchor: Vector) -> Self {
        self.anchor1 = JointAnchor::FromGlobal(anchor);
        self
    }

    /// Sets the global [`JointAnchor`] on the second body.
    #[inline]
    pub const fn with_global_anchor_2(mut self, anchor: Vector) -> Self {
        self.anchor2 = JointAnchor::FromGlobal(anchor);
        self
    }

    /// Returns the [`JointAnchor`] on the first body.
    ///
    /// This is stored as [`JointAnchor::Local`] after the first physics step
    /// after the joint was initialized.
    #[inline]
    pub const fn anchor1(&self) -> JointAnchor {
        self.anchor1
    }

    /// Returns the [`JointAnchor`] on the second body.
    ///
    /// This is stored as [`JointAnchor::Local`] after the first physics step
    /// after the joint was initialized.
    #[inline]
    pub const fn anchor2(&self) -> JointAnchor {
        self.anchor2
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
        update_local_anchors.in_set(JointSet::PrepareAnchors),
    );
}

fn update_local_anchors(
    mut joints: Query<&mut RevoluteJoint, Changed<RevoluteJoint>>,
    bodies: Query<(&Position, &Rotation, &RigidBody)>,
) {
    for mut joint in &mut joints {
        if matches!(joint.anchor1, JointAnchor::Local(_))
            && matches!(joint.anchor2, JointAnchor::Local(_))
        {
            continue;
        }

        let Ok([(pos1, rot1, rb1), (pos2, rot2, _)]) = bodies.get_many(joint.entities()) else {
            continue;
        };

        let [anchor1, anchor2] = JointAnchor::compute_local_anchors(
            joint.anchor1,
            joint.anchor2,
            pos1.0,
            pos2.0,
            rot1,
            rot2,
            rb1.is_dynamic(),
        );
        joint.anchor1 = anchor1;
        joint.anchor2 = anchor2;
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

        let JointAnchor::Local(local_anchor1) = self.anchor1 else {
            return;
        };
        let JointAnchor::Local(local_anchor2) = self.anchor2 else {
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
