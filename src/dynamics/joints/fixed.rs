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
    /// The joint anchor point on the first body.
    pub anchor1: JointAnchor,
    /// The joint anchor point on the second body.
    pub anchor2: JointAnchor,
    /// The rotation of the second body relative to the first body in radiands.
    ///
    /// This defines the target relative orientation of the bodies.
    #[cfg(feature = "2d")]
    pub reference_rotation: Scalar,
    /// The rotation of the second body relative to the first body.
    ///
    /// This defines the target relative orientation of the bodies.
    #[cfg(feature = "3d")]
    pub reference_rotation: Quaternion,
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
            anchor1: JointAnchor::Auto,
            anchor2: JointAnchor::Auto,
            #[cfg(feature = "2d")]
            reference_rotation: 0.0,
            #[cfg(feature = "3d")]
            reference_rotation: Quaternion::IDENTITY,
            point_compliance: 0.0,
            angle_compliance: 0.0,
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
        update_local_anchors.in_set(JointSet::PrepareAnchors),
    );
}

fn update_local_anchors(
    mut joints: Query<&mut FixedJoint, Changed<FixedJoint>>,
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

        // TODO: Use weighted COM average for dynamic Auto-Auto pairs.
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
