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

/// A distance joint keeps the attached bodies at a certain distance from each other while while allowing rotation around all axes.
///
/// Distance joints can be useful for things like springs, muscles, and mass-spring networks.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct DistanceJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// The joint anchor point on the first body.
    pub anchor1: JointAnchor,
    /// The joint anchor point on the second body.
    pub anchor2: JointAnchor,
    /// The extents of the allowed relative translation between the attached bodies.
    pub limits: DistanceLimit,
    /// The joint's compliance, the inverse of stiffness (m / N).
    pub compliance: Scalar,
}

impl EntityConstraint<2> for DistanceJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl DistanceJoint {
    /// Creates a new [`DistanceJoint`] between two entities.
    #[inline]
    pub const fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            anchor1: JointAnchor::ZERO,
            anchor2: JointAnchor::ZERO,
            limits: DistanceLimit::ZERO,
            compliance: 0.0,
        }
    }

    /// Sets the global anchor point on both bodies.
    ///
    /// This configures the [`JointTranslation`] of each [`JointFrame`].
    #[inline]
    pub const fn with_anchor(mut self, anchor: Vector) -> Self {
        self.anchor1 = JointAnchor::FromGlobal(anchor);
        self.anchor2 = JointAnchor::FromGlobal(anchor);
        self
    }

    /// Sets the local anchor point on the first body.
    ///
    /// This configures the [`JointTranslation`] of the first [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor1(mut self, anchor: Vector) -> Self {
        self.anchor1 = JointAnchor::Local(anchor);
        self
    }

    /// Sets the local anchor point on the second body.
    ///
    /// This configures the [`JointTranslation`] of the second [`JointFrame`].
    #[inline]
    pub const fn with_local_anchor2(mut self, anchor: Vector) -> Self {
        self.anchor2 = JointAnchor::Local(anchor);
        self
    }

    /// Returns the local anchor point on the first body.
    ///
    /// If the [`JointTranslation`] is set to [`FromGlobal`](JointTranslation::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor1(&self) -> Option<Vector> {
        match self.anchor1 {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local anchor point on the second body.
    ///
    /// If the [`JointTranslation`] is set to [`FromGlobal`](JointTranslation::FromGlobal),
    /// and the local anchor has not yet been computed, this will return `None`.
    #[inline]
    pub const fn local_anchor2(&self) -> Option<Vector> {
        match self.anchor2 {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Sets the joint's rest length, or distance the bodies will be kept at.
    #[inline]
    #[deprecated(note = "Use `with_limits` instead.", since = "0.4.0")]
    pub const fn with_rest_length(mut self, rest_length: Scalar) -> Self {
        self.limits = DistanceLimit::new(rest_length, rest_length);
        self
    }

    /// Sets the minimum and maximum distances between the attached bodies.
    #[inline]
    pub const fn with_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.limits = DistanceLimit::new(min, max);
        self
    }

    /// Sets the joint's compliance (inverse of stiffness, m / N).
    #[inline]
    pub const fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.compliance = compliance;
        self
    }
}

impl MapEntities for DistanceJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}

pub(super) fn plugin(app: &mut App) {
    app.register_type::<DistanceJoint>();
    app.add_systems(
        PhysicsSchedule,
        update_local_anchors.in_set(JointSet::PrepareAnchors),
    );
}

fn update_local_anchors(
    mut joints: Query<&mut DistanceJoint, Changed<DistanceJoint>>,
    bodies: Query<(&Position, &Rotation)>,
) {
    for mut joint in &mut joints {
        if matches!(joint.anchor1, JointAnchor::Local(_))
            && matches!(joint.anchor2, JointAnchor::Local(_))
        {
            continue;
        }

        let Ok([(pos1, rot1), (pos2, rot2)]) = bodies.get_many(joint.entities()) else {
            continue;
        };

        let [anchor1, anchor2] =
            JointAnchor::compute_local(joint.anchor1, joint.anchor2, pos1.0, pos2.0, rot1, rot2);
        joint.anchor1 = anchor1;
        joint.anchor2 = anchor2;
    }
}

impl DebugRenderConstraint<2> for DistanceJoint {
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
