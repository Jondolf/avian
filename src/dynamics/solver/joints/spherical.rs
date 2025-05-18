//! [`SphericalJoint`] component.

use crate::prelude::*;
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};
use dynamics::solver::softness_parameters::{SoftnessCoefficients, SoftnessParameters};

use super::{point_constraint::PointConstraintSolverData, swing_limit::SwingLimitSolverData};

/// A spherical joint prevents relative translation of the attached bodies while allowing rotation around all axes.
///
/// Spherical joints can be useful for things like pendula, chains, ragdolls etc.
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
#[require(SphericalJointSolverData)]
pub struct SphericalJoint {
    /// The point-to-point constraint of the joint.
    pub point_constraint: PointConstraint,

    /// The swing limit of the joint.
    pub swing_limit: Option<SwingLimit>,
}

/// Cached data required by the impulse-based solver for [`SphericalJoint`].
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct SphericalJointSolverData {
    /// The point-to-point constraint part of the spherical joint.
    pub point_constraint: PointConstraintSolverData,

    /// The swing limit constraint part of the spherical joint.
    pub swing_limit: SwingLimitSolverData,

    /// The effective mass of the point-to-point constraint.
    pub effective_point_constraint_mass: SymmetricMatrix3,

    /// Coefficients computed for the spring parameters of the constraint.
    pub coefficients: SoftnessCoefficients,
}

impl EntityConstraint<2> for SphericalJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.point_constraint.entity1, self.point_constraint.entity2]
    }
}

impl ImpulseJoint for SphericalJoint {
    type SolverData = SphericalJointSolverData;

    fn prepare(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut SphericalJointSolverData,
        delta_secs: Scalar,
    ) {
        // Prepare the point-to-point constraint.
        self.point_constraint
            .prepare(body1, body2, &mut solver_data.point_constraint, delta_secs);

        // Prepare the swing limit constraint.
        if let Some(swing_limit) = &self.swing_limit {
            swing_limit.prepare(body1, body2, &mut solver_data.swing_limit, delta_secs);
        }
    }

    fn warm_start(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &SphericalJointSolverData,
    ) {
        // Warm start the point-to-point constraint.
        self.point_constraint
            .warm_start(body1, body2, &solver_data.point_constraint);

        // Warm start the swing limit constraint.
        if let Some(swing_limit) = &self.swing_limit {
            swing_limit.warm_start(body1, body2, &solver_data.swing_limit);
        }
    }

    fn solve(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &mut SphericalJointSolverData,
        delta_secs: Scalar,
        use_bias: bool,
    ) {
        // Solve the point-to-point constraint.
        self.point_constraint.solve(
            body1,
            body2,
            &mut solver_data.point_constraint,
            delta_secs,
            use_bias,
        );

        // Solve the swing limit constraint.
        if let Some(swing_limit) = &self.swing_limit {
            swing_limit.solve(
                body1,
                body2,
                &mut solver_data.swing_limit,
                delta_secs,
                use_bias,
            );
        }
    }
}

impl SphericalJoint {
    /// Creates a new spherical joint between two entities.
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            point_constraint: PointConstraint::new(entity1, entity2),
            swing_limit: None,
        }
    }

    /// Sets the anchor point of the joint in the local space of the first entity.
    pub fn with_local_anchor_1(mut self, anchor: Vector) -> Self {
        self.point_constraint.local_anchor1 = anchor;
        self
    }

    /// Sets the anchor point of the joint in the local space of the second entity.
    pub fn with_local_anchor_2(mut self, anchor: Vector) -> Self {
        self.point_constraint.local_anchor2 = anchor;
        self
    }

    /// Sets the swing axis attached to the first body in its local space.
    pub fn with_local_swing_axis1(mut self, axis: Vector) -> Self {
        let swing_limit = self.swing_limit.get_or_insert(SwingLimit::new(PI));
        swing_limit.local_axis1 = axis;
        self
    }

    /// Sets the swing axis attached to the second body in its local space.
    pub fn with_local_swing_axis2(mut self, axis: Vector) -> Self {
        let swing_limit = self.swing_limit.get_or_insert(SwingLimit::new(PI));
        swing_limit.local_axis2 = axis;
        self
    }

    /// Sets the limits of the allowed relative rotation around the swing axis.
    pub fn with_swing_limits(mut self, max_angle: Scalar) -> Self {
        let swing_limit = self.swing_limit.get_or_insert(SwingLimit::new(PI));
        swing_limit.max_angle = max_angle;
        self
    }

    /// Sets the softness parameters of the joint.
    pub fn with_point_softness(mut self, softness: SoftnessParameters) -> Self {
        self.point_constraint.stiffness = softness;
        self
    }

    /// Sets the softness parameters of the swing limit.
    pub fn with_swing_limit_softness(mut self, softness: SoftnessParameters) -> Self {
        let swing_limit = self.swing_limit.get_or_insert(SwingLimit::new(PI));
        swing_limit.stiffness = softness;
        self
    }
}

impl MapEntities for SphericalJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.point_constraint.entity1 = entity_mapper.get_mapped(self.point_constraint.entity1);
        self.point_constraint.entity2 = entity_mapper.get_mapped(self.point_constraint.entity2);
    }
}

impl ConstraintDebugRender for SphericalJoint {
    fn render(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        color: Color,
        gizmos: &mut Gizmos<PhysicsGizmos>,
    ) {
        let r1 =
            body1.global_center_of_mass() + *body1.rotation * self.point_constraint.local_anchor1;
        let r2 =
            body2.global_center_of_mass() + *body2.rotation * self.point_constraint.local_anchor2;

        #[cfg(feature = "2d")]
        {
            gizmos.circle_2d(r1, 0.05, color);
            gizmos.circle_2d(r2, 0.05, color);
        }
        #[cfg(feature = "3d")]
        {
            gizmos.sphere(r1, 0.05, color);
            gizmos.sphere(r2, 0.05, color);
        }
    }
}
