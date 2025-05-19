//! [`WeldJoint`] component.

use crate::{
    dynamics::solver::solver_body::{SolverBody, SolverBodyInertia},
    prelude::*,
};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};
use dynamics::solver::softness_parameters::SoftnessParameters;

use super::{
    fixed_angle_constraint::FixedAngleConstraintSolverData,
    point_constraint::PointConstraintSolverData,
};

// TODO: It's possible to solve the position and rotation constraint simultaneously, see Bepu.
/// A weld joint prevents all relative movement of the attached bodies.
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
#[require(WeldJointSolverData)]
pub struct WeldJoint {
    /// The point-to-point constraint part of the joint.
    pub point_constraint: PointConstraint,

    /// The fixed angle constraint part of the joint.
    pub fixed_angle_constraint: FixedAngleConstraint,
}

/// Cached data required by the impulse-based solver for [`WeldJoint`].
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct WeldJointSolverData {
    /// The point-to-point constraint part of the weld joint.
    pub point_constraint: PointConstraintSolverData,

    /// The fixed angle constraint part of the weld joint.
    pub fixed_angle_constraint: FixedAngleConstraintSolverData,
}

impl EntityConstraint<2> for WeldJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.point_constraint.entity1, self.point_constraint.entity2]
    }
}

impl ImpulseJoint for WeldJoint {
    type SolverData = WeldJointSolverData;

    fn prepare(
        &mut self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut WeldJointSolverData,
        delta_secs: Scalar,
    ) {
        // Prepare the point-to-point constraint.
        self.point_constraint
            .prepare(body1, body2, &mut solver_data.point_constraint, delta_secs);

        // Prepare the fixed angle constraint.
        self.fixed_angle_constraint.prepare(
            body1,
            body2,
            &mut solver_data.fixed_angle_constraint,
            delta_secs,
        );
    }

    fn warm_start(
        &self,
        body1: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        inertia2: &SolverBodyInertia,
        solver_data: &WeldJointSolverData,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        let r1 = solver_data.point_constraint.point_constraint.r1;
        let r2 = solver_data.point_constraint.point_constraint.r2;
        let point_impulse = solver_data.point_constraint.point_constraint.impulse;
        let angular_impulse = solver_data.fixed_angle_constraint.angular_impulse;

        body1.linear_velocity -= point_impulse * inv_mass1;
        body1.angular_velocity -= inv_inertia1 * (cross(r1, point_impulse) + angular_impulse);

        body2.linear_velocity += point_impulse * inv_mass2;
        body2.angular_velocity += inv_inertia2 * (cross(r2, point_impulse) + angular_impulse);
    }

    fn solve(
        &self,
        body1: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        inertia2: &SolverBodyInertia,
        solver_data: &mut WeldJointSolverData,
        delta_secs: Scalar,
        use_bias: bool,
    ) {
        // Solve the fixed angle constraint.
        self.fixed_angle_constraint.solve(
            body1,
            inertia1,
            body2,
            inertia2,
            &mut solver_data.fixed_angle_constraint,
            delta_secs,
            use_bias,
        );

        // Solve the point-to-point constraint.
        self.point_constraint.solve(
            body1,
            inertia1,
            body2,
            inertia2,
            &mut solver_data.point_constraint,
            delta_secs,
            use_bias,
        );
    }

    fn relative_dominance(&self) -> i16 {
        self.point_constraint.relative_dominance
    }
}

impl WeldJoint {
    /// Creates a new weld joint between two entities.
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            point_constraint: PointConstraint::new(entity1, entity2),
            fixed_angle_constraint: FixedAngleConstraint::new(),
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

    /// Sets the softness parameters for the joint.
    pub fn with_softness(mut self, stiffness: SoftnessParameters) -> Self {
        self.point_constraint.stiffness = stiffness;
        self
    }
}

impl MapEntities for WeldJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.point_constraint.entity1 = entity_mapper.get_mapped(self.point_constraint.entity1);
        self.point_constraint.entity2 = entity_mapper.get_mapped(self.point_constraint.entity2);
    }
}
