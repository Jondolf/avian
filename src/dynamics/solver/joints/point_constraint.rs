//! [`PointConstraint`] component.

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
use dynamics::solver::softness_parameters::{SoftnessCoefficients, SoftnessParameters};

use super::point_constraint_part::PointConstraintPart;

/// A point-to-point constraint that prevents relative translation of the attached bodies.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
#[require(PointConstraintSolverData)]
pub struct PointConstraint {
    /// First entity constrained by the joint.
    pub entity1: Entity,

    /// Second entity constrained by the joint.
    pub entity2: Entity,

    /// Attachment point on the first body.
    pub local_anchor1: Vector,

    /// Attachment point on the second body.
    pub local_anchor2: Vector,

    /// Soft constraint parameters for tuning the stiffness and damping of the joint.
    pub stiffness: SoftnessParameters,

    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
}

/// Cached data required by the impulse-based solver for [`PointConstraint`].
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct PointConstraintSolverData {
    pub coefficients: SoftnessCoefficients,
    pub point_constraint: PointConstraintPart,
    #[cfg(feature = "2d")]
    pub effective_mass: Matrix2,
    #[cfg(feature = "3d")]
    pub effective_mass: SymmetricMatrix3,
}

impl EntityConstraint<2> for PointConstraint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl ImpulseJoint for PointConstraint {
    type SolverData = PointConstraintSolverData;

    fn prepare(
        &mut self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut PointConstraintSolverData,
        delta_secs: Scalar,
    ) {
        // Update the world-space anchor points.
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        solver_data.point_constraint.r1 = *body1.rotation * local_r1;
        solver_data.point_constraint.r2 = *body2.rotation * local_r2;

        // Update the center difference.
        solver_data.point_constraint.center_difference = body2.position.0 - body1.position.0;

        let inverse_mass_sum = body1.mass.inverse() + body2.mass.inverse();
        let i1 = body1.effective_global_angular_inertia().inverse();
        let i2 = body2.effective_global_angular_inertia().inverse();

        // A revolute joint is a point-to-point constraint with optional limits.
        // It tries to align the points p2 and p1.
        //
        // Position constraint:
        //
        // C = p2 - p1 = x2 + r2 - x1 - r1 = 0
        //
        // where x1 and x2 are the positions of the bodies, and r1 and r2 are world-space anchor points
        // relative to the center of mass of the first and second body respectively.
        //
        // Velocity constraint:
        //
        // C' = v_r2 - v_r1
        //    = v2 + cross(w2, r2) - v1 - cross(w1, r1)
        //    = v2 + r2_skew * w2 - v1 - r2_skew * w1
        //    = 0
        //
        // where v1 and v2 are linear velocities, w1 and w2 are angular velocities,
        // and r_skew is a skew-symmetric matrix for vector r (see `SymmetricMatrix3::skew`).
        //
        // Jacobian:
        //
        //      lin1   ang1   lin2   ang2
        //     [ -E, -r1_skew, E, r2_skew ] J_trans
        // J = [  0, -axis_x,  0, axis_x  ] J_rot_x (3D only)
        //     [  0, -axis_y,  0, axis_y  ] J_rot_y (3D only)
        //
        // where E is the identity matrix, and axis_x and axis_y are the two orthogonal axes.
        //
        // Mass matrix:
        //
        //     [ m1E   0   0   0  ]
        // M = [  0    I1  0   0  ]
        //     [  0    0  m2E  0  ]
        //     [  0    0   0   I2 ]
        //
        // Effective inverse mass matrix in 2D:
        //
        // K = J * M^-1 * J^T
        //   = [  m1 + m2 + r1_y^2 * i1 + r2_y^2 * i2, -r1_y * r1_x * i1 - r2_y * r2_x * i2 ]
        //     [ -r1_y * r1_x * i1 - r2_y * r2_x * i2,  m1 + m2 + r1_x^2 * i1 + r2_x^2 * i2 ]
        //
        // Effective inverse mass matrix in 3D:
        //
        // TODO

        solver_data.effective_mass = solver_data
            .point_constraint
            .effective_inverse_mass(
                solver_data.point_constraint.r1,
                solver_data.point_constraint.r2,
                inverse_mass_sum,
                &i1,
                &i2,
            )
            .inverse();

        solver_data.coefficients = self.stiffness.compute_coefficients(delta_secs);

        // Prepare the relative dominance.
        self.relative_dominance = body1.dominance() - body2.dominance();
    }

    fn warm_start(
        &self,
        body1: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        inertia2: &SolverBodyInertia,
        solver_data: &PointConstraintSolverData,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        let r1 = body1.delta_rotation * solver_data.point_constraint.r1;
        let r2 = body2.delta_rotation * solver_data.point_constraint.r2;
        let point_impulse = solver_data.point_constraint.impulse;

        body1.linear_velocity -= point_impulse * inv_mass1;
        body1.angular_velocity -= inv_inertia1 * cross(r1, point_impulse);

        body2.linear_velocity += point_impulse * inv_mass2;
        body2.angular_velocity += inv_inertia2 * cross(r2, point_impulse);
    }

    fn solve(
        &self,
        body1: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        inertia2: &SolverBodyInertia,
        solver_data: &mut PointConstraintSolverData,
        _delta_secs: Scalar,
        use_bias: bool,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        let r1 = body1.delta_rotation * solver_data.point_constraint.r1;
        let r2 = body2.delta_rotation * solver_data.point_constraint.r2;

        // Solving without bias is done after position integration,
        // which can change rotation. Recompute the effective mass.
        if !use_bias {
            // TODO: Handle locked axes properly.
            let inverse_mass_sum = (inv_mass1 + inv_mass2).max_element();
            solver_data.effective_mass = solver_data
                .point_constraint
                .effective_inverse_mass(r1, r2, inverse_mass_sum, &inv_inertia1, &inv_inertia2)
                .inverse();
        }

        let impulse = solver_data.point_constraint.compute_incremental_impulse(
            body1,
            body2,
            r1,
            r2,
            &solver_data.effective_mass,
            &solver_data.coefficients,
            use_bias,
        );

        solver_data.point_constraint.impulse += impulse;

        body1.linear_velocity -= impulse * inv_mass1;
        body1.angular_velocity -= inv_inertia1 * cross(r1, impulse);

        body2.linear_velocity += impulse * inv_mass2;
        body2.angular_velocity += inv_inertia2 * cross(r2, impulse);
    }

    fn relative_dominance(&self) -> i16 {
        self.relative_dominance
    }
}

impl PointConstraint {
    /// Creates a new point-to-point constraint between two entities.
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            stiffness: SoftnessParameters::new(1.0, 30.0),
            relative_dominance: 0,
        }
    }

    /// Sets the anchor point of the joint in the local space of the first entity.
    pub fn with_local_anchor_1(self, anchor: Vector) -> Self {
        Self {
            local_anchor1: anchor,
            ..self
        }
    }

    /// Sets the anchor point of the joint in the local space of the second entity.
    pub fn with_local_anchor_2(self, anchor: Vector) -> Self {
        Self {
            local_anchor2: anchor,
            ..self
        }
    }

    /// Sets the softness parameters for the joint.
    pub fn with_softness(self, stiffness: SoftnessParameters) -> Self {
        Self { stiffness, ..self }
    }
}

impl MapEntities for PointConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}

impl ConstraintDebugRender for PointConstraint {
    fn render(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        color: Color,
        gizmos: &mut Gizmos<PhysicsGizmos>,
    ) {
        let r1 = body1.global_center_of_mass() + *body1.rotation * self.local_anchor1;
        let r2 = body2.global_center_of_mass() + *body2.rotation * self.local_anchor2;

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
