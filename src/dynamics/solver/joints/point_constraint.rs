//! [`PointConstraint`] component.

use crate::prelude::*;
use bevy::{
    ecs::{
        component::{ComponentHooks, StorageType},
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};
use dynamics::solver::softness_parameters::{SoftnessCoefficients, SoftnessParameters};

use super::point_constraint_part::PointConstraintPart;

/// A point-to-point constraint that prevents relative translation of the attached bodies.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
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
}

impl Component for PointConstraint {
    const STORAGE_TYPE: StorageType = StorageType::Table;

    fn register_component_hooks(hooks: &mut ComponentHooks) {
        hooks.on_add(|mut world, entity, _| {
            world
                .commands()
                .entity(entity)
                .insert(PointConstraintSolverData::default());
        });
    }
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
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut PointConstraintSolverData,
        delta_secs: Scalar,
    ) {
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        let r1 = *body1.rotation * local_r1;
        let r2 = *body2.rotation * local_r2;

        // TODO: Support frames.
        solver_data.point_constraint.center_difference =
            body2.current_position() - body1.current_position();

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

        solver_data.effective_mass =
            PointConstraintPart::compute_inverse_effective_mass(inverse_mass_sum, &i1, &i2, r1, r2)
                .inverse();

        solver_data.coefficients = self.stiffness.compute_coefficients(delta_secs);
    }

    fn warm_start(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &PointConstraintSolverData,
    ) {
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        let r1 = *body1.rotation * local_r1;
        let r2 = *body2.rotation * local_r2;

        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_global_angular_inertia().inverse();
        let inv_inertia2 = body2.effective_global_angular_inertia().inverse();

        if body1.rb.is_dynamic() {
            body1.linear_velocity.0 -= solver_data.point_constraint.impulse * inv_mass1;
            body1.angular_velocity.0 -=
                inv_inertia1 * cross(r1, solver_data.point_constraint.impulse);
        }
        if body2.rb.is_dynamic() {
            body2.linear_velocity.0 += solver_data.point_constraint.impulse * inv_mass2;
            body2.angular_velocity.0 +=
                inv_inertia2 * cross(r2, solver_data.point_constraint.impulse);
        }
    }

    fn solve(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &mut PointConstraintSolverData,
        delta_secs: Scalar,
        use_bias: bool,
    ) {
        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_global_angular_inertia().inverse();
        let inv_inertia2 = body2.effective_global_angular_inertia().inverse();

        // TODO: Cache these.
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        let r1 = *body1.rotation * local_r1;
        let r2 = *body2.rotation * local_r2;

        let impulse = PointConstraintPart::compute_incremental_impulse(
            body1,
            body2,
            r1,
            r2,
            solver_data.point_constraint.impulse,
            solver_data.point_constraint.center_difference,
            &solver_data.effective_mass,
            &solver_data.coefficients,
            use_bias,
        );

        solver_data.point_constraint.impulse += impulse;

        if body1.rb.is_dynamic() {
            body1.linear_velocity.0 -= impulse * inv_mass1;
            body1.angular_velocity.0 -= inv_inertia1 * cross(r1, impulse);
        }
        if body2.rb.is_dynamic() {
            body2.linear_velocity.0 += impulse * inv_mass2;
            body2.angular_velocity.0 += inv_inertia2 * cross(r2, impulse);
        }
    }
}

impl PointConstraint {
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            stiffness: SoftnessParameters::new(1.0, 0.125 / (1.0 / 60.0)),
        }
    }

    pub fn with_local_anchor_1(self, anchor: Vector) -> Self {
        Self {
            local_anchor1: anchor,
            ..self
        }
    }

    pub fn with_local_anchor_2(self, anchor: Vector) -> Self {
        Self {
            local_anchor2: anchor,
            ..self
        }
    }

    pub fn with_softness(self, stiffness: SoftnessParameters) -> Self {
        Self { stiffness, ..self }
    }
}

impl MapEntities for PointConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
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
