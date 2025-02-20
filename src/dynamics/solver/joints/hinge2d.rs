//! [`HingeJoint`] component.

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

/// A hinge joint prevents relative translation of the attached bodies, but allows rotation.
///
/// Hinges can be useful for things like wheels, fans, revolving doors etc.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
pub struct HingeJoint {
    /// The first entity constrained by the joint.
    pub entity1: Entity,

    /// The second entity constrained by the joint.
    pub entity2: Entity,

    /// The attachment point expressed in the local space of the first body.
    pub local_anchor1: Vector,

    /// The attachment point expressed in the local space of the second body.
    pub local_anchor2: Vector,

    /// The extents of the allowed relative rotation of the bodies.
    pub angle_limit: Option<AngleLimit>,

    /// Soft constraint parameters for tuning the stiffness and damping of the joint.
    pub stiffness: SoftnessParameters,
}

impl Component for HingeJoint {
    const STORAGE_TYPE: StorageType = StorageType::Table;

    fn register_component_hooks(hooks: &mut ComponentHooks) {
        hooks.on_add(|mut world, entity, _| {
            world
                .commands()
                .entity(entity)
                .insert(HingeJointSolverData::default());
        });
    }
}

/// Cached data required by the impulse-based solver for [`HingeJoint`].
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct HingeJointSolverData {
    /// The point-to-point constraint part of the hinge joint.
    pub point_constraint: PointConstraintPart,

    /// The relative rotation between the two bodies.
    pub rotation_difference: Scalar,

    /// The effective mass of the point-to-point constraint.
    pub effective_point_constraint_mass: Matrix2,

    /// The effective mass of the angle limit constraint.
    pub axial_mass: f32,

    /// The accumulated impulse applied by the lower angle limit.
    pub lower_impulse: f32,

    /// The accumulated impulse applied by the upper angle limit.
    pub upper_impulse: f32,

    /// Coefficients computed for the spring parameters of the constraint.
    pub coefficients: SoftnessCoefficients,
}

impl EntityConstraint<2> for HingeJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl ImpulseJoint for HingeJoint {
    type SolverData = HingeJointSolverData;

    fn prepare(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut HingeJointSolverData,
        delta_secs: Scalar,
    ) {
        // Update the world-space anchor points.
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        solver_data.point_constraint.r1 = *body1.rotation * local_r1;
        solver_data.point_constraint.r2 = *body2.rotation * local_r2;

        // TODO: Support a rotation offset.
        // Update the center difference and rotation difference.
        solver_data.point_constraint.center_difference =
            body2.current_position() - body1.current_position();
        solver_data.rotation_difference = body1.rotation.angle_between(*body2.rotation);

        let inverse_mass_sum = body1.mass.inverse() + body2.mass.inverse();
        let i1 = body1.effective_global_angular_inertia().inverse();
        let i2 = body2.effective_global_angular_inertia().inverse();

        // Update the effective mass of the point-to-point constraint.
        solver_data.effective_point_constraint_mass = solver_data
            .point_constraint
            .effective_inverse_mass(inverse_mass_sum, &i1, &i2)
            .inverse();

        // Update the effective mass of the angle limit constraint.
        let inverse_axial_mass = i1 + i2;
        solver_data.axial_mass = inverse_axial_mass;

        if solver_data.axial_mass > 0.0 {
            solver_data.axial_mass = 1.0 / solver_data.axial_mass;
        }

        solver_data.coefficients = self.stiffness.compute_coefficients(delta_secs);
    }

    fn warm_start(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &HingeJointSolverData,
    ) {
        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_global_angular_inertia().inverse();
        let inv_inertia2 = body2.effective_global_angular_inertia().inverse();

        let r1 = solver_data.point_constraint.r1;
        let r2 = solver_data.point_constraint.r2;
        let point_impulse = solver_data.point_constraint.impulse;
        let axial_impulse = solver_data.lower_impulse - solver_data.upper_impulse;

        if body1.rb.is_dynamic() {
            body1.linear_velocity.0 -= point_impulse * inv_mass1;
            body1.angular_velocity.0 -= inv_inertia1 * (cross(r1, point_impulse) + axial_impulse);
        }
        if body2.rb.is_dynamic() {
            body2.linear_velocity.0 += point_impulse * inv_mass2;
            body2.angular_velocity.0 += inv_inertia2 * (cross(r2, point_impulse) + axial_impulse);
        }
    }

    fn solve(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &mut HingeJointSolverData,
        delta_secs: Scalar,
        use_bias: bool,
    ) {
        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_global_angular_inertia().inverse();
        let inv_inertia2 = body2.effective_global_angular_inertia().inverse();

        // Solving without bias is done after position integration, which can change rotation.
        // Recompute the anchor points and effective mass.
        if !use_bias {
            let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
            let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
            solver_data.point_constraint.r1 = *body1.rotation * local_r1;
            solver_data.point_constraint.r2 = *body2.rotation * local_r2;

            let inverse_mass_sum = body1.mass.inverse() + body2.mass.inverse();
            solver_data.effective_point_constraint_mass = solver_data
                .point_constraint
                .effective_inverse_mass(inverse_mass_sum, &inv_inertia1, &inv_inertia2)
                .inverse();
        }

        // Limits
        if let Some(limit) = self.angle_limit {
            let angle = solver_data.rotation_difference;

            // Lower limit

            // Angle constraint. Satisfied when C = 0.
            let c = angle - limit.min;

            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;

            if c > 0.0 {
                // Speculative
                bias = c / delta_secs;
            } else if use_bias {
                bias = solver_data.coefficients.bias * c;
                mass_scale = solver_data.coefficients.mass_scale;
                impulse_scale = solver_data.coefficients.impulse_scale;
            }

            // Angular velocity constraint. Satisfied when C' = 0.
            let c_vel = body2.angular_velocity.0 - body1.angular_velocity.0;

            let mut impulse = -solver_data.axial_mass * mass_scale * (c_vel + bias)
                - impulse_scale * solver_data.lower_impulse;
            let old_impulse = solver_data.lower_impulse;
            solver_data.lower_impulse = (solver_data.lower_impulse + impulse).max(0.0);
            impulse = solver_data.lower_impulse - old_impulse;

            if body1.rb.is_dynamic() {
                body1.angular_velocity.0 -= inv_inertia1 * impulse;
            }
            if body2.rb.is_dynamic() {
                body2.angular_velocity.0 += inv_inertia2 * impulse;
            }

            // Upper limit

            // Angle constraint. Satisfied when C = 0.
            let c = limit.max - angle;

            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;

            if c > 0.0 {
                // Speculative
                bias = c / delta_secs;
            } else if use_bias {
                bias = solver_data.coefficients.bias * c;
                mass_scale = solver_data.coefficients.mass_scale;
                impulse_scale = solver_data.coefficients.impulse_scale;
            }

            // Angular velocity constraint. Satisfied when C' = 0.
            let c_vel = body1.angular_velocity.0 - body2.angular_velocity.0;

            let mut impulse = -solver_data.axial_mass * mass_scale * (c_vel + bias)
                - impulse_scale * solver_data.upper_impulse;
            let old_impulse = solver_data.upper_impulse;
            solver_data.upper_impulse = (solver_data.upper_impulse + impulse).max(0.0);
            impulse = solver_data.upper_impulse - old_impulse;

            if body1.rb.is_dynamic() {
                body1.angular_velocity.0 += inv_inertia1 * impulse;
            }
            if body2.rb.is_dynamic() {
                body2.angular_velocity.0 -= inv_inertia2 * impulse;
            }
        }

        // Solve point-to-point constraint.
        let impulse = solver_data.point_constraint.compute_incremental_impulse(
            body1,
            body2,
            &solver_data.effective_point_constraint_mass,
            &solver_data.coefficients,
            use_bias,
        );

        solver_data.point_constraint.impulse += impulse;

        if body1.rb.is_dynamic() {
            body1.linear_velocity.0 -= impulse * inv_mass1;
            body1.angular_velocity.0 -=
                inv_inertia1 * cross(solver_data.point_constraint.r1, impulse);
        }
        if body2.rb.is_dynamic() {
            body2.linear_velocity.0 += impulse * inv_mass2;
            body2.angular_velocity.0 +=
                inv_inertia2 * cross(solver_data.point_constraint.r2, impulse);
        }
    }
}

impl HingeJoint {
    /// Creates a new hinge joint between two entities.
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            angle_limit: None,
            stiffness: SoftnessParameters::new(1.0, 10.0),
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

    /// Sets the limits of the allowed relative rotation around the `aligned_axis`.
    pub fn with_angle_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            angle_limit: Some(AngleLimit::new(min, max)),
            ..self
        }
    }

    /// Sets the softness parameters for the joint.
    pub fn with_softness(self, stiffness: SoftnessParameters) -> Self {
        Self { stiffness, ..self }
    }
}

impl MapEntities for HingeJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}
