//! [`PrismaticJoint`] component.

use crate::{
    dynamics::solver::{
        softness_parameters::SoftnessCoefficients,
        solver_body::{SolverBody, SolverBodyInertia},
    },
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

// TODO: It's possible to solve the position and rotation constraint simultaneously, see Bepu.
/// A weld joint prevents all relative movement of the attached bodies.
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
#[require(WeldJointSolverData)]
pub struct PrismaticJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,

    /// Second entity constrained by the joint.
    pub entity2: Entity,

    /// Attachment point on the first body.
    pub local_anchor1: Vector,

    /// Attachment point on the second body.
    pub local_anchor2: Vector,

    /// The local axis of the joint.
    pub local_axis1: Vector,

    /// The distance limit of the joint.
    pub limit: Option<DistanceLimit>,

    /// Soft constraint parameters for tuning the stiffness and damping of the joint.
    pub stiffness: SoftnessParameters,

    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
}

/// Cached data required by the impulse-based solver for [`PrismaticJoint`].
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct WeldJointSolverData {
    pub r1: Vector,
    pub r2: Vector,
    pub axis1: Vector,
    pub center_difference: Vector,
    pub rotation_difference: Scalar,
    pub effective_mass: Scalar,
    pub impulse: Vector,
    pub lower_impulse: Scalar,
    pub upper_impulse: Scalar,
    pub coefficients: SoftnessCoefficients,
}

impl EntityConstraint<2> for PrismaticJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl ImpulseJoint for PrismaticJoint {
    type SolverData = WeldJointSolverData;

    fn prepare(
        &mut self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut WeldJointSolverData,
        delta_secs: Scalar,
    ) {
        // Update the world-space anchor points and axis.
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        solver_data.r1 = *body1.rotation * local_r1;
        solver_data.r2 = *body2.rotation * local_r2;
        solver_data.axis1 = *body1.rotation * self.local_axis1;

        // Update the center difference and rotation difference.
        // TODO: This should use the global center of mass.
        solver_data.center_difference = body2.position.0 - body1.position.0;
        solver_data.rotation_difference = body1.rotation.angle_between(*body2.rotation);

        // Compute the effective mass.
        let inverse_mass_sum = body1.mass.inverse() + body2.mass.inverse();
        let i1 = body1.effective_global_angular_inertia().inverse();
        let i2 = body2.effective_global_angular_inertia().inverse();

        let d = solver_data.center_difference + solver_data.r2 - solver_data.r1;
        let a1 = cross(d + solver_data.r1, solver_data.axis1);
        let a2 = cross(solver_data.r2, solver_data.axis1);

        let k = inverse_mass_sum + i1 * a1 * a1 + i2 * a2 * a2;
        solver_data.effective_mass = k.recip_or_zero();

        // Compute the softness coefficients.
        solver_data.coefficients = self.stiffness.compute_coefficients(delta_secs);
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

        let r1 = body1.delta_rotation * solver_data.r1;
        let r2 = body2.delta_rotation * solver_data.r2;
        let d =
            body2.delta_position - body1.delta_position + solver_data.center_difference + r2 - r1;
        let axis1 = body1.delta_rotation * solver_data.axis1;

        // Axial constraint
        let a1 = cross(d + r1, axis1);
        let a2 = cross(r2, axis1);
        let axial_impulse = solver_data.lower_impulse - solver_data.upper_impulse;

        // Perpendicular constraint
        let perp1 = axis1.perp();
        let s1 = cross(d + r1, perp1);
        let s2 = cross(r2, perp1);
        let perp_impulse = solver_data.impulse.x;
        let angle_impulse = solver_data.impulse.y;

        let p = axial_impulse * axis1 + perp_impulse * perp1;
        let l1 = axial_impulse * a1 + perp_impulse * s1 + angle_impulse;
        let l2 = axial_impulse * a2 + perp_impulse * s2 + angle_impulse;

        body1.linear_velocity -= inv_mass1 * p;
        body1.angular_velocity -= inv_inertia1 * l1;

        body2.linear_velocity += inv_mass2 * p;
        body2.angular_velocity += inv_inertia2 * l2;
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
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();
        // TODO: Handle locked axes properly.
        let inv_mass_sum = (inv_mass1 + inv_mass2).max_element();

        // Update the world-space anchor points and axis.
        let r1 = body1.delta_rotation * solver_data.r1;
        let r2 = body2.delta_rotation * solver_data.r2;
        let d =
            body2.delta_position - body1.delta_position + solver_data.center_difference + r2 - r1;
        let axis1 = body1.delta_rotation * solver_data.axis1;
        let translation = d.dot(axis1);

        // Axial constraint
        let a1 = cross(d + r1, axis1);
        let a2 = cross(r2, axis1);

        if let Some(limit) = self.limit {
            // Lower limit
            let c = translation - limit.min;

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

            let c_vel = axis1.dot(body2.linear_velocity - body1.linear_velocity)
                + a2 * body2.angular_velocity
                - a1 * body1.angular_velocity;

            let mut impulse = -solver_data.effective_mass * mass_scale * (c_vel + bias)
                - impulse_scale * solver_data.lower_impulse;
            let old_impulse = solver_data.lower_impulse;
            solver_data.lower_impulse = (solver_data.lower_impulse + impulse).max(0.0);
            impulse = solver_data.lower_impulse - old_impulse;

            let p = impulse * axis1;
            let l1 = impulse * a1;
            let l2 = impulse * a2;

            body1.linear_velocity -= inv_mass1 * p;
            body1.angular_velocity -= inv_inertia1 * l1;
            body2.linear_velocity += inv_mass2 * p;
            body2.angular_velocity += inv_inertia2 * l2;

            // Upper limit
            // Note: Signs are flipped to keep C positive when the constraint is satisfied.
            let c = limit.max - translation;
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

            // Sign flipped
            let c_vel = axis1.dot(body1.linear_velocity - body2.linear_velocity)
                + a1 * body1.angular_velocity
                - a2 * body2.angular_velocity;

            let mut impulse = -solver_data.effective_mass * mass_scale * (c_vel + bias)
                - impulse_scale * solver_data.upper_impulse;
            let old_impulse = solver_data.upper_impulse;
            solver_data.upper_impulse = (solver_data.upper_impulse + impulse).max(0.0);
            impulse = solver_data.upper_impulse - old_impulse;

            let p = impulse * axis1;
            let l1 = impulse * a1;
            let l2 = impulse * a2;

            // Sign flipped
            body1.linear_velocity += inv_mass1 * p;
            body1.angular_velocity += inv_inertia1 * l1;
            body2.linear_velocity -= inv_mass2 * p;
            body2.angular_velocity -= inv_inertia2 * l2;
        }

        // Solve the prismatic constraint in block form.
        let perp1 = axis1.perp();

        // Perpendicular constraint
        let s1 = cross(d + r1, perp1);
        let s2 = cross(r2, perp1);

        let c_dot = Vector::new(
            perp1.dot(body2.linear_velocity - body1.linear_velocity) + s2 * body2.angular_velocity
                - s1 * body1.angular_velocity,
            body2.angular_velocity - body1.angular_velocity,
        );

        let mut bias = Vector::ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;

        if use_bias {
            let c = Vector::new(
                d.dot(perp1),
                body1.delta_rotation.angle_between(body2.delta_rotation)
                    + solver_data.rotation_difference,
            );
            bias = solver_data.coefficients.bias * c;
            mass_scale = solver_data.coefficients.mass_scale;
            impulse_scale = solver_data.coefficients.impulse_scale;
        }

        let k11 = inv_mass_sum + inv_inertia1 * s1 * s1 + inv_inertia2 * s2 * s2;
        let k12 = inv_inertia1 * s1 + inv_inertia2 * s2;
        let mut k22 = inv_inertia1 + inv_inertia2;

        if k22 == 0.0 {
            // For bodies with fixed rotation.
            k22 = 1.0;
        }

        let k = Matrix2::from_cols_array(&[k11, k12, k12, k22]);
        let b = k.inverse() * (c_dot + bias);

        let impulse = -mass_scale * b - impulse_scale * solver_data.impulse;

        solver_data.impulse += impulse;

        let p = impulse.x * perp1;
        let l1 = impulse.x * s1 + impulse.y;
        let l2 = impulse.x * s2 + impulse.y;

        body1.linear_velocity -= inv_mass1 * p;
        body1.angular_velocity -= inv_inertia1 * l1;
        body2.linear_velocity += inv_mass2 * p;
        body2.angular_velocity += inv_inertia2 * l2;
    }

    fn relative_dominance(&self) -> i16 {
        self.relative_dominance
    }
}

impl PrismaticJoint {
    /// Creates a new weld joint between two entities.
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            local_axis1: Vector::X,
            limit: None,
            stiffness: SoftnessParameters::new(1.0, 30.0),
            relative_dominance: 0,
        }
    }

    /// Sets the anchor point of the joint in the local space of the first entity.
    pub fn with_local_anchor_1(mut self, anchor: Vector) -> Self {
        self.local_anchor1 = anchor;
        self
    }

    /// Sets the anchor point of the joint in the local space of the second entity.
    pub fn with_local_anchor_2(mut self, anchor: Vector) -> Self {
        self.local_anchor2 = anchor;
        self
    }

    /// Sets the local axis of the joint.
    pub fn with_local_axis(mut self, axis: Vector) -> Self {
        self.local_axis1 = axis;
        self
    }

    /// Sets the distance limit of the joint.
    pub fn with_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.limit = Some(DistanceLimit::new(min, max));
        self
    }

    /// Sets the softness parameters for the joint.
    pub fn with_softness(mut self, stiffness: SoftnessParameters) -> Self {
        self.stiffness = stiffness;
        self
    }
}

impl MapEntities for PrismaticJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}
