use crate::{
    dynamics::solver::solver_body::{SolverBody, SolverBodyInertia},
    prelude::*,
};
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};
use dynamics::solver::softness_parameters::{SoftnessCoefficients, SoftnessParameters};

#[cfg(feature = "3d")]
use super::swing_limit::SwingLimitSolverData;

/// The angular part of a hinge joint.
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
#[require(AngularHingeSolverData)]
pub struct AngularHinge {
    /// First entity constrained by the joint.
    pub entity1: Entity,

    /// Second entity constrained by the joint.
    pub entity2: Entity,

    /// The hinge axis in the local space of the first entity.
    #[cfg(feature = "3d")]
    pub local_axis1: Vector,

    /// The hinge axis in the local space of the second entity.
    #[cfg(feature = "3d")]
    pub local_axis2: Vector,

    /// The extents of the allowed relative rotation of the bodies around the `aligned_axis`.
    pub angle_limit: Option<AngleLimit>,

    /// The swing limit of the hinge joint.
    #[cfg(feature = "3d")]
    pub swing_limit: SwingLimit,

    /// Soft constraint parameters for tuning the stiffness and damping of the joint.
    pub stiffness: SoftnessParameters,

    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
}

/// Cached data required by the impulse-based solver for [`AngularHinge`].
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct AngularHingeSolverData {
    pub axis1: Vector,
    pub axis2: Vector,
    pub coefficients: SoftnessCoefficients,
    #[cfg(feature = "2d")]
    pub rotation_difference: Scalar,
    #[cfg(feature = "3d")]
    pub rotation_difference: Quaternion,
    #[cfg(feature = "3d")]
    pub swing_limit: SwingLimitSolverData,
    #[cfg(feature = "2d")]
    pub effective_mass: f32,
    #[cfg(feature = "3d")]
    pub effective_mass: Matrix2,
    #[cfg(feature = "3d")]
    pub jacobian: Matrix2x3,
    #[cfg(feature = "3d")]
    pub impulse: Vector2,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
}

impl EntityConstraint<2> for AngularHinge {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl ImpulseJoint for AngularHinge {
    type SolverData = AngularHingeSolverData;

    fn prepare(
        &mut self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut AngularHingeSolverData,
        delta_secs: Scalar,
    ) {
        #[cfg(feature = "2d")]
        {
            solver_data.rotation_difference = body1.rotation.angle_between(*body2.rotation);
        }
        #[cfg(feature = "3d")]
        {
            solver_data.rotation_difference = body1.rotation.0.inverse() * body2.rotation.0;
        }

        let i1 = body1.effective_global_angular_inertia().inverse();
        let i2 = body2.effective_global_angular_inertia().inverse();

        #[cfg(feature = "2d")]
        {
            // Effective mass for angular hinge constraint
            solver_data.effective_mass = i1 + i2;

            if solver_data.effective_mass > 0.0 {
                solver_data.effective_mass = 1.0 / solver_data.effective_mass;
            }
        }

        #[cfg(feature = "3d")]
        {
            solver_data.axis1 = body1.rotation * self.local_axis1;
            solver_data.axis2 = body2.rotation * self.local_axis2;

            let (axis1, mut axis2) = self.local_axis1.any_orthonormal_pair();

            // Our implementation expects this to be flipped.
            axis2 *= -1.0;

            solver_data.jacobian = AngularHinge::jacobian(body1.rotation, axis1, axis2);
            let effective_inverse_mass =
                AngularHinge::effective_inverse_mass(i1, i2, solver_data.jacobian);
            solver_data.effective_mass = effective_inverse_mass.inverse();
        }

        solver_data.coefficients = self.stiffness.compute_coefficients(delta_secs);

        #[cfg(feature = "2d")]
        if solver_data.effective_mass > 0.0 {
            solver_data.effective_mass = 1.0 / solver_data.effective_mass;
        }

        #[cfg(feature = "3d")]
        {
            self.swing_limit
                .prepare(body1, body2, &mut solver_data.swing_limit, delta_secs);
        }

        // Prepare the relative dominance.
        self.relative_dominance = body1.dominance() - body2.dominance();
    }

    fn warm_start(
        &self,
        body1: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        inertia2: &SolverBodyInertia,
        solver_data: &AngularHingeSolverData,
    ) {
        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        #[cfg(feature = "3d")]
        {
            self.swing_limit
                .warm_start(body1, inertia1, body2, inertia2, &solver_data.swing_limit);
        }

        #[cfg(feature = "2d")]
        {
            let axial_impulse = solver_data.lower_impulse - solver_data.upper_impulse;
            body1.angular_velocity -= inv_inertia1 * axial_impulse;
            body2.angular_velocity += inv_inertia2 * axial_impulse;
        }

        #[cfg(feature = "3d")]
        {
            let impulse_to_velocity1 = solver_data.jacobian * inv_inertia1;
            let neg_impulse_to_velocity2 = solver_data.jacobian * inv_inertia2;
            body1.angular_velocity += impulse_to_velocity1 * solver_data.impulse;
            body2.angular_velocity -= neg_impulse_to_velocity2 * solver_data.impulse;
        }
    }

    fn solve(
        &self,
        body1: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        inertia2: &SolverBodyInertia,
        solver_data: &mut AngularHingeSolverData,
        delta_secs: Scalar,
        use_bias: bool,
    ) {
        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        #[cfg(feature = "3d")]
        {
            self.swing_limit.solve(
                body1,
                inertia1,
                body2,
                inertia2,
                &mut solver_data.swing_limit,
                delta_secs,
                use_bias,
            );
        }

        // Limits
        #[cfg(feature = "2d")]
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
            let c_vel = body2.angular_velocity - body1.angular_velocity;

            let mut impulse = -solver_data.effective_mass * mass_scale * (c_vel + bias)
                - impulse_scale * solver_data.lower_impulse;
            let old_impulse = solver_data.lower_impulse;
            solver_data.lower_impulse = (solver_data.lower_impulse + impulse).max(0.0);
            impulse = solver_data.lower_impulse - old_impulse;

            body1.angular_velocity -= inv_inertia1 * impulse;
            body2.angular_velocity += inv_inertia2 * impulse;

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
            let c_vel = body1.angular_velocity - body2.angular_velocity;

            let mut impulse = -solver_data.effective_mass * mass_scale * (c_vel + bias)
                - impulse_scale * solver_data.upper_impulse;
            let old_impulse = solver_data.upper_impulse;
            solver_data.upper_impulse = (solver_data.upper_impulse + impulse).max(0.0);
            impulse = solver_data.upper_impulse - old_impulse;

            body1.angular_velocity += inv_inertia1 * impulse;
            body2.angular_velocity -= inv_inertia2 * impulse;
        }

        #[cfg(feature = "3d")]
        {
            let effective_mass = &solver_data.effective_mass;

            // 1. Compute biased velocity errors
            let hinge_velocity_error = AngularHinge::velocity_error(
                body1.angular_velocity,
                body2.angular_velocity,
                solver_data.jacobian,
            );
            let mut hinge_bias = Vector2::ZERO;

            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;

            if use_bias {
                let axis1 = body1.delta_rotation * solver_data.axis1;
                let axis2 = body2.delta_rotation * solver_data.axis2;
                let error_angles = Self::get_error_angles(axis1, axis2, solver_data.jacobian);
                // Negation: We want to oppose the error.
                hinge_bias = error_angles * -solver_data.coefficients.bias;

                mass_scale = solver_data.coefficients.mass_scale;
                impulse_scale = solver_data.coefficients.impulse_scale;
            }

            hinge_bias = effective_mass.mul_vec2(hinge_bias);
            let mut csi = mass_scale * effective_mass.mul_vec2(hinge_velocity_error);

            csi = hinge_bias - (impulse_scale * solver_data.impulse + csi);

            solver_data.impulse += csi;

            let impulse_to_velocity1 = solver_data.jacobian * inv_inertia1;
            let neg_impulse_to_velocity2 = solver_data.jacobian * inv_inertia2;

            body1.angular_velocity += impulse_to_velocity1 * csi;
            body2.angular_velocity -= neg_impulse_to_velocity2 * csi;
        }
    }

    fn relative_dominance(&self) -> i16 {
        self.relative_dominance
    }
}

impl AngularHinge {
    /// Computes the error angles for the angular hinge constraint.
    #[cfg(feature = "3d")]
    pub fn get_error_angles(axis1: Vector, axis2: Vector, jacobian1: Matrix2x3) -> Vector2 {
        let (jacobian_x, jacobian_y) = (jacobian1.row(0), jacobian1.row(1));

        let axis2_dot_x = axis2.dot(jacobian_x);
        let axis2_dot_y = axis2.dot(jacobian_y);

        let to_remove_x = jacobian_x * axis2_dot_x;
        let to_remove_y = jacobian_y * axis2_dot_y;

        let mut axis2_on_x_plane = axis2 - to_remove_x;
        let mut axis2_on_y_plane = axis2 - to_remove_y;

        let x_length = axis2_on_x_plane.length();
        let y_length = axis2_on_y_plane.length();

        axis2_on_x_plane /= x_length;
        axis2_on_y_plane /= y_length;

        let epsilon = Vector::splat(1e-7);
        let use_fallback_x = Vector::splat(x_length).cmplt(epsilon);
        let use_fallback_y = Vector::splat(y_length).cmplt(epsilon);

        axis2_on_x_plane = Vector::select(use_fallback_x, axis1, axis2_on_x_plane);
        axis2_on_y_plane = Vector::select(use_fallback_y, axis1, axis2_on_y_plane);

        let axis2x_dot_axis1 = axis2_on_x_plane.dot(axis1);
        let axis2y_dot_axis1 = axis2_on_y_plane.dot(axis1);

        let mut error_angles = Vector2::new(
            axis2x_dot_axis1.clamp(-1.0, 1.0).acos(),
            axis2y_dot_axis1.clamp(-1.0, 1.0).acos(),
        );

        let axis2x_dot_jacobian_y = axis2_on_x_plane.dot(jacobian_y);
        let axis2y_dot_jacobian_x = axis2_on_y_plane.dot(jacobian_x);

        // TODO: Eliminate branching
        if axis2x_dot_jacobian_y >= 0.0 {
            error_angles.x = -error_angles.x;
        }
        if axis2y_dot_jacobian_x < 0.0 {
            error_angles.y = -error_angles.y;
        }

        error_angles
    }

    /// Creates a new angular hinge constraint between two entities.
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            #[cfg(feature = "3d")]
            local_axis1: Vector3::Z,
            #[cfg(feature = "3d")]
            local_axis2: Vector3::Z,
            angle_limit: None,
            #[cfg(feature = "3d")]
            swing_limit: SwingLimit::new(0.5),
            stiffness: SoftnessParameters::new(1.0, 0.125 / (1.0 / 60.0)),
            relative_dominance: 0,
        }
    }
}

impl AngularHinge {
    /// Sets the axis that the bodies should be aligned on.
    #[cfg(feature = "3d")]
    pub fn with_aligned_axis(self, axis: Vector) -> Self {
        Self {
            local_axis1: axis,
            local_axis2: axis,
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

    /// Sets the softness parameters for the constraint.
    pub fn with_softness(self, stiffness: SoftnessParameters) -> Self {
        Self { stiffness, ..self }
    }
}

impl MapEntities for AngularHinge {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}

#[cfg(feature = "3d")]
impl ConstraintDebugRender for AngularHinge {
    fn render(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        color: Color,
        gizmos: &mut Gizmos<PhysicsGizmos>,
    ) {
        let hinge_axis1 = *body1.rotation * self.local_axis1;
        let hinge_axis2 = *body2.rotation * self.local_axis2;

        gizmos.arrow(
            body1.current_position(),
            body1.current_position() + hinge_axis1,
            bevy::color::palettes::tailwind::CYAN_400,
        );
        gizmos.arrow(
            body2.current_position(),
            body2.current_position() + hinge_axis2,
            bevy::color::palettes::css::PINK,
        );
        gizmos.sphere(body1.current_position(), 0.05, color);
        gizmos.sphere(body2.current_position(), 0.05, color);
    }
}

impl AngularHinge {
    /// Computes the Jacobian matrix for the angular hinge constraint.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn jacobian(rotation1: &Rotation, local_axis1: Vector, local_axis2: Vector) -> Matrix2x3 {
        Matrix2x3::from_rows(rotation1 * local_axis1, rotation1 * local_axis2)
    }

    /// Computes the velocity error for the angular hinge constraint.
    #[cfg(feature = "3d")]
    pub fn velocity_error(ang_vel1: Vector, ang_vel2: Vector, jacobian: Matrix2x3) -> Vector2 {
        (ang_vel1 - ang_vel2) * jacobian
    }

    /// Computes the effective inverse mass for the angular hinge constraint.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn effective_inverse_mass(
        inverse_angular_inertia1: SymmetricMatrix3,
        inverse_angular_inertia2: SymmetricMatrix3,
        jacobian: Matrix2x3,
    ) -> Matrix2 {
        let hinge_inertia1 = jacobian * inverse_angular_inertia1;
        let hinge_inertia2 = jacobian * inverse_angular_inertia2;
        let hinge_angular_contribution1 = hinge_inertia1.mul_by_transposed(jacobian);
        let neg_hinge_angular_contribution2 = hinge_inertia2.mul_by_transposed(jacobian);
        hinge_angular_contribution1 + neg_hinge_angular_contribution2
    }
}
