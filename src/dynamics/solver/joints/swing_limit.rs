use crate::prelude::*;
use bevy::{
    color::palettes::{css::PINK, tailwind::CYAN_400},
    ecs::{
        component::{ComponentHooks, StorageType},
        entity::{EntityMapper, MapEntities},
    },
    prelude::*,
};
use dynamics::solver::softness_parameters::{SoftnessCoefficients, SoftnessParameters};

/// The angular part of a hinge joint.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct SwingLimit {
    /// The hinge axis in the local space of the first entity.
    pub local_axis1: Vector,

    /// The hinge axis in the local space of the second entity.
    pub local_axis2: Vector,

    /// The extents of the allowed relative rotation of the bodies around the `aligned_axis`.
    pub max_angle: Scalar,

    /// Soft constraint parameters for tuning the stiffness and damping of the joint.
    pub stiffness: SoftnessParameters,
}

impl Component for SwingLimit {
    const STORAGE_TYPE: StorageType = StorageType::Table;

    fn register_component_hooks(hooks: &mut ComponentHooks) {
        hooks.on_add(|mut world, entity, _| {
            world
                .commands()
                .entity(entity)
                .insert(SwingLimitSolverData::default());
        });
    }
}

/// Cached data required by the impulse-based solver for [`SwingLimit`].
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct SwingLimitSolverData {
    /// The minimum dot product between the hinge axis and the relative rotation of the bodies
    /// that the constraint tries to maintain.
    pub min_dot: Scalar,
    pub coefficients: SoftnessCoefficients,
    pub rotation_difference: Quaternion,
    pub effective_mass: Scalar,
    pub jacobian: Vector,
    pub impulse: Scalar,
}

impl SwingLimit {
    pub fn prepare(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut SwingLimitSolverData,
        delta_secs: Scalar,
    ) {
        #[cfg(feature = "3d")]
        {
            solver_data.rotation_difference = body1.rotation.0.inverse() * body2.rotation.0;
        }

        let i1 = body1.effective_world_inv_inertia();
        let i2 = body2.effective_world_inv_inertia();

        solver_data.jacobian = SwingLimit::jacobian(
            body1.rotation,
            body2.rotation,
            self.local_axis1,
            self.local_axis2,
        );

        let impulse_to_velocity1 = i1 * solver_data.jacobian;
        let neg_impulse_to_velocity2 = i2 * solver_data.jacobian;

        let angular_contribution1 = impulse_to_velocity1.dot(solver_data.jacobian);
        let angular_contribution2 = neg_impulse_to_velocity2.dot(solver_data.jacobian);

        // TODO: Apply mass scale here already?
        solver_data.effective_mass = 1.0 / (angular_contribution1 + angular_contribution2);

        // TODO: Cross-platform determinism
        solver_data.min_dot = self.max_angle.cos();

        solver_data.coefficients = self.stiffness.compute_coefficients(delta_secs);
    }

    pub fn warm_start(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &SwingLimitSolverData,
    ) {
        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        let impulse_to_velocity1 = inv_inertia1 * solver_data.jacobian;
        let neg_impulse_to_velocity2 = inv_inertia2 * solver_data.jacobian;

        if body1.rb.is_dynamic() {
            body1.angular_velocity.0 += impulse_to_velocity1 * solver_data.impulse;
        }
        if body2.rb.is_dynamic() {
            body2.angular_velocity.0 -= neg_impulse_to_velocity2 * solver_data.impulse;
        }
    }

    pub fn solve(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &mut SwingLimitSolverData,
        delta_secs: Scalar,
        use_bias: bool,
    ) {
        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        let effective_mass = &solver_data.effective_mass;

        // 1. Compute biased velocity errors
        let hinge_velocity_error =
            (body1.angular_velocity.0 - body2.angular_velocity.0).dot(solver_data.jacobian);

        let mut hinge_bias = 0.0;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;

        let axis1 = *body1.rotation * self.local_axis1;
        let axis2 = *body2.rotation * self.local_axis2;
        let axis_dot = axis1.dot(axis2);
        let error = axis_dot - solver_data.min_dot;

        if error > 0.0 {
            hinge_bias = -error / delta_secs;
        } else if use_bias {
            // Negation: We want to oppose the error.
            hinge_bias = -error * solver_data.coefficients.bias;

            mass_scale = solver_data.coefficients.mass_scale;
            impulse_scale = solver_data.coefficients.impulse_scale;
        }

        let mut csi = mass_scale * effective_mass * (hinge_bias - hinge_velocity_error)
            - impulse_scale * solver_data.impulse;

        let new_impulse = (solver_data.impulse + csi).max(0.0);
        csi = new_impulse - solver_data.impulse;
        solver_data.impulse = new_impulse;

        let impulse_to_velocity1 = inv_inertia1 * solver_data.jacobian;
        let neg_impulse_to_velocity2 = inv_inertia2 * solver_data.jacobian;

        if body1.rb.is_dynamic() {
            body1.angular_velocity.0 += impulse_to_velocity1 * csi;
        }
        if body2.rb.is_dynamic() {
            body2.angular_velocity.0 -= neg_impulse_to_velocity2 * csi;
        }
    }
}

impl SwingLimit {
    /// Creates a new angular hinge constraint between two entities.
    pub fn new(max_angle: Scalar) -> Self {
        Self {
            local_axis1: Vector3::X,
            local_axis2: Vector3::X,
            max_angle,
            stiffness: SoftnessParameters::new(1.0, 60.0),
        }
    }
}

impl SwingLimit {
    /// Sets the axis that the bodies should be aligned on.
    pub fn with_aligned_axis(self, axis: Vector) -> Self {
        Self {
            local_axis1: axis,
            local_axis2: axis,
            ..self
        }
    }

    /// Sets the softness parameters for the constraint.
    pub fn with_softness(self, stiffness: SoftnessParameters) -> Self {
        Self { stiffness, ..self }
    }
}

#[cfg(feature = "3d")]
impl ConstraintDebugRender for SwingLimit {
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
            CYAN_400,
        );
        gizmos.arrow(
            body2.current_position(),
            body2.current_position() + hinge_axis2,
            PINK,
        );
        gizmos.sphere(body1.current_position(), default(), 0.05, color);
        gizmos.sphere(body2.current_position(), default(), 0.05, color);
    }
}

impl SwingLimit {
    /// Computes the Jacobian matrix for the angular hinge constraint.
    #[inline]
    pub fn jacobian(
        rotation1: &Rotation,
        rotation2: &Rotation,
        local_axis1: Vector,
        local_axis2: Vector,
    ) -> Vector {
        let axis1 = rotation1 * local_axis1;
        let axis2 = rotation2 * local_axis2;
        let jacobian = axis1.cross(axis2);

        // If the axes are parallel, there is no unique solution, so we pick one arbitrarily.
        // Note that this causes a discontinuity in the length of the Jacobian at the poles, but it's fine :)
        let fallback_jacobian = axis1.any_orthonormal_vector();
        let length_squared = jacobian.length_squared();
        if length_squared < 1e-7 {
            fallback_jacobian
        } else {
            jacobian
        }
    }

    /// Computes the velocity error for the angular hinge constraint.
    pub fn velocity_error(ang_vel1: Vector, ang_vel2: Vector, jacobian: Matrix2x3) -> Vector2 {
        (ang_vel1 - ang_vel2) * jacobian
    }

    /// Computes the effective inverse mass for the angular hinge constraint.
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
