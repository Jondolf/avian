use crate::{
    dynamics::solver::solver_body::{SolverBody, SolverBodyInertia},
    prelude::*,
};
use bevy::prelude::*;
use dynamics::solver::softness_parameters::{SoftnessCoefficients, SoftnessParameters};

/// A constraint that restricts axes of two bodies to stay within a maximum swing angle.
///
/// This can be used to apply angle limits to a hinge joint, or to restrict swing
/// for a spherical joint, for example.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct SwingLimit {
    /// The swing axis attached to the first body in its local space.
    pub local_axis1: Vector,

    /// The swing axis attached to the second body in its local space.
    pub local_axis2: Vector,

    /// The maximum allowed swing angle between the two world-space axes
    /// that the constraint tries to maintain.
    pub max_angle: Scalar,

    /// Spring parameters for tuning the stiffness and damping of the constraint.
    pub stiffness: SoftnessParameters,

    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
}

/// Cached data required by the impulse-based solver for [`SwingLimit`].
#[derive(Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct SwingLimitSolverData {
    /// The axis attached to the first body in world space.
    pub axis1: Vector,

    /// The axis attached to the second body in world space.
    pub axis2: Vector,

    /// The minimum dot product between the two axes that the constraint tries to maintain.
    pub min_dot: Scalar,

    /// The Jacobian for the axis on the first body.
    ///
    /// Note that J1 == -J2.
    pub jacobian1: Vector,

    /// The effective mass of the constraint.
    ///
    /// K^-1 = 1 / (J * M^-1 * J^T).
    pub effective_mass: Scalar,

    // TODO: Profile if these impulse-to-velocity values are worth caching.
    /// Transforms a constraint-space impulse into a velocity change for the first body.
    ///
    /// J1 * I1^-1
    pub impulse_to_velocity1: Vector,

    /// Transforms a constraint-space impulse into a negated velocity change for the second body.
    ///
    /// J1 * I2^-1
    pub neg_impulse_to_velocity2: Vector,

    /// Coefficients computed for the spring parameters of the constraint.
    pub coefficients: SoftnessCoefficients,

    /// The accumulated impulse applied by the constraint, expressed in constraint space.
    ///
    /// Multiply this by the Jacobian to get the impulse in world space.
    pub impulse: Scalar,
}

// Derivation of the rotation constraint, velocity constraint, and Jacobian:
//
// The swing limit aims to keep an axis on body 1 within a certain angle of an axis on body 2.
// This is implemented as a speculative inequality constraint.
//
// The rotation constraint is:
//
// C = dot(axis1, axis2) >= cos(max_angle) = min_dot
//
// The velocity constraint is:
//
// C' = dot(d/dt(axis1), axis2) + dot(axis1, d/dt(axis2)) >= 0
// C' = dot(ω1 x axis1, axis2) + dot(axis1, ω2 x axis2) >= 0
// C' = dot(axis1 x axis2, ω1) + dot(ω2, axis2 x axis1) >= 0
//
// By inspection, the Jacobians for axes 1 and 2 are:
//
// J1 = axis1 x axis2
// J2 = axis2 x axis1
//
// Because J1 == -J2, it is enough to compute J1.
//
// References:
//
// - Bepu Physics v2: SwingLimit.cs

impl ImpulseJoint for SwingLimit {
    type SolverData = SwingLimitSolverData;

    fn prepare(
        &mut self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut SwingLimitSolverData,
        delta_secs: Scalar,
    ) {
        let i1 = body1.effective_global_angular_inertia().inverse();
        let i2 = body2.effective_global_angular_inertia().inverse();

        // Update the Jacobian for the axis attached to the first body.
        // Note that J1 == -J2, so we only need to compute J1.
        solver_data.axis1 = body1.rotation * self.local_axis1;
        solver_data.axis2 = body2.rotation * self.local_axis2;
        solver_data.jacobian1 = SwingLimit::jacobian1(solver_data.axis1, solver_data.axis2);

        // Update the effective mass and intermediary impulse-to-velocity values.
        self.update_effective_mass(i1, i2, solver_data);

        // Update the minimum dot product to maintain between the two axes.
        #[cfg(not(feature = "libm"))]
        {
            solver_data.min_dot = self.max_angle.cos();
        }
        #[cfg(feature = "libm")]
        {
            solver_data.min_dot = libm::cos(self.max_angle as f64) as Scalar;
        }

        solver_data.coefficients = self.stiffness.compute_coefficients(delta_secs);
    }

    fn warm_start(
        &self,
        body1: &mut SolverBody,
        _inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        _inertia2: &SolverBodyInertia,
        solver_data: &SwingLimitSolverData,
    ) {
        body1.angular_velocity += solver_data.impulse_to_velocity1 * solver_data.impulse;
        body2.angular_velocity -= solver_data.neg_impulse_to_velocity2 * solver_data.impulse;
    }

    fn solve(
        &self,
        body1: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        inertia2: &SolverBodyInertia,
        solver_data: &mut SwingLimitSolverData,
        delta_secs: Scalar,
        use_bias: bool,
    ) {
        let i1 = inertia1.effective_inv_angular_inertia();
        let i2 = inertia2.effective_inv_angular_inertia();

        let mut axis1 = solver_data.axis1;
        let mut axis2 = solver_data.axis2;

        // Solving without bias is done after position integration, which can change rotation.
        // Recompute the Jacobian, effective mass, and impulse-to-velocity values with the new axes.
        if !use_bias {
            axis1 = body1.delta_rotation * axis1;
            axis2 = body2.delta_rotation * axis2;
            solver_data.jacobian1 = SwingLimit::jacobian1(axis1, axis2);

            self.update_effective_mass(i1, i2, solver_data);
        }

        let effective_mass = &solver_data.effective_mass;

        // Compute the rotation error.
        //
        // C = dot(axis1, axis2) - min_dot
        let axis_dot = axis1.dot(axis2);
        let rotation_error = axis_dot - solver_data.min_dot;

        // Compute the velocity error.
        //
        // C' = dot(axis1 x axis2, ω1) + dot(ω2, axis2 x axis1)
        //    = dot(J1, ω1) + dot(ω2, -J1)
        //    = dot(ω1 - ω2, J1)
        let velocity_error =
            (body1.angular_velocity - body2.angular_velocity).dot(solver_data.jacobian1);

        // Compute the incremental impulse in constraint space.
        let mut csi = if rotation_error > 0.0 {
            // Speculative limit: Push back the part of the velocity
            // that would cause the limit to be exceeded.
            -effective_mass * (velocity_error + rotation_error / delta_secs)
        } else if use_bias {
            // Limit using bias: Incorporate softness parameters.
            let bias = solver_data.coefficients.bias * rotation_error;
            let scaled_mass = solver_data.coefficients.mass_scale * effective_mass;
            let scaled_impulse = solver_data.coefficients.impulse_scale * solver_data.impulse;
            -scaled_mass * (velocity_error + bias) - scaled_impulse
        } else {
            // Limit without bias: Solve normally without softness parameters.
            -effective_mass * velocity_error
        };

        // Clamp the accumulated impulse.
        let new_impulse = (solver_data.impulse + csi).max(0.0);
        csi = new_impulse - solver_data.impulse;
        solver_data.impulse = new_impulse;

        // Apply the clamped incremental impulse.
        body1.angular_velocity += solver_data.impulse_to_velocity1 * csi;
        body2.angular_velocity -= solver_data.neg_impulse_to_velocity2 * csi;
    }

    fn relative_dominance(&self) -> i16 {
        self.relative_dominance
    }
}

impl SwingLimit {
    /// Computes the Jacobian of the swing limit constraint for an axis
    /// attached to the first body.
    ///
    /// `axis1` and `axis2` are world-space axes attached to the first
    /// and second body, respectively.
    ///
    /// Note that J1 == -J2.
    #[inline]
    pub fn jacobian1(axis1: Vector, axis2: Vector) -> Vector {
        // J1 = axis1 x axis2
        let jacobian = axis1.cross(axis2);

        // If the axes are parallel, there is no unique solution, so we pick one arbitrarily.
        // Note that this causes a discontinuity in the length of the Jacobian at the poles,
        // but we'll just ignore that :)
        let fallback_jacobian = axis1.any_orthonormal_vector();
        let length_squared = jacobian.length_squared();
        if length_squared < 1e-7 {
            fallback_jacobian
        } else {
            jacobian
        }
    }

    /// Updates the effective mass and intermediary impulse-to-velocity values
    /// cached in the given solver data.
    pub fn update_effective_mass(
        &self,
        inverse_angular_inertia1: SymmetricMatrix3,
        inverse_angular_inertia2: SymmetricMatrix3,
        data: &mut SwingLimitSolverData,
    ) {
        // Effective inverse mass:
        // K = J * M^-1 * J^T
        //
        // Note that J1 == -J2 in this case.

        // Cache intermediate result J * M^-1.
        data.impulse_to_velocity1 = inverse_angular_inertia1 * data.jacobian1;
        data.neg_impulse_to_velocity2 = inverse_angular_inertia2 * data.jacobian1;

        // We can use the dot product because a^T * b = dot(a, b) if a and b are column vectors.
        let angular_contribution1 = data.impulse_to_velocity1.dot(data.jacobian1);
        let neg_angular_contribution2 = data.neg_impulse_to_velocity2.dot(data.jacobian1);

        // Return the effective mass K^-1.
        data.effective_mass = (angular_contribution1 + neg_angular_contribution2).recip_or_zero()
    }
}

impl SwingLimit {
    /// Creates a new angular hinge constraint between two entities.
    pub fn new(max_angle: Scalar) -> Self {
        Self {
            local_axis1: Vector3::X,
            local_axis2: Vector3::X,
            max_angle,
            // TODO: Tune these values
            stiffness: SoftnessParameters::new(1.0, 10.0),
            relative_dominance: 0,
        }
    }

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
