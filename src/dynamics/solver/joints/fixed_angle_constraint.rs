//! [`FixedAngleConstraint`] component.

use crate::{
    dynamics::solver::solver_body::{SolverBody, SolverBodyInertia},
    prelude::*,
};
use bevy::prelude::*;
use dynamics::solver::softness_parameters::{SoftnessCoefficients, SoftnessParameters};

/// An angle constraint that prevents relative rotation of the attached bodies.
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
#[require(FixedAngleConstraintSolverData)]
pub struct FixedAngleConstraint {
    /// Soft constraint parameters for tuning the stiffness and damping of the joint.
    pub stiffness: SoftnessParameters,

    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
}

impl Default for FixedAngleConstraint {
    fn default() -> Self {
        Self::new()
    }
}

/// Cached data required by the impulse-based solver for [`FixedAngleConstraint`].
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct FixedAngleConstraintSolverData {
    /// The relative rotation between the two bodies.
    #[cfg(feature = "2d")]
    pub rotation_difference: Scalar,

    /// The relative rotation between the two bodies.
    #[cfg(feature = "3d")]
    pub rotation_difference: Quaternion,

    /// The effective mass of the angle constraint.
    #[cfg(feature = "2d")]
    pub effective_mass: f32,

    /// The effective mass of the angle constraint.
    #[cfg(feature = "3d")]
    pub effective_mass: SymmetricMatrix3,

    /// Coefficients computed for the spring parameters of the constraint.
    pub coefficients: SoftnessCoefficients,

    /// The angular impulse applied to the first body.
    #[cfg(feature = "2d")]
    pub angular_impulse: Scalar,

    /// The angular impulse applied to the first body.
    #[cfg(feature = "3d")]
    pub angular_impulse: Vector,
}

impl ImpulseJoint for FixedAngleConstraint {
    type SolverData = FixedAngleConstraintSolverData;

    fn prepare(
        &mut self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        solver_data: &mut FixedAngleConstraintSolverData,
        delta_secs: Scalar,
    ) {
        // TODO: Support a rotation offset.
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

        // Update the effective mass of the angle limit constraint.
        let inverse_effective_mass = i1 + i2;

        #[cfg(feature = "2d")]
        {
            solver_data.effective_mass = inverse_effective_mass.recip_or_zero();
        }
        #[cfg(feature = "3d")]
        {
            solver_data.effective_mass = inverse_effective_mass.inverse();
        }

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
        solver_data: &FixedAngleConstraintSolverData,
    ) {
        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();
        body1.angular_velocity -= inv_inertia1 * solver_data.angular_impulse;
        body2.angular_velocity += inv_inertia2 * solver_data.angular_impulse;
    }

    fn solve(
        &self,
        body1: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        body2: &mut SolverBody,
        inertia2: &SolverBodyInertia,
        solver_data: &mut FixedAngleConstraintSolverData,
        _delta_secs: Scalar,
        use_bias: bool,
    ) {
        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        // Angle constraint. Satisfied when C = 0.

        #[cfg(feature = "2d")]
        let mut bias = 0.0;
        #[cfg(feature = "3d")]
        let mut bias = Vector::ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;

        // TODO: Also use bias if angular_hz > 0.0
        if use_bias {
            #[cfg(feature = "2d")]
            let c = body2.delta_rotation.angle_between(body1.delta_rotation)
                + solver_data.rotation_difference;

            #[cfg(feature = "3d")]
            let c: Vector = (body2.delta_rotation.0.inverse()
                * body1.delta_rotation.0
                * solver_data.rotation_difference)
                .to_euler(EulerRot::XYZ)
                .into();

            bias = solver_data.coefficients.bias * c;
            mass_scale = solver_data.coefficients.mass_scale;
            impulse_scale = solver_data.coefficients.impulse_scale;
        }

        // Angular velocity constraint. Satisfied when C' = 0.
        let c_vel = body2.angular_velocity - body1.angular_velocity;

        let impulse = -solver_data.effective_mass * mass_scale * (c_vel + bias)
            - impulse_scale * solver_data.angular_impulse;
        solver_data.angular_impulse += impulse;

        body1.angular_velocity -= inv_inertia1 * impulse;
        body2.angular_velocity += inv_inertia2 * impulse;
    }

    fn relative_dominance(&self) -> i16 {
        self.relative_dominance
    }
}

impl FixedAngleConstraint {
    /// Creates a new hinge joint between two entities.
    pub fn new() -> Self {
        Self {
            stiffness: SoftnessParameters::new(1.0, 30.0),
            relative_dominance: 0,
        }
    }

    /// Sets the softness parameters for the joint.
    pub fn with_softness(mut self, stiffness: SoftnessParameters) -> Self {
        self.stiffness = stiffness;
        self
    }
}
