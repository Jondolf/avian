use crate::prelude::*;
use bevy::prelude::*;
use dynamics::solver::softness_parameters::SoftnessCoefficients;

#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct PointConstraintPart {
    pub center_difference: Vector,
    pub r1: Vector,
    pub r2: Vector,
    pub impulse: Vector,
}

#[cfg(feature = "2d")]
type EffectiveMass = Matrix2;
#[cfg(feature = "3d")]
type EffectiveMass = SymmetricMatrix3;

impl PointConstraintPart {
    pub fn compute_incremental_impulse(
        &self,
        body1: &RigidBodyQueryItem,
        body2: &RigidBodyQueryItem,
        effective_mass: &EffectiveMass,
        softness: &SoftnessCoefficients,
        use_bias: bool,
    ) -> Vector {
        let mut bias = Vector::ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;

        if use_bias {
            let separation =
                Self::position_error(body1, body2, self.r1, self.r2, self.center_difference);
            bias = softness.bias * separation;
            mass_scale = softness.mass_scale;
            impulse_scale = softness.impulse_scale;
        }

        let velocity_error = Self::velocity_error(body1, body2, self.r1, self.r2);

        // Compute the impulse.
        -effective_mass.mul_scalar(mass_scale) * (velocity_error + bias)
            - impulse_scale * self.impulse
    }

    pub fn position_error(
        body1: &RigidBodyQueryItem,
        body2: &RigidBodyQueryItem,
        r1: Vector,
        r2: Vector,
        center_difference: Vector,
    ) -> Vector {
        let delta_separation =
            (body2.accumulated_translation.0 - body1.accumulated_translation.0) + (r2 - r1);
        delta_separation + center_difference
    }

    pub fn velocity_error(
        body1: &RigidBodyQueryItem,
        body2: &RigidBodyQueryItem,
        r1: Vector,
        r2: Vector,
    ) -> Vector {
        let velocity1 = body1.velocity_at_point(r1);
        let velocity2 = body2.velocity_at_point(r2);
        velocity2 - velocity1
    }

    /// Computes the inverse effective mass matrix for the constraint.
    #[cfg(feature = "2d")]
    #[inline]
    pub fn effective_inverse_mass(
        &self,
        inverse_mass_sum: Scalar,
        inverse_angular_inertia1: &Scalar,
        inverse_angular_inertia2: &Scalar,
    ) -> Matrix2 {
        let k00 = inverse_mass_sum
            + self.r1.y.powi(2) * inverse_angular_inertia1
            + self.r2.y.powi(2) * inverse_angular_inertia2;
        let k10 = -self.r1.y * self.r1.x * inverse_angular_inertia1
            - self.r2.y * self.r2.x * inverse_angular_inertia2;
        let k01 = k10;
        let k11 = inverse_mass_sum
            + self.r1.x.powi(2) * inverse_angular_inertia1
            + self.r2.x.powi(2) * inverse_angular_inertia2;
        Matrix2::from_cols_array(&[k00, k10, k01, k11])
    }

    #[cfg(feature = "3d")]
    #[inline]
    pub fn effective_inverse_mass(
        &self,
        inverse_mass_sum: Scalar,
        inverse_angular_inertia1: &SymmetricMatrix3,
        inverse_angular_inertia2: &SymmetricMatrix3,
    ) -> SymmetricMatrix3 {
        let mut effective_inverse_mass;
        let angular_contribution1 = inverse_angular_inertia1.skew(self.r1);
        let angular_contribution2 = inverse_angular_inertia2.skew(self.r2);
        effective_inverse_mass = angular_contribution1 + angular_contribution2;
        effective_inverse_mass.m00 += inverse_mass_sum;
        effective_inverse_mass.m11 += inverse_mass_sum;
        effective_inverse_mass.m22 += inverse_mass_sum;
        effective_inverse_mass
    }
}
