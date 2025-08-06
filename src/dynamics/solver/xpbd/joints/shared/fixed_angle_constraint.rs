use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a [`FixedAngleConstraint`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct FixedAngleConstraintShared {
    /// The target rotation difference between the two bodies.
    #[cfg(feature = "2d")]
    pub rotation_difference: Scalar,
    /// The target rotation difference between the two bodies.
    #[cfg(feature = "3d")]
    pub rotation_difference: Quaternion,
    /// The Lagrange multiplier for the constraint.
    pub lagrange: Scalar,
}

impl XpbdConstraintSolverData for FixedAngleConstraintShared {
    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }
}

impl FixedAngleConstraintShared {
    /// Prepares the constraint with the given rotations and reference rotation.
    pub fn prepare(
        &mut self,
        rotation1: &Rotation,
        rotation2: &Rotation,
        #[cfg(feature = "2d")] reference_rotation: Scalar,
        #[cfg(feature = "3d")] reference_rotation: Quaternion,
    ) {
        // Prepare the base rotation difference.
        #[cfg(feature = "2d")]
        {
            self.rotation_difference = rotation1.angle_between(*rotation2) + reference_rotation;
        }
        #[cfg(feature = "3d")]
        {
            self.rotation_difference = rotation1.0 * rotation2.inverse().0 * reference_rotation;
        }
    }

    /// Solves the constraint for the given bodies.
    pub fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        compliance: Scalar,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_inertia2 = inertia2.effective_inv_angular_inertia();

        #[cfg(feature = "2d")]
        let difference =
            self.rotation_difference + body1.delta_rotation.angle_between(body2.delta_rotation);
        #[cfg(feature = "3d")]
        // TODO: The XPBD paper doesn't have this minus sign, but it seems to be needed for stability.
        //       The angular correction code might have a wrong sign elsewhere.
        let difference = -2.0
            * (self.rotation_difference
                * body1.delta_rotation.0
                * body2.delta_rotation.0.inverse())
            .xyz();

        // Align orientation
        let mut lagrange = self.lagrange;
        self.align_orientation(
            body1,
            body2,
            inv_inertia1,
            inv_inertia2,
            difference,
            &mut lagrange,
            compliance,
            dt,
        );
        self.lagrange = lagrange;
    }
}

impl AngularConstraint for FixedAngleConstraintShared {}
