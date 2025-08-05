//! [`RevoluteJoint`] component.

use crate::{
    dynamics::solver::{
        joints::PointConstraintShared,
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct RevoluteJointSolverData {
    pub(super) point_constraint: PointConstraintShared,
    #[cfg(feature = "2d")]
    pub(super) rotation_difference: Scalar,
    #[cfg(feature = "3d")]
    pub(super) axis1: Vector,
    #[cfg(feature = "3d")]
    pub(super) axis2: Vector,
    pub(super) align_lagrange: Scalar,
    pub(super) limit_lagrange: Scalar,
}

impl XpbdConstraintSolverData for RevoluteJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.point_constraint.clear_lagrange_multipliers();
        self.align_lagrange = 0.0;
        self.limit_lagrange = 0.0;
    }
}

impl XpbdConstraint<2> for RevoluteJoint {
    type SolverData = RevoluteJointSolverData;

    fn prepare(
        &self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        solver_data: &mut RevoluteJointSolverData,
    ) {
        let JointAnchor::Local(local_anchor1) = self.anchor1 else {
            return;
        };
        let JointAnchor::Local(local_anchor2) = self.anchor2 else {
            return;
        };

        // Prepare the point-to-point constraint.
        solver_data
            .point_constraint
            .prepare(bodies, local_anchor1, local_anchor2);

        // Prepare the base rotation difference.
        #[cfg(feature = "2d")]
        {
            solver_data.rotation_difference = bodies[0].rotation.angle_between(*bodies[1].rotation);
        }
        #[cfg(feature = "3d")]
        {
            // Prepare the base axes.
            solver_data.axis1 = bodies[0].rotation * self.aligned_axis;
            solver_data.axis2 = bodies[1].rotation * self.aligned_axis;
        }
    }

    fn solve(
        &self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut RevoluteJointSolverData,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        // Get the effective inverse angular inertia of the bodies.
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        #[cfg(feature = "3d")]
        {
            // Constrain the relative rotation of the bodies, only allowing rotation around one free axis
            let a1 = body1.delta_rotation * solver_data.axis1;
            let a2 = body2.delta_rotation * solver_data.axis2;
            let difference = a1.cross(a2);

            let mut lagrange = solver_data.align_lagrange;
            self.align_orientation(
                body1,
                body2,
                inv_angular_inertia1,
                inv_angular_inertia2,
                difference,
                &mut lagrange,
                self.align_compliance,
                dt,
            );
            solver_data.align_lagrange = lagrange;
        }

        // Apply angle limits when rotating around the free axis
        self.apply_angle_limits(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            solver_data,
            dt,
        );

        // Align positions
        solver_data
            .point_constraint
            .solve([body1, body2], inertias, self.point_compliance, dt);
    }
}

impl RevoluteJoint {
    /// Applies angle limits to limit the relative rotation of the bodies around the `aligned_axis`.
    fn apply_angle_limits(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inv_angular_inertia1: SymmetricTensor,
        inv_angular_inertia2: SymmetricTensor,
        solver_data: &mut RevoluteJointSolverData,
        dt: Scalar,
    ) {
        let Some(Some(correction)) = self.angle_limit.map(|angle_limit| {
            #[cfg(feature = "2d")]
            {
                let rotation_difference = solver_data.rotation_difference
                    + body1.delta_rotation.angle_between(body2.delta_rotation);
                angle_limit.compute_correction(rotation_difference, PI)
            }
            #[cfg(feature = "3d")]
            {
                // [n, n1, n2] = [a1, b1, b2], where [a, b, c] are perpendicular unit axes on the bodies.
                let a1 = body1.delta_rotation * self.aligned_axis;
                let b1 = a1.any_orthonormal_vector();
                let b2 = body2.delta_rotation * self.aligned_axis.any_orthonormal_vector();
                angle_limit.compute_correction(a1, b1, b2, PI)
            }
        }) else {
            return;
        };

        let mut lagrange = solver_data.limit_lagrange;
        self.align_orientation(
            body1,
            body2,
            inv_angular_inertia1,
            inv_angular_inertia2,
            correction,
            &mut lagrange,
            self.align_compliance,
            dt,
        );
        solver_data.limit_lagrange = lagrange;
    }
}

impl PositionConstraint for RevoluteJoint {}

impl AngularConstraint for RevoluteJoint {}
