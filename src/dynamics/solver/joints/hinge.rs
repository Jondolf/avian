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

/// A hinge joint prevents relative movement of the attached bodies, except for rotation around one `aligned_axis`.
///
/// Hinges can be useful for things like wheels, fans, revolving doors etc.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, MapEntities, PartialEq)]
pub struct HingeJoint {
    /// First entity constrained by the joint.
    pub entity1: Entity,

    /// Second entity constrained by the joint.
    pub entity2: Entity,

    /// Attachment point on the first body.
    pub local_anchor1: Vector,

    /// Attachment point on the second body.
    pub local_anchor2: Vector,

    /// A unit vector that controls which axis should be aligned for both bodies.
    #[cfg(feature = "3d")]
    pub aligned_axis: Vector,

    /// The extents of the allowed relative rotation of the bodies around the `aligned_axis`.
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
    pub coefficients: SoftnessCoefficients,
    pub point_constraint: PointConstraintPart,
    #[cfg(feature = "2d")]
    pub rotation_difference: Scalar,
    #[cfg(feature = "3d")]
    pub rotation_difference: Quaternion,
    #[cfg(feature = "2d")]
    pub effective_point_constraint_mass: Matrix2,
    #[cfg(feature = "2d")]
    pub axial_mass: f32,
    #[cfg(feature = "3d")]
    pub effective_mass: SymmetricMatrix5,
    #[cfg(feature = "3d")]
    pub hinge_jacobian: Matrix2x3,
    #[cfg(feature = "3d")]
    pub angular_impulse: Vector2,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
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
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        let r1 = *body1.rotation * local_r1;
        let r2 = *body2.rotation * local_r2;

        // TODO: Support frames.
        solver_data.point_constraint.center_difference =
            body2.current_position() - body1.current_position();
        #[cfg(feature = "2d")]
        {
            solver_data.rotation_difference = body1.rotation.angle_between(*body2.rotation);
        }
        #[cfg(feature = "3d")]
        {
            solver_data.rotation_difference = body1.rotation.0.inverse() * body2.rotation.0;
        }

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

        #[cfg(feature = "2d")]
        {
            solver_data.effective_point_constraint_mass =
                PointConstraintPart::compute_inverse_effective_mass(
                    inverse_mass_sum,
                    &i1,
                    &i2,
                    r1,
                    r2,
                )
                .inverse();

            // Effective mass for angular hinge constraint
            solver_data.axial_mass = i1 + i2;

            if solver_data.axial_mass > 0.0 {
                solver_data.axial_mass = 1.0 / solver_data.axial_mass;
            }
        }

        #[cfg(feature = "3d")]
        {
            let mut effective_inverse_mass = SymmetricMatrix5::IDENTITY;

            // Point-to-point constraint (upper left 3x3 block)
            effective_inverse_mass.a = PointConstraintPart::compute_inverse_effective_mass(
                inverse_mass_sum,
                &i1,
                &i2,
                r1,
                r2,
            );

            let (axis1, mut axis2) = self.aligned_axis.any_orthonormal_pair();

            // Our implementation expects this to be flipped.
            axis2 *= -1.0;

            // Angular hinge (lower right 2x2 block)
            solver_data.hinge_jacobian = AngularHinge::jacobian(body1.rotation, axis1, axis2);
            let hinge_inertia1 = solver_data.hinge_jacobian * i1;
            let hinge_inertia2 = solver_data.hinge_jacobian * i2;
            let hinge_angular_contribution1 =
                hinge_inertia1.mul_by_transposed(solver_data.hinge_jacobian);
            let hinge_angular_contribution2 =
                hinge_inertia2.mul_by_transposed(solver_data.hinge_jacobian);
            effective_inverse_mass.d = hinge_angular_contribution1 + hinge_angular_contribution2;

            // Angular hinge (off-diagonal 2x3 block)
            let off_diagonal_x = hinge_inertia1.row(0).cross(r1) + hinge_inertia2.row(0).cross(r2);
            let off_diagonal_y = hinge_inertia1.row(1).cross(r1) + hinge_inertia2.row(1).cross(r2);
            effective_inverse_mass.b = Matrix2x3::from_rows(off_diagonal_x, off_diagonal_y);

            // TODO: Could do an LDLT solve here.
            solver_data.effective_mass = effective_inverse_mass.inverse();
        }

        solver_data.coefficients = self.stiffness.compute_coefficients(delta_secs);
    }

    fn warm_start(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        solver_data: &HingeJointSolverData,
    ) {
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        let r1 = *body1.rotation * local_r1;
        let r2 = *body2.rotation * local_r2;

        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_global_angular_inertia().inverse();
        let inv_inertia2 = body2.effective_global_angular_inertia().inverse();

        #[cfg(feature = "2d")]
        {
            let axial_impulse = solver_data.lower_impulse - solver_data.upper_impulse;

            if body1.rb.is_dynamic() {
                body1.linear_velocity.0 -= solver_data.point_constraint.impulse * inv_mass1;
                body1.angular_velocity.0 -= inv_inertia1
                    * (cross(r1, solver_data.point_constraint.impulse) + axial_impulse);
            }
            if body2.rb.is_dynamic() {
                body2.linear_velocity.0 += solver_data.point_constraint.impulse * inv_mass2;
                body2.angular_velocity.0 += inv_inertia2
                    * (cross(r2, solver_data.point_constraint.impulse) + axial_impulse);
            }
        }

        #[cfg(feature = "3d")]
        {
            let ball_socket_angular_impulse1 = r1.cross(solver_data.point_constraint.impulse);
            let hinge_angular_impulse1 = solver_data.angular_impulse * solver_data.hinge_jacobian;
            let angular_impulse1 = ball_socket_angular_impulse1 + hinge_angular_impulse1;

            let ball_socket_angular_impulse2 = solver_data.point_constraint.impulse.cross(r2);
            let angular_impulse2 = ball_socket_angular_impulse2 - hinge_angular_impulse1;

            if body1.rb.is_dynamic() {
                body1.linear_velocity.0 += solver_data.point_constraint.impulse * inv_mass1;
                body1.angular_velocity.0 += inv_inertia1 * angular_impulse1;
            }
            if body2.rb.is_dynamic() {
                body2.linear_velocity.0 += solver_data.point_constraint.impulse * inv_mass2;
                body2.angular_velocity.0 += inv_inertia2 * angular_impulse2;
            }
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

        // Limits
        if let Some(limit) = self.angle_limit {
            #[cfg(feature = "2d")]
            {
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
        }

        // Solve point-to-point constraint.

        // TODO: Cache these.
        let local_r1 = self.local_anchor1 - body1.center_of_mass.0;
        let local_r2 = self.local_anchor2 - body2.center_of_mass.0;
        let r1 = *body1.rotation * local_r1;
        let r2 = *body2.rotation * local_r2;

        #[cfg(feature = "2d")]
        {
            let impulse = PointConstraintPart::compute_incremental_impulse(
                body1,
                body2,
                r1,
                r2,
                solver_data.point_constraint.impulse,
                solver_data.point_constraint.center_difference,
                &solver_data.effective_point_constraint_mass,
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

        #[cfg(feature = "3d")]
        {
            let effective_mass = &solver_data.effective_mass;

            // 1. Compute biased velocity errors
            let mut point_constraint_velocity_error =
                PointConstraintPart::velocity_error(body1, body2, r1, r2);
            let mut hinge_velocity_error = AngularHinge::velocity_error(
                body1.angular_velocity.0,
                body2.angular_velocity.0,
                solver_data.hinge_jacobian,
            );
            let mut point_constraint_velocity_bias = Vector::ZERO;
            let mut hinge_bias = Vector2::ZERO;

            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;

            if use_bias {
                let separation = PointConstraintPart::position_error(
                    body1,
                    body2,
                    r1,
                    r2,
                    solver_data.point_constraint.center_difference,
                );
                point_constraint_velocity_bias = solver_data.coefficients.bias * separation;

                let axis1 = *body1.rotation * self.aligned_axis;
                let axis2 = *body2.rotation * self.aligned_axis;
                let error_angles =
                    AngularHinge::get_error_angles(axis1, axis2, solver_data.hinge_jacobian);
                println!("Error angles: {:?}", error_angles);
                // Negation: We want to oppose the error.
                hinge_bias = error_angles * -solver_data.coefficients.bias;

                mass_scale = solver_data.coefficients.mass_scale;
                impulse_scale = solver_data.coefficients.impulse_scale;
            }

            point_constraint_velocity_error =
                point_constraint_velocity_bias - point_constraint_velocity_error;
            hinge_velocity_error = hinge_bias - hinge_velocity_error;

            // 2. Transform the velocity errors by the effective mass to get the impulse.
            let (mut point_constraint_csi, mut hinge_csi) = effective_mass
                .transform_pair(point_constraint_velocity_error, hinge_velocity_error);

            // 3. Scale by mass scale and remove scaled accumulated impulse
            point_constraint_csi *= mass_scale;
            hinge_csi *= mass_scale;
            point_constraint_csi -= impulse_scale * solver_data.point_constraint.impulse;
            hinge_csi -= impulse_scale * solver_data.angular_impulse;

            solver_data.point_constraint.impulse += point_constraint_csi;
            solver_data.angular_impulse += hinge_csi;

            let ball_socket_angular_impulse1 = r1.cross(point_constraint_csi);
            let hinge_angular_impulse1 = hinge_csi * solver_data.hinge_jacobian;
            let angular_impulse1 = ball_socket_angular_impulse1 + hinge_angular_impulse1;

            let ball_socket_angular_impulse2 = point_constraint_csi.cross(r2);
            let angular_impulse2 = ball_socket_angular_impulse2 - hinge_angular_impulse1;

            if point_constraint_csi.is_finite()
                && angular_impulse1.is_finite()
                && angular_impulse2.is_finite()
            {
                println!("Point constraint csi: {:?}", point_constraint_csi);
                println!("Angular impulse: {:?}", hinge_velocity_error);
                if body1.rb.is_dynamic() {
                    body1.linear_velocity.0 += point_constraint_csi * inv_mass1;
                    body1.angular_velocity.0 += inv_inertia1 * angular_impulse1;
                }
                if body2.rb.is_dynamic() {
                    body2.linear_velocity.0 += point_constraint_csi * inv_mass2;
                    body2.angular_velocity.0 += inv_inertia2 * angular_impulse2;
                }
            }
        }
    }
}

impl HingeJoint {
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            #[cfg(feature = "3d")]
            aligned_axis: Vector3::Z,
            angle_limit: None,
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
}

impl HingeJoint {
    /// Sets the axis that the bodies should be aligned on.
    #[cfg(feature = "3d")]
    pub fn with_aligned_axis(self, axis: Vector) -> Self {
        Self {
            aligned_axis: axis,
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

#[cfg(feature = "3d")]
impl ConstraintDebugRender for HingeJoint {
    fn render(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        color: Color,
        gizmos: &mut Gizmos<PhysicsGizmos>,
    ) {
        let r1 = body1.global_center_of_mass() + *body1.rotation * self.local_anchor1;
        let r2 = body2.global_center_of_mass() + *body2.rotation * self.local_anchor2;
        let hinge_axis1 = *body1.rotation * self.aligned_axis;
        let hinge_axis2 = *body2.rotation * self.aligned_axis;

        gizmos.arrow(
            r1,
            r1 + hinge_axis1,
            bevy::color::palettes::tailwind::CYAN_400,
        );
        gizmos.arrow(r2, r2 + hinge_axis2, bevy::color::palettes::css::PINK);
        gizmos.sphere(r1, 0.05, color);
        gizmos.sphere(r2, 0.05, color);
    }
}

#[cfg(test)]
mod tests {

    #[cfg(feature = "3d")]
    #[test]
    fn error_angles() {
        use super::{super::AngularHinge, Matrix2x3, Vector2, Vector3};

        let error = AngularHinge::get_error_angles(
            Vector3::Z,
            Vector3::Z,
            Matrix2x3::from_rows(Vector3::Y, Vector3::X),
        );

        assert_eq!(error, Vector2::ZERO);
    }
}
