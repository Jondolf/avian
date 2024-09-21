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

/// A hinge joint prevents relative movement of the attached bodies, except for rotation around one `aligned_axis`.
///
/// Hinges can be useful for things like wheels, fans, revolving doors etc.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
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
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct HingeJointSolverData {
    pub coefficients: SoftnessCoefficients,
    pub center_difference: Vector,
    #[cfg(feature = "2d")]
    pub rotation_difference: Scalar,
    #[cfg(feature = "3d")]
    pub rotation_difference: Quaternion,
    #[cfg(feature = "2d")]
    pub pivot_mass: Mat2,
    #[cfg(feature = "2d")]
    pub axial_mass: f32,
    #[cfg(feature = "3d")]
    pub effective_mass: SymmetricMatrix5,
    #[cfg(feature = "3d")]
    pub hinge_jacobian: Matrix2x3,
    pub linear_impulse: Vector,
    #[cfg(feature = "3d")]
    pub angular_impulse: Vector2,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
}

impl ImpulseJoint for HingeJoint {
    type SolverData = HingeJointSolverData;

    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

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
        solver_data.center_difference = body2.current_position() - body1.current_position();
        #[cfg(feature = "2d")]
        {
            solver_data.rotation_difference = body1.rotation.angle_between(*body2.rotation);
        }
        #[cfg(feature = "3d")]
        {
            solver_data.rotation_difference = body1.rotation.0.inverse() * body2.rotation.0;
        }

        let inverse_mass_sum = body1.inverse_mass.0 + body2.inverse_mass.0;
        let i1 = body1.effective_world_inv_inertia();
        let i2 = body2.effective_world_inv_inertia();

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
            // Effective mass for point-to-point constraint
            // TODO: Abstract this.
            let k00 = inverse_mass_sum + r1.y.powi(2) * i1 + r2.y.powi(2) * i2;
            let k10 = -r1.y * r1.x * i1 - r2.y * r2.x * i2;
            let k01 = k10;
            let k11 = inverse_mass_sum + r1.x.powi(2) * i1 + r2.x.powi(2) * i2;
            solver_data.pivot_mass = Mat2::from_cols_array(&[k00, k10, k01, k11]).inverse();

            // Effective mass for angular hinge constraint
            solver_data.axial_mass = i1 + i2;
        }
        #[cfg(feature = "3d")]
        {
            let mut effective_inverse_mass = SymmetricMatrix5::IDENTITY;
            let (axis1, axis2) = self.aligned_axis.any_orthonormal_pair();

            // Point-to-point constraint (upper left 3x3 block)
            // TODO: Abstract this.
            let angular_contribution1 = i1.skew(r1);
            let angular_contribution2 = i2.skew(r2);
            effective_inverse_mass.a = angular_contribution1 + angular_contribution2;
            effective_inverse_mass.a.m00 += inverse_mass_sum;
            effective_inverse_mass.a.m11 += inverse_mass_sum;
            effective_inverse_mass.a.m22 += inverse_mass_sum;

            // Angular hinge (lower right 2x2 block)
            solver_data.hinge_jacobian =
                Matrix2x3::from_rows(body1.rotation.0 * axis1, body2.rotation.0 * axis2);
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

            solver_data.effective_mass = effective_inverse_mass.inverse();
        }

        solver_data.coefficients = SoftnessParameters::new(1.0, (0.125 / delta_secs).min(60.0))
            .compute_coefficients(delta_secs);

        #[cfg(feature = "2d")]
        if solver_data.axial_mass > 0.0 {
            solver_data.axial_mass = 1.0 / solver_data.axial_mass;
        }
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

        let inv_mass1 = body1.effective_inv_mass();
        let inv_mass2 = body2.effective_inv_mass();
        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

        let axial_impulse = solver_data.lower_impulse - solver_data.upper_impulse;

        if body1.rb.is_dynamic() {
            body1.linear_velocity.0 -= solver_data.linear_impulse * inv_mass1;
            body1.angular_velocity.0 -=
                inv_inertia1 * (cross(r1, solver_data.linear_impulse) + axial_impulse);
        }
        if body2.rb.is_dynamic() {
            body2.linear_velocity.0 += solver_data.linear_impulse * inv_mass2;
            body2.angular_velocity.0 +=
                inv_inertia2 * (cross(r2, solver_data.linear_impulse) + axial_impulse);
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
        let inv_mass1 = body1.effective_inv_mass();
        let inv_mass2 = body2.effective_inv_mass();
        let inv_inertia1 = body1.effective_world_inv_inertia();
        let inv_inertia2 = body2.effective_world_inv_inertia();

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

        let mut bias = Vector::ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;

        if use_bias {
            let delta_separation =
                (body2.accumulated_translation.0 - body1.accumulated_translation.0) + (r2 - r1);
            let separation = delta_separation + solver_data.center_difference;
            bias = solver_data.coefficients.bias * separation;
            mass_scale = solver_data.coefficients.mass_scale;
            impulse_scale = solver_data.coefficients.impulse_scale;
        }

        #[cfg(feature = "2d")]
        let effective_mass = solver_data.pivot_mass;
        #[cfg(feature = "3d")]
        let effective_mass = solver_data.effective_mass;

        #[cfg(feature = "2d")]
        {
            // Linear velocity constraint.
            // C' = v2 + cross(w2, r2) - v1 - cross(w1, r1) = 0
            let c_vel = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);

            let impulse = -mass_scale * effective_mass * (c_vel + bias)
                - impulse_scale * solver_data.linear_impulse;
            solver_data.linear_impulse += impulse;

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
            let ball_socket_error = solver_data.center_difference + r2 - r1;
            let ball_socket_bias_vel = ball_socket_error * bias;

            let error_angles = self.get_error_angles(effective_mass.b);
            let hinge_bias_velocity = error_angles * solver_data.coefficients.bias;

            let ball_socket_angular1 = body1.angular_velocity.cross(r1);
            let ball_socket_angular2 = body2.angular_velocity.cross(r2);
            let hinge1 = body1.angular_velocity.0 * solver_data.hinge_jacobian;
            let neg_hinge2 = body2.angular_velocity.0 * solver_data.hinge_jacobian;

            let ball_socket_angular = ball_socket_angular1 + ball_socket_angular2;
            let ball_socket_linear = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);
            let mut ball_socket = ball_socket_bias_vel - (ball_socket_angular + ball_socket_linear);
            let mut hinge = hinge_bias_velocity - (hinge1 - neg_hinge2);

            effective_mass.transform_pair(&mut ball_socket, &mut hinge);

            let linear_impulse =
                -mass_scale * ball_socket - impulse_scale * solver_data.linear_impulse;
            solver_data.linear_impulse += linear_impulse;

            let angular_impulse = -mass_scale * hinge - impulse_scale * solver_data.angular_impulse;
            solver_data.angular_impulse += angular_impulse;

            let ball_socket_angular_impulse1 = r1.cross(linear_impulse);
            let hinge_angular_impulse1 = angular_impulse * solver_data.hinge_jacobian;
            let angular_impulse1 = ball_socket_angular_impulse1 + hinge_angular_impulse1;

            let ball_socket_angular_impulse2 = r2.cross(linear_impulse);
            let angular_impulse2 = ball_socket_angular_impulse2 - hinge_angular_impulse1;

            if body1.rb.is_dynamic() {
                body1.linear_velocity.0 -= linear_impulse * inv_mass1;
                //body1.angular_velocity.0 -= inv_inertia1 * cross(r1, angular_impulse1);
            }
            if body2.rb.is_dynamic() {
                body2.linear_velocity.0 += linear_impulse * inv_mass2;
                //body2.angular_velocity.0 += inv_inertia2 * cross(r2, angular_impulse2);
            }
        }
    }

    fn local_anchor_1(&self) -> Vector {
        self.local_anchor1
    }

    fn local_anchor_2(&self) -> Vector {
        self.local_anchor2
    }
}

impl HingeJoint {
    #[cfg(feature = "3d")]
    fn get_error_angles(&self, jacobian1: Matrix2x3) -> Vector2 {
        // TODO: Cache these.
        let (axis1, axis2) = self.aligned_axis.any_orthonormal_pair();
        let (jacobian_x, jacobian_y) = (jacobian1.row(0), jacobian1.row(1));

        let axis2_dot_x = axis2.dot(jacobian_x);
        let axis2_dot_y = axis2.dot(jacobian_y);

        let to_remove_x = jacobian_x * axis2_dot_x;
        let to_remove_y = jacobian_y * axis2_dot_y;

        let mut axis2_on_x_plane = axis2 - to_remove_x;
        let mut axis2_on_y_plane = axis2 - to_remove_y;

        let x_length = axis2_on_x_plane.length();
        let y_length = axis2_on_y_plane.length();
        let scale_x = Vector::ONE / x_length;
        let scale_y = Vector::ONE / x_length;

        axis2_on_x_plane *= scale_x;
        axis2_on_y_plane *= scale_y;

        let epsilon = Vector::splat(1e-7);
        let use_fallback_x = Vector::splat(x_length).cmple(epsilon);
        let use_fallback_y = Vector::splat(y_length).cmple(epsilon);

        axis2_on_x_plane = Vector::select(use_fallback_x, axis1, axis2_on_x_plane);
        axis2_on_y_plane = Vector::select(use_fallback_y, axis1, axis2_on_y_plane);

        let axis2x_dot_axis1 = axis2_on_x_plane.dot(axis1);
        let axis2y_dot_axis1 = axis2_on_y_plane.dot(axis1);

        let mut error_angles = Vector2::new(axis2x_dot_axis1.acos(), axis2y_dot_axis1.acos());

        let axis2x_dot_jacobian_y = axis2_on_x_plane.dot(jacobian_y);
        let axis2y_dot_jacobian_x = axis2_on_y_plane.dot(jacobian_x);

        error_angles.x *= (axis2x_dot_jacobian_y < 0.0) as u8 as Scalar;
        error_angles.y *= (axis2y_dot_jacobian_x >= 0.0) as u8 as Scalar;

        error_angles
    }

    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            #[cfg(feature = "3d")]
            aligned_axis: Vector3::Z,
            angle_limit: None,
            stiffness: SoftnessParameters::new(1.0, 0.125 * (1.0 / (60.0 * 8.0))),
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

    #[cfg(feature = "3d")]
    fn get_delta_q(&self, rot1: &Rotation, rot2: &Rotation) -> Vector3 {
        let a1 = *rot1 * self.aligned_axis;
        let a2 = *rot2 * self.aligned_axis;
        a1.cross(a2)
    }
}

impl MapEntities for HingeJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}
