use crate::prelude::*;
#[cfg(feature = "3d")]
use bevy::prelude::*;

#[cfg(feature = "3d")]
pub(crate) fn get_rotated_inertia_tensor(inertia_tensor: Mat3, rot: Quat) -> Mat3 {
    let rot_mat3 = Mat3::from_quat(rot);
    (rot_mat3 * inertia_tensor) * rot_mat3.transpose()
}

/// Calculates velocity correction caused by dynamic friction.
pub(crate) fn get_dynamic_friction(
    tangent_vel: Vector,
    friction_a: &Friction,
    friction_b: &Friction,
    normal_lagrange: f32,
    sub_dt: f32,
) -> Vector {
    let tangent_vel_magnitude = tangent_vel.length();

    // Avoid division by zero when normalizing the vector later.
    // We compare against epsilon to avoid potential floating point precision problems.
    if tangent_vel_magnitude.abs() <= f32::EPSILON {
        return Vector::ZERO;
    }

    // Average of the bodies' dynamic friction coefficients
    let dynamic_friction_coefficient =
        (friction_a.dynamic_coefficient + friction_b.dynamic_coefficient) * 0.5;

    let normal_force = normal_lagrange / sub_dt.powi(2);

    // Velocity update caused by dynamic friction, never exceeds the magnitude of the tangential velocity itself
    -tangent_vel / tangent_vel_magnitude
        * (sub_dt * dynamic_friction_coefficient * normal_force.abs()).min(tangent_vel_magnitude)
}

/// Calculates velocity correction caused by restitution.
pub(crate) fn get_restitution(
    normal: Vector,
    normal_vel: f32,
    pre_solve_normal_vel: f32,
    restitution_a: &Restitution,
    restitution_b: &Restitution,
    gravity: Vector,
    sub_dt: f32,
) -> Vector {
    let mut restitution_coefficient = (restitution_a.0 + restitution_b.0) * 0.5;

    // If normal velocity is small enough, use restitution of 0 to avoid jittering
    if normal_vel.abs() <= 2.0 * gravity.length() * sub_dt {
        restitution_coefficient = 0.0;
    }

    normal * (-normal_vel + (-restitution_coefficient * pre_solve_normal_vel).min(0.0))
}
