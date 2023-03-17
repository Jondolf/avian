use super::super::prelude::*;

#[cfg(feature = "2d")]
pub(crate) fn make_isometry(pos: Vector, rot: &Rot) -> Isometry<Scalar> {
    Isometry::<Scalar>::new(pos.into(), (*rot).into())
}

#[cfg(feature = "3d")]
pub(crate) fn make_isometry(pos: Vector, rot: &Rot) -> Isometry<Scalar> {
    Isometry::<Scalar>::new(pos.into(), rot.to_scaled_axis().into())
}

#[cfg(feature = "3d")]
pub(crate) fn get_rotated_inertia_tensor(inertia_tensor: Matrix3, rot: Quaternion) -> Matrix3 {
    let rot_mat3 = Matrix3::from_quat(rot);
    (rot_mat3 * inertia_tensor) * rot_mat3.transpose()
}

/// Calculates velocity correction caused by dynamic friction.
pub(crate) fn get_dynamic_friction(
    tangent_vel: Vector,
    friction1: &Friction,
    friction2: &Friction,
    normal_lagrange: Scalar,
    sub_dt: Scalar,
) -> Vector {
    let tangent_vel_magnitude = tangent_vel.length();

    // Avoid division by zero when normalizing the vector later.
    // We compare against epsilon to avoid potential floating point precision problems.
    if tangent_vel_magnitude.abs() <= Scalar::EPSILON {
        return Vector::ZERO;
    }

    // Average of the bodies' dynamic friction coefficients
    let dynamic_friction_coefficient =
        (friction1.dynamic_coefficient + friction2.dynamic_coefficient) * 0.5;

    let normal_force = normal_lagrange / sub_dt.powi(2);

    // Velocity update caused by dynamic friction, never exceeds the magnitude of the tangential velocity itself
    -tangent_vel / tangent_vel_magnitude
        * (sub_dt * dynamic_friction_coefficient * normal_force.abs()).min(tangent_vel_magnitude)
}

/// Calculates velocity correction caused by restitution.
pub(crate) fn get_restitution(
    normal: Vector,
    normal_vel: Scalar,
    pre_solve_normal_vel: Scalar,
    restitution1: &Restitution,
    restitution2: &Restitution,
    gravity: Vector,
    sub_dt: Scalar,
) -> Vector {
    let mut restitution_coefficient = (restitution1.0 + restitution2.0) * 0.5;

    // If normal velocity is small enough, use restitution of 0 to avoid jittering
    if normal_vel.abs() <= 2.0 * gravity.length() * sub_dt {
        restitution_coefficient = 0.0;
    }

    normal * (-normal_vel + (-restitution_coefficient * pre_solve_normal_vel).min(0.0))
}
