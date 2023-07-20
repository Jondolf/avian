//! Miscallaneous utility functions.

use crate::prelude::*;

#[cfg(feature = "2d")]
pub(crate) fn make_isometry(pos: Vector, rot: &Rotation) -> Isometry<Scalar> {
    Isometry::<Scalar>::new(pos.into(), (*rot).into())
}

#[cfg(feature = "3d")]
pub(crate) fn make_isometry(pos: Vector, rot: &Rotation) -> Isometry<Scalar> {
    Isometry::<Scalar>::new(pos.into(), rot.to_scaled_axis().into())
}

pub(crate) fn entity_from_index_and_gen(index: u32, generation: u32) -> bevy::prelude::Entity {
    bevy::prelude::Entity::from_bits((generation as u64) << 32 | index as u64)
}

#[cfg(feature = "3d")]
pub(crate) fn get_rotated_inertia_tensor(inertia_tensor: Matrix3, rot: Quaternion) -> Matrix3 {
    let rot_mat3 = Matrix3::from_quat(rot);
    (rot_mat3 * inertia_tensor) * rot_mat3.transpose()
}

/// Calculates impulse magnitude correction caused by dynamic friction.
pub(crate) fn get_dynamic_friction(
    tangent_speed: Scalar,
    generalized_mass_sum: Scalar,
    coefficient: Scalar,
    normal_lagrange: Scalar,
    sub_dt: Scalar,
) -> Scalar {
    let normal_impulse = normal_lagrange / sub_dt;

    // Velocity update caused by dynamic friction, never exceeds the magnitude of the tangential velocity itself
    -(coefficient * normal_impulse.abs()).min(tangent_speed / generalized_mass_sum)
}

/// Calculates speed correction caused by restitution.
pub(crate) fn get_restitution(
    normal_vel: Scalar,
    pre_solve_normal_vel: Scalar,
    mut coefficient: Scalar,
    gravity: Vector,
    sub_dt: Scalar,
) -> Scalar {
    // If normal velocity is small enough, use restitution of 0 to avoid jittering
    if normal_vel.abs() <= 2.0 * gravity.length() * sub_dt {
        coefficient = 0.0;
    }

    -normal_vel + (-coefficient * pre_solve_normal_vel).min(0.0)
}
