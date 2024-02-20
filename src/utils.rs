//! Miscallaneous utility functions.

use crate::prelude::*;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use parry::math::Isometry;

#[cfg(all(
    feature = "2d",
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub(crate) fn make_isometry(
    position: impl Into<Position>,
    rotation: impl Into<Rotation>,
) -> Isometry<Scalar> {
    let position: Position = position.into();
    let rotation: Rotation = rotation.into();
    Isometry::<Scalar>::new(position.0.into(), rotation.into())
}

#[cfg(all(
    feature = "3d",
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub(crate) fn make_isometry(
    position: impl Into<Position>,
    rotation: impl Into<Rotation>,
) -> Isometry<Scalar> {
    let position: Position = position.into();
    let rotation: Rotation = rotation.into();
    Isometry::<Scalar>::new(position.0.into(), rotation.to_scaled_axis().into())
}

#[cfg(feature = "3d")]
pub(crate) fn get_rotated_inertia_tensor(inertia_tensor: Matrix3, rot: Quaternion) -> Matrix3 {
    let rot_mat3 = Matrix3::from_quat(rot);
    (rot_mat3 * inertia_tensor) * rot_mat3.transpose()
}

/// Computes translation of `Position` based on center of mass rotation and translation
pub(crate) fn get_pos_translation(
    com_translation: &AccumulatedTranslation,
    previous_rotation: &Rotation,
    rotation: &Rotation,
    center_of_mass: &CenterOfMass,
) -> Vector {
    com_translation.0 + previous_rotation.rotate(center_of_mass.0)
        - rotation.rotate(center_of_mass.0)
}

/// Computes the magnitude of the impulse caused by dynamic friction.
pub(crate) fn compute_dynamic_friction(
    tangent_speed: Scalar,
    generalized_inv_mass_sum: Scalar,
    coefficient: Scalar,
    normal_lagrange: Scalar,
    sub_dt: Scalar,
) -> Scalar {
    let normal_impulse = normal_lagrange / sub_dt;

    // Compute impulse caused by dynamic friction, clamped to never exceed the tangential speed.
    // Note: This is handled differently from the XPBD paper because it treated mass incorrectly.
    -(coefficient * normal_impulse.abs()).min(tangent_speed / generalized_inv_mass_sum)
}

/// Computes the speed correction caused by restitution.
pub(crate) fn compute_restitution(
    normal_speed: Scalar,
    pre_solve_normal_speed: Scalar,
    coefficient: Scalar,
    _gravity: Vector,
    _sub_dt: Scalar,
) -> Scalar {
    // TODO: The XPBD paper has this, but it seems to be prevent bounces in cases
    // where bodies should clearly be bouncing.
    // Maybe change the threshold to be even lower? Or is this even needed at all?
    /*
    // If normal velocity is small enough, use restitution of 0 to avoid jittering
    if normal_speed.abs() <= 2.0 * gravity.length() * sub_dt {
        coefficient = 0.0;
    }
    */

    -normal_speed + (-coefficient * pre_solve_normal_speed).min(0.0)
}
