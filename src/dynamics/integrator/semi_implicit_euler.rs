//! The *semi-implicit* or *symplectic* Euler [integration](super) scheme.
//!
//! [Semi-implicit Euler](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method)
//! integration is the most common integration scheme because it is simpler and more
//! efficient than implicit Euler integration, has great energy conservation,
//! and provides much better accuracy than explicit Euler integration.
//!
//! Semi-implicit Euler integration evalutes the acceleration at
//! the current timestep and the velocity at the next timestep:
//!
//! ```text
//! v = v_0 + a * Δt (linear velocity)
//! ω = ω_0 + α * Δt (angular velocity)
//! ```
//!
//! and computes the new position:
//!
//! ```text
//! x = x_0 + v * Δt (position)
//! θ = θ_0 + ω * Δt (rotation)
//! ```
//!
//! This order is opposite to explicit Euler integration, which uses the velocity
//! at the current timestep instead of the next timestep. The explicit approach
//! can lead to bodies gaining energy over time, which is why the semi-implicit
//! approach is typically preferred.

use super::*;

/// Integrates velocity based on the given forces in order to find
/// the linear and angular velocity after `delta_seconds` have passed.
///
/// This uses [semi-implicit (symplectic) Euler integration](self).
#[allow(clippy::too_many_arguments)]
pub fn integrate_velocity(
    lin_vel: &mut Vector,
    ang_vel: &mut AngularValue,
    force: Vector,
    torque: TorqueValue,
    inv_mass: Scalar,
    inv_inertia: impl Into<InverseInertia>,
    rotation: Rotation,
    locked_axes: LockedAxes,
    gravity: Vector,
    delta_seconds: Scalar,
) {
    let inv_inertia = inv_inertia.into();

    // Compute linear acceleration.
    let lin_acc = linear_acceleration(force, inv_mass, locked_axes, gravity);

    // Compute next linear velocity.
    // v = v_0 + a * Δt
    let next_lin_vel = *lin_vel + lin_acc * delta_seconds;
    if next_lin_vel != *lin_vel {
        *lin_vel = next_lin_vel;
    }

    // Compute angular acceleration.
    let ang_acc = angular_acceleration(torque, inv_inertia.rotated(&rotation).0, locked_axes);

    // Compute angular velocity delta.
    // Δω = α * Δt
    #[allow(unused_mut)]
    let mut delta_ang_vel = ang_acc * delta_seconds;

    #[cfg(feature = "3d")]
    {
        // In 3D, we should also handle gyroscopic motion, which accounts for
        // non-spherical shapes that may wobble as they spin in the air.
        //
        // Gyroscopic motion happens when the inertia tensor is not uniform, causing
        // the angular momentum to point in a different direction than the angular velocity.
        //
        // The gyroscopic torque is τ = ω x Iω.
        //
        // However, the basic semi-implicit approach can blow up, as semi-implicit Euler
        // extrapolates velocity and the gyroscopic torque is quadratic in the angular velocity.
        // Thus, we use implicit Euler, which is much more accurate and stable, although slightly more expensive.
        let effective_inertia = locked_axes.apply_to_rotation(inv_inertia.inverse().0);
        delta_ang_vel += solve_gyroscopic_torque(
            *ang_vel,
            rotation.0,
            Inertia(effective_inertia),
            delta_seconds,
        );
    }

    if delta_ang_vel != AngularVelocity::ZERO.0 && delta_ang_vel.is_finite() {
        *ang_vel += delta_ang_vel;
    }
}

/// Integrates position and rotation based on the given velocities in order to
/// find the position and rotation after `delta_seconds` have passed.
///
/// This uses [semi-implicit (symplectic) Euler integration](self).
pub fn integrate_position(
    pos: &mut Vector,
    rot: &mut Rotation,
    lin_vel: Vector,
    ang_vel: AngularValue,
    locked_axes: LockedAxes,
    delta_seconds: Scalar,
) {
    let lin_vel = locked_axes.apply_to_vec(lin_vel);

    // x = x_0 + v * Δt
    let next_pos = *pos + lin_vel * delta_seconds;

    if next_pos != *pos && next_pos.is_finite() {
        *pos = next_pos;
    }

    // Effective inverse inertia along each rotational axis
    let ang_vel = locked_axes.apply_to_angular_velocity(ang_vel);

    // θ = θ_0 + ω * Δt
    #[cfg(feature = "2d")]
    {
        let delta_rot = Rotation::radians(ang_vel * delta_seconds);
        if delta_rot != Rotation::IDENTITY && delta_rot.is_finite() {
            *rot *= delta_rot;
        }
    }
    #[cfg(feature = "3d")]
    {
        // This is a bit more complicated because quaternions are weird.
        // Maybe there's a simpler and more numerically stable way?
        let delta_rot = Quaternion::from_vec4(
            (ang_vel * delta_seconds / 2.0).extend(rot.w * delta_seconds / 2.0),
        ) * rot.0;
        if delta_rot.w != 0.0 && delta_rot.is_finite() {
            rot.0 = (rot.0 + delta_rot).normalize();
        }
    }
}

/// Computes linear acceleration based on the given forces and mass.
pub fn linear_acceleration(
    force: Vector,
    inv_mass: Scalar,
    locked_axes: LockedAxes,
    gravity: Vector,
) -> Vector {
    // Effective inverse mass along each axis
    let inv_mass = locked_axes.apply_to_vec(Vector::splat(inv_mass));

    if inv_mass != Vector::ZERO && inv_mass.is_finite() {
        // Newton's 2nd law for translational movement:
        //
        // F = m * a
        // a = F / m
        //
        // where a is the acceleration, F is the force, and m is the mass.
        //
        // `gravity` below is the gravitational acceleration,
        // so it doesn't need to be divided by mass.
        force * inv_mass + locked_axes.apply_to_vec(gravity)
    } else {
        Vector::ZERO
    }
}

/// Computes angular acceleration based on the current angular velocity, torque, and inertia.
#[cfg_attr(
    feature = "3d",
    doc = "
Note that this does not account for gyroscopic motion. To compute the gyroscopic angular velocity
correction, use `solve_gyroscopic_torque`."
)]
pub fn angular_acceleration(
    torque: TorqueValue,
    world_inv_inertia: InertiaValue,
    locked_axes: LockedAxes,
) -> AngularValue {
    // Effective inverse inertia along each axis
    let effective_inv_inertia = locked_axes.apply_to_rotation(world_inv_inertia);

    if effective_inv_inertia != InverseInertia::ZERO.0 && effective_inv_inertia.is_finite() {
        // Newton's 2nd law for rotational movement:
        //
        // τ = I * α
        // α = τ / I
        //
        // where α (alpha) is the angular acceleration,
        // τ (tau) is the torque, and I is the moment of inertia.
        world_inv_inertia * torque
    } else {
        AngularValue::ZERO
    }
}

/// Computes the angular correction caused by gyroscopic motion,
/// which may cause objects with non-uniform angular inertia to wobble
/// while spinning.
#[cfg(feature = "3d")]
pub fn solve_gyroscopic_torque(
    ang_vel: Vector,
    rotation: Quaternion,
    local_inertia: Inertia,
    delta_seconds: Scalar,
) -> Vector {
    // Based on the "Gyroscopic Motion" section of Erin Catto's GDC 2015 slides on Numerical Methods.
    // https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf

    // Convert angular velocity to body coordinates so that we can use the local angular inertia
    let local_ang_vel = rotation.inverse() * ang_vel;

    // Compute body-space angular momentum
    let angular_momentum = local_inertia.0 * local_ang_vel;

    // Compute Jacobian
    let jacobian = local_inertia.0
        + delta_seconds
            * (skew_symmetric_mat3(local_ang_vel) * local_inertia.0
                - skew_symmetric_mat3(angular_momentum));

    // Residual vector
    let f = delta_seconds * local_ang_vel.cross(angular_momentum);

    // Do one Newton-Raphson iteration
    let delta_ang_vel = -jacobian.inverse() * f;

    // Convert back to world coordinates
    rotation * delta_ang_vel
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn semi_implicit_euler() {
        let mut position = Vector::ZERO;
        let mut rotation = Rotation::default();

        let mut linear_velocity = Vector::ZERO;
        #[cfg(feature = "2d")]
        let mut angular_velocity = 2.0;
        #[cfg(feature = "3d")]
        let mut angular_velocity = Vector::Z * 2.0;

        let inv_mass = 1.0;
        #[cfg(feature = "2d")]
        let inv_inertia = 1.0;
        #[cfg(feature = "3d")]
        let inv_inertia = Matrix3::IDENTITY;

        let gravity = Vector::NEG_Y * 9.81;

        // Step by 100 steps of 0.1 seconds
        for _ in 0..100 {
            integrate_velocity(
                &mut linear_velocity,
                &mut angular_velocity,
                default(),
                default(),
                inv_mass,
                inv_inertia,
                rotation,
                default(),
                gravity,
                1.0 / 10.0,
            );
            integrate_position(
                &mut position,
                &mut rotation,
                linear_velocity,
                angular_velocity,
                default(),
                1.0 / 10.0,
            );
        }

        // Euler methods have some precision issues, but this seems weirdly inaccurate.
        assert_relative_eq!(position, Vector::NEG_Y * 490.5, epsilon = 10.0);

        #[cfg(feature = "2d")]
        assert_relative_eq!(
            rotation.as_radians(),
            Rotation::radians(20.0).as_radians(),
            epsilon = 0.00001
        );
        #[cfg(feature = "3d")]
        assert_relative_eq!(
            rotation.0,
            Quaternion::from_rotation_z(20.0),
            epsilon = 0.01
        );

        assert_relative_eq!(linear_velocity, Vector::NEG_Y * 98.1, epsilon = 0.0001);
        #[cfg(feature = "2d")]
        assert_relative_eq!(angular_velocity, 2.0, epsilon = 0.00001);
        #[cfg(feature = "3d")]
        assert_relative_eq!(angular_velocity, Vector::Z * 2.0, epsilon = 0.00001);
    }
}
