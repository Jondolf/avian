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
    inv_inertia: InertiaValue,
    locked_axes: LockedAxes,
    gravity: Vector,
    delta_seconds: Scalar,
) {
    // Compute linear acceleration.
    let lin_acc = linear_acceleration(force, inv_mass, locked_axes, gravity);

    // Compute next linear velocity.
    // v = v_0 + a * Δt
    let next_lin_vel = *lin_vel + lin_acc * delta_seconds;
    if next_lin_vel != *lin_vel {
        *lin_vel = next_lin_vel;
    }

    // Compute angular acceleration.
    let ang_acc = angular_acceleration(*ang_vel, torque, inv_inertia, locked_axes);

    // Compute next angular velocity.
    // ω = ω_0 + α * Δt
    let delta_ang_vel = ang_acc * delta_seconds;
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
        force * inv_mass + gravity
    } else {
        Vector::ZERO
    }
}

/// Computes angular acceleration based on the current angular velocity, torque, and inertia.
#[allow(unused_variables)]
pub fn angular_acceleration(
    ang_vel: AngularValue,
    torque: TorqueValue,
    inv_inertia: InertiaValue,
    locked_axes: LockedAxes,
) -> AngularValue {
    // Effective inverse inertia along each axis
    let inv_inertia = locked_axes.apply_to_rotation(inv_inertia);

    if inv_inertia != InverseInertia::ZERO.0 && inv_inertia.is_finite() {
        // Newton's 2nd law for rotational movement:
        //
        // τ = I * α
        // α = τ / I
        //
        // where α (alpha) is the angular acceleration,
        // τ (tau) is the torque, and I is the moment of inertia.
        #[cfg(feature = "2d")]
        {
            inv_inertia * torque
        }
        #[cfg(feature = "3d")]
        {
            inv_inertia * (torque - ang_vel.cross(inv_inertia.inverse() * ang_vel))
        }
    } else {
        AngularValue::ZERO
    }
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
