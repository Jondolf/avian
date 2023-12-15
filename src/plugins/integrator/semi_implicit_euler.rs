//! The *semi-implicit* or *symplectic* Euler [integration](integrator) scheme.
//!
//! Semi-implicit Euler integration is the most common integration scheme because it
//! is simpler and more efficient than implicit Euler integration, has great energy
//! conservation, and provides much better accuracy than explicit Euler integration.
//!
//! Semi-implicicit Euler integration evalutes the acceleration at
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

/// Integrates forces and torque according to Newton's 2nd law
/// in order to find the velocity after `delta_seconds` have passed.
///
/// This uses [semi-implicit Euler integration](self). See [`integrator`] for alternatives.
#[allow(clippy::too_many_arguments)]
pub fn integrate_forces(
    lin_vel: &mut Vector,
    ang_vel: &mut AngularValue,
    force: Vector,
    torque: TorqueValue,
    inv_mass: f32,
    inv_inertia: InertiaValue,
    locked_axes: LockedAxes,
    gravity: Vector,
    delta_seconds: f32,
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
    let next_ang_vel = *ang_vel + ang_acc * delta_seconds;
    if next_ang_vel != *ang_vel && next_ang_vel.is_finite() {
        *ang_vel = next_ang_vel;
    }
}

/// Integrates velocity in order to find the position and rotation after `delta_seconds` have passed.
///
/// This uses [semi-implicit Euler integration](self). See [`integrator`] for alternatives.
pub fn integrate_velocity(
    pos: &mut Vector,
    rot: &mut Rotation,
    lin_vel: Vector,
    ang_vel: AngularValue,
    locked_axes: LockedAxes,
    delta_seconds: f32,
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
        let next_rot = Rotation::from_radians(ang_vel * delta_seconds);
        if next_rot != *rot && next_rot.is_finite() {
            *rot = next_rot;
        }
    }
    #[cfg(feature = "3d")]
    {
        let q = Quaternion::from_vec4(ang_vel.extend(0.0)) * rot.0;
        let effective_dq = locked_axes
            .apply_to_angular_velocity(delta_seconds * 0.5 * q.xyz())
            .extend(delta_seconds * 0.5 * q.w);
        // avoid triggering bevy's change detection unnecessarily
        let delta = Quaternion::from_vec4(effective_dq);
        if delta.w != 0.0 {
            rot.0 = (rot.0 + delta).normalize();
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn velocity_verlet() {
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
            integrate_forces(
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
            integrate_velocity(
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
            Rotation::from_radians(20.0).as_radians(),
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
