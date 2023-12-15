//! The Verlet [integration](integrator) scheme.

use super::*;

/// Integrates velocity in order to find the position and rotation after `delta_seconds` have passed.
/// With Verlet integration, this should typically be run before [`integrate_forces`].
///
/// This uses [velocity Verlet](self), a popular variant of Verlet integration.
/// See [`integrator`] for alternatives.
#[allow(clippy::too_many_arguments)]
pub fn integrate_velocity(
    pos: &mut Vector,
    rot: &mut Rotation,
    lin_vel: Vector,
    ang_vel: AngularValue,
    lin_acc: Vector,
    ang_acc: AngularValue,
    locked_axes: LockedAxes,
    delta_seconds: f32,
) {
    let delta_seconds_sq = delta_seconds.powi(2);

    // Compute next position using velocity and acceleration.
    // x_next = x + v * Δt + 1/2 * a * (Δt)^2
    let next_pos = *pos + lin_vel * delta_seconds + 0.5 * lin_acc * delta_seconds_sq;
    if next_pos != *pos && next_pos.is_finite() {
        *pos = next_pos;
    }

    // Rotation delta
    let delta_rot = locked_axes
        .apply_to_angular_velocity(ang_vel * delta_seconds + 0.5 * ang_acc * delta_seconds_sq);

    // θ = θ_0 + ω * Δt
    #[cfg(feature = "2d")]
    {
        let next_rot = *rot + Rotation::from_radians(delta_rot);
        if next_rot != *rot && next_rot.is_finite() {
            *rot = next_rot;
        }
    }
    #[cfg(feature = "3d")]
    {
        // This is a bit more complicated because quaternions are weird.
        // Maybe there's a simpler and more numerically stable way?
        let delta_rotation =
            Quaternion::from_vec4((delta_rot / 2.0).extend(rot.w * delta_seconds / 2.0)) * rot.0;
        if delta_rotation.w != 0.0 && delta_rotation.is_finite() {
            rot.0 = (rot.0 + delta_rotation).normalize();
        }
    }
}

/// Integrates forces and torque according to Newton's 2nd law
/// in order to find the velocity after `delta_seconds` have passed.
/// With Verlet integration, this should typically be run after [`integrate_forces`].
///
/// This uses [velocity Verlet](self), a popular variant of verlet integration.
/// See [`integrator`] for alternatives.
#[allow(clippy::too_many_arguments)]
pub fn integrate_forces(
    lin_vel: &mut Vector,
    ang_vel: &mut AngularValue,
    lin_acc: Vector,
    ang_acc: AngularValue,
    force: Vector,
    torque: AngularValue,
    inv_mass: f32,
    inv_inertia: InertiaValue,
    locked_axes: LockedAxes,
    gravity: Vector,
    delta_seconds: f32,
) {
    // Compute next linear acceleration.
    let next_lin_acc = linear_acceleration(force, inv_mass, locked_axes, gravity);

    // Compute next linear velocity using current and next acceleration.
    // v_next = v + 1/2 * (a + a_next) * Δt
    let next_lin_vel = *lin_vel + 0.5 * (lin_acc + next_lin_acc) * delta_seconds;
    if next_lin_vel != *lin_vel {
        *lin_vel = next_lin_vel;
    }

    // Compute next angular acceleration.
    let next_ang_acc = angular_acceleration(*ang_vel, torque, inv_inertia, locked_axes);

    // Compute next angular velocity using current and next acceleration.
    // ω_next = ω + (α + α_next) * Δt
    let next_ang_vel = *ang_vel + (ang_acc + next_ang_acc) * delta_seconds;
    if next_ang_vel != *ang_vel {
        *ang_vel = next_ang_vel;
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
            integrate_velocity(
                &mut position,
                &mut rotation,
                linear_velocity,
                angular_velocity,
                default(),
                default(),
                default(),
                1.0 / 10.0,
            );
            integrate_forces(
                &mut linear_velocity,
                &mut angular_velocity,
                gravity,
                default(),
                default(),
                default(),
                inv_mass,
                inv_inertia,
                default(),
                gravity,
                1.0 / 10.0,
            );
        }

        assert_relative_eq!(position, Vector::NEG_Y * 490.5, epsilon = 0.00001);

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
