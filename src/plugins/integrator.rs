//! Integrates Newton's 2nd law of motion, applying forces and moving entities according to their velocities.
//!
//! See [`IntegratorPlugin`].

#![allow(clippy::type_complexity)]

use crate::prelude::*;
use bevy::prelude::*;

/// Integrates Newton's 2nd law of motion, applying forces and moving entities according to their velocities.
///
/// This acts as a prediction for the next positions and orientations of the bodies. The [solver] corrects these predicted
/// positions to follow the rules set by the [constraints].
///
/// The integration scheme used is very closely related to implicit Euler integration.
///
/// The integration systems run in [`SubstepSet::Integrate`].
pub struct IntegratorPlugin;

impl Plugin for IntegratorPlugin {
    fn build(&self, app: &mut App) {
        app.get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first")
            .add_systems(
                (
                    integrate_pos_2d,
                    integrate_pos_3d,
                    integrate_rot_2d,
                    integrate_rot_3d,
                )
                    .in_set(SubstepSet::Integrate),
            );
        app.get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first")
            .add_systems(
                (apply_impulses_2d, apply_impulses_3d)
                    .after(PhysicsStepSet::BroadPhase)
                    .before(PhysicsStepSet::Substeps),
            )
            .add_systems(
                (clear_forces_and_impulses_2d, clear_forces_and_impulses_3d)
                    .after(PhysicsStepSet::SpatialQuery),
            );

        app.get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first")
            .add_systems(
                (apply_translation_2d, apply_translation_3d).in_set(SubstepSet::ApplyTranslation),
            );
    }
}

/// Explicitly integrates the positions and linear velocities of bodies taking only external forces
/// like gravity into account. This acts as a prediction for the next positions of the bodies.
fn integrate_pos_2d(
    mut bodies: Query<
        (
            &RigidBody2d,
            &mut Position2d,
            &mut PreviousPosition2d,
            &mut AccumulatedTranslation2d,
            &mut LinearVelocity2d,
            Option<&LinearDamping>,
            Option<&GravityScale>,
            &ExternalForce2d,
            &Mass,
            &InverseMass,
            Option<&LockedAxes2d>,
        ),
        Without<Sleeping>,
    >,
    gravity: Res<Gravity>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (
        rb,
        pos,
        mut prev_pos,
        mut translation,
        mut lin_vel,
        lin_damping,
        gravity_scale,
        external_force,
        mass,
        inv_mass,
        locked_axes,
    ) in &mut bodies
    {
        prev_pos.0 = pos.0;

        if rb.is_static() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes2d::default(), |locked_axes| *locked_axes);

        // Apply damping, gravity and other external forces
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = lin_damping {
                lin_vel.0 *= 1.0 / (1.0 + delta_secs * damping.0);
            }

            let effective_mass = locked_axes.apply_to_vec(Vector2::splat(mass.0));
            let effective_inv_mass = locked_axes.apply_to_vec(Vector2::splat(inv_mass.0));

            // Apply forces
            let gravitation_force = effective_mass
                * Vector2::from_math(gravity.0)
                * gravity_scale.map_or(1.0, |scale| scale.0);
            let external_forces = gravitation_force + external_force.force();
            let delta_lin_vel = delta_secs * external_forces * effective_inv_mass;
            // avoid triggering bevy's change detection unnecessarily
            if delta_lin_vel != Vector2::ZERO {
                lin_vel.0 += delta_lin_vel;
            }
        }
        if lin_vel.0 != Vector2::ZERO {
            translation.0 += locked_axes.apply_to_vec(delta_secs * lin_vel.0);
        }
    }
}

/// Explicitly integrates the positions and linear velocities of bodies taking only external forces
/// like gravity into account. This acts as a prediction for the next positions of the bodies.
fn integrate_pos_3d(
    mut bodies: Query<
        (
            &RigidBody3d,
            &mut Position3d,
            &mut PreviousPosition3d,
            &mut AccumulatedTranslation3d,
            &mut LinearVelocity3d,
            Option<&LinearDamping>,
            Option<&GravityScale>,
            &ExternalForce3d,
            &Mass,
            &InverseMass,
            Option<&LockedAxes3d>,
        ),
        Without<Sleeping>,
    >,
    gravity: Res<Gravity>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (
        rb,
        pos,
        mut prev_pos,
        mut translation,
        mut lin_vel,
        lin_damping,
        gravity_scale,
        external_force,
        mass,
        inv_mass,
        locked_axes,
    ) in &mut bodies
    {
        prev_pos.0 = pos.0;

        if rb.is_static() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes3d::default(), |locked_axes| *locked_axes);

        // Apply damping, gravity and other external forces
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = lin_damping {
                lin_vel.0 *= 1.0 / (1.0 + delta_secs * damping.0);
            }

            let effective_mass = locked_axes.apply_to_vec(Vector3::splat(mass.0));
            let effective_inv_mass = locked_axes.apply_to_vec(Vector3::splat(inv_mass.0));

            // Apply forces
            let gravitation_force =
                effective_mass * gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0);
            let external_forces = gravitation_force + external_force.force();
            let delta_lin_vel = delta_secs * external_forces * effective_inv_mass;
            // avoid triggering bevy's change detection unnecessarily
            if delta_lin_vel != Vector3::ZERO {
                lin_vel.0 += delta_lin_vel;
            }
        }
        if lin_vel.0 != Vector3::ZERO {
            translation.0 += locked_axes.apply_to_vec(delta_secs * lin_vel.0);
        }
    }
}

/// Explicitly integrates the rotations and angular velocities of bodies taking only external torque into account.
/// This acts as a prediction for the next rotations of the bodies.
#[cfg(feature = "2d")]
fn integrate_rot_2d(
    mut bodies: Query<
        (
            &RigidBody2d,
            &mut Rotation2d,
            &mut PreviousRotation2d,
            &mut AngularVelocity2d,
            Option<&AngularDamping>,
            &ExternalForce2d,
            &ExternalTorque2d,
            &Inertia2d,
            &InverseInertia2d,
            Option<&LockedAxes2d>,
        ),
        Without<Sleeping>,
    >,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (
        rb,
        mut rot,
        mut prev_rot,
        mut ang_vel,
        ang_damping,
        external_force,
        external_torque,
        _inertia,
        inv_inertia,
        locked_axes,
    ) in &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes2d::default(), |locked_axes| *locked_axes);

        // Apply damping and external torque
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = ang_damping {
                // avoid triggering bevy's change detection unnecessarily
                if ang_vel.0 != 0.0 && damping.0 != 0.0 {
                    ang_vel.0 *= 1.0 / (1.0 + delta_secs * damping.0);
                }
            }

            let effective_inv_inertia = locked_axes.apply_to_rotation(inv_inertia.0);

            // Apply external torque
            let delta_ang_vel = delta_secs
                * effective_inv_inertia
                * (external_torque.torque() + external_force.torque());
            // avoid triggering bevy's change detection unnecessarily
            if delta_ang_vel != 0.0 {
                ang_vel.0 += delta_ang_vel;
            }
        }
        // avoid triggering bevy's change detection unnecessarily
        let delta = locked_axes.apply_to_angular_velocity(delta_secs * ang_vel.0);
        if delta != 0.0 {
            *rot += Rotation2d::from_radians(delta);
        }
    }
}

/// Explicitly integrates the rotations and angular velocities of bodies taking only external torque into account.
/// This acts as a prediction for the next rotations of the bodies.
#[cfg(feature = "3d")]
fn integrate_rot_3d(
    mut bodies: Query<
        (
            &RigidBody3d,
            &mut Rotation3d,
            &mut PreviousRotation3d,
            &mut AngularVelocity3d,
            Option<&AngularDamping>,
            &ExternalForce3d,
            &ExternalTorque3d,
            &Inertia3d,
            &InverseInertia3d,
            Option<&LockedAxes3d>,
        ),
        Without<Sleeping>,
    >,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (
        rb,
        mut rot,
        mut prev_rot,
        mut ang_vel,
        ang_damping,
        external_force,
        external_torque,
        inertia,
        inv_inertia,
        locked_axes,
    ) in &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes3d::default(), |locked_axes| *locked_axes);

        // Apply damping and external torque
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = ang_damping {
                // avoid triggering bevy's change detection unnecessarily
                if ang_vel.0 != Vector3::ZERO && damping.0 != 0.0 {
                    ang_vel.0 *= 1.0 / (1.0 + delta_secs * damping.0);
                }
            }

            let effective_inertia = locked_axes.apply_to_rotation(inertia.rotated(&rot).0);
            let effective_inv_inertia = locked_axes.apply_to_rotation(inv_inertia.rotated(&rot).0);

            // Apply external torque
            let delta_ang_vel = delta_secs
                * effective_inv_inertia
                * ((external_torque.torque() + external_force.torque())
                    - ang_vel.0.cross(effective_inertia * ang_vel.0));
            // avoid triggering bevy's change detection unnecessarily
            if delta_ang_vel != Vector3::ZERO {
                ang_vel.0 += delta_ang_vel;
            }
        }

        let q = Quaternion::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
        let effective_dq = locked_axes
            .apply_to_angular_velocity(delta_secs * 0.5 * q.xyz())
            .extend(delta_secs * 0.5 * q.w);
        // avoid triggering bevy's change detection unnecessarily
        let delta = Quaternion::from_vec4(effective_dq);
        if delta != Quaternion::IDENTITY {
            rot.0 = (rot.0 + delta).normalize();
        }
    }
}

fn apply_impulses_2d(
    mut bodies: Query<
        (
            &RigidBody2d,
            &mut ExternalImpulse2d,
            &mut ExternalAngularImpulse2d,
            &mut LinearVelocity2d,
            &mut AngularVelocity2d,
            &InverseMass,
            &InverseInertia2d,
            Option<&LockedAxes2d>,
        ),
        Without<Sleeping>,
    >,
) {
    for (rb, impulse, ang_impulse, mut lin_vel, mut ang_vel, inv_mass, inv_inertia, locked_axes) in
        &mut bodies
    {
        if !rb.is_dynamic() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes2d::default(), |locked_axes| *locked_axes);

        let effective_inv_mass = locked_axes.apply_to_vec(Vector2::splat(inv_mass.0));
        let effective_inv_inertia = locked_axes.apply_to_rotation(inv_inertia.0);

        // avoid triggering bevy's change detection unnecessarily
        let delta_lin_vel = impulse.impulse() * effective_inv_mass;
        let delta_ang_vel =
            effective_inv_inertia * (ang_impulse.impulse() + impulse.angular_impulse());

        if delta_lin_vel != Vector2::ZERO {
            lin_vel.0 += delta_lin_vel;
        }
        if delta_ang_vel != AngularVelocity2d::ZERO.0 {
            ang_vel.0 += delta_ang_vel;
        }
    }
}

fn apply_impulses_3d(
    mut bodies: Query<
        (
            &RigidBody3d,
            &mut ExternalImpulse3d,
            &mut ExternalAngularImpulse3d,
            &mut LinearVelocity3d,
            &mut AngularVelocity3d,
            &Rotation3d,
            &InverseMass,
            &InverseInertia3d,
            Option<&LockedAxes3d>,
        ),
        Without<Sleeping>,
    >,
) {
    for (
        rb,
        impulse,
        ang_impulse,
        mut lin_vel,
        mut ang_vel,
        rotation,
        inv_mass,
        inv_inertia,
        locked_axes,
    ) in &mut bodies
    {
        if !rb.is_dynamic() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes3d::default(), |locked_axes| *locked_axes);

        let effective_inv_mass = locked_axes.apply_to_vec(Vector3::splat(inv_mass.0));
        let effective_inv_inertia = locked_axes.apply_to_rotation(inv_inertia.rotated(rotation).0);

        // avoid triggering bevy's change detection unnecessarily
        let delta_lin_vel = impulse.impulse() * effective_inv_mass;
        let delta_ang_vel =
            effective_inv_inertia * (ang_impulse.impulse() + impulse.angular_impulse());

        if delta_lin_vel != Vector3::ZERO {
            lin_vel.0 += delta_lin_vel;
        }
        if delta_ang_vel != AngularVelocity3d::ZERO.0 {
            ang_vel.0 += delta_ang_vel;
        }
    }
}

fn clear_forces_and_impulses_2d(
    mut forces: Query<
        (
            &mut ExternalForce2d,
            &mut ExternalTorque2d,
            &mut ExternalImpulse2d,
            &mut ExternalAngularImpulse2d,
        ),
        Or<(
            Changed<ExternalForce2d>,
            Changed<ExternalTorque2d>,
            Changed<ExternalImpulse2d>,
            Changed<ExternalAngularImpulse2d>,
        )>,
    >,
) {
    for (mut force, mut torque, mut impulse, mut angular_ímpulse) in &mut forces {
        if !force.persistent {
            force.clear();
        }
        if !torque.persistent {
            torque.clear();
        }
        if !impulse.persistent {
            impulse.clear();
        }
        if !angular_ímpulse.persistent {
            angular_ímpulse.clear();
        }
    }
}

fn clear_forces_and_impulses_3d(
    mut forces: Query<
        (
            &mut ExternalForce3d,
            &mut ExternalTorque3d,
            &mut ExternalImpulse3d,
            &mut ExternalAngularImpulse3d,
        ),
        Or<(
            Changed<ExternalForce3d>,
            Changed<ExternalTorque3d>,
            Changed<ExternalImpulse3d>,
            Changed<ExternalAngularImpulse3d>,
        )>,
    >,
) {
    for (mut force, mut torque, mut impulse, mut angular_ímpulse) in &mut forces {
        if !force.persistent {
            force.clear();
        }
        if !torque.persistent {
            torque.clear();
        }
        if !impulse.persistent {
            impulse.clear();
        }
        if !angular_ímpulse.persistent {
            angular_ímpulse.clear();
        }
    }
}

fn apply_translation_2d(
    mut bodies: Query<
        (&RigidBody2d, &mut Position2d, &mut AccumulatedTranslation2d),
        Changed<AccumulatedTranslation2d>,
    >,
) {
    for (rb, mut pos, mut translation) in &mut bodies {
        if rb.is_static() {
            continue;
        }

        pos.0 += translation.0;
        translation.0 = Vector2::ZERO;
    }
}

fn apply_translation_3d(
    mut bodies: Query<
        (&RigidBody3d, &mut Position3d, &mut AccumulatedTranslation3d),
        Changed<AccumulatedTranslation3d>,
    >,
) {
    for (rb, mut pos, mut translation) in &mut bodies {
        if rb.is_static() {
            continue;
        }

        pos.0 += translation.0;
        translation.0 = Vector3::ZERO;
    }
}
