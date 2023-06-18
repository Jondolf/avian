//! The integrator explicitly integrates the positions and velocities of bodies. See [`IntegratorPlugin`].

use crate::prelude::*;
use bevy::prelude::*;

/// The `IntegratorPlugin` explicitly integrates the positions and velocities of bodies taking only external forces like gravity into account. This acts as a prediction for the next positions and orientations of the bodies.
pub struct IntegratorPlugin;

impl Plugin for IntegratorPlugin {
    fn build(&self, app: &mut App) {
        app.get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first")
            .add_systems((integrate_pos, integrate_rot).in_set(SubstepSet::Integrate));
    }
}

type PosIntegrationComponents = (
    &'static RigidBody,
    &'static mut Position,
    &'static mut PreviousPosition,
    &'static mut LinearVelocity,
    &'static ExternalForce,
    &'static Mass,
);

/// Explicitly integrates the positions and linear velocities of bodies taking only external forces like gravity into account. This acts as a prediction for the next positions of the bodies.
fn integrate_pos(
    mut bodies: Query<PosIntegrationComponents, Without<Sleeping>>,
    gravity: Res<Gravity>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, mut pos, mut prev_pos, mut lin_vel, external_force, mass) in &mut bodies {
        prev_pos.0 = pos.0;

        if rb.is_static() {
            continue;
        }

        // Apply gravity and other external forces
        if rb.is_dynamic() {
            let gravitation_force = mass.0 * gravity.0;
            let external_forces = gravitation_force + external_force.0;
            lin_vel.0 += sub_dt.0 * external_forces / mass.0;
        }

        pos.0 += sub_dt.0 * lin_vel.0;
    }
}

type RotIntegrationComponents = (
    &'static RigidBody,
    &'static mut Rotation,
    &'static mut PreviousRotation,
    &'static mut AngularVelocity,
    &'static ExternalTorque,
    &'static Inertia,
    &'static InverseInertia,
);

/// Explicitly integrates the rotations and angular velocities of bodies taking only external torque into account. This acts as a prediction for the next rotations of the bodies.
#[cfg(feature = "2d")]
fn integrate_rot(
    mut bodies: Query<RotIntegrationComponents, Without<Sleeping>>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, mut rot, mut prev_rot, mut ang_vel, external_torque, _inertia, inverse_inertia) in
        &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        // Apply external torque
        if rb.is_dynamic() {
            ang_vel.0 += sub_dt.0 * inverse_inertia.0 * external_torque.0;
        }

        *rot += Rotation::from_radians(sub_dt.0 * ang_vel.0);
    }
}

/// Explicitly integrates the rotations and angular velocities of bodies taking only external torque into account. This acts as a prediction for the next rotations of the bodies.
#[cfg(feature = "3d")]
fn integrate_rot(
    mut bodies: Query<RotIntegrationComponents, Without<Sleeping>>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, mut rot, mut prev_rot, mut ang_vel, external_torque, inertia, inverse_inertia) in
        &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        // Apply external torque
        if rb.is_dynamic() {
            let delta_ang_vel = sub_dt.0
                * inverse_inertia.rotated(&rot).0
                * (external_torque.0 - ang_vel.0.cross(inertia.rotated(&rot).0 * ang_vel.0));
            ang_vel.0 += delta_ang_vel;
        }

        let q = Quaternion::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
        let (x, y, z, w) = (
            rot.x + sub_dt.0 * 0.5 * q.x,
            rot.y + sub_dt.0 * 0.5 * q.y,
            rot.z + sub_dt.0 * 0.5 * q.z,
            rot.w + sub_dt.0 * 0.5 * q.w,
        );
        rot.0 = Quaternion::from_xyzw(x, y, z, w).normalize();
    }
}
