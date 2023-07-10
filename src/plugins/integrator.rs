//! Integrates Newton's 2nd law of motion, applying forces and moving entities according to their velocities.
//!
//! See [`IntegratorPlugin`].

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
            .add_systems((integrate_pos, integrate_rot).in_set(SubstepSet::Integrate));
        app.get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first")
            .add_systems(
                clear_external_force_and_torque
                    .after(PhysicsStepSet::Substeps)
                    .before(PhysicsStepSet::Sleeping),
            );
    }
}

type PosIntegrationComponents = (
    &'static RigidBody,
    &'static mut Position,
    &'static mut PreviousPosition,
    &'static mut LinearVelocity,
    Option<&'static LinearDamping>,
    Option<&'static GravityScale>,
    &'static ExternalForce,
    &'static Mass,
);

/// Explicitly integrates the positions and linear velocities of bodies taking only external forces
/// like gravity into account. This acts as a prediction for the next positions of the bodies.
fn integrate_pos(
    mut bodies: Query<PosIntegrationComponents, Without<Sleeping>>,
    gravity: Res<Gravity>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (
        rb,
        mut pos,
        mut prev_pos,
        mut lin_vel,
        lin_damping,
        gravity_scale,
        external_force,
        mass,
    ) in &mut bodies
    {
        prev_pos.0 = pos.0;

        if rb.is_static() {
            continue;
        }

        // Apply damping, gravity and other external forces
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = lin_damping {
                lin_vel.0 *= 1.0 / (1.0 + sub_dt.0 * damping.0);
            }

            // Apply forces
            let gravitation_force = mass.0 * gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0);
            let external_forces = gravitation_force + external_force.force();
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
    Option<&'static AngularDamping>,
    &'static ExternalForce,
    &'static ExternalTorque,
    &'static Inertia,
    &'static InverseInertia,
);

/// Explicitly integrates the rotations and angular velocities of bodies taking only external torque into account.
/// This acts as a prediction for the next rotations of the bodies.
#[cfg(feature = "2d")]
fn integrate_rot(
    mut bodies: Query<RotIntegrationComponents, Without<Sleeping>>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (
        rb,
        mut rot,
        mut prev_rot,
        mut ang_vel,
        ang_damping,
        external_force,
        external_torque,
        _inertia,
        inverse_inertia,
    ) in &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        // Apply damping and external torque
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = ang_damping {
                ang_vel.0 *= 1.0 / (1.0 + sub_dt.0 * damping.0);
            }

            // Apply external torque
            ang_vel.0 +=
                sub_dt.0 * inverse_inertia.0 * (external_torque.torque + external_force.torque());
        }

        *rot += Rotation::from_radians(sub_dt.0 * ang_vel.0);
    }
}

/// Explicitly integrates the rotations and angular velocities of bodies taking only external torque into account.
/// This acts as a prediction for the next rotations of the bodies.
#[cfg(feature = "3d")]
fn integrate_rot(
    mut bodies: Query<RotIntegrationComponents, Without<Sleeping>>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (
        rb,
        mut rot,
        mut prev_rot,
        mut ang_vel,
        ang_damping,
        external_force,
        external_torque,
        inertia,
        inverse_inertia,
    ) in &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        // Apply damping and external torque
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = ang_damping {
                ang_vel.0 *= 1.0 / (1.0 + sub_dt.0 * damping.0);
            }

            // Apply external torque
            let delta_ang_vel = sub_dt.0
                * inverse_inertia.rotated(&rot).0
                * ((external_torque.torque + external_force.torque())
                    - ang_vel.0.cross(inertia.rotated(&rot).0 * ang_vel.0));
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

type ForceComponents = (&'static mut ExternalForce, &'static mut ExternalTorque);
type ForceComponentsChanged = Or<(Changed<ExternalForce>, Changed<ExternalTorque>)>;

fn clear_external_force_and_torque(mut forces: Query<ForceComponents, ForceComponentsChanged>) {
    for (mut force, mut torque) in &mut forces {
        // Clear external force if it's not persistent
        if !force.persistent {
            force.clear();
        }
        // Clear external torque if it's not persistent
        if !torque.persistent {
            torque.clear();
        }
    }
}
