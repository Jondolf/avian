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
                    .after(PhysicsSet::Substeps)
                    .before(PhysicsSet::Sleeping),
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
    &'static InverseMass,
    Option<&'static LockedAxes>,
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
        inv_mass,
        locked_axes,
    ) in &mut bodies
    {
        prev_pos.0 = pos.0;

        if rb.is_static() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        // Apply damping, gravity and other external forces
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = lin_damping {
                lin_vel.0 *= 1.0 / (1.0 + sub_dt.0 * damping.0);
            }

            let mass = locked_axes.apply_to_vec(Vector::splat(mass.0));
            let effective_inv_mass = locked_axes.apply_to_vec(Vector::splat(inv_mass.0));

            // Apply forces
            let gravitation_force = mass * gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0);
            let external_forces = gravitation_force + external_force.force();
            lin_vel.0 += sub_dt.0 * external_forces * effective_inv_mass;
        }

        pos.0 += locked_axes.apply_to_vec(sub_dt.0 * lin_vel.0);
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
    Option<&'static LockedAxes>,
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
        inv_inertia,
        locked_axes,
    ) in &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        // Apply damping and external torque
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = ang_damping {
                ang_vel.0 *= 1.0 / (1.0 + sub_dt.0 * damping.0);
            }

            let effective_inv_inertia = locked_axes.apply_to_rotation(inv_inertia.0);

            // Apply external torque
            ang_vel.0 += sub_dt.0
                * effective_inv_inertia
                * (external_torque.torque + external_force.torque());
        }

        *rot += Rotation::from_radians(locked_axes.apply_to_rotation(sub_dt.0 * ang_vel.0));
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
        inv_inertia,
        locked_axes,
    ) in &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        // Apply damping and external torque
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = ang_damping {
                ang_vel.0 *= 1.0 / (1.0 + sub_dt.0 * damping.0);
            }

            let effective_inertia = locked_axes.apply_to_rotation(inertia.rotated(&rot).0);
            let effective_inv_inertia = locked_axes.apply_to_rotation(inv_inertia.rotated(&rot).0);

            // Apply external torque
            let delta_ang_vel = sub_dt.0
                * effective_inv_inertia
                * ((external_torque.torque + external_force.torque())
                    - ang_vel.0.cross(effective_inertia * ang_vel.0));
            ang_vel.0 += delta_ang_vel;
        }

        let q = Quaternion::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
        let effective_dq = locked_axes
            .apply_to_vec(sub_dt.0 * 0.5 * q.xyz())
            .extend(sub_dt.0 * 0.5 * q.w);
        rot.0 = (rot.0 + Quaternion::from_vec4(effective_dq)).normalize();
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
