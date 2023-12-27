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
                apply_impulses
                    .after(PhysicsStepSet::BroadPhase)
                    .before(PhysicsStepSet::Substeps),
            )
            .add_systems(clear_forces_and_impulses.after(PhysicsStepSet::SpatialQuery));
    }
}

type PosIntegrationComponents = (
    &'static RigidBody,
    &'static Position,
    &'static mut PreviousPosition,
    &'static mut AccumulatedTranslation,
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

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        // Apply damping, gravity and other external forces
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = lin_damping {
                lin_vel.0 *= 1.0 / (1.0 + delta_secs * damping.0);
            }

            let effective_mass = locked_axes.apply_to_vec(Vector::splat(mass.0));
            let effective_inv_mass = locked_axes.apply_to_vec(Vector::splat(inv_mass.0));

            // Apply forces
            let gravitation_force =
                effective_mass * gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0);
            let external_forces = gravitation_force + external_force.force();
            let delta_lin_vel = delta_secs * external_forces * effective_inv_mass;
            // avoid triggering bevy's change detection unnecessarily
            if delta_lin_vel != Vector::ZERO {
                lin_vel.0 += delta_lin_vel;
            }
        }
        if lin_vel.0 != Vector::ZERO {
            translation.0 += locked_axes.apply_to_vec(delta_secs * lin_vel.0);
        }
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
fn integrate_rot(mut bodies: Query<RotIntegrationComponents, Without<Sleeping>>, time: Res<Time>) {
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

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

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
            *rot += Rotation::from_radians(delta);
        }
    }
}

/// Explicitly integrates the rotations and angular velocities of bodies taking only external torque into account.
/// This acts as a prediction for the next rotations of the bodies.
#[cfg(feature = "3d")]
fn integrate_rot(mut bodies: Query<RotIntegrationComponents, Without<Sleeping>>, time: Res<Time>) {
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

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        // Apply damping and external torque
        if rb.is_dynamic() {
            // Apply damping
            if let Some(damping) = ang_damping {
                // avoid triggering bevy's change detection unnecessarily
                if ang_vel.0 != Vector::ZERO && damping.0 != 0.0 {
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
            if delta_ang_vel != Vector::ZERO {
                ang_vel.0 += delta_ang_vel;
            }
        }

        let q = Quaternion::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
        let effective_dq = locked_axes
            .apply_to_angular_velocity(delta_secs * 0.5 * q.xyz())
            .extend(delta_secs * 0.5 * q.w);
        // avoid triggering bevy's change detection unnecessarily
        let delta = Quaternion::from_vec4(effective_dq);
        if delta != Quaternion::from_xyzw(0.0, 0.0, 0.0, 0.0) {
            rot.0 = (rot.0 + delta).normalize();
        }
    }
}

type ImpulseQueryComponents = (
    &'static RigidBody,
    &'static mut ExternalImpulse,
    &'static mut ExternalAngularImpulse,
    &'static mut LinearVelocity,
    &'static mut AngularVelocity,
    &'static Rotation,
    &'static InverseMass,
    &'static InverseInertia,
    Option<&'static LockedAxes>,
);

fn apply_impulses(mut bodies: Query<ImpulseQueryComponents, Without<Sleeping>>) {
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

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        let effective_inv_mass = locked_axes.apply_to_vec(Vector::splat(inv_mass.0));
        let effective_inv_inertia = locked_axes.apply_to_rotation(inv_inertia.rotated(rotation).0);

        // avoid triggering bevy's change detection unnecessarily
        let delta_lin_vel = impulse.impulse() * effective_inv_mass;
        let delta_ang_vel =
            effective_inv_inertia * (ang_impulse.impulse() + impulse.angular_impulse());

        if delta_lin_vel != Vector::ZERO {
            lin_vel.0 += delta_lin_vel;
        }
        if delta_ang_vel != AngularVelocity::ZERO.0 {
            ang_vel.0 += delta_ang_vel;
        }
    }
}

type ForceComponents = (
    &'static mut ExternalForce,
    &'static mut ExternalTorque,
    &'static mut ExternalImpulse,
    &'static mut ExternalAngularImpulse,
);
type ForceComponentsChanged = Or<(
    Changed<ExternalForce>,
    Changed<ExternalTorque>,
    Changed<ExternalImpulse>,
    Changed<ExternalAngularImpulse>,
)>;

/// Responsible for clearing forces and impulses on bodies.
///
/// Runs in [`PhysicsSchedule`], after [`PhysicsStepSet::SpatialQuery`].
pub fn clear_forces_and_impulses(mut forces: Query<ForceComponents, ForceComponentsChanged>) {
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
