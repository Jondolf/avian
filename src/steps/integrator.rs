use crate::prelude::*;
use bevy::prelude::*;

pub struct IntegratorPlugin;

impl Plugin for IntegratorPlugin {
    fn build(&self, app: &mut App) {
        app.get_schedule_mut(XpbdSubstepSchedule)
            .expect("add XpbdSubstepSchedule first")
            .add_systems((integrate_pos, integrate_rot).in_set(PhysicsSubstep::Integrate));
    }
}

/// Applies forces and predicts the next position and velocity for all dynamic bodies.
fn integrate_pos(
    mut bodies: Query<(
        &RigidBody,
        &mut Pos,
        &mut PrevPos,
        &mut LinVel,
        &ExternalForce,
        &Mass,
    )>,
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

/// Integrates rotations for all dynamic bodies.
#[cfg(feature = "2d")]
fn integrate_rot(
    mut bodies: Query<(
        &RigidBody,
        &mut Rot,
        &mut PrevRot,
        &mut AngVel,
        &ExternalTorque,
        &InvInertia,
    )>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, mut rot, mut prev_rot, mut ang_vel, external_torque, inv_inertia) in &mut bodies {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        // Apply external torque
        if rb.is_dynamic() {
            ang_vel.0 += sub_dt.0 * inv_inertia.0 * external_torque.0;
        }

        *rot += Rot::from_radians(sub_dt.0 * ang_vel.0);
    }
}

/// Integrates rotations for all dynamic bodies.
#[cfg(feature = "3d")]
fn integrate_rot(
    mut bodies: Query<(
        &RigidBody,
        &mut Rot,
        &mut PrevRot,
        &mut AngVel,
        &ExternalTorque,
        &Inertia,
        &InvInertia,
    )>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, mut rot, mut prev_rot, mut ang_vel, external_torque, inertia, inv_inertia) in
        &mut bodies
    {
        prev_rot.0 = *rot;

        if rb.is_static() {
            continue;
        }

        // Apply external torque
        if rb.is_dynamic() {
            let delta_ang_vel = sub_dt.0
                * inv_inertia.rotated(&rot).0
                * (external_torque.0 - ang_vel.cross(inertia.rotated(&rot).0 * ang_vel.0));
            ang_vel.0 += delta_ang_vel;
        }

        let q = Quat::from_vec4(ang_vel.extend(0.0)) * rot.0;
        let (x, y, z, w) = (
            rot.x + sub_dt.0 * 0.5 * q.x,
            rot.y + sub_dt.0 * 0.5 * q.y,
            rot.z + sub_dt.0 * 0.5 * q.z,
            rot.w + sub_dt.0 * 0.5 * q.w,
        );
        rot.0 = Quat::from_xyzw(x, y, z, w).normalize();
    }
}
