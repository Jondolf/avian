use crate::{
    narrow_phase::{Collision, Collisions},
    prelude::*,
    utils::{get_dynamic_friction, get_restitution},
};
use bevy::prelude::*;
use constraints::penetration::PenetrationConstraint;

pub struct SolverPlugin;

impl Plugin for SolverPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PenetrationConstraints>()
            .add_system_set_to_stage(
                FixedUpdateStage,
                SystemSet::new()
                    .before(PhysicsStep::SolvePos)
                    .with_system(clear_penetration_constraint_lagrange)
                    .with_system(clear_joint_lagrange::<FixedJoint>)
                    .with_system(clear_joint_lagrange::<RevoluteJoint>)
                    .with_system(clear_joint_lagrange::<SphericalJoint>)
                    .with_system(clear_joint_lagrange::<PrismaticJoint>),
            )
            .add_system_set_to_stage(
                FixedUpdateStage,
                SystemSet::new()
                    .label(PhysicsStep::SolvePos)
                    .after(PhysicsStep::NarrowPhase)
                    .with_system(solve_pos)
                    .with_system(joint_constraints::<FixedJoint>)
                    .with_system(joint_constraints::<RevoluteJoint>)
                    .with_system(joint_constraints::<SphericalJoint>)
                    .with_system(joint_constraints::<PrismaticJoint>),
            )
            .add_system_set_to_stage(
                FixedUpdateStage,
                SystemSet::new()
                    .label(PhysicsStep::UpdateVel)
                    .after(PhysicsStep::SolvePos)
                    .with_system(update_lin_vel)
                    .with_system(update_ang_vel),
            )
            .add_system_set_to_stage(
                FixedUpdateStage,
                SystemSet::new()
                    .label(PhysicsStep::SolveVel)
                    .after(PhysicsStep::UpdateVel)
                    .with_system(solve_vel)
                    .with_system(joint_damping::<FixedJoint>)
                    .with_system(joint_damping::<RevoluteJoint>)
                    .with_system(joint_damping::<SphericalJoint>)
                    .with_system(joint_damping::<PrismaticJoint>),
            );
    }
}

/// Stores penetration constraints for colliding entity pairs.
#[derive(Resource, Debug, Default)]
pub(crate) struct PenetrationConstraints(pub Vec<PenetrationConstraint>);

fn clear_penetration_constraint_lagrange(
    mut penetration_constraints: ResMut<PenetrationConstraints>,
) {
    for constraint in &mut penetration_constraints.0 {
        constraint.clear_lagrange_multipliers();
    }
}

fn clear_joint_lagrange<T: Joint>(mut joints: Query<&mut T>) {
    for mut joint in &mut joints {
        joint.clear_lagrange_multipliers();
    }
}

/// Solves position constraints for dynamic-dynamic interactions.
fn solve_pos(
    mut bodies: Query<RigidBodyQuery>,
    collisions: Res<Collisions>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    sub_dt: Res<SubDeltaTime>,
) {
    penetration_constraints.0.clear();

    // Handle non-penetration constraints
    for ((ent_a, ent_b), collision) in collisions.0.iter() {
        if let Ok([mut body1, mut body2]) = bodies.get_many_mut([*ent_a, *ent_b]) {
            // No need to solve collisions if neither of the bodies is dynamic
            if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                continue;
            }

            let mut constraint = PenetrationConstraint::new(*ent_a, *ent_b, *collision);
            constraint.constrain(&mut body1, &mut body2, sub_dt.0);
            penetration_constraints.0.push(constraint);
        }
    }
}

fn joint_constraints<T: Joint>(
    mut bodies: Query<RigidBodyQuery>,
    mut joints: Query<&mut T, Without<Pos>>,
    num_pos_iters: Res<NumPosIters>,
    sub_dt: Res<SubDeltaTime>,
) {
    for _j in 0..num_pos_iters.0 {
        for mut joint in &mut joints {
            // Get components for entity a and b
            if let Ok([mut body1, mut body2]) = bodies.get_many_mut(joint.entities()) {
                // No need to solve constraints if neither of the bodies is dynamic
                if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                    continue;
                }

                joint.constrain(&mut body1, &mut body2, sub_dt.0);
            }
        }
    }
}

/// Updates the linear velocity of all dynamic bodies based on the change in position from the previous step.
fn update_lin_vel(
    mut bodies: Query<(&RigidBody, &Pos, &PrevPos, &mut LinVel, &mut PreSolveLinVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, pos, prev_pos, mut lin_vel, mut pre_solve_lin_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() {
            pre_solve_lin_vel.0 = Vector::ZERO;
            lin_vel.0 = Vector::ZERO;
            continue;
        }

        pre_solve_lin_vel.0 = lin_vel.0;
        // v = (x - x_prev) / h
        lin_vel.0 = (pos.0 - prev_pos.0) / sub_dt.0;
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the previous step.
#[cfg(feature = "2d")]
fn update_ang_vel(
    mut bodies: Query<(&RigidBody, &Rot, &PrevRot, &mut AngVel, &mut PreSolveAngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() {
            pre_solve_ang_vel.0 = 0.0;
            ang_vel.0 = 0.0;
            continue;
        }

        pre_solve_ang_vel.0 = ang_vel.0;
        ang_vel.0 = (rot.mul(prev_rot.inv())).as_radians() / sub_dt.0;
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the previous step.
#[cfg(feature = "3d")]
fn update_ang_vel(
    mut bodies: Query<(&RigidBody, &Rot, &PrevRot, &mut AngVel, &mut PreSolveAngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() {
            pre_solve_ang_vel.0 = Vec3::ZERO;
            ang_vel.0 = Vec3::ZERO;
            continue;
        }

        pre_solve_ang_vel.0 = ang_vel.0;

        let delta_rot = rot.mul_quat(prev_rot.inverse());
        ang_vel.0 = 2.0 * delta_rot.xyz() / sub_dt.0;

        if delta_rot.w < 0.0 {
            ang_vel.0 = -ang_vel.0;
        }
    }
}

/// Solves the new velocities of dynamic bodies after dynamic-dynamic collisions.
#[allow(clippy::type_complexity)]
fn solve_vel(
    mut bodies: Query<RigidBodyQuery>,
    penetration_constraints: Res<PenetrationConstraints>,
    gravity: Res<Gravity>,
    sub_dt: Res<SubDeltaTime>,
) {
    for constraint in penetration_constraints.0.iter().cloned() {
        let Collision {
            entity_a,
            entity_b,
            world_r_a: r_a,
            world_r_b: r_b,
            normal,
            ..
        } = constraint.collision_data;

        if let Ok([mut body1, mut body2]) = bodies.get_many_mut([entity_a, entity_b]) {
            if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                continue;
            }

            // Compute pre-solve relative normal velocities at the contact point (used for restitution)
            let pre_solve_contact_vel_a =
                compute_contact_vel(body1.pre_solve_lin_vel.0, body1.pre_solve_ang_vel.0, r_a);
            let pre_solve_contact_vel_b =
                compute_contact_vel(body2.pre_solve_lin_vel.0, body2.pre_solve_ang_vel.0, r_b);
            let pre_solve_relative_vel = pre_solve_contact_vel_a - pre_solve_contact_vel_b;
            let pre_solve_normal_vel = normal.dot(pre_solve_relative_vel);

            // Compute relative normal and tangential velocities at the contact point (equation 29)
            let contact_vel_a = compute_contact_vel(body1.lin_vel.0, body1.ang_vel.0, r_a);
            let contact_vel_b = compute_contact_vel(body2.lin_vel.0, body2.ang_vel.0, r_b);
            let relative_vel = contact_vel_a - contact_vel_b;
            let normal_vel = normal.dot(relative_vel);
            let tangent_vel = relative_vel - normal * normal_vel;

            let inv_inertia1 = body1.inv_inertia.rotated(&body1.rot).0;
            let inv_inertia2 = body2.inv_inertia.rotated(&body2.rot).0;

            // Compute generalized inverse masses
            let w_a = PenetrationConstraint::get_generalized_inverse_mass(
                body1.rb,
                body1.inv_mass.0,
                inv_inertia1,
                r_a,
                normal,
            );
            let w_b = PenetrationConstraint::get_generalized_inverse_mass(
                body2.rb,
                body2.inv_mass.0,
                inv_inertia2,
                r_b,
                normal,
            );

            // Compute dynamic friction
            let friction_impulse = get_dynamic_friction(
                tangent_vel,
                body1.friction,
                body2.friction,
                constraint.normal_lagrange,
                sub_dt.0,
            );

            // Compute restitution
            let restitution_impulse = get_restitution(
                normal,
                normal_vel,
                pre_solve_normal_vel,
                body1.restitution,
                body2.restitution,
                gravity.0,
                sub_dt.0,
            );

            let delta_v = friction_impulse + restitution_impulse;

            // Compute velocity impulse and apply velocity updates (equation 33)
            let p = delta_v / (w_a + w_b);
            if body1.rb.is_dynamic() {
                body1.lin_vel.0 += p / body1.mass.0;
                body1.ang_vel.0 += compute_delta_ang_vel(inv_inertia1, r_a, p);
            }
            if body2.rb.is_dynamic() {
                body2.lin_vel.0 -= p / body2.mass.0;
                body2.ang_vel.0 -= compute_delta_ang_vel(inv_inertia2, r_b, p);
            }
        }
    }
}

fn joint_damping<T: Joint>(
    mut bodies: Query<(&RigidBody, &mut LinVel, &mut AngVel, &InvMass)>,
    joints: Query<&T, Without<RigidBody>>,
    sub_dt: Res<SubDeltaTime>,
) {
    for joint in &joints {
        if let Ok(
            [(rb_a, mut lin_vel_a, mut ang_vel_a, inv_mass_a), (rb_b, mut lin_vel_b, mut ang_vel_b, inv_mass_b)],
        ) = bodies.get_many_mut(joint.entities())
        {
            let delta_omega =
                (ang_vel_b.0 - ang_vel_a.0) * (joint.damping_ang() * sub_dt.0).min(1.0);

            if rb_a.is_dynamic() {
                ang_vel_a.0 += delta_omega;
            }
            if rb_b.is_dynamic() {
                ang_vel_b.0 -= delta_omega;
            }

            let delta_v = (lin_vel_b.0 - lin_vel_a.0) * (joint.damping_lin() * sub_dt.0).min(1.0);

            let w_a = if rb_a.is_dynamic() { inv_mass_a.0 } else { 0.0 };
            let w_b = if rb_b.is_dynamic() { inv_mass_b.0 } else { 0.0 };

            if w_a + w_b <= f32::EPSILON {
                continue;
            }

            let p = delta_v / (w_a + w_b);

            if rb_a.is_dynamic() {
                lin_vel_a.0 += p * inv_mass_a.0;
            }
            if rb_b.is_dynamic() {
                lin_vel_b.0 -= p * inv_mass_b.0;
            }
        }
    }
}

#[cfg(feature = "2d")]
fn compute_contact_vel(lin_vel: Vector, ang_vel: f32, r: Vector) -> Vector {
    lin_vel + ang_vel * r.perp()
}

#[cfg(feature = "3d")]
fn compute_contact_vel(lin_vel: Vector, ang_vel: Vector, r: Vector) -> Vector {
    lin_vel + ang_vel.cross(r)
}

#[cfg(feature = "2d")]
fn compute_delta_ang_vel(inv_inertia: f32, r: Vector, p: Vector) -> f32 {
    inv_inertia * r.perp_dot(p)
}

#[cfg(feature = "3d")]
fn compute_delta_ang_vel(inv_inertia: Mat3, r: Vector, p: Vector) -> Vector {
    inv_inertia * r.cross(p)
}
