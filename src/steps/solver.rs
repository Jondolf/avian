use crate::{
    collision::*,
    prelude::*,
    steps::broad_phase::BroadCollisionPairs,
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
                    .after(PhysicsStep::Integrate)
                    .with_system(penetration_constraints)
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
pub struct PenetrationConstraints(pub Vec<PenetrationConstraint>);

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

/// Iterates through broad phase collision pairs, checks which ones are actually colliding, and creates penetration constraints for them.
fn penetration_constraints(
    mut bodies: Query<(RigidBodyQuery, &ColliderShape)>,
    broad_collision_pairs: Res<BroadCollisionPairs>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    sub_dt: Res<SubDeltaTime>,
) {
    penetration_constraints.0.clear();

    for (ent1, ent2) in broad_collision_pairs.0.iter() {
        if let Ok([(mut body1, collider_shape1), (mut body2, collider_shape2)]) =
            bodies.get_many_mut([*ent1, *ent2])
        {
            if let Some(collision) = get_collision(
                *ent1,
                *ent2,
                body1.pos.0,
                body2.pos.0,
                body1.local_com.0,
                body2.local_com.0,
                &body1.rot,
                &body2.rot,
                &collider_shape1.0,
                &collider_shape2.0,
            ) {
                let mut constraint = PenetrationConstraint::new(*ent1, *ent2, collision);
                constraint.constrain(&mut body1, &mut body2, sub_dt.0);
                penetration_constraints.0.push(constraint);
            }
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
    for constraint in penetration_constraints.0.iter() {
        let Collision {
            entity1,
            entity2,
            world_r1: r1,
            world_r2: r2,
            normal,
            ..
        } = constraint.collision_data;

        if let Ok([mut body1, mut body2]) = bodies.get_many_mut([entity1, entity2]) {
            if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                continue;
            }

            // Compute pre-solve relative normal velocities at the contact point (used for restitution)
            let pre_solve_contact_vel1 =
                compute_contact_vel(body1.pre_solve_lin_vel.0, body1.pre_solve_ang_vel.0, r1);
            let pre_solve_contact_vel2 =
                compute_contact_vel(body2.pre_solve_lin_vel.0, body2.pre_solve_ang_vel.0, r2);
            let pre_solve_relative_vel = pre_solve_contact_vel1 - pre_solve_contact_vel2;
            let pre_solve_normal_vel = normal.dot(pre_solve_relative_vel);

            // Compute relative normal and tangential velocities at the contact point (equation 29)
            let contact_vel1 = compute_contact_vel(body1.lin_vel.0, body1.ang_vel.0, r1);
            let contact_vel2 = compute_contact_vel(body2.lin_vel.0, body2.ang_vel.0, r2);
            let relative_vel = contact_vel1 - contact_vel2;
            let normal_vel = normal.dot(relative_vel);
            let tangent_vel = relative_vel - normal * normal_vel;

            let inv_inertia1 = body1.inv_inertia.rotated(&body1.rot).0;
            let inv_inertia2 = body2.inv_inertia.rotated(&body2.rot).0;

            // Compute generalized inverse masses
            let w1 = PenetrationConstraint::get_generalized_inverse_mass(
                body1.rb,
                body1.inv_mass.0,
                inv_inertia1,
                r1,
                normal,
            );
            let w2 = PenetrationConstraint::get_generalized_inverse_mass(
                body2.rb,
                body2.inv_mass.0,
                inv_inertia2,
                r2,
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
            let p = delta_v / (w1 + w2);
            if body1.rb.is_dynamic() {
                body1.lin_vel.0 += p / body1.mass.0;
                body1.ang_vel.0 += compute_delta_ang_vel(inv_inertia1, r1, p);
            }
            if body2.rb.is_dynamic() {
                body2.lin_vel.0 -= p / body2.mass.0;
                body2.ang_vel.0 -= compute_delta_ang_vel(inv_inertia2, r2, p);
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
            [(rb1, mut lin_vel1, mut ang_vel1, inv_mass1), (rb2, mut lin_vel2, mut ang_vel2, inv_mass2)],
        ) = bodies.get_many_mut(joint.entities())
        {
            let delta_omega = (ang_vel2.0 - ang_vel1.0) * (joint.damping_ang() * sub_dt.0).min(1.0);

            if rb1.is_dynamic() {
                ang_vel1.0 += delta_omega;
            }
            if rb2.is_dynamic() {
                ang_vel2.0 -= delta_omega;
            }

            let delta_v = (lin_vel2.0 - lin_vel1.0) * (joint.damping_lin() * sub_dt.0).min(1.0);

            let w1 = if rb1.is_dynamic() { inv_mass1.0 } else { 0.0 };
            let w2 = if rb2.is_dynamic() { inv_mass2.0 } else { 0.0 };

            if w1 + w2 <= f32::EPSILON {
                continue;
            }

            let p = delta_v / (w1 + w2);

            if rb1.is_dynamic() {
                lin_vel1.0 += p * inv_mass1.0;
            }
            if rb2.is_dynamic() {
                lin_vel2.0 -= p * inv_mass2.0;
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
