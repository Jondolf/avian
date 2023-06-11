//! The XPBD solver is responsible for constraint projection, velocity updates, and velocity corrections. See [`SolverPlugin`].

use crate::{
    collision::*,
    prelude::*,
    steps::broad_phase::BroadCollisionPairs,
    utils::{get_dynamic_friction, get_restitution},
};
use bevy::prelude::*;
use constraints::penetration::PenetrationConstraint;

/// The `SolverPlugin` is reponsible for constraint projection, velocity updates, and velocity corrections caused by dynamic friction, restitution and damping.
///
/// ## Steps
///
/// Below are the three main steps of the `SolverPlugin`.
///
/// 1. **Constraint projection**: Constraints are handled by looping through them and applying positional and angular corrections to the bodies in order to satisfy the constraints.
///
/// 2. **Velocity update**: The velocities of bodies are updated based on positional and rotational changes from the last step.
///
/// 3. **Velocity solve**: Velocity corrections caused by dynamic friction, restitution and damping are applied.
///
/// In the case of collisions, [`PenetrationConstraint`]s are created for each contact pair. The constraints are resolved by moving the bodies so that they no longer penetrate. Then, the velocities are updated, and velocity corrections caused by dynamic friction and restitution are applied.
pub struct SolverPlugin;

impl Plugin for SolverPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PenetrationConstraints>();

        let substeps = app
            .get_schedule_mut(XpbdSubstepSchedule)
            .expect("add XpbdSubstepSchedule first");

        substeps.configure_sets(
            (
                SubsteppingSet::Integrate,
                SubsteppingSet::SolvePos,
                SubsteppingSet::UpdateVel,
                SubsteppingSet::SolveVel,
            )
                .chain(),
        );

        substeps.add_systems(
            (
                penetration_constraints,
                solve_constraint::<FixedJoint, 2>,
                solve_constraint::<RevoluteJoint, 2>,
                solve_constraint::<SphericalJoint, 2>,
                solve_constraint::<PrismaticJoint, 2>,
            )
                .chain()
                .in_set(SubsteppingSet::SolvePos),
        );

        substeps.add_systems((update_lin_vel, update_ang_vel).in_set(SubsteppingSet::UpdateVel));

        substeps.add_systems(
            (
                solve_vel,
                joint_damping::<FixedJoint>,
                joint_damping::<RevoluteJoint>,
                joint_damping::<SphericalJoint>,
                joint_damping::<PrismaticJoint>,
            )
                .chain()
                .in_set(SubsteppingSet::SolveVel),
        );
    }
}

/// Stores penetration constraints for colliding entity pairs.
#[derive(Resource, Debug, Default)]
pub struct PenetrationConstraints(pub Vec<PenetrationConstraint>);

/// Iterates through broad phase collision pairs, checks which ones are actually colliding, and uses [`PenetrationConstraint`]s to resolve the collisions.
fn penetration_constraints(
    mut commands: Commands,
    mut bodies: Query<(RigidBodyQuery, &ColliderShape, Option<&Sleeping>)>,
    broad_collision_pairs: Res<BroadCollisionPairs>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    sub_dt: Res<SubDeltaTime>,
) {
    penetration_constraints.0.clear();

    for (ent1, ent2) in broad_collision_pairs.0.iter() {
        if let Ok(
            [(mut body1, collider_shape1, sleeping1), (mut body2, collider_shape2, sleeping2)],
        ) = bodies.get_many_mut([*ent1, *ent2])
        {
            let inactive1 = body1.rb.is_static() || sleeping1.is_some();
            let inactive2 = body2.rb.is_static() || sleeping2.is_some();

            // No collision if one of the bodies is static and the other one is sleeping.
            if inactive1 && inactive2 {
                continue;
            }

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
                // When an active body collides with a sleeping body, wake up the sleeping body.
                if sleeping1.is_some() {
                    commands.entity(*ent1).remove::<Sleeping>();
                } else if sleeping2.is_some() {
                    commands.entity(*ent2).remove::<Sleeping>();
                }

                let mut constraint: PenetrationConstraint =
                    PenetrationConstraint::new(*ent1, *ent2, collision);
                constraint.solve([&mut body1, &mut body2], sub_dt.0);
                penetration_constraints.0.push(constraint);
            }
        }
    }
}

/// Iterates through all joints and solves the constraints.
pub fn solve_constraint<C: XpbdConstraint<ENTITY_COUNT> + Component, const ENTITY_COUNT: usize>(
    mut commands: Commands,
    mut bodies: Query<(RigidBodyQuery, Option<&Sleeping>)>,
    mut constraints: Query<&mut C, Without<RigidBody>>,
    num_pos_iters: Res<NumPosIters>,
    sub_dt: Res<SubDeltaTime>,
) {
    // Clear Lagrange multipliers
    constraints
        .iter_mut()
        .for_each(|mut c| c.clear_lagrange_multipliers());

    for _j in 0..num_pos_iters.0 {
        for mut constraint in &mut constraints {
            // Get components for entities
            if let Ok(mut bodies) = bodies.get_many_mut(constraint.entities()) {
                let none_dynamic = bodies.iter().all(|(body, _)| !body.rb.is_dynamic());
                let all_inactive = bodies
                    .iter()
                    .all(|(body, sleeping)| body.rb.is_static() || sleeping.is_some());

                // No constraint solving if none of the bodies is dynamic,
                // or if all of the bodies are either static or sleeping
                if none_dynamic || all_inactive {
                    continue;
                }

                // At least one of the participating bodies is active, so wake up any sleeping bodies
                for (body, sleeping) in &bodies {
                    if sleeping.is_some() {
                        commands.entity(body.entity).remove::<Sleeping>();
                    }
                }

                // Get the bodies as an array and solve the constraint
                if let Ok(bodies) = bodies
                    .iter_mut()
                    .map(|(ref mut body, _)| body)
                    .collect::<Vec<&mut RigidBodyQueryItem>>()
                    .try_into()
                {
                    constraint.solve(bodies, sub_dt.0);
                }
            }
        }
    }
}

/// Updates the linear velocity of all dynamic bodies based on the change in position from the previous step.
fn update_lin_vel(
    mut bodies: Query<
        (&RigidBody, &Pos, &PrevPos, &mut LinVel, &mut PreSolveLinVel),
        Without<Sleeping>,
    >,
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
    mut bodies: Query<
        (&RigidBody, &Rot, &PrevRot, &mut AngVel, &mut PreSolveAngVel),
        Without<Sleeping>,
    >,
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
    mut bodies: Query<
        (&RigidBody, &Rot, &PrevRot, &mut AngVel, &mut PreSolveAngVel),
        Without<Sleeping>,
    >,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() {
            pre_solve_ang_vel.0 = Vector::ZERO;
            ang_vel.0 = Vector::ZERO;
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

/// Applies velocity corrections caused by dynamic friction and restitution.
#[allow(clippy::type_complexity)]
fn solve_vel(
    mut bodies: Query<RigidBodyQuery, Without<Sleeping>>,
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

            let inv_inertia1 = body1.world_inv_inertia().0;
            let inv_inertia2 = body2.world_inv_inertia().0;

            // Compute generalized inverse masses
            let w1 = constraint.compute_generalized_inverse_mass(&body1, r1, normal);
            let w2 = constraint.compute_generalized_inverse_mass(&body2, r2, normal);

            // Compute dynamic friction
            let friction_impulse = get_dynamic_friction(
                tangent_vel,
                body1.friction.combine(*body2.friction).dynamic_coefficient,
                constraint.normal_lagrange,
                sub_dt.0,
            );

            // Compute restitution
            let restitution_impulse = get_restitution(
                normal,
                normal_vel,
                pre_solve_normal_vel,
                body1.restitution.combine(*body2.restitution).coefficient,
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

/// Applies velocity corrections caused by joint damping.
fn joint_damping<T: Joint>(
    mut bodies: Query<(&RigidBody, &mut LinVel, &mut AngVel, &InvMass), Without<Sleeping>>,
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

            if w1 + w2 <= Scalar::EPSILON {
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
fn compute_contact_vel(lin_vel: Vector, ang_vel: Scalar, r: Vector) -> Vector {
    lin_vel + ang_vel * r.perp()
}

#[cfg(feature = "3d")]
fn compute_contact_vel(lin_vel: Vector, ang_vel: Vector, r: Vector) -> Vector {
    lin_vel + ang_vel.cross(r)
}

#[cfg(feature = "2d")]
fn compute_delta_ang_vel(inv_inertia: Scalar, r: Vector, p: Vector) -> Scalar {
    inv_inertia * r.perp_dot(p)
}

#[cfg(feature = "3d")]
fn compute_delta_ang_vel(inv_inertia: Matrix3, r: Vector, p: Vector) -> Vector {
    inv_inertia * r.cross(p)
}
