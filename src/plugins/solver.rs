//! Solves positional and angular [constraints], updates velocities and solves velocity constraints
//! (dynamic [friction](Friction), [restitution](Restitution) and [joint damping](joints#damping)).
//!
//! See [`SolverPlugin`].

use crate::{
    prelude::*,
    utils::{compute_dynamic_friction, compute_restitution},
};
use bevy::prelude::*;
use constraints::penetration::PenetrationConstraint;

/// Solves positional and angular [constraints], updates velocities and solves velocity constraints
/// (dynamic [friction](Friction) and [restitution](Restitution) and [joint damping](joints#damping)).
///
/// ## Steps
///
/// Below are the three main steps of the `SolverPlugin`.
///
/// 1. **Constraint projection**: Constraints are handled by looping through them and applying positional and angular corrections
/// to the bodies in order to satisfy the constraints. Runs in [`SubstepSet::SolveConstraints`] and [`SubstepSet::SolveUserConstraints`].
///
/// 2. **Velocity update**: The velocities of bodies are updated based on positional and rotational changes from the last step.
/// Runs in [`SubstepSet::UpdateVelocities`].
///
/// 3. **Velocity solve**: Velocity corrections caused by dynamic friction, restitution and joint damping are applied.
/// Runs in [`SubstepSet::SolveVelocities`].
///
/// In the case of collisions, [`PenetrationConstraint`]s are created for each contact pair.
/// The constraints are resolved by moving the bodies so that they no longer penetrate.
/// Then, the velocities are updated, and velocity corrections caused by dynamic friction and restitution are applied.
pub struct SolverPlugin;

impl Plugin for SolverPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PenetrationConstraints>();

        let substeps = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");

        substeps.add_systems(
            (
                penetration_constraints,
                solve_constraint::<FixedJoint, 2>,
                solve_constraint::<RevoluteJoint, 2>,
                solve_constraint::<SphericalJoint, 2>,
                solve_constraint::<PrismaticJoint, 2>,
                solve_constraint::<DistanceJoint, 2>,
            )
                .chain()
                .in_set(SubstepSet::SolveConstraints),
        );

        substeps.add_systems((update_lin_vel, update_ang_vel).in_set(SubstepSet::UpdateVelocities));

        substeps.add_systems(
            (
                solve_vel,
                joint_damping::<FixedJoint>,
                joint_damping::<RevoluteJoint>,
                joint_damping::<SphericalJoint>,
                joint_damping::<PrismaticJoint>,
                joint_damping::<DistanceJoint>,
            )
                .chain()
                .in_set(SubstepSet::SolveVelocities),
        );

        substeps.add_systems(apply_translation.in_set(SubstepSet::ApplyTranslation));
    }
}

/// Stores penetration constraints for colliding entity pairs.
#[derive(Resource, Debug, Default)]
pub struct PenetrationConstraints(pub Vec<PenetrationConstraint>);

/// Iterates through broad phase collision pairs, checks which ones are actually colliding, and uses [`PenetrationConstraint`]s to resolve the collisions.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
fn penetration_constraints(
    mut commands: Commands,
    mut bodies: Query<(RigidBodyQuery, Option<&Sensor>, Option<&Sleeping>)>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    collisions: Res<Collisions>,
    sub_dt: Res<SubDeltaTime>,
) {
    penetration_constraints.0.clear();

    for ((entity1, entity2), contacts) in collisions
        .0
        .iter()
        .filter(|(_, contacts)| contacts.during_current_substep)
    {
        if let Ok([bundle1, bundle2]) = bodies.get_many_mut([*entity1, *entity2]) {
            let (mut body1, sensor1, sleeping1) = bundle1;
            let (mut body2, sensor2, sleeping2) = bundle2;

            let inactive1 = body1.rb.is_static() || sleeping1.is_some();
            let inactive2 = body2.rb.is_static() || sleeping2.is_some();

            // No collision if one of the bodies is static and the other one is sleeping.
            if inactive1 && inactive2 {
                continue;
            }

            // Create and solve constraint if both colliders are solid
            if sensor1.is_none() && sensor2.is_none() {
                // When an active body collides with a sleeping body, wake up the sleeping body
                if sleeping1.is_some() {
                    commands.entity(*entity1).remove::<Sleeping>();
                } else if sleeping2.is_some() {
                    commands.entity(*entity2).remove::<Sleeping>();
                }

                for contact_manifold in contacts.manifolds.iter() {
                    for contact in contact_manifold.contacts.iter() {
                        let mut constraint = PenetrationConstraint::new(&body1, &body2, *contact);
                        constraint.solve([&mut body1, &mut body2], sub_dt.0);
                        penetration_constraints.0.push(constraint);
                    }
                }
            }
        }
    }
}

/// Iterates through the constraints of a given type and solves them. Sleeping bodies are woken up when
/// active bodies interact with them in a constraint.
///
/// Note that this system only works for constraints that are modeled as entities.
/// If you store constraints in a resource, you must create your own system for solving them.
///
/// ## User constraints
///
/// To create a new constraint, implement [`XpbdConstraint`] for a component, get the [`SubstepSchedule`] and add this system into
/// the [`SubstepSet::SolveUserConstraints`] set.
/// You must provide the number of entities in the constraint using generics.
///
/// It should look something like this:
///
/// ```ignore
/// let substeps = app
///     .get_schedule_mut(SubstepSchedule)
///     .expect("add SubstepSchedule first");
///
/// substeps.add_systems(
///     solve_constraint::<YourConstraint, ENTITY_COUNT>
///         .in_set(SubstepSet::SolveUserConstraints),
/// );
/// ```
pub fn solve_constraint<C: XpbdConstraint<ENTITY_COUNT> + Component, const ENTITY_COUNT: usize>(
    mut commands: Commands,
    mut bodies: Query<(RigidBodyQuery, Option<&Sleeping>)>,
    mut constraints: Query<&mut C, Without<RigidBody>>,
    sub_dt: Res<SubDeltaTime>,
) {
    // Clear Lagrange multipliers
    constraints
        .iter_mut()
        .for_each(|mut c| c.clear_lagrange_multipliers());

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

/// Updates the linear velocity of all dynamic bodies based on the change in position from the previous step.
#[allow(clippy::type_complexity)]
fn update_lin_vel(
    mut bodies: Query<
        (
            &RigidBody,
            &Position,
            &PreviousPosition,
            &AccumulatedTranslation,
            &mut LinearVelocity,
            &mut PreSolveLinearVelocity,
        ),
        Without<Sleeping>,
    >,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, pos, prev_pos, translation, mut lin_vel, mut pre_solve_lin_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && lin_vel.0 != Vector::ZERO {
            lin_vel.0 = Vector::ZERO;
        }

        pre_solve_lin_vel.0 = lin_vel.0;

        if rb.is_dynamic() {
            // v = (x - x_prev) / h
            let new_lin_vel = (pos.0 - prev_pos.0 + translation.0) / sub_dt.0;
            // avoid triggering bevy's change detection unnecessarily
            if new_lin_vel != lin_vel.0 {
                lin_vel.0 = new_lin_vel;
            }
        }
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the previous step.
#[cfg(feature = "2d")]
fn update_ang_vel(
    mut bodies: Query<
        (
            &RigidBody,
            &Rotation,
            &PreviousRotation,
            &mut AngularVelocity,
            &mut PreSolveAngularVelocity,
        ),
        Without<Sleeping>,
    >,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && ang_vel.0 != 0.0 {
            ang_vel.0 = 0.0;
        }

        pre_solve_ang_vel.0 = ang_vel.0;

        if rb.is_dynamic() {
            let new_ang_vel = (rot.mul(prev_rot.inverse())).as_radians() / sub_dt.0;
            // avoid triggering bevy's change detection unnecessarily
            if new_ang_vel != ang_vel.0 {
                ang_vel.0 = new_ang_vel;
            }
        }
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the previous step.
#[cfg(feature = "3d")]
fn update_ang_vel(
    mut bodies: Query<
        (
            &RigidBody,
            &Rotation,
            &PreviousRotation,
            &mut AngularVelocity,
            &mut PreSolveAngularVelocity,
        ),
        Without<Sleeping>,
    >,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && ang_vel.0 != Vector::ZERO {
            ang_vel.0 = Vector::ZERO;
        }

        pre_solve_ang_vel.0 = ang_vel.0;

        if rb.is_dynamic() {
            let delta_rot = rot.mul_quat(prev_rot.inverse().0);
            let mut new_ang_vel = 2.0 * delta_rot.xyz() / sub_dt.0;
            if delta_rot.w < 0.0 {
                new_ang_vel = -new_ang_vel;
            }
            // avoid triggering bevy's change detection unnecessarily
            if new_ang_vel != ang_vel.0 {
                ang_vel.0 = new_ang_vel;
            }
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
        if let Ok([mut body1, mut body2]) = bodies.get_many_mut(constraint.entities()) {
            if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                continue;
            }

            // Skip constraint if it didn't apply a correction
            if constraint.normal_lagrange == 0.0 {
                continue;
            }

            let normal = constraint.contact.global_normal1(&body1.rotation);
            let r1 = body1.rotation.rotate(constraint.r1);
            let r2 = body2.rotation.rotate(constraint.r2);

            // Compute pre-solve relative normal velocities at the contact point (used for restitution)
            let pre_solve_contact_vel1 = compute_contact_vel(
                body1.pre_solve_linear_velocity.0,
                body1.pre_solve_angular_velocity.0,
                r1,
            );
            let pre_solve_contact_vel2 = compute_contact_vel(
                body2.pre_solve_linear_velocity.0,
                body2.pre_solve_angular_velocity.0,
                r2,
            );
            let pre_solve_relative_vel = pre_solve_contact_vel1 - pre_solve_contact_vel2;
            let pre_solve_normal_speed = normal.dot(pre_solve_relative_vel);

            // Compute relative normal and tangential velocities at the contact point (equation 29)
            let contact_vel1 =
                compute_contact_vel(body1.linear_velocity.0, body1.angular_velocity.0, r1);
            let contact_vel2 =
                compute_contact_vel(body2.linear_velocity.0, body2.angular_velocity.0, r2);
            let relative_vel = contact_vel1 - contact_vel2;

            let normal_speed = normal.dot(relative_vel);
            let tangent_vel = relative_vel - normal * normal_speed;
            let tangent_speed = tangent_vel.length();

            let inv_mass1 = body1.effective_inv_mass();
            let inv_mass2 = body2.effective_inv_mass();
            let inv_inertia1 = body1.effective_world_inv_inertia();
            let inv_inertia2 = body2.effective_world_inv_inertia();

            let mut p = Vector::ZERO;

            // Compute restitution
            let restitution_speed = compute_restitution(
                normal_speed,
                pre_solve_normal_speed,
                body1.restitution.combine(*body2.restitution).coefficient,
                gravity.0,
                sub_dt.0,
            );
            if restitution_speed.abs() > Scalar::EPSILON {
                let w1 = constraint.compute_generalized_inverse_mass(&body1, r1, normal);
                let w2 = constraint.compute_generalized_inverse_mass(&body2, r2, normal);
                p += restitution_speed / (w1 + w2) * normal;
            }

            // Compute dynamic friction
            if tangent_speed > Scalar::EPSILON {
                let tangent_dir = tangent_vel / tangent_speed;
                let w1 = constraint.compute_generalized_inverse_mass(&body1, r1, tangent_dir);
                let w2 = constraint.compute_generalized_inverse_mass(&body2, r2, tangent_dir);
                let friction_impulse = compute_dynamic_friction(
                    tangent_speed,
                    w1 + w2,
                    body1.friction.combine(*body2.friction).dynamic_coefficient,
                    constraint.normal_lagrange,
                    sub_dt.0,
                );
                p += friction_impulse * tangent_dir;
            }

            if body1.rb.is_dynamic() {
                let delta_lin_vel = p * inv_mass1;
                let delta_ang_vel = compute_delta_ang_vel(inv_inertia1, r1, p);

                if delta_lin_vel != Vector::ZERO {
                    body1.linear_velocity.0 += delta_lin_vel;
                }
                if delta_ang_vel != AngularVelocity::ZERO.0 {
                    body1.angular_velocity.0 += delta_ang_vel;
                }
            }
            if body2.rb.is_dynamic() {
                let delta_lin_vel = p * inv_mass2;
                let delta_ang_vel = compute_delta_ang_vel(inv_inertia2, r2, p);

                if delta_lin_vel != Vector::ZERO {
                    body2.linear_velocity.0 -= delta_lin_vel;
                }
                if delta_ang_vel != AngularVelocity::ZERO.0 {
                    body2.angular_velocity.0 -= delta_ang_vel;
                }
            }
        }
    }
}

/// Applies velocity corrections caused by joint damping.
pub fn joint_damping<T: Joint>(
    mut bodies: Query<
        (
            &RigidBody,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &InverseMass,
        ),
        Without<Sleeping>,
    >,
    joints: Query<&T, Without<RigidBody>>,
    sub_dt: Res<SubDeltaTime>,
) {
    for joint in &joints {
        if let Ok(
            [(rb1, mut lin_vel1, mut ang_vel1, inv_mass1), (rb2, mut lin_vel2, mut ang_vel2, inv_mass2)],
        ) = bodies.get_many_mut(joint.entities())
        {
            let delta_omega =
                (ang_vel2.0 - ang_vel1.0) * (joint.damping_angular() * sub_dt.0).min(1.0);

            if rb1.is_dynamic() {
                ang_vel1.0 += delta_omega;
            }
            if rb2.is_dynamic() {
                ang_vel2.0 -= delta_omega;
            }

            let delta_v = (lin_vel2.0 - lin_vel1.0) * (joint.damping_linear() * sub_dt.0).min(1.0);

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

fn apply_translation(
    mut bodies: Query<
        (&RigidBody, &mut Position, &mut AccumulatedTranslation),
        Changed<AccumulatedTranslation>,
    >,
) {
    for (rb, mut pos, mut translation) in &mut bodies {
        if rb.is_static() {
            continue;
        }

        pos.0 += translation.0;
        translation.0 = Vector::ZERO;
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
fn compute_delta_ang_vel(inverse_inertia: Scalar, r: Vector, p: Vector) -> Scalar {
    inverse_inertia * r.perp_dot(p)
}

#[cfg(feature = "3d")]
fn compute_delta_ang_vel(inverse_inertia: Matrix3, r: Vector, p: Vector) -> Vector {
    inverse_inertia * r.cross(p)
}
