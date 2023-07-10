//! Solves positional and angular [constraints], updates velocities and solves velocity constraints
//! (dynamic [friction](Friction), [restitution](Restitution) and [joint damping](joints#damping)).
//!
//! See [`SolverPlugin`].

use crate::{
    collision::*,
    prelude::*,
    utils::{get_dynamic_friction, get_restitution},
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
        app.init_resource::<PenetrationConstraints>()
            .add_event::<Collision>()
            .add_event::<CollisionStarted>()
            .add_event::<CollisionEnded>();

        let substeps = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");

        substeps.configure_sets(
            (
                SubstepSet::Integrate,
                SubstepSet::SolveConstraints,
                SubstepSet::SolveUserConstraints,
                SubstepSet::UpdateVelocities,
                SubstepSet::SolveVelocities,
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
            )
                .chain()
                .in_set(SubstepSet::SolveVelocities),
        );
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
    mut bodies: Query<(
        RigidBodyQuery,
        &Collider,
        Option<&Sensor>,
        Option<&CollisionLayers>,
        Option<&mut CollidingEntities>,
        Option<&Sleeping>,
    )>,
    broad_collision_pairs: Res<BroadCollisionPairs>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    mut collision_ev_writer: EventWriter<Collision>,
    mut collision_started_ev_writer: EventWriter<CollisionStarted>,
    mut collision_ended_ev_writer: EventWriter<CollisionEnded>,
    sub_dt: Res<SubDeltaTime>,
) {
    let mut collision_events = Vec::with_capacity(broad_collision_pairs.0.len());
    let mut started_collisions = Vec::new();
    let mut ended_collisions = Vec::new();

    penetration_constraints.0.clear();

    for (ent1, ent2) in broad_collision_pairs.0.iter() {
        if let Ok([bundle1, bundle2]) = bodies.get_many_mut([*ent1, *ent2]) {
            let (mut body1, collider1, sensor1, layers1, colliding_entities1, sleeping1) = bundle1;
            let (mut body2, collider2, sensor2, layers2, colliding_entities2, sleeping2) = bundle2;

            let layers1 = layers1.map_or(CollisionLayers::default(), |l| *l);
            let layers2 = layers2.map_or(CollisionLayers::default(), |l| *l);

            // Skip collision if collision layers are incompatible
            if !layers1.interacts_with(layers2) {
                continue;
            }

            let inactive1 = body1.rb.is_static() || sleeping1.is_some();
            let inactive2 = body2.rb.is_static() || sleeping2.is_some();

            // No collision if one of the bodies is static and the other one is sleeping.
            if inactive1 && inactive2 {
                continue;
            }

            if let Some(contact) = compute_contact(
                *ent1,
                *ent2,
                body1.position.0,
                body2.position.0,
                &body1.rotation,
                &body2.rotation,
                collider1,
                collider2,
            ) {
                collision_events.push(Collision(contact));

                let mut collision_started_1 = false;
                let mut collision_started_2 = false;

                // Add entity to set of colliding entities
                if let Some(mut entities) = colliding_entities1 {
                    collision_started_1 = entities.insert(*ent2);
                }
                if let Some(mut entities) = colliding_entities2 {
                    collision_started_2 = entities.insert(*ent1);
                }

                if collision_started_1 || collision_started_2 {
                    started_collisions.push(CollisionStarted(*ent1, *ent2));
                }

                // When an active body collides with a sleeping body, wake up the sleeping body
                if sleeping1.is_some() {
                    commands.entity(*ent1).remove::<Sleeping>();
                } else if sleeping2.is_some() {
                    commands.entity(*ent2).remove::<Sleeping>();
                }

                // Create and solve constraint if both colliders are solid
                if sensor1.is_none() && sensor2.is_none() {
                    let mut constraint = PenetrationConstraint::new(&body1, &body2, contact);
                    constraint.solve([&mut body1, &mut body2], sub_dt.0);
                    penetration_constraints.0.push(constraint);
                }
            } else {
                let mut collision_ended_1 = false;
                let mut collision_ended_2 = false;

                // Remove entity from set of colliding entities
                if let Some(mut entities) = colliding_entities1 {
                    collision_ended_1 = entities.remove(ent2);
                }
                if let Some(mut entities) = colliding_entities2 {
                    collision_ended_2 = entities.remove(ent1);
                }

                if collision_ended_1 || collision_ended_2 {
                    ended_collisions.push(CollisionEnded(*ent1, *ent2));
                }
            }
        }
    }

    collision_ev_writer.send_batch(collision_events);
    collision_started_ev_writer.send_batch(started_collisions);
    collision_ended_ev_writer.send_batch(ended_collisions);
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
    num_pos_iters: Res<IterationCount>,
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
        (
            &RigidBody,
            &Position,
            &PreviousPosition,
            &mut LinearVelocity,
            &mut PreSolveLinearVelocity,
        ),
        Without<Sleeping>,
    >,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, pos, prev_pos, mut lin_vel, mut pre_solve_lin_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() {
            lin_vel.0 = Vector::ZERO;
        }

        pre_solve_lin_vel.0 = lin_vel.0;

        if rb.is_dynamic() {
            // v = (x - x_prev) / h
            lin_vel.0 = (pos.0 - prev_pos.0) / sub_dt.0;
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
        if rb.is_static() {
            ang_vel.0 = 0.0;
        }

        pre_solve_ang_vel.0 = ang_vel.0;

        if rb.is_dynamic() {
            ang_vel.0 = (rot.mul(prev_rot.inverse())).as_radians() / sub_dt.0;
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
        if rb.is_static() {
            ang_vel.0 = Vector::ZERO;
        }

        pre_solve_ang_vel.0 = ang_vel.0;

        if rb.is_dynamic() {
            let delta_rot = rot.mul_quat(prev_rot.inverse().0);
            ang_vel.0 = 2.0 * delta_rot.xyz() / sub_dt.0;

            if delta_rot.w < 0.0 {
                ang_vel.0 = -ang_vel.0;
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
        let Contact {
            entity1,
            entity2,
            normal,
            ..
        } = constraint.contact;

        if let Ok([mut body1, mut body2]) = bodies.get_many_mut([entity1, entity2]) {
            if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                continue;
            }

            // Compute pre-solve relative normal velocities at the contact point (used for restitution)
            let pre_solve_contact_vel1 = compute_contact_vel(
                body1.pre_solve_linear_velocity.0,
                body1.pre_solve_angular_velocity.0,
                constraint.world_r1,
            );
            let pre_solve_contact_vel2 = compute_contact_vel(
                body2.pre_solve_linear_velocity.0,
                body2.pre_solve_angular_velocity.0,
                constraint.world_r2,
            );
            let pre_solve_relative_vel = pre_solve_contact_vel1 - pre_solve_contact_vel2;
            let pre_solve_normal_vel = normal.dot(pre_solve_relative_vel);

            // Compute relative normal and tangential velocities at the contact point (equation 29)
            let contact_vel1 = compute_contact_vel(
                body1.linear_velocity.0,
                body1.angular_velocity.0,
                constraint.world_r1,
            );
            let contact_vel2 = compute_contact_vel(
                body2.linear_velocity.0,
                body2.angular_velocity.0,
                constraint.world_r2,
            );
            let relative_vel = contact_vel1 - contact_vel2;
            let normal_vel = normal.dot(relative_vel);
            let tangent_vel = relative_vel - normal * normal_vel;

            let inv_mass1 = body1.effective_inv_mass();
            let inv_mass2 = body2.effective_inv_mass();
            let inv_inertia1 = body1.effective_world_inv_inertia();
            let inv_inertia2 = body2.effective_world_inv_inertia();

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
            let delta_v_length = delta_v.length();

            if delta_v_length <= Scalar::EPSILON {
                continue;
            }

            let delta_v_dir = delta_v / delta_v_length;

            // Compute generalized inverse masses
            let w1 = constraint.compute_generalized_inverse_mass(
                &body1,
                constraint.world_r1,
                delta_v_dir,
            );
            let w2 = constraint.compute_generalized_inverse_mass(
                &body2,
                constraint.world_r2,
                delta_v_dir,
            );

            // Compute velocity impulse and apply velocity updates (equation 33)
            let p = delta_v / (w1 + w2);
            if body1.rb.is_dynamic() {
                body1.linear_velocity.0 += p * inv_mass1;
                body1.angular_velocity.0 +=
                    compute_delta_ang_vel(inv_inertia1, constraint.world_r1, p);
            }
            if body2.rb.is_dynamic() {
                body2.linear_velocity.0 -= p * inv_mass2;
                body2.angular_velocity.0 -=
                    compute_delta_ang_vel(inv_inertia2, constraint.world_r2, p);
            }
        }
    }
}

/// Applies velocity corrections caused by joint damping.
fn joint_damping<T: Joint>(
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
