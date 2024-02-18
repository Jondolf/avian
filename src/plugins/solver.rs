//! Solves positional and angular [constraints], updates velocities and solves velocity constraints
//! (dynamic [friction](Friction), [restitution](Restitution) and [joint damping](joints#damping)).
//!
//! See [`SolverPlugin`].

use crate::{
    prelude::*,
    utils::{compute_dynamic_friction, compute_restitution, get_pos_translation},
};
use bevy::{
    ecs::query::{Has, QueryData},
    prelude::*,
};
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

        substeps.add_systems(store_contact_impulses.in_set(SubstepSet::StoreImpulses));

        substeps.add_systems(apply_translation.in_set(SubstepSet::ApplyTranslation));
    }
}

/// Stores penetration constraints for colliding entity pairs.
#[derive(Resource, Debug, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct PenetrationConstraints(pub Vec<PenetrationConstraint>);

/// A `WorldQuery` to make code handling colliders in collisions cleaner.
#[derive(QueryData)]
struct ColliderQuery<'w> {
    entity: Entity,
    parent: Option<&'w ColliderParent>,
    transform: Option<&'w ColliderTransform>,
    is_sensor: Has<Sensor>,
    friction: Option<&'w Friction>,
    restitution: Option<&'w Restitution>,
}

/// Iterates through broad phase collision pairs, checks which ones are actually colliding, and uses [`PenetrationConstraint`]s to resolve the collisions.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
fn penetration_constraints(
    mut commands: Commands,
    mut bodies: Query<(
        RigidBodyQuery,
        Option<&Name>,
        Option<&Sensor>,
        Option<&Sleeping>,
    )>,
    colliders: Query<ColliderQuery>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    mut collisions: ResMut<Collisions>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    penetration_constraints.0.clear();

    for ((collider_entity1, collider_entity2), contacts) in collisions
        .get_internal_mut()
        .iter_mut()
        .filter(|(_, contacts)| contacts.during_current_substep)
    {
        // Don't collide with self
        if collider_entity1 == collider_entity2 {
            continue;
        }

        // Get colliders
        let Ok([collider1, collider2]) = colliders.get_many([*collider_entity1, *collider_entity2])
        else {
            continue;
        };

        let collider_parent1 = collider1.parent.map_or(*collider_entity1, |p| p.get());
        let collider_parent2 = collider2.parent.map_or(*collider_entity2, |p| p.get());

        // Reset penetration state for this substep.
        // This is set to true if any of the contacts is penetrating.
        contacts.during_current_substep = false;

        if let Ok([bundle1, bundle2]) = bodies.get_many_mut([collider_parent1, collider_parent2]) {
            let (mut body1, name1, sensor1, sleeping1) = bundle1;
            let (mut body2, name2, sensor2, sleeping2) = bundle2;

            let inactive1 = body1.rb.is_static() || sleeping1.is_some();
            let inactive2 = body2.rb.is_static() || sleeping2.is_some();

            let body1_is_sensor = contacts.entity1 == body1.entity && sensor1.is_some();
            let body2_is_sensor = contacts.entity2 == body2.entity && sensor2.is_some();

            // No collision response if both bodies are static or sleeping
            // or if either of the colliders is a sensor collider.
            if (inactive1 && inactive2)
                || body1_is_sensor
                || body2_is_sensor
                || collider1.is_sensor
                || collider2.is_sensor
            {
                continue;
            }

            // When an active body collides with a sleeping body, wake up the sleeping body.
            if sleeping1.is_some() {
                commands.entity(body1.entity).remove::<Sleeping>();
            } else if sleeping2.is_some() {
                commands.entity(body2.entity).remove::<Sleeping>();
            }

            // Get combined friction and restitution coefficients of the colliders
            // or the bodies they are attached to.
            let friction = collider1
                .friction
                .unwrap_or(body1.friction)
                .combine(*collider2.friction.unwrap_or(body2.friction));
            let restitution = collider1
                .restitution
                .unwrap_or(body1.restitution)
                .combine(*collider2.restitution.unwrap_or(body2.restitution));

            // Create and solve penetration constraints for each contact.
            for (manifold_index, manifold) in contacts.manifolds.iter().enumerate() {
                for contact in manifold.contacts.iter() {
                    // Add collider transforms to local contact points
                    let contact = ContactData {
                        point1: collider1.transform.map_or(contact.point1, |t| {
                            t.rotation.rotate(contact.point1) + t.translation
                        }),
                        point2: collider2.transform.map_or(contact.point2, |t| {
                            t.rotation.rotate(contact.point2) + t.translation
                        }),
                        normal1: collider1
                            .transform
                            .map_or(contact.normal1, |t| t.rotation.rotate(contact.normal1)),
                        normal2: collider2
                            .transform
                            .map_or(contact.normal2, |t| t.rotation.rotate(contact.normal2)),
                        ..*contact
                    };

                    let mut constraint = PenetrationConstraint {
                        friction,
                        restitution,
                        ..PenetrationConstraint::new(
                            &body1,
                            &body2,
                            *collider_entity1,
                            *collider_entity2,
                            contact,
                            manifold_index,
                        )
                    };
                    constraint.solve([&mut body1, &mut body2], delta_secs);
                    penetration_constraints.0.push(constraint);

                    // Set collision as penetrating for this frame and substep.
                    // This is used for detecting when the collision has started or ended.
                    if contact.penetration > Scalar::EPSILON {
                        contacts.during_current_frame = true;
                        contacts.during_current_substep = true;
                    }
                }
            }

            if contacts.during_current_substep
                && (body1.rb.is_added() || body2.rb.is_added())
                && body1.rb.is_dynamic()
                && body2.rb.is_dynamic()
            {
                // if the RigidBody entity has a name, use that for debug.
                let debug_id1 = match name1 {
                    Some(n) => format!("{:?} ({n})", body1.entity),
                    None => format!("{:?}", body1.entity),
                };
                let debug_id2 = match name2 {
                    Some(n) => format!("{:?} ({n})", body2.entity),
                    None => format!("{:?}", body2.entity),
                };
                warn!(
                    "{} and {} are overlapping at spawn, which can result in explosive behavior.",
                    debug_id1, debug_id2,
                );
                debug!("{} is at {}", debug_id1, body1.position.0);
                debug!("{} is at {}", debug_id2, body2.position.0);
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
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

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
                constraint.solve(bodies, delta_secs);
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
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (rb, pos, prev_pos, translation, mut lin_vel, mut pre_solve_lin_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && lin_vel.0 != Vector::ZERO {
            lin_vel.0 = Vector::ZERO;
        }

        pre_solve_lin_vel.0 = lin_vel.0;

        if rb.is_dynamic() {
            // v = (x - x_prev) / h
            let new_lin_vel = (pos.0 - prev_pos.0 + translation.0) / delta_secs;
            // avoid triggering bevy's change detection unnecessarily
            if new_lin_vel != lin_vel.0 && new_lin_vel.is_finite() {
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
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && ang_vel.0 != 0.0 {
            ang_vel.0 = 0.0;
        }

        pre_solve_ang_vel.0 = ang_vel.0;

        if rb.is_dynamic() {
            let new_ang_vel = (rot.mul(prev_rot.inverse())).as_radians() / delta_secs;
            // avoid triggering bevy's change detection unnecessarily
            if new_ang_vel != ang_vel.0 && new_ang_vel.is_finite() {
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
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && ang_vel.0 != Vector::ZERO {
            ang_vel.0 = Vector::ZERO;
        }

        pre_solve_ang_vel.0 = ang_vel.0;

        if rb.is_dynamic() {
            let delta_rot = rot.mul_quat(prev_rot.inverse().0);
            let mut new_ang_vel = 2.0 * delta_rot.xyz() / delta_secs;
            if delta_rot.w < 0.0 {
                new_ang_vel = -new_ang_vel;
            }
            // avoid triggering bevy's change detection unnecessarily
            if new_ang_vel != ang_vel.0 && new_ang_vel.is_finite() {
                ang_vel.0 = new_ang_vel;
            }
        }
    }
}

/// Applies velocity corrections caused by dynamic friction and restitution.
#[allow(clippy::type_complexity)]
fn solve_vel(
    mut bodies: Query<RigidBodyQuery, Without<Sleeping>>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    gravity: Res<Gravity>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for constraint in penetration_constraints.0.iter_mut() {
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
                constraint.restitution.coefficient,
                gravity.0,
                delta_secs,
            );
            if restitution_speed.abs() > Scalar::EPSILON {
                let w1 = constraint.compute_generalized_inverse_mass(&body1, r1, normal);
                let w2 = constraint.compute_generalized_inverse_mass(&body2, r2, normal);
                let restitution_impulse = restitution_speed / (w1 + w2);
                p += restitution_impulse * normal;
                constraint.contact.normal_impulse += restitution_impulse;
            }

            // Compute dynamic friction
            if tangent_speed > Scalar::EPSILON {
                let tangent = tangent_vel / tangent_speed;
                let w1 = constraint.compute_generalized_inverse_mass(&body1, r1, tangent);
                let w2 = constraint.compute_generalized_inverse_mass(&body2, r2, tangent);
                let friction_impulse = compute_dynamic_friction(
                    tangent_speed,
                    w1 + w2,
                    constraint.friction.dynamic_coefficient,
                    constraint.normal_lagrange,
                    delta_secs,
                );
                p += friction_impulse * tangent;
                constraint.contact.tangent_impulse += friction_impulse;
            }

            if body1.rb.is_dynamic() && body1.dominance() <= body2.dominance() {
                let delta_lin_vel = p * inv_mass1;
                let delta_ang_vel = compute_delta_ang_vel(inv_inertia1, r1, p);

                if delta_lin_vel != Vector::ZERO {
                    body1.linear_velocity.0 += delta_lin_vel;
                }
                if delta_ang_vel != AngularVelocity::ZERO.0 {
                    body1.angular_velocity.0 += delta_ang_vel;
                }
            }
            if body2.rb.is_dynamic() && body2.dominance() <= body1.dominance() {
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
#[allow(clippy::type_complexity)]
pub fn joint_damping<T: Joint>(
    mut bodies: Query<
        (
            &RigidBody,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &InverseMass,
            Option<&Dominance>,
        ),
        Without<Sleeping>,
    >,
    joints: Query<&T, Without<RigidBody>>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for joint in &joints {
        if let Ok(
            [(rb1, mut lin_vel1, mut ang_vel1, inv_mass1, dominance1), (rb2, mut lin_vel2, mut ang_vel2, inv_mass2, dominance2)],
        ) = bodies.get_many_mut(joint.entities())
        {
            let delta_omega =
                (ang_vel2.0 - ang_vel1.0) * (joint.damping_angular() * delta_secs).min(1.0);

            if rb1.is_dynamic() {
                ang_vel1.0 += delta_omega;
            }
            if rb2.is_dynamic() {
                ang_vel2.0 -= delta_omega;
            }

            let delta_v =
                (lin_vel2.0 - lin_vel1.0) * (joint.damping_linear() * delta_secs).min(1.0);

            let w1 = if rb1.is_dynamic() { inv_mass1.0 } else { 0.0 };
            let w2 = if rb2.is_dynamic() { inv_mass2.0 } else { 0.0 };

            if w1 + w2 <= Scalar::EPSILON {
                continue;
            }

            let p = delta_v / (w1 + w2);

            let dominance1 = dominance1.map_or(0, |dominance| dominance.0);
            let dominance2 = dominance2.map_or(0, |dominance| dominance.0);

            if rb1.is_dynamic() && (!rb2.is_dynamic() || dominance1 <= dominance2) {
                lin_vel1.0 += p * inv_mass1.0;
            }
            if rb2.is_dynamic() && (!rb1.is_dynamic() || dominance2 <= dominance1) {
                lin_vel2.0 -= p * inv_mass2.0;
            }
        }
    }
}

fn store_contact_impulses(
    constraints: Res<PenetrationConstraints>,
    mut collisions: ResMut<Collisions>,
) {
    for constraint in constraints.0.iter() {
        let Some(collision) =
            collisions.get_mut(constraint.collider_entity1, constraint.collider_entity2)
        else {
            continue;
        };
        if let Some(Some(contact)) = collision
            .manifolds
            .get_mut(constraint.manifold_index)
            .map(|m| m.contacts.get_mut(constraint.contact.index))
        {
            contact.normal_impulse = constraint.contact.normal_impulse;
            contact.tangent_impulse = constraint.contact.tangent_impulse;

            collision.total_normal_impulse += contact.normal_impulse.abs();
            collision.total_tangent_impulse += contact.tangent_impulse.abs();
        }
    }
}

#[allow(clippy::type_complexity)]
fn apply_translation(
    mut bodies: Query<
        (
            &RigidBody,
            &mut Position,
            &Rotation,
            &PreviousRotation,
            &mut AccumulatedTranslation,
            &CenterOfMass,
        ),
        Changed<AccumulatedTranslation>,
    >,
) {
    for (rb, mut pos, rot, prev_rot, mut translation, center_of_mass) in &mut bodies {
        if rb.is_static() {
            continue;
        }

        // We must also account for the translation caused by rotations around the center of mass,
        // as it may be offset from `Position`.
        pos.0 += get_pos_translation(&translation, prev_rot, rot, center_of_mass);
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
