//! XPBD systems and core logic.
//!
//! The math and physics are primarily from [Matthias Müller's paper titled "Detailed Rigid Body Simulation with Extended Position Based Dynamics"](https://matthias-research.github.io/pages/publications/PBDBodies.pdf).

use crate::{
    components::*,
    constraints::{penetration::PenetrationConstraint, Constraint, PositionConstraint},
    prelude::Joint,
    resources::*,
    utils::*,
    *,
};

use bevy::prelude::*;
use parry::{bounding_volume::BoundingVolume, math::Isometry};

/// Collects bodies that are potentially colliding and adds non-penetration constraints for them.
pub(crate) fn collect_collision_pairs(
    bodies: Query<(Entity, &ColliderAabbWithMargin, &RigidBody)>,
    mut broad_collision_pairs: ResMut<BroadCollisionPairs>,
) {
    broad_collision_pairs.0.clear();

    for [(ent_a, aabb_a, rb_a), (ent_b, aabb_b, rb_b)] in bodies.iter_combinations() {
        // At least one of the bodies is dynamic and their AABBs intersect
        if (rb_a.is_dynamic() || rb_b.is_dynamic()) && aabb_a.intersects(&aabb_b.0) {
            broad_collision_pairs.0.push((ent_a, ent_b));
        }
    }
}

/// Applies forces and predicts the next position and velocity for all dynamic bodies.
pub(crate) fn integrate_pos(
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
    for (rb, mut pos, mut prev_pos, mut lin_vel, external_force, mass) in bodies.iter_mut() {
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
pub(crate) fn integrate_rot(
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
    for (rb, mut rot, mut prev_rot, mut ang_vel, external_torque, inv_inertia) in bodies.iter_mut()
    {
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
pub(crate) fn integrate_rot(
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
        bodies.iter_mut()
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

pub(crate) fn clear_penetration_constraint_lagrange(
    mut penetration_constraints: ResMut<PenetrationConstraints>,
) {
    for constraint in penetration_constraints.0.iter_mut() {
        constraint.clear_lagrange_multipliers();
    }
}

pub(crate) fn clear_joint_lagrange<T: Joint>(mut joints: Query<&mut T>) {
    for mut joint in joints.iter_mut() {
        joint.clear_lagrange_multipliers();
    }
}

/// Solves position constraints for dynamic-dynamic interactions.
pub(crate) fn solve_pos(
    mut bodies: Query<(RigidBodyQuery, &ColliderShape)>,
    broad_collision_pairs: Res<BroadCollisionPairs>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    sub_dt: Res<SubDeltaTime>,
) {
    penetration_constraints.0.clear();

    // Handle non-penetration constraints
    for (ent_a, ent_b) in broad_collision_pairs.0.iter().cloned() {
        // Get components for entity a and b
        if let Ok([(mut body1, collider_shape_a), (mut body2, collider_shape_b)]) =
            bodies.get_many_mut([ent_a, ent_b])
        {
            // No need to solve collisions if neither of the bodies is dynamic
            if !body1.rb.is_dynamic() && !body2.rb.is_dynamic() {
                continue;
            }

            // Detect if the entities are actually colliding and get contact data
            if let Some(contact) = get_contact(
                ent_a,
                ent_b,
                body1.pos.0,
                body2.pos.0,
                body1.local_com.0,
                body2.local_com.0,
                &body1.rot,
                &body2.rot,
                &collider_shape_a.0,
                &collider_shape_b.0,
            ) {
                let mut constraint = PenetrationConstraint::new(ent_a, ent_b, contact);

                constraint.constrain(&mut body1, &mut body2, sub_dt.0);

                penetration_constraints.0.push(constraint);
            }
        }
    }
}

pub(crate) fn joint_constraints<T: Joint>(
    mut bodies: Query<RigidBodyQuery>,
    mut joints: Query<&mut T, Without<Pos>>,
    num_pos_iters: Res<NumPosIters>,
    sub_dt: Res<SubDeltaTime>,
) {
    for _j in 0..num_pos_iters.0 {
        for mut joint in joints.iter_mut() {
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
pub(crate) fn update_lin_vel(
    mut bodies: Query<(&RigidBody, &Pos, &PrevPos, &mut LinVel, &mut PreSolveLinVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, pos, prev_pos, mut lin_vel, mut pre_solve_lin_vel) in bodies.iter_mut() {
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
pub(crate) fn update_ang_vel(
    mut bodies: Query<(&RigidBody, &Rot, &PrevRot, &mut AngVel, &mut PreSolveAngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in bodies.iter_mut() {
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
pub(crate) fn update_ang_vel(
    mut bodies: Query<(&RigidBody, &Rot, &PrevRot, &mut AngVel, &mut PreSolveAngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, rot, prev_rot, mut ang_vel, mut pre_solve_ang_vel) in bodies.iter_mut() {
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
pub(crate) fn solve_vel(
    mut bodies: Query<RigidBodyQuery>,
    penetration_constraints: Res<PenetrationConstraints>,
    gravity: Res<Gravity>,
    sub_dt: Res<SubDeltaTime>,
) {
    for constraint in penetration_constraints.0.iter().cloned() {
        let Contact {
            entity_a,
            entity_b,
            world_r_a: r_a,
            world_r_b: r_b,
            normal,
            ..
        } = constraint.contact_data;

        if let Ok([mut body1, mut body2]) = bodies.get_many_mut([entity_a, entity_b]) {
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
                body1.inv_mass.0,
                inv_inertia1,
                r_a,
                normal,
            );
            let w_b = PenetrationConstraint::get_generalized_inverse_mass(
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

pub(crate) fn joint_damping<T: Joint>(
    mut bodies: Query<(&RigidBody, &mut LinVel, &mut AngVel, &InvMass)>,
    joints: Query<&T, Without<RigidBody>>,
    sub_dt: Res<SubDeltaTime>,
) {
    for joint in joints.iter() {
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
            let w_a = inv_mass_a.0;
            let w_b = inv_mass_b.0;

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

/// Copies positions from the physics world to bevy Transforms
#[cfg(feature = "2d")]
pub(crate) fn sync_transforms(mut bodies: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in bodies.iter_mut() {
        transform.translation = pos.extend(0.0);
        transform.rotation = (*rot).into();
    }
}

/// Copies positions from the physics world to bevy Transforms
#[cfg(feature = "3d")]
pub(crate) fn sync_transforms(mut bodies: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in bodies.iter_mut() {
        transform.translation = pos.0;
        transform.rotation = rot.0;
    }
}

type AABBChanged = Or<(
    Changed<ColliderShape>,
    Changed<ColliderAabb>,
    Changed<Pos>,
    Changed<Rot>,
    Changed<LinVel>,
)>;

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
pub(crate) fn update_aabb(
    mut bodies: Query<(ColliderQuery, &Pos, &Rot, Option<&LinVel>), AABBChanged>,
) {
    let safety_margin_factor = 2.0 * DELTA_TIME;

    for (mut collider, pos, rot, lin_vel) in bodies.iter_mut() {
        // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
        let lin_vel = if let Some(lin_vel) = lin_vel {
            lin_vel.0
        } else {
            Vector::ZERO
        };
        collider.aabb.0 = collider
            .shape
            .compute_aabb(&Isometry::new(pos.0.into(), (*rot).into()));
        collider.aabb_with_margin.0 = collider
            .aabb
            .with_margin(safety_margin_factor * lin_vel.length());
    }
}

pub(crate) fn update_sub_delta_time(substeps: Res<NumSubsteps>, mut sub_dt: ResMut<SubDeltaTime>) {
    sub_dt.0 = DELTA_TIME / substeps.0 as f32;
}

type MassPropsChanged = Or<(
    Changed<Mass>,
    Changed<InvMass>,
    Changed<Inertia>,
    Changed<InvInertia>,
    Changed<ColliderShape>,
    Changed<ColliderMassProperties>,
)>;

/// Updates each body's mass properties whenever their dependant mass properties or the body's ,[`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
pub(crate) fn update_mass_props(
    mut bodies: Query<(MassPropsQueryMut, Option<ColliderQuery>), MassPropsChanged>,
) {
    for (mut mass_props, collider) in bodies.iter_mut() {
        if mass_props.mass.is_changed() && mass_props.mass.0 >= f32::EPSILON {
            mass_props.inv_mass.0 = 1.0 / mass_props.mass.0;
        }

        if let Some(mut collider) = collider {
            // Subtract previous collider mass props from the body's mass props
            mass_props -= collider.prev_mass_props.0;

            // Update previous and current collider mass props
            collider.prev_mass_props.0 = *collider.mass_props;
            *collider.mass_props = ColliderMassProperties::from_shape_and_density(
                &collider.shape.0,
                collider.mass_props.density,
            );

            // Add new collider mass props to the body's mass props
            mass_props += *collider.mass_props;
        }

        if mass_props.mass.0 < f32::EPSILON {
            mass_props.mass.0 = f32::EPSILON;
        }
        if mass_props.inv_mass.0 < f32::EPSILON {
            mass_props.inv_mass.0 = f32::EPSILON;
        }
    }
}
