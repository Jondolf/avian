//! XPBD systems and core logic.
//!
//! The math and physics are primarily from [Matthias Müller's paper titled "Detailed Rigid Body Simulation with Extended Position Based Dynamics"](https://matthias-research.github.io/pages/publications/PBDBodies.pdf).

use crate::{
    components::*,
    constraints::{
        joints::SphericalJoint, penetration::PenetrationConstraint, Constraint, PositionConstraint,
    },
    resources::*,
    utils::*,
    *,
};

use bevy::prelude::*;
use parry::bounding_volume::BoundingVolume;

/// Collects bodies that are potentially colliding and adds non-penetration constraints for them.
pub(crate) fn collect_collision_pairs(
    bodies: Query<(Entity, &Collider, &RigidBody)>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
) {
    penetration_constraints.0.clear();

    for [(ent_a, collider_a, rb_a), (ent_b, collider_b, rb_b)] in bodies.iter_combinations() {
        // At least one of the bodies is dynamic and their AABBs intersect
        if (*rb_a == RigidBody::Dynamic || *rb_b == RigidBody::Dynamic)
            && collider_a.aabb.intersects(&collider_b.aabb)
        {
            penetration_constraints
                .0
                .push(PenetrationConstraint::new_with_compliance(
                    ent_b, ent_a, 0.0,
                ));
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
        &mut PreSolveLinVel,
        &MassProperties,
    )>,
    gravity: Res<Gravity>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, mut pos, mut prev_pos, mut vel, mut pre_solve_vel, mass_props) in bodies.iter_mut() {
        prev_pos.0 = pos.0;

        if *rb != RigidBody::Static {
            let gravitation_force = mass_props.mass * gravity.0;
            let external_forces = gravitation_force;
            vel.0 += sub_dt.0 * external_forces / mass_props.mass;
        }
        pos.0 += sub_dt.0 * vel.0;
        pre_solve_vel.0 = vel.0;
    }
}

/// Integrates rotations for all dynamic bodies.
#[cfg(feature = "2d")]
pub(crate) fn integrate_rot(
    mut bodies: Query<(&mut Rot, &mut PrevRot, &AngVel, &mut PreSolveAngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (mut rot, mut prev_rot, ang_vel, mut pre_solve_ang_vel) in bodies.iter_mut() {
        prev_rot.0 = *rot;
        *rot += Rot::from_radians(sub_dt.0 * ang_vel.0);
        pre_solve_ang_vel.0 = ang_vel.0;
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
        &mut PreSolveAngVel,
        &MassProperties,
    )>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rb, mut rot, mut prev_rot, mut ang_vel, mut pre_solve_ang_vel, mass_props) in
        bodies.iter_mut()
    {
        prev_rot.0 = *rot;

        if *rb != RigidBody::Static {
            let delta_ang_vel = sub_dt.0
                * get_rotated_inertia_tensor(mass_props.inv_inertia, rot.0)
                * (-ang_vel
                    .cross(get_rotated_inertia_tensor(mass_props.inertia, rot.0) * ang_vel.0));
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

        pre_solve_ang_vel.0 = ang_vel.0;
    }
}

pub(crate) fn clear_constraint_lagrange(
    mut joints: Query<&mut SphericalJoint>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
) {
    for mut joint in joints.iter_mut() {
        joint.clear_lagrange_multipliers();
    }
    for constraint in penetration_constraints.0.iter_mut() {
        constraint.clear_lagrange_multipliers();
    }
}

/// Solves position constraints for dynamic-dynamic interactions.
pub(crate) fn solve_pos(
    mut bodies: Query<(&mut Pos, &mut Rot, &Collider, &MassProperties, &RigidBody)>,
    mut penetration_constraints: ResMut<PenetrationConstraints>,
    mut contacts: ResMut<Contacts>,
    sub_dt: Res<SubDeltaTime>,
) {
    // Handle non-penetration constraints
    for constraint in penetration_constraints.0.iter_mut() {
        // Get components for entity a and b
        if let Ok(
            [(mut pos_a, mut rot_a, collider_a, mass_props_a, rb_a), (mut pos_b, mut rot_b, collider_b, mass_props_b, rb_b)],
        ) = bodies.get_many_mut([constraint.entity_a, constraint.entity_b])
        {
            // No need to solve collisions if both bodies are static
            if *rb_a == RigidBody::Static && *rb_b == RigidBody::Static {
                continue;
            }

            // Detect if the entities are actually colliding and get contact data
            if let Some(contact) = get_contact(
                constraint.entity_a,
                constraint.entity_b,
                pos_a.0,
                pos_b.0,
                mass_props_a.local_center_of_mass,
                mass_props_b.local_center_of_mass,
                &rot_a,
                &rot_b,
                &collider_a.shape,
                &collider_b.shape,
            ) {
                // Apply non-penetration constraints for dynamic-dynamic interactions
                constraint.constrain(
                    rb_a,
                    rb_b,
                    &mut pos_a,
                    &mut pos_b,
                    &mut rot_a,
                    &mut rot_b,
                    mass_props_a,
                    mass_props_b,
                    contact,
                    sub_dt.0,
                );

                contacts.0.push((contact, constraint.normal_lagrange));
            }
        }
    }
}

pub(crate) fn joint_constraints(
    mut bodies: Query<(&RigidBody, &mut Pos, &mut Rot, &MassProperties), With<MassProperties>>,
    mut joints: Query<&mut SphericalJoint, Without<Pos>>,
    sub_dt: Res<SubDeltaTime>,
    num_pos_iters: Res<NumPosIters>,
) {
    for _ in 0..num_pos_iters.0 {
        for mut joint in joints.iter_mut() {
            // Get components for entity a and b
            if let Ok(
                [(rb_a, mut pos_a, mut rot_a, mass_props_a), (rb_b, mut pos_b, mut rot_b, mass_props_b)],
            ) = bodies.get_many_mut([joint.entity_a, joint.entity_b])
            {
                // No need to solve constraints if both bodies are static
                if *rb_a == RigidBody::Static && *rb_b == RigidBody::Static {
                    continue;
                }

                joint.constrain(
                    rb_a,
                    rb_b,
                    &mut pos_a,
                    &mut pos_b,
                    &mut rot_a,
                    &mut rot_b,
                    mass_props_a,
                    mass_props_b,
                    sub_dt.0,
                );
            }
        }
    }
}

/// Updates the linear velocity of all dynamic bodies based on the change in position from the previous step.
pub(crate) fn update_lin_vel(
    mut bodies: Query<(&Pos, &PrevPos, &mut LinVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (pos, prev_pos, mut vel) in bodies.iter_mut() {
        // v = (x - x_prev) / h
        vel.0 = (pos.0 - prev_pos.0) / sub_dt.0;
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the previous step.
#[cfg(feature = "2d")]
pub(crate) fn update_ang_vel(
    mut bodies: Query<(&Rot, &PrevRot, &mut AngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rot, prev_rot, mut ang_vel) in bodies.iter_mut() {
        ang_vel.0 = (rot.mul(prev_rot.inv())).as_radians() / sub_dt.0;
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the previous step.
#[cfg(feature = "3d")]
pub(crate) fn update_ang_vel(
    mut bodies: Query<(&Rot, &PrevRot, &mut AngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rot, prev_rot, mut ang_vel) in bodies.iter_mut() {
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
    mut bodies: Query<(
        &Rot,
        &mut LinVel,
        &mut AngVel,
        &PreSolveLinVel,
        &PreSolveAngVel,
        &Restitution,
        &Friction,
        &MassProperties,
        &RigidBody,
    )>,
    contacts: Res<Contacts>,
    gravity: Res<Gravity>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (
        Contact {
            entity_a,
            entity_b,
            r_a,
            r_b,
            normal,
            ..
        },
        normal_lagrange,
    ) in contacts.0.iter().cloned()
    {
        if let Ok(
            [(
                rot_a,
                mut lin_vel_a,
                mut ang_vel_a,
                pre_solve_lin_vel_a,
                pre_solve_ang_vel_a,
                restitution_a,
                friction_a,
                mass_props_a,
                rb_a,
            ), (
                rot_b,
                mut lin_vel_b,
                mut ang_vel_b,
                pre_solve_lin_vel_b,
                pre_solve_ang_vel_b,
                restitution_b,
                friction_b,
                mass_props_b,
                rb_b,
            )],
        ) = bodies.get_many_mut([entity_a, entity_b])
        {
            // Compute pre-solve relative normal velocities at the contact point (used for restitution)
            let pre_solve_contact_vel_a =
                compute_contact_vel(pre_solve_lin_vel_a.0, pre_solve_ang_vel_a.0, r_a);
            let pre_solve_contact_vel_b =
                compute_contact_vel(pre_solve_lin_vel_b.0, pre_solve_ang_vel_b.0, r_b);
            let pre_solve_relative_vel = pre_solve_contact_vel_a - pre_solve_contact_vel_b;
            let pre_solve_normal_vel = normal.dot(pre_solve_relative_vel);

            // Compute relative normal and tangential velocities at the contact point (equation 29)
            let contact_vel_a = compute_contact_vel(lin_vel_a.0, ang_vel_a.0, r_a);
            let contact_vel_b = compute_contact_vel(lin_vel_b.0, ang_vel_b.0, r_b);
            let relative_vel = contact_vel_a - contact_vel_b;
            let normal_vel = normal.dot(relative_vel);
            let tangent_vel = relative_vel - normal * normal_vel;

            // Compute generalized inverse masses
            let w_a = PenetrationConstraint::get_generalized_inverse_mass(
                &mass_props_a.with_rotation(rot_a),
                r_a,
                normal,
            );
            let w_b = PenetrationConstraint::get_generalized_inverse_mass(
                &mass_props_b.with_rotation(rot_b),
                r_b,
                normal,
            );

            // Compute dynamic friction
            let friction_impulse = get_dynamic_friction(
                tangent_vel,
                friction_a,
                friction_b,
                normal_lagrange,
                sub_dt.0,
            );

            // Compute restitution
            let restitution_impulse = get_restitution(
                normal,
                normal_vel,
                pre_solve_normal_vel,
                restitution_a,
                restitution_b,
                gravity.0,
                sub_dt.0,
            );

            let delta_v = friction_impulse + restitution_impulse;

            // Compute velocity impulse and apply velocity updates (equation 33)
            let p = delta_v / (w_a + w_b);
            if *rb_a != RigidBody::Static {
                lin_vel_a.0 += p / mass_props_a.mass;
                ang_vel_a.0 += compute_delta_ang_vel(mass_props_a.inv_inertia, r_a, p);
            }
            if *rb_b != RigidBody::Static {
                lin_vel_b.0 -= p / mass_props_b.mass;
                ang_vel_b.0 -= compute_delta_ang_vel(mass_props_b.inv_inertia, r_b, p);
            }
        }
    }
}

pub(crate) fn joint_damping(
    mut bodies: Query<(&RigidBody, &mut LinVel, &mut AngVel, &MassProperties)>,
    joints: Query<&SphericalJoint, Without<RigidBody>>,
    sub_dt: Res<SubDeltaTime>,
) {
    for joint in joints.iter() {
        if let Ok(
            [(rb_a, mut lin_vel_a, mut ang_vel_a, mass_props_a), (rb_b, mut lin_vel_b, mut ang_vel_b, mass_props_b)],
        ) = bodies.get_many_mut([joint.entity_a, joint.entity_b])
        {
            let delta_omega = (ang_vel_b.0 - ang_vel_a.0) * (joint.damping_ang * sub_dt.0).min(1.0);

            if *rb_a != RigidBody::Static {
                ang_vel_a.0 += delta_omega;
            }
            if *rb_b != RigidBody::Static {
                ang_vel_b.0 -= delta_omega;
            }

            let delta_v = (lin_vel_b.0 - lin_vel_a.0) * (joint.damping_lin * sub_dt.0).min(1.0);
            let w_a = mass_props_a.inv_mass;
            let w_b = mass_props_b.inv_mass;

            let p = delta_v / (w_a + w_b);
            if *rb_a != RigidBody::Static {
                lin_vel_a.0 += p / mass_props_a.mass;
            }
            if *rb_b != RigidBody::Static {
                lin_vel_b.0 -= p / mass_props_b.mass;
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

/// Clears all contact resources.
pub(crate) fn clear_contacts(mut contacts: ResMut<Contacts>) {
    contacts.0.clear();
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
    Changed<Collider>,
    Changed<Pos>,
    Changed<Rot>,
    Changed<LinVel>,
)>;

/// Updates the AABBs of all bodies.
pub(crate) fn update_aabb(
    mut bodies: Query<(&mut Collider, &Pos, &Rot, Option<&LinVel>), AABBChanged>,
) {
    for (mut collider, pos, rot, lin_vel) in bodies.iter_mut() {
        // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
        let safety_margin_factor = 2.0 * DELTA_TIME;
        let lin_vel = if let Some(lin_vel) = lin_vel {
            lin_vel.0
        } else {
            Vector::ZERO
        };
        let collider_shape = collider.shape.clone();
        collider.update_aabb_with_margin(
            &pos.0,
            rot,
            &collider_shape,
            safety_margin_factor * lin_vel.length(),
        );
    }
}

pub(crate) fn update_sub_delta_time(substeps: Res<NumSubsteps>, mut sub_dt: ResMut<SubDeltaTime>) {
    sub_dt.0 = DELTA_TIME / substeps.0 as f32;
}

type MassPropsChanged = Or<(Changed<ExplicitMassProperties>, Changed<Collider>)>;

/// Updates each body's [`MassProperties`] whenever their [`ExplicitMassProperties`] or [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
pub(crate) fn update_mass_props(
    mut bodies: Query<
        (
            &mut MassProperties,
            Option<&ExplicitMassProperties>,
            Option<&mut Collider>,
        ),
        MassPropsChanged,
    >,
) {
    for (mut mass_props, explicit_mass_props, collider) in bodies.iter_mut() {
        let mut new_mass_props = MassProperties::ZERO;

        if let Some(explicit_mass_props) = explicit_mass_props {
            new_mass_props = explicit_mass_props.0;
        }

        if let Some(mut collider) = collider {
            collider.update_mass_props();

            new_mass_props.mass += collider.mass_properties.mass;
            new_mass_props.inv_mass += collider.mass_properties.inv_mass;
            new_mass_props.inertia += collider.mass_properties.inertia;
            new_mass_props.inv_inertia += collider.mass_properties.inv_inertia;
            new_mass_props.local_center_of_mass += collider.mass_properties.local_center_of_mass;
        }

        *mass_props = new_mass_props;
    }
}
