//! XPBD systems and core logic.
//!
//! The math and physics are primarily from [Matthias Müller's paper titled "Detailed Rigid Body Simulation with Extended Position Based Dynamics"](https://matthias-research.github.io/pages/publications/PBDBodies.pdf).

use crate::{
    components::*, constraints::penetration::PenetrationConstraint, resources::*, utils::*,
    Contact, DELTA_TIME,
};

use bevy::prelude::*;
use parry2d::bounding_volume::BoundingVolume;

/// Collects bodies that are potentially colliding and adds non-penetration constraints for them.
pub(crate) fn collect_collision_pairs(
    query: Query<(Entity, &Collider, Option<&MassProperties>)>,
    mut dynamic_penetration_constraints: ResMut<DynamicPenetrationConstraints>,
    mut static_penetration_constraints: ResMut<StaticPenetrationConstraints>,
) {
    dynamic_penetration_constraints.0.clear();
    static_penetration_constraints.0.clear();

    for [(ent_a, collider_a, mass_props_a), (ent_b, collider_b, mass_props_b)] in
        query.iter_combinations()
    {
        let is_dynamic_a = mass_props_a.is_some();
        let is_dynamic_b = mass_props_b.is_some();

        // At least one of the bodies is dynamic and their AABBs intersect
        if (is_dynamic_a || is_dynamic_b) && collider_a.aabb.intersects(&collider_b.aabb) {
            if is_dynamic_a && is_dynamic_b {
                // Both are dynamic
                dynamic_penetration_constraints
                    .0
                    .push(PenetrationConstraint::new_with_compliance(
                        ent_a, ent_b, 0.0,
                    ));
            } else if is_dynamic_a {
                // A is dynamic, B is static
                static_penetration_constraints
                    .0
                    .push(PenetrationConstraint::new_with_compliance(
                        ent_a, ent_b, 0.0,
                    ));
            } else if is_dynamic_b {
                // B is dynamic, A is static
                static_penetration_constraints
                    .0
                    .push(PenetrationConstraint::new_with_compliance(
                        ent_b, ent_a, 0.0,
                    ));
            }
        }
    }
}

/// Applies forces and predicts the next position and velocity for all dynamic bodies.
pub(crate) fn integrate_pos(
    mut query: Query<(
        &mut Pos,
        &mut PrevPos,
        &mut LinVel,
        &mut PreSolveLinVel,
        &MassProperties,
    )>,
    gravity: Res<Gravity>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (mut pos, mut prev_pos, mut vel, mut pre_solve_vel, mass_props) in query.iter_mut() {
        prev_pos.0 = pos.0;

        let gravitation_force = mass_props.mass * gravity.0;
        let external_forces = gravitation_force;
        vel.0 += sub_dt.0 * external_forces / mass_props.mass;
        pos.0 += sub_dt.0 * vel.0;
        pre_solve_vel.0 = vel.0;
    }
}

/// Integrates rotations for all dynamic bodies.
pub(crate) fn integrate_rot(
    mut query: Query<(&mut Rot, &mut PrevRot, &AngVel, &mut PreSolveAngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (mut rot, mut prev_rot, ang_vel, mut pre_solve_ang_vel) in query.iter_mut() {
        prev_rot.cos = rot.cos;
        prev_rot.sin = rot.sin;
        *rot += Rot::from_radians(sub_dt.0 * ang_vel.0);
        pre_solve_ang_vel.0 = ang_vel.0;
    }
}

/// Solves position constraints for dynamic-dynamic interactions.
pub(crate) fn solve_pos_dynamics(
    mut query: Query<(&mut Pos, &mut Rot, &Collider, &MassProperties)>,
    mut penetration_constraints: ResMut<DynamicPenetrationConstraints>,
    mut contacts: ResMut<DynamicContacts>,
    sub_dt: Res<SubDeltaTime>,
) {
    // Handle non-penetration constraints
    for constraint in penetration_constraints.0.iter_mut() {
        // Get components for entity a and b
        if let Ok(
            [(mut pos_a, mut rot_a, collider_a, mass_props_a), (mut pos_b, mut rot_b, collider_b, mass_props_b)],
        ) = query.get_many_mut([constraint.entity_a, constraint.entity_b])
        {
            // Detect if the entities are actually colliding and get contact data
            if let Some(contact) = get_contact(
                constraint.entity_a,
                constraint.entity_b,
                pos_a.0,
                pos_b.0,
                rot_a.as_radians(),
                rot_b.as_radians(),
                &collider_a.shape,
                &collider_b.shape,
            ) {
                // Apply non-penetration constraints for dynamic-dynamic interactions
                constraint.constrain_dynamic(
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

/// Solves position constraints for dynamic-static interactions.
pub(crate) fn solve_pos_statics(
    mut dynamics: Query<(&mut Pos, &mut Rot, &Collider, &MassProperties)>,
    statics: Query<(&Pos, &Rot, &Collider), Without<MassProperties>>,
    mut penetration_constraints: ResMut<StaticPenetrationConstraints>,
    mut contacts: ResMut<StaticContacts>,
    sub_dt: Res<SubDeltaTime>,
) {
    // Handle non-penetration constraints
    for constraint in penetration_constraints.0.iter_mut() {
        // Get components for dynamic entity a
        if let Ok((mut pos_a, mut rot_a, collider_a, mass_props_a)) =
            dynamics.get_mut(constraint.entity_a)
        {
            // Get components for static entity b
            if let Ok((pos_b, rot_b, collider_b)) = statics.get(constraint.entity_b) {
                // Detect if the entities are actually colliding and get contact data
                if let Some(contact) = get_contact(
                    constraint.entity_a,
                    constraint.entity_b,
                    pos_a.0,
                    pos_b.0,
                    rot_a.as_radians(),
                    rot_b.as_radians(),
                    &collider_a.shape,
                    &collider_b.shape,
                ) {
                    // Apply non-penetration constraints for dynamic-static interactions
                    constraint.constrain_static(
                        &mut pos_a,
                        &mut rot_a,
                        mass_props_a,
                        contact,
                        sub_dt.0,
                    );

                    contacts.0.push((contact, constraint.normal_lagrange));
                }
            }
        }
    }
}

/// Updates the linear velocity of all dynamic bodies based on the change in position from the previous step.
pub(crate) fn update_lin_vel(
    mut query: Query<(&Pos, &PrevPos, &mut LinVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (pos, prev_pos, mut vel) in query.iter_mut() {
        // v = (x - x_prev) / h
        vel.0 = (pos.0 - prev_pos.0) / sub_dt.0;
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the previous step.
pub(crate) fn update_ang_vel(
    mut query: Query<(&Rot, &PrevRot, &mut AngVel)>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (rot, prev_rot, mut ang_vel) in query.iter_mut() {
        ang_vel.0 = (rot.mul(prev_rot.inv())).as_radians() / sub_dt.0;
    }
}

/// Solves the new velocities of dynamic bodies after dynamic-dynamic collisions.
#[allow(clippy::type_complexity)]
pub(crate) fn solve_vel_dynamics(
    mut query: Query<(
        &mut LinVel,
        &mut AngVel,
        &PreSolveLinVel,
        &PreSolveAngVel,
        &Restitution,
        &Friction,
        &MassProperties,
    )>,
    contacts: Res<DynamicContacts>,
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
                mut lin_vel_a,
                mut ang_vel_a,
                pre_solve_lin_vel_a,
                pre_solve_ang_vel_a,
                restitution_a,
                friction_a,
                mass_props_a,
            ), (
                mut lin_vel_b,
                mut ang_vel_b,
                pre_solve_lin_vel_b,
                pre_solve_ang_vel_b,
                restitution_b,
                friction_b,
                mass_props_b,
            )],
        ) = query.get_many_mut([entity_a, entity_b])
        {
            let MassProperties {
                mass: mass_a,
                inv_mass: inv_mass_a,
                inv_inertia: inv_inertia_a,
                ..
            } = *mass_props_a;
            let MassProperties {
                mass: mass_b,
                inv_mass: inv_mass_b,
                inv_inertia: inv_inertia_b,
                ..
            } = *mass_props_b;

            // Compute pre-solve relative normal velocities at the contact point (used for restitution)
            let pre_solve_contact_vel_a =
                pre_solve_lin_vel_a.0 + pre_solve_ang_vel_a.0 * r_a.perp();
            let pre_solve_contact_vel_b =
                pre_solve_lin_vel_b.0 + pre_solve_ang_vel_b.0 * r_b.perp();
            let pre_solve_relative_vel = pre_solve_contact_vel_a - pre_solve_contact_vel_b;
            let pre_solve_normal_vel = normal.dot(pre_solve_relative_vel);

            // Compute relative normal and tangential velocities at the contact point (equation 29)
            let contact_vel_a = lin_vel_a.0 + ang_vel_a.0 * r_a.perp();
            let contact_vel_b = lin_vel_b.0 + ang_vel_b.0 * r_b.perp();
            let relative_vel = contact_vel_a - contact_vel_b;
            let normal_vel = normal.dot(relative_vel);
            let tangent_vel = relative_vel - normal * normal_vel;

            // Compute generalized inverse masses (equations 2-3)
            let w_a = inv_mass_a + inv_inertia_a * r_a.perp_dot(normal).powi(2);
            let w_b = inv_mass_b + inv_inertia_b * r_b.perp_dot(normal).powi(2);

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
            );

            // Compute velocity impulse and apply velocity updates (equation 33)
            let p = (restitution_impulse + friction_impulse) / (w_a + w_b);
            lin_vel_a.0 += p / mass_a;
            lin_vel_b.0 -= p / mass_b;
            ang_vel_a.0 += inv_inertia_a * r_a.perp_dot(p);
            ang_vel_b.0 -= inv_inertia_b * r_b.perp_dot(p);
        }
    }
}

/// Solves the new velocities of dynamic bodies after dynamic-static collisions.
#[allow(clippy::type_complexity)]
pub(crate) fn solve_vel_statics(
    mut dynamics: Query<(
        &mut LinVel,
        &mut AngVel,
        &PreSolveLinVel,
        &PreSolveAngVel,
        &Restitution,
        &Friction,
        &MassProperties,
    )>,
    statics: Query<(&Restitution, &Friction), Without<MassProperties>>,
    contacts: Res<StaticContacts>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (
        Contact {
            entity_a,
            entity_b,
            r_a: r,
            normal,
            ..
        },
        normal_lagrange,
    ) in contacts.0.iter().cloned()
    {
        if let Ok((
            mut lin_vel,
            mut ang_vel,
            pre_solve_lin_vel,
            pre_solve_ang_vel,
            restitution_a,
            friction_a,
            mass_props,
        )) = dynamics.get_mut(entity_a)
        {
            if let Ok((restitution_b, friction_b)) = statics.get(entity_b) {
                let MassProperties {
                    mass,
                    inv_mass,
                    inv_inertia,
                    ..
                } = *mass_props;

                // Compute pre-solve normal velocity at the contact point (used for restitution)
                let pre_solve_contact_vel = pre_solve_lin_vel.0 + pre_solve_ang_vel.0 * r.perp();
                let pre_solve_normal_vel = normal.dot(pre_solve_contact_vel);

                // Compute normal and tangential velocity at the contact point (equation 29)
                let contact_vel = lin_vel.0 + ang_vel.0 * r.perp();
                let normal_vel = normal.dot(contact_vel);
                let tangent_vel = contact_vel - normal * normal_vel;

                // Compute generalized inverse mass (equation 2)
                let w_a = inv_mass + inv_inertia * r.perp_dot(normal).powi(2);

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
                );

                // Compute velocity impulse and apply velocity updates (equation 33)
                let p = (restitution_impulse + friction_impulse) / w_a;
                lin_vel.0 += p / mass;
                ang_vel.0 += inv_inertia * r.perp_dot(p);
            }
        }
    }
}

/// Clears all contact resources.
pub(crate) fn clear_contacts(
    mut contacts: ResMut<DynamicContacts>,
    mut static_contacts: ResMut<StaticContacts>,
) {
    contacts.0.clear();
    static_contacts.0.clear();
}

/// Copies positions from the physics world to bevy Transforms
pub(crate) fn sync_transforms(mut query: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in query.iter_mut() {
        transform.translation = pos.0.extend(0.0);
        transform.rotation = (*rot).into();
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
    mut query: Query<(&mut Collider, &Pos, &Rot, Option<&LinVel>), AABBChanged>,
) {
    for (mut collider, pos, rot, lin_vel) in query.iter_mut() {
        // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
        let safety_margin_factor = 2.0 * DELTA_TIME;
        let lin_vel = if let Some(lin_vel) = lin_vel {
            lin_vel.0
        } else {
            Vec2::ZERO
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

pub(crate) fn update_sub_delta_time(substeps: Res<XPBDSubsteps>, mut sub_dt: ResMut<SubDeltaTime>) {
    sub_dt.0 = DELTA_TIME / substeps.0 as f32;
}

type MassPropsChanged = Or<(Changed<ExplicitMassProperties>, Changed<Collider>)>;

/// Updates each body's [`MassProperties`] whenever their [`ExplicitMassProperties`] or [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
pub(crate) fn update_mass_props(
    mut query: Query<
        (
            &mut MassProperties,
            Option<&ExplicitMassProperties>,
            Option<&mut Collider>,
        ),
        MassPropsChanged,
    >,
) {
    for (mut mass_props, explicit_mass_props, collider) in query.iter_mut() {
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
