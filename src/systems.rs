//! XPBD systems and core logic.
//!
//! The math and physics are primarily from [Matthias Müller's paper titled "Detailed Rigid Body Simulation with Extended Position Based Dynamics"](https://matthias-research.github.io/pages/publications/PBDBodies.pdf).

use crate::{components::*, resources::*, utils::*, Contact, DELTA_TIME};

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
        &Density,
    )>,
    gravity: Res<Gravity>,
    sub_dt: Res<SubDeltaTime>,
) {
    for (mut pos, mut prev_pos, mut vel, mut pre_solve_vel, mass) in query.iter_mut() {
        prev_pos.0 = pos.0;

        let gravitation_force = mass.0 * gravity.0;
        let external_forces = gravitation_force;
        vel.0 += sub_dt.0 * external_forces / mass.0;
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
    mut query: Query<(
        &mut Pos,
        &mut Rot,
        &PrevPos,
        &Friction,
        &Collider,
        &MassProperties,
    )>,
    mut penetration_constraints: ResMut<DynamicPenetrationConstraints>,
    mut contacts: ResMut<DynamicContacts>,
    sub_dt: Res<SubDeltaTime>,
) {
    // Handle non-penetration constraints
    for constraint in penetration_constraints.0.iter_mut() {
        // Get components for entity a and b
        if let Ok(
            [(mut pos_a, mut rot_a, prev_pos_a, friction_a, collider_a, mass_props_a), (mut pos_b, mut rot_b, prev_pos_b, friction_b, collider_b, mass_props_b)],
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
                constrain_penetration_dynamics(
                    &mut pos_a,
                    &mut pos_b,
                    &mut rot_a,
                    &mut rot_b,
                    prev_pos_a,
                    prev_pos_b,
                    friction_a,
                    friction_b,
                    mass_props_a,
                    mass_props_b,
                    contact,
                    &mut constraint.lagrange,
                    constraint.compliance,
                    sub_dt.0,
                );

                contacts.0.push(contact);
            }
        }
    }
}

/// Solves position constraints for dynamic-static interactions.
pub(crate) fn solve_pos_statics(
    mut dynamics: Query<(
        &mut Pos,
        &mut Rot,
        &PrevPos,
        &Friction,
        &Collider,
        &MassProperties,
    )>,
    statics: Query<(&Pos, &Rot, &Friction, &Collider), Without<MassProperties>>,
    mut penetration_constraints: ResMut<StaticPenetrationConstraints>,
    mut contacts: ResMut<StaticContacts>,
    sub_dt: Res<SubDeltaTime>,
) {
    // Handle non-penetration constraints
    for constraint in penetration_constraints.0.iter_mut() {
        // Get components for dynamic entity a
        if let Ok((mut pos_a, mut rot_a, prev_pos_a, friction_a, collider_a, mass_props_a)) =
            dynamics.get_mut(constraint.entity_a)
        {
            // Get components for static entity b
            if let Ok((pos_b, rot_b, friction_b, collider_b)) = statics.get(constraint.entity_b) {
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
                    constrain_penetration_statics(
                        &mut pos_a,
                        &mut rot_a,
                        prev_pos_a,
                        friction_a,
                        friction_b,
                        mass_props_a,
                        contact,
                        &mut constraint.lagrange,
                        constraint.compliance,
                        sub_dt.0,
                    );

                    contacts.0.push(contact);
                }
            }
        }
    }
}

/// Solves overlap between two dynamic bodies according to their masses.
#[allow(clippy::too_many_arguments)]
fn constrain_penetration_dynamics(
    pos_a: &mut Pos,
    pos_b: &mut Pos,
    rot_a: &mut Rot,
    rot_b: &mut Rot,
    prev_pos_a: &PrevPos,
    prev_pos_b: &PrevPos,
    friction_a: &Friction,
    friction_b: &Friction,
    mass_props_a: &MassProperties,
    mass_props_b: &MassProperties,
    contact: Contact,
    lagrange: &mut f32,
    compliance: f32,
    sub_dt: f32,
) {
    let Contact {
        normal,
        penetration,
        r_a,
        r_b,
        ..
    } = contact;
    let MassProperties {
        inv_mass: inv_mass_a,
        inv_inertia: inv_inertia_a,
        ..
    } = mass_props_a;
    let MassProperties {
        inv_mass: inv_mass_b,
        inv_inertia: inv_inertia_b,
        ..
    } = mass_props_b;

    // Compute generalized inverse masses (equations 2-3)
    let w_a = inv_mass_a + inv_inertia_a * r_a.perp_dot(normal).powi(2);
    let w_b = inv_mass_b + inv_inertia_b * r_b.perp_dot(normal).powi(2);

    // Compute Lagrange multiplier updates (equations 4-5)
    let a = compliance / sub_dt.powi(2);
    let delta_lagrange = (-penetration - a * *lagrange) / (w_a + w_b + a);
    *lagrange += delta_lagrange;

    // Positional impulse
    let p = delta_lagrange * normal;

    // Update positions and rotations of the bodies (equations 6-9)
    pos_a.0 += p * w_a;
    pos_b.0 -= p * w_b;
    *rot_a += Rot::from_radians(inv_inertia_a * r_a.perp_dot(p));
    *rot_b += Rot::from_radians(inv_inertia_b * r_b.perp_dot(-p));

    // Compute static friction
    let friction = get_static_friction(
        pos_a.0 - prev_pos_a.0,
        pos_b.0 - prev_pos_b.0,
        friction_a,
        friction_b,
        normal,
        delta_lagrange,
    );
    // Apply static friction
    pos_a.0 -= friction;
    pos_b.0 += friction;
}

/// Solves overlap between a dynamic body and a static body.
#[allow(clippy::too_many_arguments)]
fn constrain_penetration_statics(
    pos: &mut Pos,
    rot: &mut Rot,
    prev_pos: &PrevPos,
    friction_a: &Friction,
    friction_b: &Friction,
    mass_props: &MassProperties,
    contact: Contact,
    lagrange: &mut f32,
    compliance: f32,
    sub_dt: f32,
) {
    let Contact {
        normal,
        penetration,
        r_a,
        ..
    } = contact;
    let MassProperties {
        inv_mass,
        inv_inertia,
        ..
    } = *mass_props;

    // Compute generalized inverse mass (equation 2)
    let w_a = inv_mass + inv_inertia * r_a.perp_dot(normal).powi(2);

    // Compute Lagrange multiplier updates (equations 4-5)
    let a = compliance / sub_dt.powi(2);
    let delta_lagrange = (-penetration - a * *lagrange) / (w_a + a);
    *lagrange += delta_lagrange;

    // Positional impulse
    let p = delta_lagrange * normal;

    // Update position and rotation of the dynamic body (equations 6 and 8)
    pos.0 += p * inv_mass;
    *rot += Rot::from_radians(inv_inertia * r_a.perp_dot(p));

    // Compute and apply static friction
    pos.0 -= get_static_friction(
        pos.0 - prev_pos.0,
        Vec2::ZERO,
        friction_a,
        friction_b,
        normal,
        delta_lagrange,
    );
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
        ang_vel.0 = (prev_rot.inv().mul(*rot)).as_radians() / sub_dt.0;
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
        &Density,
        &Collider,
    )>,
    contacts: Res<DynamicContacts>,
    sub_dt: Res<SubDeltaTime>,
) {
    for Contact {
        entity_a,
        entity_b,
        r_a,
        r_b,
        normal,
        penetration,
        ..
    } in contacts.0.iter().cloned()
    {
        if let Ok(
            [(
                mut lin_vel_a,
                mut ang_vel_a,
                pre_solve_lin_vel_a,
                pre_solve_ang_vel_a,
                restitution_a,
                friction_a,
                density_a,
                collider_a,
            ), (
                mut lin_vel_b,
                mut ang_vel_b,
                pre_solve_lin_vel_b,
                pre_solve_ang_vel_b,
                restitution_b,
                friction_b,
                density_b,
                collider_b,
            )],
        ) = query.get_many_mut([entity_a, entity_b])
        {
            let MassProperties {
                mass: mass_a,
                inv_mass: inv_mass_a,
                inv_inertia: inv_inertia_a,
                ..
            } = collider_a.mass_properties(density_a.0);
            let MassProperties {
                mass: mass_b,
                inv_mass: inv_mass_b,
                inv_inertia: inv_inertia_b,
                ..
            } = collider_b.mass_properties(density_b.0);

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
            let friction_impulse =
                get_dynamic_friction(tangent_vel, penetration, friction_a, friction_b, sub_dt.0);

            // Compute restitution
            let restitution_impulse = get_restitution(
                normal,
                normal_vel,
                pre_solve_normal_vel,
                restitution_a,
                restitution_b,
            );

            // Compute velocity impulse and apply velocity updates (equation 33)
            let p = (restitution_impulse - friction_impulse) / (w_a + w_b);
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
        &Density,
        &Collider,
    )>,
    statics: Query<(&Restitution, &Friction), Without<Density>>,
    contacts: Res<StaticContacts>,
    sub_dt: Res<SubDeltaTime>,
) {
    for Contact {
        entity_a,
        entity_b,
        r_a: r,
        normal,
        penetration,
        ..
    } in contacts.0.iter().cloned()
    {
        if let Ok((
            mut lin_vel,
            mut ang_vel,
            pre_solve_lin_vel,
            pre_solve_ang_vel,
            restitution_a,
            friction_a,
            density,
            collider,
        )) = dynamics.get_mut(entity_a)
        {
            if let Ok((restitution_b, friction_b)) = statics.get(entity_b) {
                let MassProperties {
                    mass,
                    inv_mass,
                    inv_inertia,
                    ..
                } = collider.mass_properties(density.0);

                // Compute pre-solve normal velocity at the contact point (used for restitution)
                let pre_solve_contact_vel = pre_solve_lin_vel.0 + pre_solve_ang_vel.0 * r.perp();
                let pre_solve_normal_vel = Vec2::dot(pre_solve_contact_vel, normal);

                // Compute normal and tangential velocity at the contact point (equation 29)
                let contact_vel = lin_vel.0 + ang_vel.0 * r.perp();
                let normal_vel = Vec2::dot(contact_vel, normal);
                let tangent_vel = contact_vel - normal * normal_vel;

                // Compute generalized inverse mass (equation 2)
                let w_a = inv_mass + inv_inertia * r.perp_dot(normal).powi(2);

                // Compute dynamic friction
                let friction_impulse = get_dynamic_friction(
                    tangent_vel,
                    penetration,
                    friction_a,
                    friction_b,
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
                let p = (restitution_impulse - friction_impulse) / w_a;
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

type MassPropsChanged = Or<(Changed<Density>, Changed<Collider>)>;

pub(crate) fn update_mass_props(
    mut query: Query<(&mut MassProperties, &Density, &Collider), MassPropsChanged>,
) {
    for (mut mass_props, density, collider) in query.iter_mut() {
        *mass_props = collider.mass_properties(density.0);
    }
}
