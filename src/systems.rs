use crate::{components::*, resources::*, DELTA_TIME, SUB_DT};

use bevy::prelude::*;
use parry2d::{math::Isometry, shape::SharedShape};

/// Collects bodies that are potentially colliding and stores them in pairs in the [`CollisionPairs`] array.
pub(crate) fn collect_collision_pairs(
    query: Query<(Entity, &Pos, &Vel, &ParticleCollider)>,
    mut collision_pairs: ResMut<CollisionPairs>,
) {
    collision_pairs.0.clear();

    let k = 2.0; // Safety margin multiplier bigger than 1 to account for sudden accelerations
    let safety_margin_factor = k * DELTA_TIME;

    let mut iter = query.iter_combinations();
    while let Some([(ent_a, pos_a, vel_a, collider_a), (ent_b, pos_b, vel_b, collider_b)]) =
        iter.fetch_next()
    {
        let vel_a_len = vel_a.length();

        let ab = pos_b.0 - pos_a.0;
        let ab_len = ab.length();
        let safety_margin = safety_margin_factor * (vel_a_len + vel_b.length());

        if let Some(contact) = get_contact(pos_a.0, &collider_a.0, pos_b.0, &collider_b.0, ab_len) {
            // If distance between contact points is negative, it's a penetration.
            // A safety margin is subtracted from the computed penetration depth to make everything stabler.
            if contact.dist - safety_margin < 0.0 {
                collision_pairs.0.push((ent_a, ent_b));
            }
        }
    }
}

/// Applies forces and predicts the next position and velocity for all dynamic bodies.
pub(crate) fn integrate(
    mut query: Query<(&mut Pos, &mut PrevPos, &mut Vel, &mut PreSolveVel, &Mass)>,
    gravity: Res<Gravity>,
) {
    for (mut pos, mut prev_pos, mut vel, mut pre_solve_vel, mass) in query.iter_mut() {
        prev_pos.0 = pos.0;

        let gravitation_force = mass.0 * gravity.0;
        let external_forces = gravitation_force;
        vel.0 += SUB_DT * external_forces / mass.0;
        pos.0 += SUB_DT * vel.0;
        pre_solve_vel.0 = vel.0;
    }
}

/// Solves position constraints for dynamic-dynamic interactions.
pub(crate) fn solve_pos_dynamics(
    mut query: Query<(&mut Pos, &Mass, &ParticleCollider)>,
    collision_pairs: Res<CollisionPairs>,
    mut contacts: ResMut<DynamicContacts>,
) {
    for (ent_a, ent_b) in collision_pairs.0.iter() {
        if let Ok([(mut pos_a, mass_a, collider_a), (mut pos_b, mass_b, collider_b)]) =
            query.get_many_mut([*ent_a, *ent_b])
        {
            let ab = pos_b.0 - pos_a.0;
            let ab_len = ab.length();

            if let Some(contact) =
                get_contact(pos_a.0, &collider_a.0, pos_b.0, &collider_b.0, ab_len)
            {
                // If distance between contact points is negative, it's a penetration.
                if contact.dist < 0.0 {
                    // Contact normal
                    let n = Vec2::new(contact.normal1.x, contact.normal1.y);

                    let w_a = 1.0 / mass_a.0;
                    let w_b = 1.0 / mass_b.0;
                    let w_total = w_a + w_b;

                    pos_a.0 -= n * -contact.dist * w_a / w_total;
                    pos_b.0 += n * -contact.dist * w_b / w_total;

                    contacts.0.push((*ent_a, *ent_b, n));
                }
            }
        }
    }
}

/// Solves position constraints for dynamic-static interactions.
pub(crate) fn solve_pos_statics(
    mut dynamics: Query<(Entity, &mut Pos, &ParticleCollider), With<Mass>>,
    statics: Query<(Entity, &Pos, &ParticleCollider), Without<Mass>>,
    mut contacts: ResMut<StaticContacts>,
) {
    for (ent_a, mut pos_a, collider_a) in dynamics.iter_mut() {
        for (ent_b, pos_b, collider_b) in statics.iter() {
            let ab = pos_b.0 - pos_a.0;
            let ab_len = ab.length();
            if let Some(contact) =
                get_contact(pos_a.0, &collider_a.0, pos_b.0, &collider_b.0, ab_len)
            {
                // If distance between contact points is negative, it's a penetration.
                if contact.dist < 0.0 {
                    // Contact normal
                    let n = Vec2::new(contact.normal1.x, contact.normal1.y);
                    pos_a.0 -= n * -contact.dist;

                    contacts.0.push((ent_a, ent_b, n));
                }
            }
        }
    }
}

/// Computes one pair of contact points between two shapes.
pub(crate) fn get_contact(
    pos_a: Vec2,
    shape_a: &SharedShape,
    pos_b: Vec2,
    shape_b: &SharedShape,
    dist_prediction: f32,
) -> Option<parry2d::query::Contact> {
    parry2d::query::contact(
        &Isometry::<f32>::translation(pos_a.x, pos_a.y),
        shape_a.0.as_ref(),
        &Isometry::<f32>::translation(pos_b.x, pos_b.y),
        shape_b.0.as_ref(),
        dist_prediction,
    )
    .unwrap()
}

/// Moves objects in the physics world.
pub(crate) fn update_vel(mut query: Query<(&Pos, &PrevPos, &mut Vel)>) {
    for (pos, prev_pos, mut vel) in query.iter_mut() {
        vel.0 = (pos.0 - prev_pos.0) / SUB_DT;
    }
}

/// Solves the new velocities of dynamic bodies after dynamic-dynamic collisions.
pub(crate) fn solve_vel_dynamics(
    mut query: Query<(&mut Vel, &PreSolveVel, &Mass, &Restitution)>,
    contacts: Res<DynamicContacts>,
) {
    for (entity_a, entity_b, n) in contacts.0.iter() {
        if let Ok(
            [(mut vel_a, pre_solve_vel_a, mass_a, restitution_a), (mut vel_b, pre_solve_vel_b, mass_b, restitution_b)],
        ) = query.get_many_mut([*entity_a, *entity_b])
        {
            let pre_solve_relative_vel = pre_solve_vel_a.0 - pre_solve_vel_b.0;
            let pre_solve_normal_vel = Vec2::dot(pre_solve_relative_vel, *n);

            let relative_vel = vel_a.0 - vel_b.0;
            let normal_vel = Vec2::dot(relative_vel, *n);
            let restitution = (restitution_a.0 + restitution_b.0) * 0.5;

            let w_a = 1.0 / mass_a.0;
            let w_b = 1.0 / mass_b.0;
            let w_sum = w_a + w_b;

            let restitution_velocity = (-restitution * pre_solve_normal_vel).min(0.0);
            let vel_impulse = *n * ((-normal_vel + restitution_velocity) / w_sum);

            vel_a.0 += vel_impulse * w_a;
            vel_b.0 -= vel_impulse * w_b;
        }
    }
}

/// Solves the new velocities of dynamic bodies after dynamic-static collisions.
pub(crate) fn solve_vel_statics(
    mut dynamics: Query<(&mut Vel, &PreSolveVel, &Restitution), With<Mass>>,
    statics: Query<&Restitution, Without<Mass>>,
    contacts: Res<StaticContacts>,
) {
    for (entity_a, entity_b, n) in contacts.0.iter() {
        if let Ok((mut vel_a, pre_solve_vel_a, restitution_a)) = dynamics.get_mut(*entity_a) {
            if let Ok(restitution_b) = statics.get(*entity_b) {
                let pre_solve_normal_vel = Vec2::dot(pre_solve_vel_a.0, *n);

                let normal_vel = Vec2::dot(vel_a.0, *n);
                let restitution = (restitution_a.0 + restitution_b.0) * 0.5;

                vel_a.0 += *n * (-normal_vel + (-restitution * pre_solve_normal_vel).min(0.0));
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
pub(crate) fn sync_transforms(mut query: Query<(&mut Transform, &Pos)>) {
    for (mut transform, pos) in query.iter_mut() {
        transform.translation = pos.0.extend(0.0);
    }
}
