use crate::{components::*, *};

use bevy::prelude::*;
use parry2d::{bounding_volume::AABB, math::Isometry, na::Vector2, shape::SharedShape};

pub(crate) fn aabb_with_margin<T: Rotation>(
    pos: &Vec2,
    rot: &T,
    shape: &SharedShape,
    margin: f32,
) -> AABB {
    let mut aabb = shape.compute_aabb(&Isometry::translation(pos.x, pos.y));

    let sin = rot.sin().abs();
    let cos = rot.cos().abs();

    let size = aabb.extents().xy();
    let width = size.y * sin + size.x * cos;
    let height = size.x * sin + size.y * cos;

    let half_extents = Vector2::new(width * 0.5 + margin, height * 0.5 + margin);

    aabb.mins -= half_extents;
    aabb.maxs += half_extents;

    aabb
}

/// Computes one pair of contact points between two shapes.
#[allow(clippy::too_many_arguments)]
pub(crate) fn get_contact(
    ent_a: Entity,
    ent_b: Entity,
    pos_a: Vec2,
    pos_b: Vec2,
    rot_a: f32,
    rot_b: f32,
    shape_a: &SharedShape,
    shape_b: &SharedShape,
) -> Option<Contact> {
    if let Some(contact) = parry2d::query::contact(
        &Isometry::<f32>::new(Vector2::new(pos_a.x, pos_a.y), rot_a),
        shape_a.0.as_ref(),
        &Isometry::<f32>::new(Vector2::new(pos_b.x, pos_b.y), rot_b),
        shape_b.0.as_ref(),
        0.0,
    )
    .unwrap()
    {
        if contact.dist <= 0.0 {
            return Some(Contact {
                entity_a: ent_a,
                entity_b: ent_b,
                r_a: Vec2::new(contact.point1.x, contact.point1.y) - pos_a,
                r_b: Vec2::new(contact.point2.x, contact.point2.y) - pos_b,
                normal: Vec2::new(contact.normal1.x, contact.normal1.y),
                penetration: -contact.dist,
            });
        }
    }
    None
}

/// Calculates velocity correction caused by static friction.
pub(crate) fn get_static_friction(
    delta_pos_a: Vec2,
    delta_pos_b: Vec2,
    friction_a: &Friction,
    friction_b: &Friction,
    contact_normal: Vec2,
    normal_force: f32,
) -> Vec2 {
    let static_friction_coefficient =
        (friction_a.static_coefficient + friction_b.static_coefficient) * 0.5;
    let relative_mov = delta_pos_a - delta_pos_b;
    let tangential_mov = relative_mov - (relative_mov.dot(contact_normal)) * contact_normal;
    if tangential_mov.length() < normal_force * static_friction_coefficient {
        tangential_mov
    } else {
        Vec2::ZERO
    }
}

/// Calculates velocity correction caused by dynamic friction.
pub(crate) fn get_dynamic_friction(
    tangent_vel: Vec2,
    penetration: f32,
    friction_a: &Friction,
    friction_b: &Friction,
    sub_dt: f32,
) -> Vec2 {
    // Only call .length() once
    let tangent_vel_length = tangent_vel.length();

    // Prevent division by zero when normalizing tangent_vel
    if tangent_vel_length != 0.0 {
        // Average of the bodies' dynamic friction coefficients
        let friction_coefficient =
            (friction_a.dynamic_coefficient + friction_b.dynamic_coefficient) * 0.5;

        // Magnitude of velocity correction caused by friction
        let friction_magnitude =
            sub_dt * friction_coefficient * (penetration / sub_dt.powi(2)).abs();

        // Velocity correction vector caused by friction, never exceeds the magnitude of the velocity itself
        tangent_vel / tangent_vel_length * (friction_magnitude.min(tangent_vel_length))
    } else {
        // No tangential movement -> no friction
        Vec2::ZERO
    }
}

/// Calculates velocity correction caused by restitution.
pub(crate) fn get_restitution(
    normal: Vec2,
    normal_vel: f32,
    pre_solve_normal_vel: f32,
    restitution_a: &Restitution,
    restitution_b: &Restitution,
) -> Vec2 {
    let restitution = (restitution_a.0 + restitution_b.0) * 0.5;
    let restitution_velocity = (-restitution * pre_solve_normal_vel).min(0.0);
    normal * (restitution_velocity - normal_vel)
}
