use crate::{components::Rotation, Contact};

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
