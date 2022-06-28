use crate::prelude::*;
use bevy::prelude::*;
use parry::{bounding_volume::AABB, math::Isometry, shape::SharedShape};

pub(crate) fn aabb_with_margin(pos: &Vector, rot: &Rot, shape: &SharedShape, margin: f32) -> AABB {
    let mut aabb = shape.compute_aabb(&Isometry::new((*pos).into(), (*rot).into()));

    aabb.mins.x -= margin;
    aabb.maxs.x += margin;
    aabb.mins.y -= margin;
    aabb.maxs.y += margin;

    cfg_if! {
        if #[cfg(feature = "3d")] {
            aabb.mins.z -= margin;
            aabb.maxs.z += margin;
        }
    }

    aabb
}

/// Computes one pair of contact points between two shapes.
#[allow(clippy::too_many_arguments)]
pub(crate) fn get_contact(
    ent_a: Entity,
    ent_b: Entity,
    pos_a: Vector,
    pos_b: Vector,
    local_com_a: Vector,
    local_com_b: Vector,
    rot_a: &Rot,
    rot_b: &Rot,
    shape_a: &SharedShape,
    shape_b: &SharedShape,
) -> Option<Contact> {
    if let Some(contact) = parry::query::contact(
        &make_isometry(pos_a, rot_a),
        shape_a.0.as_ref(),
        &make_isometry(pos_b, rot_b),
        shape_b.0.as_ref(),
        0.0,
    )
    .unwrap()
    {
        let world_r_a = Vector::from(contact.point1) - pos_a + local_com_a;
        let world_r_b = Vector::from(contact.point2) - pos_b + local_com_b;

        return Some(Contact {
            entity_a: ent_a,
            entity_b: ent_b,
            local_r_a: rot_a.inv().rotate(world_r_a),
            local_r_b: rot_b.inv().rotate(world_r_b),
            world_r_a,
            world_r_b,
            normal: Vector::from(contact.normal1),
            penetration: -contact.dist,
        });
    }
    None
}

#[cfg(feature = "2d")]
pub(crate) fn make_isometry(pos: Vector, rot: &Rot) -> Isometry<f32> {
    Isometry::<f32>::new(pos.into(), (*rot).into())
}

#[cfg(feature = "3d")]
pub(crate) fn make_isometry(pos: Vector, rot: &Rot) -> Isometry<f32> {
    Isometry::<f32>::new(pos.into(), rot.to_scaled_axis().into())
}

#[cfg(feature = "3d")]
pub(crate) fn get_rotated_inertia_tensor(inertia_tensor: Mat3, rot: Quat) -> Mat3 {
    let rot_mat3 = Mat3::from_quat(rot);
    (rot_mat3 * inertia_tensor) * rot_mat3.transpose()
}

/// Calculates velocity correction caused by dynamic friction.
pub(crate) fn get_dynamic_friction(
    tangent_vel: Vector,
    friction_a: &Friction,
    friction_b: &Friction,
    normal_lagrange: f32,
    sub_dt: f32,
) -> Vector {
    let tangent_vel_magnitude = tangent_vel.length();

    // Avoid division by zero when normalizing the vector later.
    // We compare against epsilon to avoid potential floating point precision problems.
    if tangent_vel_magnitude.abs() <= f32::EPSILON {
        return Vector::ZERO;
    }

    // Average of the bodies' dynamic friction coefficients
    let dynamic_friction_coefficient =
        (friction_a.dynamic_coefficient + friction_b.dynamic_coefficient) * 0.5;

    let normal_force = normal_lagrange / sub_dt.powi(2);

    // Velocity update caused by dynamic friction, never exceeds the magnitude of the tangential velocity itself
    -tangent_vel / tangent_vel_magnitude
        * (sub_dt * dynamic_friction_coefficient * normal_force.abs()).min(tangent_vel_magnitude)
}

/// Calculates velocity correction caused by restitution.
pub(crate) fn get_restitution(
    normal: Vector,
    normal_vel: f32,
    pre_solve_normal_vel: f32,
    restitution_a: &Restitution,
    restitution_b: &Restitution,
    gravity: Vector,
    sub_dt: f32,
) -> Vector {
    let mut restitution_coefficient = (restitution_a.0 + restitution_b.0) * 0.5;

    // If normal velocity is small enough, use restitution of 0 to avoid jittering
    if normal_vel.abs() <= 2.0 * gravity.length() * sub_dt {
        restitution_coefficient = 0.0;
    }

    normal * (-normal_vel + (-restitution_coefficient * pre_solve_normal_vel).min(0.0))
}
