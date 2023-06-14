//! The broad phase is responsible for collecting potential collision pairs into the [`BroadCollisionPairs`] resource using simple AABB intersection checks. This reduces the number of required precise collision checks. See [`NarrowPhasePlugin`].

use crate::prelude::*;
use bevy::prelude::*;

/// The `NarrowPhasePlugin` is responsible for collecting potential collision pairs into the [`BroadCollisionPairs`] resource using simple AABB intersection checks.
///
/// The broad phase speeds up collision detection, as the number of accurate collision checks is greatly reduced.
pub struct NarrowPhasePlugin;

impl Plugin for NarrowPhasePlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<Collision>()
            .add_event::<CollisionStarted>()
            .add_event::<CollisionEnded>();

        let substeps = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");

        substeps.add_system(narrow_phase.in_set(SubsteppingSet::NarrowPhase));
    }
}

/// An event that is sent for each contact pair during the narrow phase. See [`NarrowPhasePlugin`].
#[derive(Clone, Debug, PartialEq)]
pub struct Collision(pub Contact);

/// An event that is sent when two entities start colliding. See [`NarrowPhasePlugin`].
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionStarted(pub Entity, pub Entity);

/// An event that is sent when two entities stop colliding. See [`NarrowPhasePlugin`].
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionEnded(pub Entity, pub Entity);

type ColliderQueryComponents = (
    &'static Collider,
    &'static Pos,
    &'static Rot,
    Option<&'static CollisionLayers>,
    Option<&'static mut CollidingEntities>,
    Option<&'static RigidBody>,
    Option<&'static Sleeping>,
);

/// Iterates through broad phase collision pairs, checks which ones are actually colliding, and sends collision events.
fn narrow_phase(
    mut colliders: Query<ColliderQueryComponents>,
    broad_collision_pairs: Res<BroadCollisionPairs>,
    mut collision_ev_writer: EventWriter<Collision>,
    mut collision_started_ev_writer: EventWriter<CollisionStarted>,
    mut collision_ended_ev_writer: EventWriter<CollisionEnded>,
) {
    let mut collision_events = Vec::with_capacity(broad_collision_pairs.0.len());
    let mut started_collisions = Vec::new();
    let mut ended_collisions = Vec::new();

    for (ent1, ent2) in broad_collision_pairs.0.iter() {
        if let Ok([collider1, collider2]) = colliders.get_many_mut([*ent1, *ent2]) {
            let (shape1, pos1, rot1, layers1, colliding_entities1, rb1, sleeping1) = collider1;
            let (shape2, pos2, rot2, layers2, colliding_entities2, rb2, sleeping2) = collider2;

            let layers1 = layers1.map_or(CollisionLayers::default(), |l| *l);
            let layers2 = layers2.map_or(CollisionLayers::default(), |l| *l);

            // Skip collision if collision layers are incompatible
            if !layers1.interacts_with(layers2) {
                continue;
            }

            let inactive1 = rb1.map_or(true, |rb| rb.is_static() || sleeping1.is_some());
            let inactive2 = rb2.map_or(true, |rb| rb.is_static() || sleeping2.is_some());

            // No collision if one of the bodies is static and the other one is sleeping
            if inactive1 && inactive2 {
                continue;
            }

            // Compute contact, add collision events and update colliding entities
            if let Some(contact) = compute_contact(
                *ent1, *ent2, pos1.0, pos2.0, rot1, rot2, &shape1.0, &shape2.0,
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

/// Data related to a contact between two colliders.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    /// First entity in a collision.
    pub entity1: Entity,
    /// Second entity in a collision.
    pub entity2: Entity,
    /// Local contact point 1 in local coordinates.
    pub local_r1: Vector,
    /// Local contact point 2 in local coordinates.
    pub local_r2: Vector,
    /// Local contact point 1 in world coordinates.
    pub world_r1: Vector,
    /// Local contact point 2 in world coordinates.
    pub world_r2: Vector,
    /// Contact normal from contact point 1 to 2.
    pub normal: Vector,
    /// Penetration depth.
    pub penetration: Scalar,
}

/// Computes one pair of collision points between two shapes.
#[allow(clippy::too_many_arguments)]
pub(crate) fn compute_contact(
    ent1: Entity,
    ent2: Entity,
    pos1: Vector,
    pos2: Vector,
    rot1: &Rot,
    rot2: &Rot,
    shape1: &Shape,
    shape2: &Shape,
) -> Option<Contact> {
    if let Ok(Some(collision)) = parry::query::contact(
        &utils::make_isometry(pos1, rot1),
        shape1.0.as_ref(),
        &utils::make_isometry(pos2, rot2),
        shape2.0.as_ref(),
        0.0,
    ) {
        let world_r1 = Vector::from(collision.point1) - pos1;
        let world_r2 = Vector::from(collision.point2) - pos2;

        return Some(Contact {
            entity1: ent1,
            entity2: ent2,
            local_r1: rot1.inv().rotate(world_r1),
            local_r2: rot2.inv().rotate(world_r2),
            world_r1,
            world_r2,
            normal: collision.normal1.into(),
            penetration: -collision.dist,
        });
    }

    None
}
