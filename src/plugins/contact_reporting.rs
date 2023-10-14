//! Sends collision events and updates [`CollidingEntities`].
//!
//! See [`ContactReportingPlugin`].

use crate::prelude::*;

/// Sends collision events and updates [`CollidingEntities`].
///
/// The following collision events are sent each frame:
///
/// - [`Collision`]
/// - [`CollisionStarted`]
/// - [`CollisionEnded`]
pub struct ContactReportingPlugin;

impl Plugin for ContactReportingPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<Collision>()
            .add_event::<CollisionStarted>()
            .add_event::<CollisionEnded>();

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(report_contacts.in_set(PhysicsStepSet::ReportContacts));
    }
}

/// A [collision event](Collider#collision-events) that is sent for each contact pair during the narrow phase.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct Collision(pub Contacts);

/// A [collision event](Collider#collision-events) that is sent when two entities start colliding.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct CollisionStarted(pub Entity, pub Entity);

/// A [collision event](Collider#collision-events) that is sent when two entities stop colliding.
#[derive(Event, Clone, Debug, PartialEq)]
pub struct CollisionEnded(pub Entity, pub Entity);

/// Sends collision events and updates [`CollidingEntities`].
fn report_contacts(
    sleeping: Query<(Ref<Position>, Ref<Rotation>)>,
    mut colliders: Query<&mut CollidingEntities>,
    mut collisions: ResMut<Collisions>,
    mut collision_ev_writer: EventWriter<Collision>,
    mut collision_started_ev_writer: EventWriter<CollisionStarted>,
    mut collision_ended_ev_writer: EventWriter<CollisionEnded>,
) {
    for ((entity1, entity2), contacts) in collisions.get_internal_mut().iter_mut() {
        if contacts.during_current_frame {
            collision_ev_writer.send(Collision(contacts.clone()));

            // Collision started
            if contacts.during_current_frame && !contacts.during_previous_frame {
                collision_started_ev_writer.send(CollisionStarted(*entity1, *entity2));
                contacts.during_previous_frame = true;

                if let Ok(mut colliding_entities1) = colliders.get_mut(*entity1) {
                    colliding_entities1.insert(*entity2);
                } else {
                    contacts.during_current_frame = false;
                }
                if let Ok(mut colliding_entities2) = colliders.get_mut(*entity2) {
                    colliding_entities2.insert(*entity1);
                } else {
                    contacts.during_current_frame = false;
                }
            }
        }

        if !contacts.during_current_frame {
            // Keep the collision active if the bodies were colliding during the previous frame
            // but neither body has moved, for example when they are sleeping.
            if let Ok([(pos1, rot1), (pos2, rot2)]) = sleeping.get_many([*entity1, *entity2]) {
                if !pos1.is_changed()
                    && !rot1.is_changed()
                    && !pos2.is_changed()
                    && !rot2.is_changed()
                {
                    contacts.during_current_frame = true;
                    continue;
                }
            }

            // Collision ended
            collision_ended_ev_writer.send(CollisionEnded(*entity1, *entity2));
            contacts.during_previous_frame = false;

            if let Ok(mut colliding_entities1) = colliders.get_mut(*entity1) {
                colliding_entities1.remove(entity2);
            }
            if let Ok(mut colliding_entities2) = colliders.get_mut(*entity2) {
                colliding_entities2.remove(entity1);
            }
        }
    }
}
