//! Sends collision events and updates [`CollidingEntities`].
//!
//! See [`ContactReportingPlugin`].

use crate::prelude::*;

/// Sends collision events and updates [`CollidingEntities`].
///
/// The following collision events are sent each frame in [`PhysicsStepSet::ReportContacts`]:
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
pub fn report_contacts(
    mut colliders: Query<&mut CollidingEntities>,
    collisions: Res<Collisions>,
    mut collision_ev_writer: EventWriter<Collision>,
    mut collision_started_ev_writer: EventWriter<CollisionStarted>,
    mut collision_ended_ev_writer: EventWriter<CollisionEnded>,
) {
    for ((entity1, entity2), contacts) in collisions.get_internal().iter() {
        if contacts.during_current_frame {
            collision_ev_writer.send(Collision(contacts.clone()));

            // Collision started
            if contacts.during_current_frame && !contacts.during_previous_frame {
                collision_started_ev_writer.send(CollisionStarted(*entity1, *entity2));

                if let Ok(mut colliding_entities1) = colliders.get_mut(*entity1) {
                    colliding_entities1.insert(*entity2);
                }
                if let Ok(mut colliding_entities2) = colliders.get_mut(*entity2) {
                    colliding_entities2.insert(*entity1);
                }
            }
        }

        // Collision ended
        if !contacts.during_current_frame {
            collision_ended_ev_writer.send(CollisionEnded(*entity1, *entity2));

            if let Ok(mut colliding_entities1) = colliders.get_mut(*entity1) {
                colliding_entities1.remove(entity2);
            }
            if let Ok(mut colliding_entities2) = colliders.get_mut(*entity2) {
                colliding_entities2.remove(entity1);
            }
        }
    }
}
