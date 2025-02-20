//! [`CollisionStarted`] and [`CollisionEnded`] events.

use bevy::prelude::*;

/// A [collision event](super#collision-events) that is sent
/// when two entities start colliding.
///
/// # Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         .add_systems(Update, print_started_collisions)
///         .run();
/// }
///
/// fn print_started_collisions(mut collision_event_reader: EventReader<CollisionStarted>) {
///     for CollisionStarted(entity1, entity2) in collision_event_reader.read() {
///         println!(
///             "Entities {} and {} started colliding",
///             entity1,
///             entity2,
///         );
///     }
/// }
/// ```
#[derive(Event, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionStarted(pub Entity, pub Entity);

/// A [collision event](super#collision-events) that is sent
/// when two entities stop colliding.
///
/// # Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         .add_systems(Update, print_ended_collisions)
///         .run();
/// }
///
/// fn print_ended_collisions(mut collision_event_reader: EventReader<CollisionEnded>) {
///     for CollisionEnded(entity1, entity2) in collision_event_reader.read() {
///         println!(
///             "Entities {} and {} stopped colliding",
///             entity1,
///             entity2,
///         );
///     }
/// }
/// ```
#[derive(Event, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionEnded(pub Entity, pub Entity);
