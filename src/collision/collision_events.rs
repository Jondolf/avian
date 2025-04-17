//! [`CollisionStarted`] and [`CollisionEnded`] events.
//!
//! Collision events are only sent if one of the entities has the [`CollisionEventsEnabled`] component.
//!
//! You can listen to these events with normal event readers:
//!
//! ```no_run
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
//!         // ...
//!         .add_systems(Update, print_collisions)
//!         .run();
//! }
//!
//! fn print_collisions(mut collision_event_reader: EventReader<CollisionStarted>) {
//!     for CollisionStarted(entity1, entity2) in collision_event_reader.read() {
//!         println!("Entities {entity1} and {entity2} are colliding");
//!     }
//! }
//! ```
//!
//! Collision events that use observers are not yet supported.

use bevy::prelude::*;

/// A [collision event](super#collision-events) that is sent when two colliders start touching.
///
/// The event is only sent if one of the entities has the [`CollisionEventsEnabled`] component.
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
///         // ...
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
#[derive(Event, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionStarted(pub Entity, pub Entity);

/// A [collision event](super#collision-events) that is sent when two colliders stop touching.
///
/// The event is only sent if one of the entities has the [`CollisionEventsEnabled`] component.
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
///         // ...
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
#[derive(Event, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionEnded(pub Entity, pub Entity);

/// A [collision event](super#collision-events) that is triggered for observers
/// when two colliders start touching.
///
/// The event is only sent if one of the entities has the [`CollisionEventsEnabled`] component.
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
///         // ...
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
#[derive(Event, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct OnCollisionStart(pub Entity);

/// A [collision event](super#collision-events) that is triggered for observers
/// when two colliders stop touching.
///
/// The event is only sent if one of the entities has the [`CollisionEventsEnabled`] component.
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
///         // ...
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
#[derive(Event, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct OnCollisionEnd(pub Entity);

/// A marker component that enables [`CollisionStarted`], and [`CollisionEnded`] events for an entity.
#[derive(Component, Clone, Copy, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug)]
pub struct CollisionEventsEnabled;
