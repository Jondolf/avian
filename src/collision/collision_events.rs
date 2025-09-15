//! Collision events for detecting when colliders start or stop touching.
//!
//! Depending on your use case, you may want to use either buffered [`Message`]s read
//! using a [`MessageReader`], or observable [`Event`]s triggered for specific entities.
//! Avian provides both options using separate event types.
//!
//! Note that collision events are only written or triggered for entities that have
//! the [`CollisionEventsEnabled`] component.
//!
//! # Buffered Events
//!
//! Avian provides two different [`Message`] types for buffered collision events:
//!
//! - [`CollisionStarted`]
//! - [`CollisionEnded`]
//!
//! These events are written when two colliders start or stop touching, and can be read
//! using a [`MessageReader`]. This can be useful for efficiently processing large numbers
//! of collision events between pairs of entities, such as for detecting bullet hits
//! or playing impact sounds when two objects collide.
//!
//! The events are only written if one of the entities has the [`CollisionEventsEnabled`] component.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn print_started_collisions(mut collision_reader: MessageReader<CollisionStarted>) {
//!     for CollisionStarted(entity1, entity2) in collision_reader.read() {
//!         println!("{entity1} and {entity2} started colliding");
//!     }
//! }
//! ```
//!
//! # Observable Events
//!
//! Avian provides two different [`Event`] types for observable collision events:
//!
//! - [`CollisionStart`]
//! - [`CollisionEnd`]
//!
//! These events are triggered for [observers](Observer) when two colliders start or stop touching.
//! This makes them good for entity-specific collision scenarios, such as for detecting when a player
//! steps on a pressure plate or enters a trigger volume.
//!
//! The events are only triggered if the target entity has the [`CollisionEventsEnabled`] component.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! #[derive(Component)]
//! struct Player;
//!
//! #[derive(Component)]
//! struct PressurePlate;
//!
//! fn setup_pressure_plates(mut commands: Commands) {
//!     commands.spawn((
//!         PressurePlate,
#![cfg_attr(feature = "2d", doc = "        Collider::rectangle(1.0, 1.0),")]
#![cfg_attr(feature = "3d", doc = "        Collider::cuboid(1.0, 0.1, 1.0),")]
//!         Sensor,
//!         // Enable collision events for this entity.
//!         CollisionEventsEnabled,
//!     ))
//!     .observe(|trigger: On<CollisionStart>, player_query: Query<&Player>| {
//!         let pressure_plate = trigger.entity;
//!         let other_entity = trigger.collider;
//!         if player_query.contains(other_entity) {
//!             println!("Player {other_entity} stepped on pressure plate {pressure_plate}");
//!         }
//!     });
//! }
//! ```

use bevy::prelude::*;

/// A [`Message`] for a [collision event](super#collision-events) that is written when two colliders start touching.
///
/// The event is only written if one of the entities has the [`CollisionEventsEnabled`] component.
///
/// Unlike [`CollisionStart`], this event is *not* triggered for observers.
/// Instead, you must use a [`MessageReader`] to read the event in a system.
/// This makes it good for efficiently processing large numbers of collision events
/// between pairs of entities.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn print_started_collisions(mut collision_reader: MessageReader<CollisionStarted>) {
///     for CollisionStarted(entity1, entity2) in collision_reader.read() {
///         println!("{entity1} and {entity2} started colliding");
///     }
/// }
/// ```
///
/// # Scheduling
///
/// The [`CollisionStarted`] event is written in the [`NarrowPhaseSet::Update`] system set,
/// but can be read at any time.
///
/// [`NarrowPhaseSet::Update`]: super::narrow_phase::NarrowPhaseSet::Update
#[derive(Message, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionStarted(pub Entity, pub Entity);

/// A [`Message`] for a [collision event](super#collision-events) that is written when two colliders stop touching.
///
/// The event is only written if one of the entities has the [`CollisionEventsEnabled`] component.
///
/// Unlike [`CollisionEnd`], this event is *not* triggered for observers.
/// Instead, you must use a [`MessageReader`] to read the event in a system.
/// This makes it good for efficiently processing large numbers of collision events
/// between pairs of entities.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn print_ended_collisions(mut collision_reader: MessageReader<CollisionEnded>) {
///     for CollisionEnded(entity1, entity2) in collision_reader.read() {
///         println!("{entity1} and {entity2} stopped colliding");
///     }
/// }
/// ```
///
/// # Scheduling
///
/// The [`CollisionEnded`] event is written in the [`NarrowPhaseSet::Update`] system set,
/// but can be read at any time.
///
/// Note that if one of the colliders was removed or the bounding boxes of the colliders stopped
/// overlapping, the [`ContactPair`] between the entities was also removed, and the contact data
/// will not be available through [`Collisions`].
///
/// [`NarrowPhaseSet::Update`]: super::narrow_phase::NarrowPhaseSet::Update
/// [`ContactPair`]: super::ContactPair
/// [`Collisions`]: super::Collisions
#[derive(Message, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionEnded(pub Entity, pub Entity);

/// A [collision event](super#collision-events) that is triggered for [observers](Observer)
/// when two colliders start touching.
///
/// The event is only triggered if the target entity has the [`CollisionEventsEnabled`] component.
///
/// Unlike [`CollisionStarted`], this event can *not* be read using a [`MessageReader`].
/// Instead, you must use an [observer](Observer). This makes it good for entity-specific
/// collision listeners.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// #[derive(Component)]
/// struct Player;
///
/// #[derive(Component)]
/// struct PressurePlate;
///
/// fn setup_pressure_plates(mut commands: Commands) {
///     commands.spawn((
///         PressurePlate,
#[cfg_attr(feature = "2d", doc = "        Collider::rectangle(1.0, 1.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::cuboid(1.0, 0.1, 1.0),")]
///         Sensor,
///         // Enable collision events for this entity.
///         CollisionEventsEnabled,
///     ))
///     .observe(|trigger: On<CollisionStart>, player_query: Query<&Player>| {
///         let pressure_plate = trigger.entity;
///         let other_entity = trigger.collider;
///         if player_query.contains(other_entity) {
///             println!("Player {other_entity} stepped on pressure plate {pressure_plate}");
///         }
///     });
/// }
/// ```
///
/// # Scheduling
///
/// The [`CollisionStart`] event is triggered after the physics step in the [`CollisionEventSystems`]
/// system set. At this point, the solver has already run and contact impulses have been updated.
///
/// [`CollisionEventSystems`]: super::narrow_phase::CollisionEventSystems
#[derive(EntityEvent, Message, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionStart {
    /// The entity of the collider that started colliding with [`collider2`](Self::collider2).
    ///
    /// For observers watching this event as an [`EntityEvent`], this is the target entity.
    #[event_target]
    pub collider1: Entity,
    /// The entity of the collider that started colliding with [`collider1`](Self::collider1).
    pub collider2: Entity,
    /// The rigid body that [`collider1`](Self::collider1) is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will be `None`.
    pub body1: Option<Entity>,
    /// The rigid body that [`collider2`](Self::collider2) is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will be `None`.
    pub body2: Option<Entity>,
    // TODO: Flags to expose the reason for the event, among other things.
    // flags: CollisionStartFlags,
}

/// A deprecated alias for [`CollisionStart`].
#[deprecated(since = "0.4.0", note = "Renamed to `CollisionStart`")]
pub type OnCollisionStart = CollisionStart;

/// A [collision event](super#collision-events) that is triggered for [observers](Observer)
/// when two colliders stop touching.
///
/// The event is only triggered if the target entity has the [`CollisionEventsEnabled`] component.
///
/// Unlike [`CollisionEnded`], this event can *not* be read using a [`MessageReader`].
/// Instead, you must use an [observer](Observer). This makes it good for entity-specific
/// collision listeners.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// #[derive(Component)]
/// struct Player;
///
/// #[derive(Component)]
/// struct PressurePlate;
///
/// fn setup_pressure_plates(mut commands: Commands) {
///     commands.spawn((
///         PressurePlate,
#[cfg_attr(feature = "2d", doc = "        Collider::rectangle(1.0, 1.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::cuboid(1.0, 0.1, 1.0),")]
///         Sensor,
///         // Enable collision events for this entity.
///         CollisionEventsEnabled,
///     ))
///     .observe(|trigger: On<CollisionEnd>, player_query: Query<&Player>| {
///         let pressure_plate = trigger.entity;
///         let other_entity = trigger.collider;
///         if player_query.contains(other_entity) {
///             println!("Player {other_entity} stepped off of pressure plate {pressure_plate}");
///         }
///     });
/// }
/// ```
///
/// # Scheduling
///
/// The [`CollisionEnd`] event is triggered after the physics step in the [`CollisionEventSystems`]
/// system set. At this point, the solver has already run and contact impulses have been updated.
///
/// Note that if one of the colliders was removed or the bounding boxes of the colliders stopped
/// overlapping, the [`ContactPair`] between the entities was also removed, and the contact data
/// will not be available through [`Collisions`].
///
/// [`CollisionEventSystems`]: super::narrow_phase::CollisionEventSystems
/// [`ContactPair`]: super::ContactPair
/// [`Collisions`]: super::Collisions
#[derive(EntityEvent, Message, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionEnd {
    /// The entity of the collider that stopped colliding with [`collider2`](Self::collider2).
    ///
    /// For observers watching this event as an [`EntityEvent`], this is the target entity.
    #[event_target]
    pub collider1: Entity,
    /// The entity of the collider that stopped colliding with [`collider1`](Self::collider1).
    pub collider2: Entity,
    /// The rigid body that [`collider1`](Self::collider1) is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will be `None`.
    pub body1: Option<Entity>,
    /// The rigid body that [`collider2`](Self::collider2) is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will be `None`.
    pub body2: Option<Entity>,
    // TODO: Flags to expose the reason for the event, among other things.
    // flags: CollisionEndFlags,
}

/// A deprecated alias for [`CollisionEnd`].
#[deprecated(since = "0.4.0", note = "Renamed to `CollisionEnd`")]
pub type OnCollisionEnd = CollisionEnd;

/// A marker component that enables [collision events](self) for an entity.
///
/// This enables both the buffered [`CollisionStarted`] and [`CollisionEnded`] events,
/// as well as the observable [`CollisionStart`] and [`CollisionEnd`] events.
#[derive(Component, Clone, Copy, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug)]
pub struct CollisionEventsEnabled;
