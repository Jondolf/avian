//! Collision detection for [`Collider`]s.
//!
//! Collision detection involves determining pairs of objects that may currently be in contact
//! (or are expected to come into contact), and computing contact data for each intersection.
//! These contacts are then used by the [solver](dynamics::solver) to generate [`ContactConstraint`]s
//! and finally resolve overlap.
//!
//! [`ContactConstraint`]: dynamics::solver::contact::ContactConstraint
//!
//! # Plugins
//!
//! In Avian, collision detection is split into two plugins:
//!
//! - [`BroadPhasePlugin`]: Finds pairs of entities with overlapping [AABBs](ColliderAabb) to reduce the number of potential contacts for the [narrow phase](narrow_phase).
//! - [`NarrowPhasePlugin`]: Updates and manages contact pairs in the [`ContactGraph`], and generates [`ContactConstraint`]s for the solver.
//!
//! Spatial queries are handled separately by the [`SpatialQueryPlugin`].
//!
//! You can also find several utility methods for computing contacts in the [`contact_query`] module.
//!
//! # Accessing Collisions
//!
//! Contact pairs found by Avian are stored in the [`ContactGraph`] resource.
//! It contains all contacs between entities with overlapping [`ColliderAabb`]s,
//! including contacts where the colliders themselves may not be touching.
//!
//! To make it easier to access relevant collision data, Avian provides a [`Collisions`]
//! system parameter that only provides touching contacts. This is a light wrapper
//! around the [`ContactGraph`] that can often be more convenient to use.
//!
//! See the documentation of [`Collisions`] for more information and usage examples.
//!
//! # Collision Events
//!
//! The following events are sent whenever two colliders start or stop touching:
//!
//! - [`CollisionStarted`]
//! - [`CollisionEnded`]
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
//!
//! # Contact Filtering and Modification
//!
//! Some advanced contact scenarios may need to filter or modify contacts
//! with user-defined logic. This can include:
//!
//! - One-way platforms
//! - Conveyor belts
//! - Non-uniform friction and restitution
//!
//! In Avian, this can be done by defining [`CollisionHooks`]. They let you hook into
//! the collision pipeline, and filter or modify contacts with (almost) full ECS access.
//!
//! See the documentation of [`CollisionHooks`] for more information and usage examples.

pub mod broad_phase;
pub mod collision_events;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub mod contact_query;
pub mod contact_types;
pub mod hooks;
pub mod narrow_phase;

pub mod collider;
pub use collider::*;

mod layers;
pub use layers::*;

mod feature_id;
pub use feature_id::PackedFeatureId;

mod diagnostics;
pub use diagnostics::CollisionDiagnostics;
