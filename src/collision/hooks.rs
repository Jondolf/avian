//! Collision hooks for filtering and modifying contacts.
use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*};

/// A trait for user-defined hooks that can filter and modify contacts.
///
/// This can be useful for advanced contact scenarios, such as:
///
/// - One-way platforms
/// - Conveyor belts
/// - Non-uniform friction
///
/// Collision hooks are more flexible than built-in filtering options like [`CollisionLayers`],
/// but can be more complex to define, and can have slightly more overhead.
/// It is recommended to use hooks only when existing options are not sufficient.
///
/// Only one set of collision hooks can be defined for a given app.
///
/// # Defining Hooks
///
/// Collision hooks can be defined by implementing the [`CollisionHooks`] trait
/// for a type implementing [`SystemParam`]. The [`SystemParam`] allows the hooks
/// to access resources, query for components, perform [spatial queries](crate::spatial_query),
/// or do almost anything else that a normal system can do.
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Define a `SystemParam` for the collision hooks.
/// #[derive(SystemParam)]
/// struct MyHooks<'w, 's> {
///     interaction_query: Query<'w, 's, &InteractionGroup>,
///     platform_query: Query<'w, 's, &Transform, With<OneWayPlatform>>,
/// }
///
/// // Implement the `CollisionHooks` trait.
/// impl CollisionHooks for MyHooks<'_, '_> {
///     fn filter_pairs(&mut self, entity1: Entity, entity2: Entity) -> bool {
///         // Only allow collisions between entities in the same interaction group.
///         // This could be a basic solution for "multiple physics worlds" that don't interact.
///         let Ok([group1, group2]) = self.interaction_query.get_many([entity1, entity2]) else {
///            return true;
///         };
///         group1.0 == group2.0
///     }
///
///     fn modify_contacts(&mut self, contacts: &mut Contacts) -> bool {
///         // Allow entities to pass through the bottom and sides of one-way platforms.
///         // See the `one_way_platform_2d` example for a full implementation.
///         let (entity1, entity2) = (contacts.entity1, contacts.entity2);
///         !is_hitting_top_of_platform(entity1, entity2, &self.platform_query, &contacts)
///     }
/// }
///
/// // A component that groups entities for interactions. Only entities in the same group can collide.
/// #[derive(Component)]
/// struct InteractionGroup(u32);
/// #
/// #
/// # fn is_hitting_top_of_platform(
/// #     entity1: Entity,
/// #     entity2: Entity,
/// #     platform_query: &Query<&Transform, With<OneWayPlatform>>,
/// #     contacts: &Contacts,
/// # ) -> bool {
/// #     todo!()
/// # }
/// ```
///
/// The hooks can be added to the app with [`PhysicsPlugins::with_collision_hooks`]:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # // No-op hooks for the example.
/// # use () as MyHooks;
/// #
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default().with_collision_hooks::<MyHooks>(),
///         ))
///         .run();
/// }
/// ```
///
/// Alternatively, manually replace the default [`BroadPhasePlugin`] and [`NarrowPhasePlugin`]
/// with instances that have the desired hooks given using generics.
///
/// # Activating Hooks
///
/// Hooks are *only* called for collisions where at least one entity has the [`ActiveCollisionHooks`] component
/// with the corresponding flags set. By default, no hooks are called.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Spawn a collider with filtering hooks enabled.
/// commands.spawn((Collider::capsule(0.5, 1.5), ActiveCollisionHooks::FILTER_PAIRS));
///
/// // Spawn a collider with both filtering and contact modification hooks enabled.
/// commands.spawn((
///     Collider::capsule(0.5, 1.5),
///     ActiveCollisionHooks::FILTER_PAIRS | ActiveCollisionHooks::MODIFY_CONTACTS
/// ));
///
/// // Alternatively, all hooks can be enabled with `ActiveCollisionHooks::ALL`.
/// commands.spawn((Collider::capsule(0.5, 1.5), ActiveCollisionHooks::ALL));
/// # }
/// ```
///
/// # Caveats
///
/// - Only one set of collision hooks can be defined for a given app.
/// - Certain components and resources may not be (mutably) accessible in hooks due to the internal system
///   having access to them simultaneously. You will get a runtime panic if you try to access them.
#[expect(unused_variables)]
pub trait CollisionHooks: SystemParam + Send + Sync {
    /// A contact pair filtering hook that determines whether contacts should be computed
    /// between `entity1` and `entity2`. If `false` is returned, contacts will not be computed.
    ///
    /// This is called in the broad phase, before [`Contacts`] have been computed for the pair.
    ///
    /// Only called if at least one entity in the contact pair has [`ActiveCollisionHooks::FILTER_PAIRS`] set.
    fn filter_pairs(&mut self, entity1: Entity, entity2: Entity, commands: &mut Commands) -> bool {
        true
    }

    /// A contact modification hook that allows modifying the contacts for a given contact pair.
    /// If `false` is returned, the contact pair will be removed.
    ///
    /// This is called in the narrow phase, after [`Contacts`] have been computed for the pair,
    /// but before constraints have been generated for the contact solver.
    ///
    /// Only called if at least one entity in the contact pair has [`ActiveCollisionHooks::MODIFY_CONTACTS`] set.
    fn modify_contacts(&self, contacts: &mut Contacts, commands: &mut Commands) -> bool {
        true
    }
}

impl CollisionHooks for () {}

/// A component with flags indicating which [`CollisionHooks`] should be called for collisions with an entity.
///
/// If either entity in a collision has hooks enabled, the hooks will be called.
///
/// Default: [`ActiveCollisionHooks::NONE`]
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Spawn a collider with filtering hooks enabled.
/// commands.spawn((Collider::capsule(0.5, 1.5), ActiveCollisionHooks::FILTER_PAIRS));
///
/// // Spawn a collider with both filtering and contact modification hooks enabled.
/// commands.spawn((
///     Collider::capsule(0.5, 1.5),
///     ActiveCollisionHooks::FILTER_PAIRS | ActiveCollisionHooks::MODIFY_CONTACTS
/// ));
///
/// // Alternatively, all hooks can be enabled with `ActiveCollisionHooks::ALL`.
/// commands.spawn((Collider::capsule(0.5, 1.5), ActiveCollisionHooks::ALL));
/// # }
/// ```
#[repr(transparent)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[derive(Component, Hash, Clone, Copy, PartialEq, Eq, Debug, Reflect)]
#[reflect(opaque, Hash, PartialEq, Debug)]
pub struct ActiveCollisionHooks(u8);

bitflags::bitflags! {
    impl ActiveCollisionHooks: u8 {
        /// Set if [`CollisionHooks::filter_contact_pairs`] should be called for collisions with this entity.
        const FILTER_PAIRS = 0b0000_0001;
        /// Set if [`CollisionHooks::modify_contacts`] should be called for collisions with this entity.
        const MODIFY_CONTACTS = 0b0000_0010;
    }
}
