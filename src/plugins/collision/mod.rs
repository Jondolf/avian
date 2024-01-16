//! Collision detection is used to detect and compute intersections between [`Collider`]s.
//!
//! In `bevy_xpbd`, collision detection is split into three plugins:
//!
//! - [`BroadPhasePlugin`]: Collects pairs of potentially colliding entities into [`BroadCollisionPairs`].
//! - [`NarrowPhasePlugin`]: Computes contacts for broad phase collision pairs and adds them to [`Collisions`].
//! - [`ContactReportingPlugin`] (optional): Sends collision events and updates [`CollidingEntities`] based on [`Collisions`].
//!
//! Spatial queries are handled by the [`SpatialQueryPlugin`].
//!
//! You can also find several utility methods for computing contacts in [`contact_query`].

pub mod broad_phase;
pub mod contact_query;
pub mod contact_reporting;
pub mod narrow_phase;

use std::marker::PhantomData;

use crate::prelude::*;
use bevy::prelude::*;
use indexmap::IndexMap;

/// A trait that generalizes over colliders. Implementing this trait
/// allows colliders to be used with the physics engine.
pub trait AnyCollider: Component {
    /// Computes the [Axis-Aligned Bounding Box](ColliderAabb) of the collider
    /// with the given position and rotation.
    #[cfg_attr(
        feature = "2d",
        doc = "\n\nThe rotation is counterclockwise and in radians."
    )]
    fn aabb(&self, position: impl Into<Position>, rotation: impl Into<Rotation>) -> ColliderAabb;

    /// Computes the swept [Axis-Aligned Bounding Box](ColliderAabb) of the collider.
    /// This corresponds to the space the shape would occupy if it moved from the given
    /// start position to the given end position.
    #[cfg_attr(
        feature = "2d",
        doc = "\n\nThe rotation is counterclockwise and in radians."
    )]
    fn swept_aabb(
        &self,
        start_position: impl Into<Position>,
        start_rotation: impl Into<Rotation>,
        end_position: impl Into<Position>,
        end_rotation: impl Into<Rotation>,
    ) -> ColliderAabb;

    /// Computes the collider's mass properties based on its shape and a given density.
    fn mass_properties(&self, density: Scalar) -> ColliderMassProperties;

    /// Computes all [`ContactManifold`]s between two colliders.
    ///
    /// Returns an empty vector if the colliders are separated by a distance greater than `prediction_distance`
    /// or if the given shapes are invalid.
    fn contact_manifolds(
        &self,
        other: &Self,
        position1: impl Into<Position>,
        rotation1: impl Into<Rotation>,
        position2: impl Into<Position>,
        rotation2: impl Into<Rotation>,
        prediction_distance: Scalar,
    ) -> Vec<ContactManifold>;
}

impl AnyCollider for Collider {
    fn mass_properties(&self, density: Scalar) -> ColliderMassProperties {
        self.mass_properties(density)
    }

    fn contact_manifolds(
        &self,
        other: &Self,
        position1: impl Into<Position>,
        rotation1: impl Into<Rotation>,
        position2: impl Into<Position>,
        rotation2: impl Into<Rotation>,
        prediction_distance: Scalar,
    ) -> Vec<ContactManifold> {
        contact_query::contact_manifolds(
            self,
            position1,
            rotation1,
            other,
            position2,
            rotation2,
            prediction_distance,
        )
    }

    fn aabb(&self, position: impl Into<Position>, rotation: impl Into<Rotation>) -> ColliderAabb {
        self.aabb(position, rotation)
    }

    fn swept_aabb(
        &self,
        start_position: impl Into<Position>,
        start_rotation: impl Into<Rotation>,
        end_position: impl Into<Position>,
        end_rotation: impl Into<Rotation>,
    ) -> ColliderAabb {
        self.swept_aabb(start_position, start_rotation, end_position, end_rotation)
    }
}

/// A trait for colliders that support scaling.
pub trait ScalableCollider: Component {
    /// Returns the global scaling factor of the collider.
    fn scale(&self) -> Vector;

    /// Sets the global scaling factor of the collider.
    ///
    /// If the scaling factor is not uniform and the resulting scaled shape
    /// can not be represented exactly, the given `detail` is used for an approximation.
    fn set_scale(&mut self, scale: Vector, detail: u32);

    /// Scales the collider by the given scaling factor.
    ///
    /// If the scaling factor is not uniform and the resulting scaled shape
    /// can not be represented exactly, the given `detail` is used for an approximation.
    fn scale_by(&mut self, factor: Vector, detail: u32) {
        self.set_scale(factor * self.scale(), detail)
    }
}

impl ScalableCollider for Collider {
    fn scale(&self) -> Vector {
        self.scale()
    }

    fn set_scale(&mut self, scale: Vector, detail: u32) {
        self.set_scale(scale, detail)
    }
}

/// A high-level plugin for collision detection.
///
/// Internally, this plugin adds several other plugins related to collision detection:
///
/// - [`BroadPhasePlugin`]
/// - [`NarrowPhasePlugin`]
/// - [`ContactReportingPlugin`]
///
/// The plugin takes a collider type. This should be [`Collider`] for
/// the vast majority of applications, but for custom collisión backends
/// you may use any collider that implements the [`AnyCollider`] trait.
pub struct CollisionPlugin<C: AnyCollider> {
    _phantom: PhantomData<C>,
}

impl<C: AnyCollider> Default for CollisionPlugin<C> {
    fn default() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

impl<C: AnyCollider> Plugin for CollisionPlugin<C> {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            BroadPhasePlugin::<C>::default(),
            NarrowPhasePlugin::<C>::default(),
            ContactReportingPlugin,
        ));
    }
}

// Collisions are stored in an `IndexMap` that uses fxhash.
// It should have faster iteration than a `HashMap` while mostly retaining other performance characteristics.
//
// `IndexMap` also preserves insertion order. This can be good or bad depending on the situation,
// but it can make spawned stacks appear more consistent and uniform, for example in the `move_marbles` example.
// ==========================================
/// A resource that stores all collision pairs.
///
/// Each colliding entity pair is associated with [`Contacts`] that can be accessed and modified
/// using the various associated methods.
///
/// ## Usage
///
/// [`Collisions`] can be accessed at almost anytime, but for modifying and filtering collisions,
/// it is recommended to use the [`PostProcessCollisions`] schedule. See its documentation
/// for more information.
///
/// ### Querying collisions
///
/// The following methods can be used for querying existing collisions:
///
/// - [`get`](Self::get) and [`get_mut`](Self::get_mut)
/// - [`iter`](Self::iter) and [`iter_mut`](Self::iter_mut)
/// - [`contains`](Self::contains)
/// - [`collisions_with_entity`](Self::collisions_with_entity) and
/// [`collisions_with_entity_mut`](Self::collisions_with_entity_mut)
///
/// The collisions can be accessed at any time, but modifications to contacts should be performed
/// in the [`PostProcessCollisions`] schedule. Otherwise, the physics solver will use the old contact data.
///
/// ### Filtering and removing collisions
///
/// The following methods can be used for filtering or removing existing collisions:
///
/// - [`retain`](Self::retain)
/// - [`remove_collision_pair`](Self::remove_collision_pair)
/// - [`remove_collisions_with_entity`](Self::remove_collisions_with_entity)
///
/// Collision filtering and removal should be done in the [`PostProcessCollisions`] schedule.
/// Otherwise, the physics solver will use the old contact data.
///
/// ### Adding new collisions
///
/// The following methods can be used for adding new collisions:
///
/// - [`insert_collision_pair`](Self::insert_collision_pair)
/// - [`extend`](Self::extend)
///
/// The most convenient place for adding new collisions is in the [`PostProcessCollisions`] schedule.
/// Otherwise, the physics solver might not have access to them in time.
///
/// ## Implementation details
///
/// Internally, the collisions are stored in an `IndexMap` that contains collisions from both the current frame
/// and the previous frame, which is used for things like [collision events](ContactReportingPlugin#collision-events).
///
/// However, the public methods only use the current frame's collisions. To access the internal data structure,
/// you can use [`get_internal`](Self::get_internal) or [`get_internal_mut`](Self::get_internal_mut).
#[derive(Resource, Clone, Debug, Default, PartialEq)]
pub struct Collisions(IndexMap<(Entity, Entity), Contacts, fxhash::FxBuildHasher>);

impl Collisions {
    /// Returns a reference to the internal `IndexMap`.
    pub fn get_internal(&self) -> &IndexMap<(Entity, Entity), Contacts, fxhash::FxBuildHasher> {
        &self.0
    }

    /// Returns a mutable reference to the internal `IndexMap`.
    pub fn get_internal_mut(
        &mut self,
    ) -> &mut IndexMap<(Entity, Entity), Contacts, fxhash::FxBuildHasher> {
        &mut self.0
    }

    /// Returns a reference to the [contacts](Contacts) stored for the given entity pair if they are colliding,
    /// else returns `None`.
    ///
    /// The order of the entities does not matter.
    pub fn get(&self, entity1: Entity, entity2: Entity) -> Option<&Contacts> {
        self.0
            .get(&(entity1, entity2))
            .filter(|contacts| contacts.during_current_frame)
            .or_else(|| {
                self.0
                    .get(&(entity2, entity1))
                    .filter(|contacts| contacts.during_current_frame)
            })
    }

    /// Returns a mutable reference to the [contacts](Contacts) stored for the given entity pair if they are colliding,
    /// else returns `None`.
    ///
    /// The order of the entities does not matter.
    pub fn get_mut(&mut self, entity1: Entity, entity2: Entity) -> Option<&mut Contacts> {
        // For lifetime reasons, the mutable borrows can't be in the same scope,
        // so we check if the key exists first (there's probably a better way though)
        if self.0.contains_key(&(entity1, entity2)) {
            self.0
                .get_mut(&(entity1, entity2))
                .filter(|contacts| contacts.during_current_frame)
        } else {
            self.0
                .get_mut(&(entity2, entity1))
                .filter(|contacts| contacts.during_current_frame)
        }
    }

    /// Returns `true` if the given entities have been in contact during this frame.
    ///
    /// The order of the entities does not matter.
    pub fn contains(&self, entity1: Entity, entity2: Entity) -> bool {
        // We can't use `contains` directly because we only want to
        // count collisions that happened during this frame.
        self.get(entity1, entity2).is_some()
    }

    /// Returns an iterator over the current collisions that have happened during the current physics frame.
    pub fn iter(&self) -> impl Iterator<Item = &Contacts> {
        self.0
            .values()
            .filter(|collision| collision.during_current_frame)
    }

    /// Returns a mutable iterator over the collisions that have happened during the current physics frame.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Contacts> {
        self.0
            .values_mut()
            .filter(|collision| collision.during_current_frame)
    }

    /// Returns an iterator over all collisions with a given entity.
    pub fn collisions_with_entity(&self, entity: Entity) -> impl Iterator<Item = &Contacts> {
        self.0
            .iter()
            .filter_map(move |((entity1, entity2), contacts)| {
                if contacts.during_current_frame && (*entity1 == entity || *entity2 == entity) {
                    Some(contacts)
                } else {
                    None
                }
            })
    }

    /// Returns an iterator over all collisions with a given entity.
    pub fn collisions_with_entity_mut(
        &mut self,
        entity: Entity,
    ) -> impl Iterator<Item = &mut Contacts> {
        self.0
            .iter_mut()
            .filter_map(move |((entity1, entity2), contacts)| {
                if contacts.during_current_frame && (*entity1 == entity || *entity2 == entity) {
                    Some(contacts)
                } else {
                    None
                }
            })
    }

    /// Inserts contact data for a collision between two entities.
    ///
    /// If a collision entry with the same entities already exists, it will be overwritten,
    /// and the old value will be returned. Otherwise, `None` is returned.
    ///
    /// **Note**: Manually inserting collisions can be error prone and should generally be avoided.
    /// If you simply want to modify existing collisions, consider using methods like [`get_mut`](Self::get_mut)
    /// or [`iter_mut`](Self::iter_mut).
    pub fn insert_collision_pair(&mut self, contacts: Contacts) -> Option<Contacts> {
        // TODO: We might want to order the data by Entity ID so that entity1, point1 etc. are for the "smaller"
        // entity ID. This requires changes elsewhere as well though.
        self.0
            .insert((contacts.entity1, contacts.entity2), contacts)
    }

    /// Extends [`Collisions`] with all collision pairs in the given iterable.
    ///
    /// This is mostly equivalent to calling [`insert_collision_pair`](Self::insert_collision_pair)
    /// for each of the collision pairs.
    pub fn extend<I: IntoIterator<Item = Contacts>>(&mut self, collisions: I) {
        // (Note: this is a copy of `std`/`hashbrown`/`indexmap`'s reservation logic.)
        // Keys may be already present or show multiple times in the iterator.
        // Reserve the entire hint lower bound if the map is empty.
        // Otherwise reserve half the hint (rounded up), so the map
        // will only resize twice in the worst case.
        let iter = collisions.into_iter();
        let reserve = if self.get_internal().is_empty() {
            iter.size_hint().0
        } else {
            (iter.size_hint().0 + 1) / 2
        };
        self.get_internal_mut().reserve(reserve);
        iter.for_each(move |contacts| {
            self.insert_collision_pair(contacts);
        });
    }

    /// Retains only the collisions for which the specified predicate returns `true`.
    /// Collisions for which the predicate returns `false` are removed.
    pub fn retain<F>(&mut self, mut keep: F)
    where
        F: FnMut(&mut Contacts) -> bool,
    {
        self.0.retain(|_, contacts| keep(contacts));
    }

    /// Removes a collision between two entites and returns its value.
    ///
    /// The order of the entities does not matter.
    pub fn remove_collision_pair(&mut self, entity1: Entity, entity2: Entity) -> Option<Contacts> {
        self.0
            .remove(&(entity1, entity2))
            .or_else(|| self.0.remove(&(entity2, entity1)))
    }

    /// Removes all collisions that involve the given entity.
    pub fn remove_collisions_with_entity(&mut self, entity: Entity) {
        self.retain(|contacts| contacts.entity1 != entity && contacts.entity2 != entity);
    }
}

/// Stores the collision pairs from the previous frame.
/// This is used for detecting when collisions have started or ended.
#[derive(Resource, Clone, Debug, Default, Deref, DerefMut, PartialEq)]
pub(super) struct PreviousCollisions(Collisions);

/// All contacts between two colliders.
///
/// The contacts are stored in contact manifolds.
/// Each manifold contains one or more contact points, and each contact
/// in a given manifold shares the same contact normal.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Contacts {
    /// First entity in the contact.
    pub entity1: Entity,
    /// Second entity in the contact.
    pub entity2: Entity,
    /// A list of contact manifolds between two colliders.
    /// Each manifold contains one or more contact points, but each contact
    /// in a given manifold shares the same contact normal.
    pub manifolds: Vec<ContactManifold>,
    /// True if the bodies have been in contact during this frame.
    pub during_current_frame: bool,
    /// True if the bodies have been in contact during this substep.
    pub during_current_substep: bool,
    /// True if the bodies were in contact during the previous frame.
    pub during_previous_frame: bool,
}

/// A contact manifold between two colliders, containing a set of contact points.
/// Each contact in a manifold shares the same contact normal.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactManifold {
    /// The contacts in this manifold.
    pub contacts: Vec<ContactData>,
    /// A contact normal shared by all contacts in this manifold,
    /// expressed in the local space of the first entity.
    pub normal1: Vector,
    /// A contact normal shared by all contacts in this manifold,
    /// expressed in the local space of the second entity.
    pub normal2: Vector,
}

impl ContactManifold {
    /// Returns the world-space contact normal pointing towards the exterior of the first entity.
    pub fn global_normal1(&self, rotation: &Rotation) -> Vector {
        rotation.rotate(self.normal1)
    }

    /// Returns the world-space contact normal pointing towards the exterior of the second entity.
    pub fn global_normal2(&self, rotation: &Rotation) -> Vector {
        rotation.rotate(self.normal2)
    }
}

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactData {
    /// Contact point on the first entity in local coordinates.
    pub point1: Vector,
    /// Contact point on the second entity in local coordinates.
    pub point2: Vector,
    /// A contact normal expressed in the local space of the first entity.
    pub normal1: Vector,
    /// A contact normal expressed in the local space of the second entity.
    pub normal2: Vector,
    /// Penetration depth.
    pub penetration: Scalar,
}

impl ContactData {
    /// Returns the global contact point on the first entity,
    /// transforming the local point by the given entity position and rotation.
    pub fn global_point1(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation.rotate(self.point1)
    }

    /// Returns the global contact point on the second entity,
    /// transforming the local point by the given entity position and rotation.
    pub fn global_point2(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation.rotate(self.point2)
    }

    /// Returns the world-space contact normal pointing towards the exterior of the first entity.
    pub fn global_normal1(&self, rotation: &Rotation) -> Vector {
        rotation.rotate(self.normal1)
    }

    /// Returns the world-space contact normal pointing towards the exterior of the second entity.
    pub fn global_normal2(&self, rotation: &Rotation) -> Vector {
        rotation.rotate(self.normal2)
    }
}
