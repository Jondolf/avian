//! Collision detection for [`Collider`]s.
//!
//! Collision detection involves determining pairs of objects that may currently be in contact
//! (or are expected to come into contact), and computing contact data for each intersection.
//! These contacts are then used by the [solver](dynamics::solver) to generate
//! [`ContactConstraint`](dynamics::solver::contact::ContactConstraint)s and finally resolve overlap.
//!
//! In Avian, collision detection is split into three plugins:
//!
//! - [`BroadPhasePlugin`]: Performs intersection tests to determine potential collisions, adding them to [`BroadCollisionPairs`].
//! - [`NarrowPhasePlugin`]: Computes [`Contacts`] for each pair in [`BroadCollisionPairs`], adding them to [`Collisions`].
//! - [`ContactReportingPlugin`] (optional): Sends collision events and updates [`CollidingEntities`] based on [`Collisions`].
//!
//! Spatial queries are handled separately by the [`SpatialQueryPlugin`].
//!
//! You can also find several utility methods for computing contacts in [`contact_query`].

pub mod broad_phase;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub mod contact_query;
pub mod contact_reporting;
pub mod narrow_phase;

pub mod collider;
pub use collider::*;

mod layers;
pub use layers::*;

mod feature_id;
pub use feature_id::PackedFeatureId;

use crate::{
    data_structures::{
        entity_data_index::EntityDataIndex,
        graph::{EdgeIndex, NodeIndex, UnGraph},
    },
    prelude::*,
};
use bevy::prelude::*;

// TODO: Add `retain` method.
// TODO: Handle edge addition/removal properly, and add `ContactFlags`.
// TODO: Update docs.

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
/// # Usage
///
/// [`Collisions`] can be accessed at almost anytime, but for modifying and filtering collisions,
/// it is recommended to use the [`PostProcessCollisions`] schedule. See its documentation
/// for more information.
///
/// ## Querying Collisions
///
/// The following methods can be used for querying existing collisions:
///
/// - [`get`](Self::get) and [`get_mut`](Self::get_mut)
/// - [`iter`](Self::iter) and [`iter_mut`](Self::iter_mut)
/// - [`contains`](Self::contains)
/// - [`collisions_with_entity`](Self::collisions_with_entity) and
///   [`collisions_with_entity_mut`](Self::collisions_with_entity_mut)
///
/// The collisions can be accessed at any time, but modifications to contacts should be performed
/// in the [`PostProcessCollisions`] schedule. Otherwise, the physics solver will use the old contact data.
///
/// ## Filtering and Removing Collisions
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
/// ## Adding New Collisions
///
/// The following methods can be used for adding new collisions:
///
/// - [`insert_collision_pair`](Self::insert_collision_pair)
/// - [`extend`](Self::extend)
///
/// The most convenient place for adding new collisions is in the [`PostProcessCollisions`] schedule.
/// Otherwise, the physics solver might not have access to them in time.
///
/// # Implementation Details
///
/// Internally, the collisions are stored in an `IndexMap` that contains collisions from both the current frame
/// and the previous frame, which is used for things like [collision events](ContactReportingPlugin#collision-events).
///
/// However, the public methods only use the current frame's collisions. To access the internal data structure,
/// you can use [`get_internal`](Self::get_internal) or [`get_internal_mut`](Self::get_internal_mut).
#[derive(Resource, Clone, Debug, Default)]
pub struct Collisions {
    // TODO: Separate intersection graph and contact graph.
    /// The contact graph where nodes are entities and edges are contact pairs.
    pub graph: UnGraph<Entity, Contacts>,
    /// A map from entities to their corresponding node indices in the contact graph.
    entity_graph_index: EntityDataIndex<NodeIndex>,
}

impl Collisions {
    /// Returns a reference to the internal `IndexMap`.
    #[deprecated(since = "0.3.0", note = "Access `graph` instead.")]
    pub fn get_internal(&self) -> &UnGraph<Entity, Contacts> {
        &self.graph
    }

    /// Returns a mutable reference to the internal `IndexMap`.
    #[deprecated(since = "0.3.0", note = "Access `graph` instead.")]
    pub fn get_internal_mut(&mut self) -> &mut UnGraph<Entity, Contacts> {
        &mut self.graph
    }

    /// Returns a reference to the [contacts](Contacts) stored for the given entity pair if they are colliding,
    /// else returns `None`.
    ///
    /// The order of the entities does not matter.
    pub fn get(&self, entity1: Entity, entity2: Entity) -> Option<&Contacts> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_graph_index.get(entity1),
            self.entity_graph_index.get(entity2),
        ) else {
            return None;
        };

        self.graph
            .find_edge(index1, index2)
            .and_then(|edge| self.graph.edge_weight(edge))
    }

    /// Returns a mutable reference to the [contacts](Contacts) stored for the given entity pair if they are colliding,
    /// else returns `None`.
    ///
    /// The order of the entities does not matter.
    pub fn get_mut(&mut self, entity1: Entity, entity2: Entity) -> Option<&mut Contacts> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_graph_index.get(entity1),
            self.entity_graph_index.get(entity2),
        ) else {
            return None;
        };

        self.graph
            .find_edge(index1, index2)
            .and_then(|edge| self.graph.edge_weight_mut(edge))
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
        self.graph.all_edge_weights()
    }

    /// Returns a mutable iterator over the collisions that have happened during the current physics frame.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Contacts> {
        self.graph.all_edge_weights_mut()
    }

    /// Returns an iterator over all collisions with a given entity.
    pub fn collisions_with_entity(&self, entity: Entity) -> impl Iterator<Item = &Contacts> {
        self.entity_graph_index
            .get(entity)
            .into_iter()
            .flat_map(move |&index| self.graph.edge_weights(index))
    }

    // TODO: Return an empty iterator instead of `None`.
    /// Returns an iterator over all collisions with a given entity.
    pub fn collisions_with_entity_mut(
        &mut self,
        entity: Entity,
    ) -> Option<impl Iterator<Item = &mut Contacts>> {
        let index = *self.entity_graph_index.get(entity)?;
        Some(self.graph.edge_weights_mut(index))
    }

    /// Inserts contact data for a collision between two entities.
    ///
    /// If a collision entry with the same entities already exists, it will be overwritten,
    /// and the old value will be returned. Otherwise, `None` is returned.
    ///
    /// **Note**: Manually inserting collisions can be error prone and should generally be avoided.
    /// If you simply want to modify existing collisions, consider using methods like [`get_mut`](Self::get_mut)
    /// or [`iter_mut`](Self::iter_mut).
    pub fn insert_collision_pair(&mut self, contacts: Contacts) {
        let (index1, index2) = self.entity_graph_index.ensure_pair_exists(
            contacts.entity1,
            contacts.entity2,
            NodeIndex::END,
        );

        if index1.is_end() {
            *index1 = self.graph.add_node(contacts.entity1);
        }

        if index2.is_end() {
            *index2 = self.graph.add_node(contacts.entity2);
        }

        if self.graph.find_edge(*index1, *index2).is_none() {
            self.graph.add_edge(*index1, *index2, contacts);
        }
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
        let reserve = if self.graph.raw_edges().is_empty() {
            iter.size_hint().0
        } else {
            (iter.size_hint().0 + 1) / 2
        };
        self.graph.reserve_edges(reserve);
        iter.for_each(move |contacts| {
            self.insert_collision_pair(contacts);
        });
    }

    // TODO: Retain

    /// Removes a collision between two entites and returns its value.
    ///
    /// The order of the entities does not matter.
    pub fn remove_collision_pair(&mut self, entity1: Entity, entity2: Entity) -> Option<Contacts> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_graph_index.get(entity1),
            self.entity_graph_index.get(entity2),
        ) else {
            return None;
        };

        self.graph
            .find_edge(index1, index2)
            .and_then(|edge| self.graph.remove_edge(edge))
    }

    /// Removes all collisions that involve the given entity.
    pub fn remove_collisions_with_entity(&mut self, entity: Entity) {
        // TODO: Avoid collecting the edges.
        let edges: Vec<EdgeIndex> = self
            .graph
            .edges(NodeIndex(entity.index()))
            .map(|edge| edge.index())
            .collect();
        edges.iter().for_each(|edge| {
            self.graph.remove_edge(*edge);
        });
    }
}

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
    /// The entity of the first body involved in the contact.
    pub body_entity1: Option<Entity>,
    /// The entity of the second body involved in the contact.
    pub body_entity2: Option<Entity>,
    /// A list of contact manifolds between two colliders.
    /// Each manifold contains one or more contact points, but each contact
    /// in a given manifold shares the same contact normal.
    pub manifolds: Vec<ContactManifold>,
    /// True if either of the colliders involved is a sensor.
    pub is_sensor: bool,
    /// True if the bodies have been in contact during this frame.
    pub during_current_frame: bool,
    /// True if the bodies were in contact during the previous frame.
    pub during_previous_frame: bool,
    /// The total normal impulse applied to the first body in a collision.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_seconds()`.
    pub total_normal_impulse: Scalar,
    /// The total tangent impulse applied to the first body in a collision.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_seconds()`.
    #[cfg(feature = "2d")]
    #[doc(alias = "total_friction_impulse")]
    pub total_tangent_impulse: Scalar,
    /// The total tangent impulse applied to the first body in a collision.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_seconds()`.
    #[cfg(feature = "3d")]
    #[doc(alias = "total_friction_impulse")]
    pub total_tangent_impulse: Vector2,
}

impl Contacts {
    /// The force corresponding to the total normal impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_seconds()`.
    pub fn total_normal_force(&self, delta_time: Scalar) -> Scalar {
        self.total_normal_impulse / delta_time
    }

    /// The force corresponding to the total tangent impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_seconds()`.
    #[cfg(feature = "2d")]
    #[doc(alias = "total_friction_force")]
    pub fn total_tangent_force(&self, delta_time: Scalar) -> Scalar {
        self.total_tangent_impulse / delta_time
    }

    /// The force corresponding to the total tangent impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_seconds()`.
    #[cfg(feature = "3d")]
    #[doc(alias = "total_friction_force")]
    pub fn total_tangent_force(&self, delta_time: Scalar) -> Vector2 {
        self.total_tangent_impulse / delta_time
    }

    /// Returns `true` if a collision started during the current frame.
    pub fn collision_started(&self) -> bool {
        !self.during_previous_frame && self.during_current_frame
    }

    /// Returns `true` if a collision stopped during the current frame.
    pub fn collision_stopped(&self) -> bool {
        self.during_previous_frame && !self.during_current_frame
    }

    /// Returns the contact with the largest penetration depth.
    ///
    /// If the objects are separated but there is still a speculative contact,
    /// the penetration depth will be negative.
    ///
    /// If there are no contacts, `None` is returned.
    pub fn find_deepest_contact(&self) -> Option<&ContactData> {
        self.manifolds
            .iter()
            .filter_map(|manifold| manifold.find_deepest_contact())
            .max_by(|a, b| {
                a.penetration
                    .partial_cmp(&b.penetration)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }
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
    /// The index of the manifold in the collision.
    pub index: usize,
}

impl ContactManifold {
    /// Returns the world-space contact normal pointing towards the exterior of the first entity.
    pub fn global_normal1(&self, rotation: &Rotation) -> Vector {
        rotation * self.normal1
    }

    /// Returns the world-space contact normal pointing towards the exterior of the second entity.
    pub fn global_normal2(&self, rotation: &Rotation) -> Vector {
        rotation * self.normal2
    }

    /// Copies impulses from previous contacts to matching contacts in `self`.
    ///
    /// Contacts are first matched based on their [feature IDs](PackedFeatureId), and if they are unknown,
    /// matching is done based on contact positions using the given `distance_threshold`
    /// for determining if points are too far away from each other to be considered matching.
    pub fn match_contacts(
        &mut self,
        previous_contacts: &[ContactData],
        distance_threshold: Scalar,
    ) {
        // The squared maximum distance for two contact points to be considered matching.
        let distance_threshold_squared = distance_threshold.powi(2);

        for contact in self.contacts.iter_mut() {
            for previous_contact in previous_contacts.iter() {
                // If the feature IDs match, copy the contact impulses over for warm starting.
                if (contact.feature_id1 == previous_contact.feature_id1
                    && contact.feature_id2 == previous_contact.feature_id2) ||
                    // we have to check both directions because the entities are sorted in order
                    // of aabb.min.x, which could have changed even the two objects in contact are the same
                    (contact.feature_id2 == previous_contact.feature_id1
                    && contact.feature_id1 == previous_contact.feature_id2)
                {
                    contact.normal_impulse = previous_contact.normal_impulse;
                    contact.tangent_impulse = previous_contact.tangent_impulse;
                    break;
                }

                let unknown_features = contact.feature_id1 == PackedFeatureId::UNKNOWN
                    || contact.feature_id2 == PackedFeatureId::UNKNOWN;

                // If the feature IDs are unknown and the contact positions match closely enough,
                // copy the contact impulses over for warm starting.
                if unknown_features
                    && (contact.point1.distance_squared(previous_contact.point1)
                        < distance_threshold_squared
                        && contact.point2.distance_squared(previous_contact.point2)
                            < distance_threshold_squared)
                    || (contact.point1.distance_squared(previous_contact.point2)
                        < distance_threshold_squared
                        && contact.point2.distance_squared(previous_contact.point1)
                            < distance_threshold_squared)
                {
                    contact.normal_impulse = previous_contact.normal_impulse;
                    contact.tangent_impulse = previous_contact.tangent_impulse;
                    break;
                }
            }
        }
    }

    /// Returns the contact with the largest penetration depth.
    ///
    /// If the objects are separated but there is still a speculative contact,
    /// the penetration depth will be negative.
    ///
    /// If there are no contacts, `None` is returned.
    pub fn find_deepest_contact(&self) -> Option<&ContactData> {
        self.contacts.iter().max_by(|a, b| {
            a.penetration
                .partial_cmp(&b.penetration)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    }
}

/// Data related to a single contact between two bodies.
///
/// If you want a contact that belongs to a [contact manifold](ContactManifold) and has more data,
/// see [`ContactData`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct SingleContact {
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

impl SingleContact {
    /// Creates a new [`SingleContact`]. The contact points and normals should be given in local space.
    pub fn new(
        point1: Vector,
        point2: Vector,
        normal1: Vector,
        normal2: Vector,
        penetration: Scalar,
    ) -> Self {
        Self {
            point1,
            point2,
            normal1,
            normal2,
            penetration,
        }
    }

    /// Returns the global contact point on the first entity,
    /// transforming the local point by the given entity position and rotation.
    pub fn global_point1(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.point1
    }

    /// Returns the global contact point on the second entity,
    /// transforming the local point by the given entity position and rotation.
    pub fn global_point2(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.point2
    }

    /// Returns the world-space contact normal pointing towards the exterior of the first entity.
    pub fn global_normal1(&self, rotation: &Rotation) -> Vector {
        rotation * self.normal1
    }

    /// Returns the world-space contact normal pointing towards the exterior of the second entity.
    pub fn global_normal2(&self, rotation: &Rotation) -> Vector {
        rotation * self.normal2
    }

    /// Flips the contact data, swapping the points and normals.
    pub fn flip(&mut self) {
        std::mem::swap(&mut self.point1, &mut self.point2);
        std::mem::swap(&mut self.normal1, &mut self.normal2);
    }

    /// Returns a flipped copy of the contact data, swapping the points and normals.
    pub fn flipped(&self) -> Self {
        Self {
            point1: self.point2,
            point2: self.point1,
            normal1: self.normal2,
            normal2: self.normal1,
            penetration: self.penetration,
        }
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
    /// The impulse applied to the first body along the normal.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_seconds()`.
    pub normal_impulse: Scalar,
    /// The impulse applied to the first body along the tangent. This corresponds to the impulse caused by friction.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_seconds()`.
    #[cfg(feature = "2d")]
    #[doc(alias = "friction_impulse")]
    pub tangent_impulse: Scalar,
    /// The impulse applied to the first body along the tangent. This corresponds to the impulse caused by friction.
    ///
    /// To get the corresponding force, divide the impulse by `Time<Substeps>::delta_seconds()`.
    #[cfg(feature = "3d")]
    #[doc(alias = "friction_impulse")]
    pub tangent_impulse: Vector2,
    /// The contact feature ID on the first shape. This indicates the ID of
    /// the vertex, edge, or face of the contact, if one can be determined.
    pub feature_id1: PackedFeatureId,
    /// The contact feature ID on the second shape. This indicates the ID of
    /// the vertex, edge, or face of the contact, if one can be determined.
    pub feature_id2: PackedFeatureId,
}

impl ContactData {
    /// Creates a new [`ContactData`]. The contact points and normals should be given in local space.
    ///
    /// [Feature IDs](PackedFeatureId) can be specified for the contact points using [`with_feature_ids`](Self::with_feature_ids).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        point1: Vector,
        point2: Vector,
        normal1: Vector,
        normal2: Vector,
        penetration: Scalar,
    ) -> Self {
        Self {
            point1,
            point2,
            normal1,
            normal2,
            penetration,
            normal_impulse: 0.0,
            tangent_impulse: default(),
            feature_id1: PackedFeatureId::UNKNOWN,
            feature_id2: PackedFeatureId::UNKNOWN,
        }
    }

    /// Sets the [feature IDs](PackedFeatureId) of the contact points.
    pub fn with_feature_ids(mut self, id1: PackedFeatureId, id2: PackedFeatureId) -> Self {
        self.feature_id1 = id1;
        self.feature_id2 = id2;
        self
    }

    /// The force corresponding to the normal impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_seconds()`.
    pub fn normal_force(&self, delta_time: Scalar) -> Scalar {
        self.normal_impulse / delta_time
    }

    /// The force corresponding to the tangent impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_seconds()`.
    #[cfg(feature = "2d")]
    #[doc(alias = "friction_force")]
    pub fn tangent_force(&self, delta_time: Scalar) -> Scalar {
        self.tangent_impulse / delta_time
    }

    /// The force corresponding to the tangent impulse applied over `delta_time`.
    ///
    /// Because contacts are solved over several substeps, `delta_time` should
    /// typically use `Time<Substeps>::delta_seconds()`.
    #[cfg(feature = "3d")]
    #[doc(alias = "friction_force")]
    pub fn tangent_force(&self, delta_time: Scalar) -> Vector2 {
        self.tangent_impulse / delta_time
    }

    /// Returns the global contact point on the first entity,
    /// transforming the local point by the given entity position and rotation.
    pub fn global_point1(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.point1
    }

    /// Returns the global contact point on the second entity,
    /// transforming the local point by the given entity position and rotation.
    pub fn global_point2(&self, position: &Position, rotation: &Rotation) -> Vector {
        position.0 + rotation * self.point2
    }

    /// Returns the world-space contact normal pointing towards the exterior of the first entity.
    pub fn global_normal1(&self, rotation: &Rotation) -> Vector {
        rotation * self.normal1
    }

    /// Returns the world-space contact normal pointing towards the exterior of the second entity.
    pub fn global_normal2(&self, rotation: &Rotation) -> Vector {
        rotation * self.normal2
    }

    /// Flips the contact data, swapping the points, normals, and feature IDs,
    /// and negating the impulses.
    pub fn flip(&mut self) {
        std::mem::swap(&mut self.point1, &mut self.point2);
        std::mem::swap(&mut self.normal1, &mut self.normal2);
        std::mem::swap(&mut self.feature_id1, &mut self.feature_id2);
        self.normal_impulse = -self.normal_impulse;
        self.tangent_impulse = -self.tangent_impulse;
    }

    /// Returns a flipped copy of the contact data, swapping the points, normals, and feature IDs,
    /// and negating the impulses.
    pub fn flipped(&self) -> Self {
        Self {
            point1: self.point2,
            point2: self.point1,
            normal1: self.normal2,
            normal2: self.normal1,
            penetration: self.penetration,
            normal_impulse: -self.normal_impulse,
            tangent_impulse: -self.tangent_impulse,
            feature_id1: self.feature_id2,
            feature_id2: self.feature_id1,
        }
    }
}
