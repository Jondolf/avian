use crate::data_structures::{
    graph::NodeIndex, pair_key::PairKey, sparse_secondary_map::SparseSecondaryEntityMap,
    stable_graph::StableUnGraph,
};
#[expect(unused_imports)]
use crate::prelude::*;
use bevy::{platform::collections::HashSet, prelude::*};

use super::{ContactEdge, ContactEdgeFlags, ContactId};

/// A resource that stores all [`ContactEdge`]s in the physics world in an [undirected graph](UnGraph).
///
/// Contact pairs exist between [colliders](Collider) that have intersecting [AABBs](ColliderAabb),
/// even if the shapes themselves are not yet touching.
///
/// For performance reasons, this does *not* store the actual contact data,
/// only how contact pairs are connected. The contact data is stored in the [`ConstraintGraph`]
/// resource, and can be accessed using the [`ContactEdge`]s stored in this resource.
///
/// For a simpler API that abstracts over this complexity, consider using the [`Collisions`]
/// system parameter.
///
/// # Usage
///
/// The following methods can be used for querying collisions:
///
/// - [`get`](Self::get) and [`get_mut`](Self::get_mut)
/// - [`iter`](Self::iter) and [`iter_mut`](Self::iter_mut)
/// - [`contains`](Self::contains)
/// - [`collisions_with`](Self::collisions_with) and
///   [`collisions_with_mut`](Self::collisions_with_mut)
/// - [`entities_colliding_with`](Self::entities_colliding_with)
///
/// For example, to iterate over all collisions with a given entity:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// #[derive(Component)]
/// struct PressurePlate;
///
/// fn activate_pressure_plates(mut query: Query<Entity, With<PressurePlate>>, contact_graph: Res<ContactGraph>) {
///     for pressure_plate in &query {
///         // Compute the total impulse applied to the pressure plate.
///         let mut total_impulse = 0.0;
///
///         for contact_pair in contact_graph.collisions_with(pressure_plate) {
///             total_impulse += contact_pair.total_normal_impulse_magnitude();
///         }
///
///         if total_impulse > 5.0 {
///             println!("Pressure plate activated!");
///         }
///     }
/// }
/// ```
///
/// While mutable access is allowed, contact modification and filtering should typically
/// be done using [`CollisionHooks`]. See the documentation for more information.
///
/// # Warning
///
/// For users, this resource is primarily for querying and reading collision data.
///
/// Directly adding, modifying, or removing contact pairs using this resource will *not* trigger any collision events,
/// wake up the entities involved, or perform any other cleanup. Only make structural modifications if you know what you are doing.
///
/// For filtering and modifying collisions, consider using [`CollisionHooks`] instead.
#[derive(Resource, Clone, Debug, Default)]
pub struct ContactGraph {
    // TODO: We could have a separate intersection graph for sensors.
    /// The internal undirected graph where nodes are entities and edges are contact pairs.
    pub edges: ContactGraphInternal,

    /// A set of all contact pairs for fast lookup.
    ///
    /// The [`PairKey`] is a unique pair of entity indices, sorted in ascending order,
    /// concatenated into a single `u64` key.
    ///
    /// Two entities have a contact pair if they have intersecting AABBs.
    pub(crate) pair_set: HashSet<PairKey>,

    /// A map from entities to their corresponding node indices in the contact graph.
    entity_to_node: SparseSecondaryEntityMap<NodeIndex>,
}

/// An undirected graph where nodes are entities and edges are contact pairs.
///
/// The graph maintains stable indices for nodes and edges even when items are removed.
///
/// This is stored internally in the [`ContactGraph`] resource, which provides a higher-level API
/// for working with contact pairs.
pub type ContactGraphInternal = StableUnGraph<Entity, ContactEdge>;

impl ContactGraph {
    /// Returns the contact pair between two entities.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get(&self, entity1: Entity, entity2: Entity) -> Option<&ContactEdge> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_to_node.get(entity1),
            self.entity_to_node.get(entity2),
        ) else {
            return None;
        };

        self.edges
            .find_edge(index1, index2)
            .and_then(|edge| self.get_by_id(edge.into()))
    }

    /// Returns the contact pair between two entities based on the contact ID.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_by_id(&self, id: ContactId) -> Option<&ContactEdge> {
        self.edges.edge_weight(id.into())
    }

    /// Returns a mutable reference to the contact pair between two entities.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_mut(&mut self, entity1: Entity, entity2: Entity) -> Option<&mut ContactEdge> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_to_node.get(entity1),
            self.entity_to_node.get(entity2),
        ) else {
            return None;
        };

        self.edges
            .find_edge(index1, index2)
            .and_then(|edge| self.edges.edge_weight_mut(edge))
    }

    /// Returns a mutable reference to the contact pair between two entities based on the contact ID.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_mut_by_id(&mut self, id: ContactId) -> Option<&mut ContactEdge> {
        self.edges.edge_weight_mut(id.into())
    }

    /// Returns `true` if the given entities have a contact pair.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    #[inline]
    pub fn contains(&self, entity1: Entity, entity2: Entity) -> bool {
        self.contains_key(&PairKey::new(entity1.index(), entity2.index()))
    }

    /// Returns `true` if the given pair key is in the contact graph.
    ///
    /// The pair key should be equivalent to `PairKey::new(entity1.index(), entity2.index())`.
    ///
    /// This method can be useful to avoid constructing a new `PairKey` when the key is already known.
    /// If the key is not available, consider using [`contains`](Self::contains) instead.
    #[inline]
    pub fn contains_key(&self, pair_key: &PairKey) -> bool {
        self.pair_set.contains(pair_key)
    }

    /// Returns an iterator yielding immutable access to all contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_touching`](Self::iter_touching) instead.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = &ContactEdge> {
        self.edges.all_edge_weights()
    }

    /// Returns an iterator yielding immutable access to all contact pairs that are currently touching.
    ///
    /// This is a subset of [`iter`](Self::iter) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_touching(&self) -> impl Iterator<Item = &ContactEdge> {
        self.iter()
            .filter(|contacts| contacts.flags.contains(ContactEdgeFlags::TOUCHING))
    }

    /// Returns a iterator yielding mutable access to all contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_touching_mut`](Self::iter_touching_mut) instead.
    #[inline]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut ContactEdge> {
        self.edges.all_edge_weights_mut()
    }

    /// Returns a iterator yielding mutable access to all contact pairs that are currently touching.
    ///
    /// This is a subset of [`iter_mut`](Self::iter_mut) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_touching_mut(&mut self) -> impl Iterator<Item = &mut ContactEdge> {
        self.iter_mut()
            .filter(|contacts| contacts.flags.contains(ContactEdgeFlags::TOUCHING))
    }

    /// Returns an iterator yielding immutable access to all contact pairs involving the given entity.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// Use [`ContactEdge::is_touching`](ContactEdge::is_touching) to determine if the actual collider shapes are touching.
    #[inline]
    pub fn collisions_with(&self, entity: Entity) -> impl Iterator<Item = &ContactEdge> {
        self.entity_to_node
            .get(entity)
            .into_iter()
            .flat_map(move |&index| self.edges.edge_weights(index))
    }

    /// Returns an iterator yielding immutable access to all entities that have a contact pair with the given entity.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    #[inline]
    pub fn entities_colliding_with(&self, entity: Entity) -> impl Iterator<Item = Entity> + '_ {
        self.entity_to_node
            .get(entity)
            .into_iter()
            .flat_map(move |&index| {
                self.edges
                    .neighbors(index)
                    .map(|index| *self.edges.node_weight(index).unwrap())
            })
    }

    /// Creates a contact pair between two entities.
    ///
    /// Returns the ID of the contact pair if it was created,
    /// or `None` if the pair already exists.
    ///
    /// # Warning
    ///
    /// Creating a collision pair with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn add_pair(&mut self, contacts: ContactEdge) -> Option<ContactId> {
        let pair_key = PairKey::new(contacts.collider1.index(), contacts.collider2.index());
        self.add_pair_with_key(contacts, pair_key)
    }

    /// Creates a contact pair between two entities with the given pair key.
    ///
    /// The key must be equivalent to `PairKey::new(contacts.entity1.index(), contacts.entity2.index())`.
    ///
    /// Returns the ID of the contact pair if it was created,
    /// or `None` if the pair already exists.
    ///
    /// This method can be useful to avoid constructing a new `PairKey` when the key is already known.
    /// If the key is not available, consider using [`add_pair`](Self::add_pair) instead.
    ///
    /// # Warning
    ///
    /// Creating a collision pair with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn add_pair_with_key(
        &mut self,
        contacts: ContactEdge,
        pair_key: PairKey,
    ) -> Option<ContactId> {
        // Add the pair to the pair set for fast lookup.
        if !self.pair_set.insert(pair_key) {
            // The pair already exists.
            return None;
        }

        // Get the indices of the entities in the graph.
        let index1 = self
            .entity_to_node
            .get_or_insert_with(contacts.collider1, || {
                self.edges.add_node(contacts.collider1)
            });
        let index2 = self
            .entity_to_node
            .get_or_insert_with(contacts.collider2, || {
                self.edges.add_node(contacts.collider2)
            });

        // Add the edge to the graph.
        Some(ContactId(self.edges.add_edge(index1, index2, contacts).0))
    }

    /// Inserts a contact pair between two entities.
    ///
    /// If a pair with the same entities already exists, it will be overwritten.
    ///
    /// # Warning
    ///
    /// Inserting a collision pair with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn insert_pair(&mut self, contacts: ContactEdge) {
        let pair_key = PairKey::new(contacts.collider1.index(), contacts.collider2.index());
        self.insert_pair_with_key(contacts, pair_key);
    }

    /// Inserts a contact pair between two entities with the given pair key.
    ///
    /// The key must be equivalent to `PairKey::new(contacts.entity1.index(), contacts.entity2.index())`.
    ///
    /// If a pair with the same entities already exists, it will be overwritten.
    ///
    /// This method can be useful to avoid constructing a new `PairKey` when the key is already known.
    /// If the key is not available, consider using [`insert_pair`](Self::insert_pair) instead.
    ///
    /// # Warning
    ///
    /// Inserting a collision pair with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn insert_pair_with_key(&mut self, contacts: ContactEdge, pair_key: PairKey) {
        // Add the pair to the pair set for fast lookup.
        self.pair_set.insert(pair_key);

        // Get the indices of the entities in the graph.
        let index1 = self
            .entity_to_node
            .get_or_insert_with(contacts.collider1, || {
                self.edges.add_node(contacts.collider1)
            });
        let index2 = self
            .entity_to_node
            .get_or_insert_with(contacts.collider2, || {
                self.edges.add_node(contacts.collider2)
            });

        // Update the edge in the graph.
        self.edges.update_edge(index1, index2, contacts);
    }

    /// Removes a contact pair between two entites and returns its value.
    ///
    /// # Warning
    ///
    /// Removing a collision pair with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    ///
    /// For filtering and modifying collisions, consider using [`CollisionHooks`] instead.
    #[inline]
    pub fn remove_pair(&mut self, entity1: Entity, entity2: Entity) -> Option<ContactEdge> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_to_node.get(entity1),
            self.entity_to_node.get(entity2),
        ) else {
            return None;
        };

        // Remove the edge from the graph.
        self.edges.find_edge(index1, index2).and_then(|edge_id| {
            let pair_key = PairKey::new(entity1.index(), entity2.index());
            self.remove_pair_by_id(&pair_key, edge_id.into())
        })
    }

    /// Removes a contact pair based on its pair key and ID and returns its value.
    ///
    /// # Warning
    ///
    /// Removing a collision pair with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    ///
    /// For filtering and modifying collisions, consider using [`CollisionHooks`] instead.
    #[inline]
    pub fn remove_pair_by_id(&mut self, pair_key: &PairKey, id: ContactId) -> Option<ContactEdge> {
        // Remove the pair from the pair set.
        self.pair_set.remove(pair_key);

        // Remove the edge from the graph.
        self.edges.remove_edge(id.into())
    }

    /// Removes the collider of the given entity from the contact graph,
    /// calling the given callback for each contact pair that is removed in the process.
    ///
    /// # Warning
    ///
    /// Removing a collider with this method will *not* trigger any collision events
    /// or wake up the entities involved. Only use this method if you know what you are doing.
    #[inline]
    pub fn remove_collider_with<F>(&mut self, entity: Entity, mut pair_callback: F)
    where
        F: FnMut(&mut StableUnGraph<Entity, ContactEdge>, ContactEdge),
    {
        // Remove the entity from the entity-to-node mapping,
        // and get the index of the node in the graph.
        let Some(index) = self.entity_to_node.remove(entity) else {
            return;
        };

        // Remove the entity from the graph.
        self.edges.remove_node_with(index, |graph, contacts| {
            let pair_key = PairKey::new(contacts.collider1.index(), contacts.collider2.index());

            pair_callback(graph, contacts);

            // Remove the pair from the pair set.
            self.pair_set.remove(&pair_key);
        });

        // Removing the node swapped the last node to its place,
        // so we need to remap the entity-to-node mapping of the swapped node.
        if let Some(swapped) = self.edges.node_weight(index).copied() {
            let swapped_index = self
                .entity_to_node
                .get_mut(swapped)
                // This should never panic.
                .expect("swapped entity has no entity-to-node mapping");
            *swapped_index = index;
        }
    }
}
