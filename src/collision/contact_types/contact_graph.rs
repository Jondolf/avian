use crate::prelude::*;
use crate::{
    data_structures::{
        graph::NodeIndex, pair_key::PairKey, sparse_secondary_map::SparseSecondaryEntityMap,
        stable_graph::StableUnGraph,
    },
    dynamics::solver::constraint_graph::ContactManifoldHandle,
};
use bevy::{platform::collections::HashSet, prelude::*};

use super::{ContactEdge, ContactId};

/// A resource that stores all [`ContactEdge`]s in the physics world in an [undirected graph](StableUnGraph),
/// and their corresponding [`ContactPair`]s.
///
/// Contact pairs exist between [colliders](Collider) that have intersecting [AABBs](ColliderAabb),
/// even if the shapes themselves are not yet touching.
///
/// For a simpler API that abstracts over this complexity, consider using the [`Collisions`]
/// system parameter.
///
/// # Usage
///
/// The following methods can be used for querying collisions:
///
/// - [`get`](Self::get) and [`get_mut`](Self::get_mut)
/// - [`iter_active`](Self::iter_active) and [`iter_active_mut`](Self::iter_active_mut)
/// - [`iter_sleeping`](Self::iter_sleeping) and [`iter_sleeping_mut`](Self::iter_sleeping_mut)
/// - [`contains`](Self::contains)
/// - [`collisions_with`](Self::collisions_with)
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
/// For advanced usage, there are also methods such as [`get_edge`](Self::get_edge) and [`get_edge_mut`](Self::get_edge_mut)
/// methods to access the [`ContactEdge`]s directly, along with variants that take a [`ContactId`] to access edges by their ID.
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

    /// Contact pairs for awake dynamic and kinematic bodies.
    active_pairs: Vec<ContactPair>,

    /// Contact pairs for sleeping dynamic bodies.
    sleeping_pairs: Vec<ContactPair>,

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
    /// Returns the contact edge between two entities.
    /// If the edge does not exist, `None` is returned.
    ///
    /// A contact edge exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_edge(&self, entity1: Entity, entity2: Entity) -> Option<&ContactEdge> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_to_node.get(entity1),
            self.entity_to_node.get(entity2),
        ) else {
            return None;
        };

        self.edges
            .find_edge(index1, index2)
            .and_then(|edge| self.get_edge_by_id(edge.into()))
    }

    /// Returns the contact edge between two entities based on their IDs.
    /// If the edge does not exist, `None` is returned.
    ///
    /// A contact edge exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_edge_by_id(&self, id: ContactId) -> Option<&ContactEdge> {
        self.edges.edge_weight(id.into())
    }

    /// Returns a mutable reference to the contact edge between two entities.
    /// If the edge does not exist, `None` is returned.
    ///
    /// A contact edge exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_edge_mut(&mut self, entity1: Entity, entity2: Entity) -> Option<&mut ContactEdge> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_to_node.get(entity1),
            self.entity_to_node.get(entity2),
        ) else {
            return None;
        };

        self.edges
            .find_edge(index1, index2)
            .and_then(|edge| self.get_edge_mut_by_id(edge.into()))
    }

    /// Returns a mutable reference to the contact edge between two entities based on their IDs.
    /// If the edge does not exist, `None` is returned.
    ///
    /// A contact edge exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactEdge::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_edge_mut_by_id(&mut self, id: ContactId) -> Option<&mut ContactEdge> {
        self.edges.edge_weight_mut(id.into())
    }

    /// Returns the contact edge and contact pair between two entities.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get(&self, entity1: Entity, entity2: Entity) -> Option<(&ContactEdge, &ContactPair)> {
        self.get_edge(entity1, entity2)
            .and_then(|edge| self.get_pair_by_edge(edge).map(|pair| (edge, pair)))
    }

    /// Returns the contact edge and contact pair between two entities based on the contact ID.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_by_id(&self, id: ContactId) -> Option<(&ContactEdge, &ContactPair)> {
        self.get_edge_by_id(id)
            .and_then(|edge| self.get_pair_by_edge(edge).map(|pair| (edge, pair)))
    }

    /// Returns the contact pair between two entities based on the [`ContactEdge`].
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_pair_by_edge(&self, edge: &ContactEdge) -> Option<&ContactPair> {
        if edge.is_sleeping() {
            self.sleeping_pairs.get(edge.pair_index)
        } else {
            self.active_pairs.get(edge.pair_index)
        }
    }

    /// Returns a mutable reference to the contact edge and contact pair between two entities.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_mut(
        &mut self,
        entity1: Entity,
        entity2: Entity,
    ) -> Option<(&mut ContactEdge, &mut ContactPair)> {
        let (Some(&index1), Some(&index2)) = (
            self.entity_to_node.get(entity1),
            self.entity_to_node.get(entity2),
        ) else {
            return None;
        };

        self.edges
            .find_edge(index1, index2)
            .and_then(|edge| self.get_mut_by_id(edge.into()))
    }

    /// Returns a mutable reference to the contact edge and contact pair between two entities based on the contact ID.
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_mut_by_id(&mut self, id: ContactId) -> Option<(&mut ContactEdge, &mut ContactPair)> {
        self.edges.edge_weight_mut(id.into()).and_then(|edge| {
            if edge.is_sleeping() {
                self.sleeping_pairs
                    .get_mut(edge.pair_index)
                    .map(|pair| (edge, pair))
            } else {
                self.active_pairs
                    .get_mut(edge.pair_index)
                    .map(|pair| (edge, pair))
            }
        })
    }

    /// Returns a mutable reference to the contact pair between two entities based on the [`ContactEdge`].
    /// If the pair does not exist, `None` is returned.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect.
    /// Use [`ContactPair::is_touching`] to determine if the actual collider shapes are touching.
    #[inline]
    pub fn get_pair_mut_by_edge(&mut self, edge: &ContactEdge) -> Option<&mut ContactPair> {
        if edge.is_sleeping() {
            self.sleeping_pairs.get_mut(edge.pair_index)
        } else {
            self.active_pairs.get_mut(edge.pair_index)
        }
    }

    /// Returns a [`ContactManifold`] of a contact pair based on the [`ContactManifoldHandle`].
    /// If the manifold does not exist, `None` is returned.
    #[inline]
    pub fn get_manifold(&self, handle: ContactManifoldHandle) -> Option<&ContactManifold> {
        let contact_pair = self.get_by_id(handle.contact_id)?.1;
        contact_pair.manifolds.get(handle.manifold_index)
    }

    /// Returns a mutable reference to a [`ContactManifold`] of a contact pair based on the [`ContactManifoldHandle`].
    /// If the manifold does not exist, `None` is returned.
    #[inline]
    pub fn get_manifold_mut(
        &mut self,
        handle: ContactManifoldHandle,
    ) -> Option<&mut ContactManifold> {
        let contact_pair = self.get_mut_by_id(handle.contact_id)?.1;
        contact_pair.manifolds.get_mut(handle.manifold_index)
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

    /// Returns a slice over all active (non-sleeping) contact pairs.
    #[inline]
    pub fn active_pairs(&self) -> &[ContactPair] {
        &self.active_pairs
    }

    /// Returns a slice over all sleeping contact pairs.
    #[inline]
    pub fn sleeping_pairs(&self) -> &[ContactPair] {
        &self.sleeping_pairs
    }

    /// Returns a mutable slice over all active (non-sleeping) contact pairs.
    #[inline]
    pub fn active_pairs_mut(&mut self) -> &mut [ContactPair] {
        &mut self.active_pairs
    }

    /// Returns a mutable slice over all sleeping contact pairs.
    #[inline]
    pub fn sleeping_pairs_mut(&mut self) -> &mut [ContactPair] {
        &mut self.sleeping_pairs
    }

    /// Returns an iterator yielding immutable access to all active (non-sleeping) contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_active_touching`](Self::iter_active_touching) instead.
    #[inline]
    pub fn iter_active(&self) -> impl Iterator<Item = &ContactPair> {
        self.active_pairs.iter()
    }

    /// Returns an iterator yielding immutable access to all active (non-sleeping) contact pairs
    /// that are currently touching.
    ///
    /// This is a subset of [`iter_active`](Self::iter_active) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_active_touching(&self) -> impl Iterator<Item = &ContactPair> {
        self.iter_active()
            .filter(|contacts| contacts.flags.contains(ContactPairFlags::TOUCHING))
    }

    /// Returns a iterator yielding mutable access to all contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_active_touching_mut`](Self::iter_active_touching_mut) instead.
    #[inline]
    pub fn iter_active_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.active_pairs.iter_mut()
    }

    /// Returns an iterator yielding mutable access to all active (non-sleeping) contact pairs
    /// that are currently touching.
    ///
    /// This is a subset of [`iter_active_mut`](Self::iter_active_mut) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_active_touching_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.iter_active_mut()
            .filter(|contacts| contacts.flags.contains(ContactPairFlags::TOUCHING))
    }

    /// Returns an iterator yielding immutable access to all sleeping contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_sleeping_touching`](Self::iter_sleeping_touching) instead.
    #[inline]
    pub fn iter_sleeping(&self) -> impl Iterator<Item = &ContactPair> {
        self.sleeping_pairs.iter()
    }

    /// Returns an iterator yielding immutable access to all sleeping contact pairs
    /// that are currently touching.
    ///
    /// This is a subset of [`iter_sleeping`](Self::iter_sleeping) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_sleeping_touching(&self) -> impl Iterator<Item = &ContactPair> {
        self.iter_sleeping()
            .filter(|contacts| contacts.flags.contains(ContactPairFlags::TOUCHING))
    }

    /// Returns an iterator yielding mutable access to all sleeping contact pairs.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// If you only want touching contacts, use [`iter_sleeping_touching_mut`](Self::iter_sleeping_touching_mut) instead.
    #[inline]
    pub fn iter_sleeping_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.sleeping_pairs.iter_mut()
    }

    /// Returns an iterator yielding mutable access to all sleeping contact pairs
    /// that are currently touching.
    ///
    /// This is a subset of [`iter_sleeping_mut`](Self::iter_sleeping_mut) that only includes pairs where the colliders are touching.
    #[inline]
    pub fn iter_sleeping_touching_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.iter_sleeping_mut()
            .filter(|contacts| contacts.flags.contains(ContactPairFlags::TOUCHING))
    }

    /// Returns an iterator yielding immutable access to all contact pairs involving the given entity.
    ///
    /// A contact pair exists between two entities if their [`ColliderAabb`]s intersect,
    /// even if the shapes themselves are not yet touching.
    ///
    /// Use [`ContactEdge::is_touching`](ContactEdge::is_touching) to determine if the actual collider shapes are touching.
    #[inline]
    pub fn collisions_with(&self, entity: Entity) -> impl Iterator<Item = &ContactPair> {
        self.entity_to_node
            .get(entity)
            .into_iter()
            .flat_map(move |&index| {
                self.edges.edge_weights(index).filter_map(|edge| {
                    if edge.is_sleeping() {
                        self.sleeping_pairs.get(edge.pair_index)
                    } else {
                        self.active_pairs.get(edge.pair_index)
                    }
                })
            })
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

        let collider1 = contacts.collider1;
        let collider2 = contacts.collider2;

        // Get the indices of the entities in the graph.
        let index1 = self
            .entity_to_node
            .get_or_insert_with(collider1, || self.edges.add_node(collider1));
        let index2 = self
            .entity_to_node
            .get_or_insert_with(collider2, || self.edges.add_node(collider2));

        // Add the edge to the graph.
        let edge_id = ContactId(self.edges.add_edge(index1, index2, contacts).0);

        // Create the contact pair. Contacts are created as active.
        let pair = ContactPair::new(collider1, collider2, edge_id);
        let pair_index = self.active_pairs.len();
        self.active_pairs.push(pair);

        // Set the contact ID and pair index of the edge.
        let edge = self.edges.edge_weight_mut(edge_id.into()).unwrap();
        edge.id = edge_id;
        edge.pair_index = pair_index;

        Some(edge_id)
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
        if let Some(edge) = self.edges.remove_edge(id.into()) {
            // Remove the contact pair from the active or sleeping pairs.
            let pair_index = edge.pair_index;
            if edge.is_sleeping() {
                let moved_index = self.sleeping_pairs.len() - 1;
                self.sleeping_pairs.swap_remove(pair_index);

                if moved_index != pair_index {
                    // Fix moved pair index.
                    let moved_contact_id = self.sleeping_pairs[pair_index].contact_id;
                    let moved_edge = self
                        .get_edge_mut_by_id(moved_contact_id)
                        .expect("moved edge should exist");
                    debug_assert!(moved_edge.pair_index == moved_index);
                    moved_edge.pair_index = pair_index;
                }

                Some(edge)
            } else {
                let moved_index = self.active_pairs.len() - 1;
                self.active_pairs.swap_remove(pair_index);

                if moved_index != pair_index {
                    // Fix moved pair index.
                    let moved_contact_id = self.active_pairs[pair_index].contact_id;
                    let moved_edge = self
                        .get_edge_mut_by_id(moved_contact_id)
                        .expect("moved edge should exist");
                    moved_edge.pair_index = pair_index;
                }

                Some(edge)
            }
        } else {
            None
        }
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

    /// Transfers contact of a body from the sleeping contacts to the active contacts,
    /// calling the given callback for each contact pair that is moved to active contacts.
    #[inline]
    pub fn wake_entity_with<F>(&mut self, entity: Entity, mut pair_callback: F)
    where
        F: FnMut(&mut ContactGraph, &ContactPair),
    {
        // Get the index of the entity in the graph.
        let Some(index) = self.entity_to_node.get(entity) else {
            return;
        };

        // Find all edges connected to the entity.
        let edges: Vec<ContactEdge> = self.edges.edge_weights(*index).cloned().collect();

        // Iterate over the edges and move sleeping contacts to active contacts.
        for edge in edges {
            let edge = self
                .edges
                .edge_weight_mut(edge.id.into())
                .expect("edge should exist");
            let pair_index = edge.pair_index;

            // Remove the sleeping contact pair.
            let moved_index = self.sleeping_pairs.len() - 1;
            let transferred_pair = self.sleeping_pairs.swap_remove(pair_index);

            // Update the pair index of the edge.
            edge.pair_index = self.active_pairs.len();

            // Call the pair callback.
            pair_callback(self, &transferred_pair);

            // Add the transferred pair to the active pairs.
            self.active_pairs.push(transferred_pair);

            // Fix moved edge.
            if moved_index != pair_index {
                let moved_contact_id = self.sleeping_pairs[pair_index].contact_id;
                let moved_edge = self
                    .get_edge_mut_by_id(moved_contact_id)
                    .expect("moved edge should exist");
                debug_assert!(moved_edge.pair_index == moved_index);
                moved_edge.pair_index = pair_index;
            }
        }
    }

    /// Transfers contact of a body from the active contacts to the sleeping contacts,
    /// calling the given callback for each contact pair that is moved to sleeping contacts.
    #[inline]
    pub fn sleep_entity_with<F>(&mut self, entity: Entity, mut pair_callback: F)
    where
        F: FnMut(&mut ContactGraph, &ContactPair),
    {
        // Get the index of the entity in the graph.
        let Some(index) = self.entity_to_node.get(entity) else {
            return;
        };

        // Find all edges connected to the entity.
        let edges: Vec<ContactEdge> = self.edges.edge_weights(*index).cloned().collect();

        // Iterate over the edges and move active contacts to sleeping contacts.
        for edge in edges {
            let edge = self
                .edges
                .edge_weight_mut(edge.id.into())
                .expect("edge should exist");
            let pair_index = edge.pair_index;

            // Remove the active contact pair.
            let moved_index = self.active_pairs.len() - 1;
            let transferred_pair = self.active_pairs.swap_remove(pair_index);

            // Update the pair index of the edge.
            edge.pair_index = self.sleeping_pairs.len();

            // Call the edge callback.
            pair_callback(self, &transferred_pair);

            // Add the transferred pair to the sleeping pairs.
            self.sleeping_pairs.push(transferred_pair);

            // Fix moved edge.
            if moved_index != pair_index {
                let moved_contact_id = self.active_pairs[pair_index].contact_id;
                let moved_edge = self
                    .get_edge_mut_by_id(moved_contact_id)
                    .expect("moved edge should exist");
                debug_assert!(moved_edge.pair_index == moved_index);
                moved_edge.pair_index = pair_index;
            }
        }
    }

    /// Clears all contact pairs and the contact graph.
    ///
    /// This can be useful to ensure that the world is in a clean state
    /// when for example reloading a scene or resetting the physics world.
    ///
    /// # Warning
    ///
    /// This does *not* clear the [`ConstraintGraph`]! You should additionally
    /// call [`ConstraintGraph::clear`], or alternatively use [`Collisions::clear`]
    /// that clears both resources.
    ///
    /// [`ConstraintGraph`]: crate::dynamics::solver::constraint_graph::ConstraintGraph
    /// [`ConstraintGraph::clear`]: crate::dynamics::solver::constraint_graph::ConstraintGraph::clear
    /// [`Collisions::clear`]: crate::collision::contact_types::Collisions::clear
    #[inline]
    pub fn clear(&mut self) {
        self.edges.clear();
        self.active_pairs.clear();
        self.sleeping_pairs.clear();
        self.pair_set.clear();
        self.entity_to_node.clear();
    }
}
