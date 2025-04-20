//! A version of petgraph's `UnGraph`, tailored for Avian's needs.
//!
//! - Index types always use `u32`.
//! - Edge connectivity information and edge weights are stored separately.
//!   Indices for connectivity data are stable when adding or removing edges,
//!   but indices for edge weights are not. This allows persistent handles
//!   to edges, while keeping fast iteration over edge weights, unlike with
//!   petgraph's `StableGraph`, which would also need to iterate over vacant edges.
//! - Edge iteration order after serialization/deserialization is preserved.
//! - Fewer iterators and helpers, and a few new ones.

use core::cmp::max;

use bevy::reflect::Reflect;
use derive_more::derive::From;

/// The index of a node in a graph structure.
///
/// This is not a stable identifier, and can be invalidated when removing nodes.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd, Eq, Ord, Hash, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct NodeIndex(pub u32);

impl NodeIndex {
    /// A special index used to denote the final node,
    /// for example at the end of an adjacency list.
    ///
    /// Equivalent to `NodeIndex(u32::MAX)`.
    pub const END: NodeIndex = NodeIndex(u32::MAX);

    /// Returns the inner `u32` value as a `usize`.
    pub fn index(self) -> usize {
        self.0 as usize
    }

    /// Returns `true` if this is the special `END` index.
    pub fn is_end(self) -> bool {
        self == NodeIndex::END
    }
}

/// The stable identifier of an edge in a graph structure.
///
/// Note that while edge IDs are stable, indices for edge weights are *not* stable,
/// and can be invalidated when removing edges.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd, Eq, Ord, Hash, Reflect, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct EdgeId(pub u32);

impl EdgeId {
    /// A placeholder edge identifier used to denote a free edge.
    ///
    /// Equivalent to `EdgeId(u32::MAX)`.
    pub const PLACEHOLDER: EdgeId = EdgeId(u32::MAX);

    /// Returns the stable identifier of the edge as a `usize`.
    pub fn id(self) -> usize {
        self.0 as usize
    }

    /// Returns `true` if this is a placeholder edge.
    pub fn is_placeholder(self) -> bool {
        self == EdgeId::PLACEHOLDER
    }
}

/// The direction of a graph edge.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Ord, Eq, Hash)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[repr(usize)]
pub enum EdgeDirection {
    /// An `Outgoing` edge is an outward edge *from* the current node.
    Outgoing = 0,
    /// An `Incoming` edge is an inbound edge *to* the current node.
    Incoming = 1,
}

impl EdgeDirection {
    /// The two possible directions for an edge.
    pub const ALL: [EdgeDirection; 2] = [EdgeDirection::Outgoing, EdgeDirection::Incoming];
}

/// The node type for a graph structure.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Node<N> {
    /// Associated node data.
    pub weight: N,
    /// Next edge in outgoing and incoming edge lists.
    next: [EdgeId; 2],
}

/// The edge type for a graph structure.
///
/// Vacant if `weight_index` is `None`.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Edge {
    /// The index of the edge weight associated with this edge.
    ///
    /// This can change when edges are removed.
    pub weight_index: Option<u32>,
    /// Next edge in outgoing and incoming edge lists.
    next: [EdgeId; 2],
    /// Start and End node index
    node: [NodeIndex; 2],
}

impl Edge {
    /// Return the source node index.
    pub fn source(&self) -> NodeIndex {
        self.node[0]
    }

    /// Return the target node index.
    pub fn target(&self) -> NodeIndex {
        self.node[1]
    }
}

/// A trait for types that can be used as edge weights
/// for a graph structure.
pub trait EdgeWeight {
    /// Returns the stable identifier for the edge weight.
    fn edge_id(&self) -> EdgeId;

    /// Sets the stable identifier for the edge weight.
    fn set_edge_id(&mut self, id: EdgeId);
}

/// A graph with undirected edges.
///
/// For example, an edge between *1* and *2* is equivalent to an edge between
/// *2* and *1*.
///
/// # Stability
///
/// Indices for edge connectivity data are stable when removing edges,
/// but indices for nodes or edge weights are not. This allows persistent
/// identifiers for edges, while keeping fast iteration over edge weights.
///
/// This is unlike petgraph's `StableGraph`, which keeps a stable order for
/// the whole graph, but requires you to iterate over vacant edges and edge weights.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct UnGraph<N, E: EdgeWeight> {
    /// The nodes of the graph.
    ///
    /// Node indices may be invalidated when removing nodes.
    nodes: Vec<Node<N>>,

    /// The edges of the graph. Vacant edges are represented as `None`.
    ///
    /// The order of edges is stable when removing edges.
    edges: Vec<Edge>,

    /// The number of occupied edges in the graph.
    edge_count: usize,

    /// Free list for edges.
    ///
    /// If not `EdgeId::PLACEHOLDER`, this points to a free edge.
    /// The edges only form a singly linked list using `edge.next[0]`
    /// to store the forward reference.
    free_edge: EdgeId,

    /// The weights associated with the edges.
    ///
    /// Edge weight indices may be invalidated when removing edges.
    edge_weights: Vec<E>,
}

impl<N, E: EdgeWeight> Default for UnGraph<N, E> {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
            edge_count: 0,
            free_edge: EdgeId::PLACEHOLDER,
            edge_weights: Vec::new(),
        }
    }
}

enum Pair<T> {
    Both(T, T),
    One(T),
    None,
}

/// Get mutable references at index `a` and `b`.
fn index_twice<T>(arr: &mut [T], a: usize, b: usize) -> Pair<&mut T> {
    if max(a, b) >= arr.len() {
        Pair::None
    } else if a == b {
        Pair::One(&mut arr[max(a, b)])
    } else {
        // safe because `a` and `b` are in bounds and distinct.
        unsafe {
            let ar = &mut *(arr.get_unchecked_mut(a) as *mut _);
            let br = &mut *(arr.get_unchecked_mut(b) as *mut _);
            Pair::Both(ar, br)
        }
    }
}

impl<N, E: EdgeWeight> UnGraph<N, E> {
    /// Creates a new [`UnGraph`] with an estimated capacity.
    pub fn with_capacity(nodes: usize, edges: usize) -> Self {
        UnGraph {
            nodes: Vec::with_capacity(nodes),
            edges: Vec::with_capacity(edges),
            edge_count: 0,
            free_edge: EdgeId::PLACEHOLDER,
            edge_weights: Vec::with_capacity(edges),
        }
    }

    /// Returns the number of nodes in the graph.
    ///
    /// Computes in **O(1)** time.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Returns the number of edges in the graph.
    ///
    /// Computes in **O(1)** time.
    pub fn edge_count(&self) -> usize {
        self.edge_count
    }

    /// Adds a node (also called vertex) with associated data `weight` to the graph.
    ///
    /// Computes in **O(1)** time.
    ///
    /// Returns the index of the new node.
    ///
    /// # Panics
    ///
    /// Panics if the graph is at the maximum number of nodes.
    pub fn add_node(&mut self, weight: N) -> NodeIndex {
        let node = Node {
            weight,
            next: [EdgeId::PLACEHOLDER, EdgeId::PLACEHOLDER],
        };
        assert!(self.nodes.len() != usize::MAX);
        let node_index = NodeIndex(self.nodes.len() as u32);
        self.nodes.push(node);
        node_index
    }

    /// Accesses the weight for node `a`.
    pub fn node_weight(&self, a: NodeIndex) -> Option<&N> {
        self.nodes.get(a.index()).map(|n| &n.weight)
    }

    /// Adds an edge from `a` to `b` to the graph, with its associated
    /// data `weight`.
    ///
    /// Returns the stable identifier of the new edge.
    ///
    /// Computes in **O(1)** time.
    ///
    /// **Note:** `UnGraph` allows adding parallel (“duplicate”) edges. If you want
    /// to avoid this, use [`.update_edge(a, b, weight)`](#method.update_edge) instead.
    ///
    /// # Panics
    ///
    /// Panics if any of the nodes don't exist or the graph is at the maximum number of edges.
    pub fn add_edge(&mut self, a: NodeIndex, b: NodeIndex, mut weight: E) -> EdgeId {
        let edge_id;
        let mut new_edge = None::<Edge>;
        {
            let edge: &mut Edge;
            let weight_index = self.edge_weights.len() as u32;

            if self.free_edge != EdgeId::PLACEHOLDER {
                // Reuse a free edge.
                edge_id = self.free_edge;
                edge = &mut self.edges[edge_id.id()];
                edge.weight_index = Some(weight_index);
                self.free_edge = edge.next[0];
                edge.node = [a, b];
            } else {
                edge_id = EdgeId(self.edges.len() as u32);
                assert!(edge_id != EdgeId::PLACEHOLDER);
                new_edge = Some(Edge {
                    weight_index: Some(weight_index),
                    node: [a, b],
                    next: [EdgeId::PLACEHOLDER; 2],
                });
                edge = new_edge.as_mut().unwrap();
            }

            match index_twice(&mut self.nodes, a.index(), b.index()) {
                Pair::None => panic!("`UnGraph::add_edge`: node indices out of bounds"),
                Pair::One(an) => {
                    edge.next = an.next;
                    an.next[0] = edge_id;
                    an.next[1] = edge_id;
                }
                Pair::Both(an, bn) => {
                    // `a` and `b` are different indices
                    edge.next = [an.next[0], bn.next[1]];
                    an.next[0] = edge_id;
                    bn.next[1] = edge_id;
                }
            }

            self.edge_count += 1;
        }

        if let Some(edge) = new_edge {
            self.edges.push(edge);
        }

        weight.set_edge_id(edge_id);
        self.edge_weights.push(weight);

        edge_id
    }

    /// Adds or updates an edge from `a` to `b`.
    /// If the edge already exists, its weight is updated.
    ///
    /// Returns the stable identifier of the affected edge.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of edges
    /// connected to `a` (and `b`, if the graph edges are undirected).
    ///
    /// # Panics
    ///
    /// Panics if any of the nodes doesn't exist.
    pub fn update_edge(&mut self, a: NodeIndex, b: NodeIndex, weight: E) -> EdgeId {
        if let Some(id) = self.find_edge(a, b) {
            *self.edge_weight_mut(id).unwrap() = weight;
            return id;
        }
        self.add_edge(a, b, weight)
    }

    /// Accesses the weight for edge `e`.
    pub fn edge_weight(&self, e: EdgeId) -> Option<&E> {
        let weight_index = self.edges.get(e.id())?.weight_index?;
        self.edge_weight_by_index(weight_index)
    }

    /// Accesses the weight for edge `e` by its index.
    pub fn edge_weight_by_index(&self, index: u32) -> Option<&E> {
        self.edge_weights.get(index as usize)
    }

    /// Accesses the weight for edge `e` mutably.
    pub fn edge_weight_mut(&mut self, e: EdgeId) -> Option<&mut E> {
        let weight_index = self.edges.get(e.id())?.weight_index?;
        self.edge_weight_by_index_mut(weight_index)
    }

    /// Accesses the weight for edge `e` by its index mutably.
    pub fn edge_weight_by_index_mut(&mut self, index: u32) -> Option<&mut E> {
        self.edge_weights.get_mut(index as usize)
    }

    /// Accesses the source and target nodes for `e`.
    pub fn edge_endpoints(&self, e: EdgeId) -> Option<(NodeIndex, NodeIndex)> {
        self.edges
            .get(e.id())
            .map(|edge| (edge.source(), edge.target()))
    }

    /// Removes `a` from the graph if it exists, calling `edge_callback` for each of its edges,
    /// and returns its weight. If it doesn't exist in the graph, returns `None`.
    ///
    /// Apart from `a`, this invalidates the last node index in the graph
    /// (that node will adopt the removed node index). Edge indices are
    /// invalidated as they would be following the removal of each edge
    /// with an endpoint in `a`.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of affected
    /// edges, including *n* calls to [`remove_edge`](Self::remove_edge),
    /// where *n* is the number of edges with an endpoint in `a`,
    /// and including the edges with an endpoint in the displaced node.
    ///
    /// # Panics
    ///
    /// Panics if `a` is out of bounds.
    pub fn remove_node_with<F>(&mut self, a: NodeIndex, mut edge_callback: F) -> Option<N>
    where
        F: FnMut(E),
    {
        assert!(a.index() < self.nodes.len(), "node index out of bounds");

        for dir in EdgeDirection::ALL {
            let k = dir as usize;

            // Remove all edges from and to this node.
            loop {
                let next = self.nodes[a.index()].next[k];
                if next == EdgeId::PLACEHOLDER {
                    break;
                }
                let edge = self.remove_edge(next).expect("edge not found for removal");
                edge_callback(edge);
            }
        }

        // Use `swap_remove` -- only the swapped-in node is going to change
        // `NodeIndex`, so we only have to walk its edges and update them.
        let node = self.nodes.swap_remove(a.index());

        // Find the edge lists of the node that had to relocate.
        // It may be that no node had to relocate, then we are done already.
        let swap_edges = match self.nodes.get(a.index()) {
            None => return Some(node.weight),
            Some(swapped_node) => swapped_node.next,
        };

        // The swapped element's old index
        let old_index = NodeIndex(self.nodes.len() as u32);
        let new_index = a;

        // Adjust the starts of the out edges, and ends of the in edges.
        for dir in EdgeDirection::ALL {
            let k = dir as usize;
            let mut edges = EdgesWalkerMut {
                edges: &mut self.edges,
                next: swap_edges[k],
                dir,
            };
            while let Some(current_edge) = edges.next_edge() {
                debug_assert!(current_edge.node[k] == old_index);
                current_edge.node[k] = new_index;
            }
        }

        Some(node.weight)
    }

    /// Removes an edge and returns its edge weight, or `None` if it didn't exist.
    ///
    /// Invalidates the edge `e` and its edge weight, and the index of the last edge weight
    /// (that edge weight will adopt the removed edge weight's index).
    ///
    /// Computes in **O(e')** time, where **e'** is the size of four particular edge lists, for
    /// the vertices of `e` and the vertices of another affected edge.
    pub fn remove_edge(&mut self, e: EdgeId) -> Option<E> {
        // Every edge is part of two lists, outgoing and incoming edges.
        // Remove it from both.
        let (is_edge, edge_node, edge_next) = match self.edges.get(e.id()) {
            None => return None,
            Some(x) => (x.weight_index.is_some(), x.node, x.next),
        };

        if !is_edge {
            return None;
        }

        // Remove the edge from its in and out lists by replacing it with
        // a link to the next in the list.
        self.change_edge_links(edge_node, e, edge_next);
        self.remove_edge_adjust_indices(e)
    }

    /// For edge `e` with endpoints `edge_node`, replaces links to it
    /// with links to `edge_next`.
    fn change_edge_links(&mut self, edge_node: [NodeIndex; 2], e: EdgeId, edge_next: [EdgeId; 2]) {
        for dir in EdgeDirection::ALL {
            let k = dir as usize;
            let node = match self.nodes.get_mut(edge_node[k].index()) {
                Some(r) => r,
                None => {
                    debug_assert!(
                        false,
                        "Edge's endpoint dir={:?} index={:?} not found",
                        dir, edge_node[k]
                    );
                    return;
                }
            };
            let first_id = node.next[k];
            if first_id == e {
                node.next[k] = edge_next[k];
            } else {
                let mut edges = EdgesWalkerMut {
                    edges: &mut self.edges,
                    next: first_id,
                    dir,
                };
                while let Some(current_edge) = edges.next_edge() {
                    if current_edge.next[k] == e {
                        current_edge.next[k] = edge_next[k];
                        // The edge can only be present once in the list.
                        break;
                    }
                }
            }
        }
    }

    fn remove_edge_adjust_indices(&mut self, e: EdgeId) -> Option<E> {
        // Clear the edge and put it in the free list.
        let edge_slot = &mut self.edges[e.id()];
        edge_slot.next = [self.free_edge, EdgeId::PLACEHOLDER];
        edge_slot.node = [NodeIndex::END; 2];
        self.free_edge = e;
        self.edge_count -= 1;
        let weight_index = edge_slot.weight_index.take().unwrap() as usize;

        // `swap_remove` the edge weight -- only the removed weight and the weight
        // swapped into place are affected, and their edges need updated weight indices.
        let edge_weight = self.edge_weights.swap_remove(weight_index);
        if let Some(Some(swapped_node)) = self
            .edge_weights
            .get(weight_index)
            .map(|weight| self.edges.get_mut(weight.edge_id().id()))
        {
            swapped_node.weight_index = Some(weight_index as u32);
        }

        Some(edge_weight)
    }

    /// Returns an iterator of all nodes with an edge connected to `a`.
    ///
    /// Produces an empty iterator if the node doesn't exist.
    ///
    /// The iterator element type is `NodeIndex`.
    pub fn neighbors(&self, a: NodeIndex) -> Neighbors {
        Neighbors {
            skip_start: a,
            edges: &self.edges,
            next: match self.nodes.get(a.index()) {
                None => [EdgeId::PLACEHOLDER, EdgeId::PLACEHOLDER],
                Some(n) => n.next,
            },
        }
    }

    /// Looks up if there is an edge from `a` to `b`.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of edges
    /// connected to `a` (and `b`, if the graph edges are undirected).
    pub fn contains_edge(&self, a: NodeIndex, b: NodeIndex) -> bool {
        self.find_edge(a, b).is_some()
    }

    /// Looks up an edge from `a` to `b`.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of edges
    /// connected to `a` (and `b`, if the graph edges are undirected).
    pub fn find_edge(&self, a: NodeIndex, b: NodeIndex) -> Option<EdgeId> {
        let node = self.nodes.get(a.index())?;
        for &dir in &EdgeDirection::ALL {
            let k = dir as usize;
            let mut edge_id = node.next[k];
            while let Some(edge) = self.edges.get(edge_id.id()) {
                if edge.node[1 - k] == b {
                    return Some(edge_id);
                }
                edge_id = edge.next[k];
            }
        }
        None
    }

    /// Returns an iterator yielding immutable access to edge weights for edges from or to `a`.
    pub fn edge_weights(&self, a: NodeIndex) -> EdgeWeights<E> {
        EdgeWeights {
            skip_start: a,
            edges: &self.edges,
            edge_weights: &self.edge_weights,
            direction: EdgeDirection::Outgoing,
            next: match self.nodes.get(a.index()) {
                None => [EdgeId::PLACEHOLDER, EdgeId::PLACEHOLDER],
                Some(n) => n.next,
            },
        }
    }

    /// Returns an iterator yielding mutable access to edge weights for edges from or to `a`.
    pub fn edge_weights_mut(&mut self, a: NodeIndex) -> EdgeWeightsMut<N, E> {
        let incoming_edge = self.first_edge(a, EdgeDirection::Incoming);
        let outgoing_edge = self.first_edge(a, EdgeDirection::Outgoing);

        EdgeWeightsMut {
            graph: self,
            incoming_edge,
            outgoing_edge,
        }
    }

    /// Accesses the internal node array.
    pub fn raw_nodes(&self) -> &[Node<N>] {
        &self.nodes
    }

    /// Accesses the internal node array mutably.
    pub fn raw_nodes_mut(&mut self) -> &mut [Node<N>] {
        &mut self.nodes
    }

    /// Accesses the internal edge array.
    ///
    /// Note that this also includes vacant edges.
    pub fn raw_edges(&self) -> &[Edge] {
        &self.edges
    }

    /// Accesses the internal edge array mutably.
    ///
    /// Note that this also includes vacant edges.
    pub fn raw_edges_mut(&mut self) -> &mut [Edge] {
        &mut self.edges
    }

    /// Accesses the internal edge weights array.
    pub fn raw_edge_weights(&self) -> &[E] {
        &self.edge_weights
    }

    /// Accesses the internal edge weights array mutably.
    pub fn raw_edge_weights_mut(&mut self) -> &mut [E] {
        &mut self.edge_weights
    }

    /// Accessor for data structure internals: returns the first edge in the given direction.
    pub fn first_edge(&self, a: NodeIndex, dir: EdgeDirection) -> Option<EdgeId> {
        match self.nodes.get(a.index()) {
            None => None,
            Some(node) => {
                let edge_id = node.next[dir as usize];
                if edge_id == EdgeId::PLACEHOLDER {
                    None
                } else {
                    Some(edge_id)
                }
            }
        }
    }

    /// Accessor for data structure internals: returns the next edge for the given direction.
    pub fn next_edge(&self, e: EdgeId, dir: EdgeDirection) -> Option<EdgeId> {
        match self.edges.get(e.id()) {
            None => None,
            Some(node) => {
                let edge_id = node.next[dir as usize];
                if edge_id == EdgeId::PLACEHOLDER {
                    None
                } else {
                    Some(edge_id)
                }
            }
        }
    }

    /// Removes all nodes and edges.
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.edges.clear();
        self.edge_weights.clear();
        self.edge_count = 0;
        self.free_edge = EdgeId::PLACEHOLDER;
    }

    /// Removes all edges.
    pub fn clear_edges(&mut self) {
        self.edges.clear();
        self.edge_weights.clear();
        self.edge_count = 0;
        self.free_edge = EdgeId::PLACEHOLDER;
        for node in &mut self.nodes {
            node.next = [EdgeId::PLACEHOLDER, EdgeId::PLACEHOLDER];
        }
    }
}

/// An iterator over the neighbors of a node.
///
/// The iterator element type is `NodeIndex`.
#[derive(Debug)]
pub struct Neighbors<'a> {
    /// The starting node to skip over.
    skip_start: NodeIndex,

    /// The edges to iterate over.
    edges: &'a [Edge],

    /// The next edge to visit.
    next: [EdgeId; 2],
}

impl Iterator for Neighbors<'_> {
    type Item = NodeIndex;

    fn next(&mut self) -> Option<NodeIndex> {
        // First any outgoing edges.
        match self.edges.get(self.next[0].id()) {
            None => {}
            Some(edge) => {
                debug_assert!(edge.weight_index.is_some());
                self.next[0] = edge.next[0];
                return Some(edge.node[1]);
            }
        }

        // Then incoming edges.
        // For an "undirected" iterator, make sure we don't double
        // count self-loops by skipping them in the incoming list.
        while let Some(edge) = self.edges.get(self.next[1].id()) {
            debug_assert!(edge.weight_index.is_some());
            self.next[1] = edge.next[1];
            if edge.node[0] != self.skip_start {
                return Some(edge.node[0]);
            }
        }
        None
    }
}

impl Clone for Neighbors<'_> {
    fn clone(&self) -> Self {
        Neighbors {
            skip_start: self.skip_start,
            edges: self.edges,
            next: self.next,
        }
    }
}

/// An iterator over edge weights for edges from or to a node.
pub struct EdgeWeights<'a, E: 'a> {
    /// The starting node to skip over.
    skip_start: NodeIndex,

    /// The edges of the graph.
    edges: &'a [Edge],

    /// The edge weights of the graph.
    edge_weights: &'a [E],

    /// The next edge to visit.
    next: [EdgeId; 2],

    /// The direction of edges.
    direction: EdgeDirection,
}

impl<'a, E> Iterator for EdgeWeights<'a, E> {
    type Item = &'a E;

    fn next(&mut self) -> Option<Self::Item> {
        // First any outgoing edges.
        let i = self.next[0].id();
        if let Some(Edge {
            weight_index: Some(weight_index),
            next,
            ..
        }) = self.edges.get(i)
        {
            self.next[0] = next[0];
            return Some(&self.edge_weights[*weight_index as usize]);
        }

        // Then incoming edges.
        while let Some(Edge {
            node,
            weight_index: Some(weight_index),
            next,
        }) = self.edges.get(self.next[1].id())
        {
            self.next[1] = next[1];

            // In any of the "both" situations, self-loops would be iterated over twice.
            // Skip them here.
            if node[0] == self.skip_start {
                continue;
            }

            return Some(&self.edge_weights[*weight_index as usize]);
        }

        None
    }
}

impl<E> Clone for EdgeWeights<'_, E> {
    fn clone(&self) -> Self {
        EdgeWeights {
            skip_start: self.skip_start,
            edges: self.edges,
            edge_weights: self.edge_weights,
            next: self.next,
            direction: self.direction,
        }
    }
}

/// An iterator over mutable references to all edge weights from or to a node.
pub struct EdgeWeightsMut<'a, N, E: EdgeWeight> {
    /// A mutable reference to the graph.
    pub graph: &'a mut UnGraph<N, E>,

    /// The next incoming edge to visit.
    pub incoming_edge: Option<EdgeId>,

    /// The next outgoing edge to visit.
    pub outgoing_edge: Option<EdgeId>,
}

impl<'a, N: Copy, E: EdgeWeight> Iterator for EdgeWeightsMut<'a, N, E> {
    type Item = &'a mut E;

    #[inline]
    fn next(&mut self) -> Option<&'a mut E> {
        if let Some(edge) = self.incoming_edge {
            self.incoming_edge = self.graph.next_edge(edge, EdgeDirection::Incoming);
            let weight = &mut self.graph.edge_weight_mut(edge).unwrap();
            return Some(unsafe { core::mem::transmute::<&mut E, &'a mut E>(weight) });
        }

        let edge = self.outgoing_edge?;
        self.outgoing_edge = self.graph.next_edge(edge, EdgeDirection::Outgoing);
        let weight = &mut self.graph.edge_weight_mut(edge).unwrap();
        Some(unsafe { core::mem::transmute::<&mut E, &'a mut E>(weight) })
    }
}

struct EdgesWalkerMut<'a> {
    edges: &'a mut [Edge],
    next: EdgeId,
    dir: EdgeDirection,
}

impl EdgesWalkerMut<'_> {
    fn next_edge(&mut self) -> Option<&mut Edge> {
        self.next().map(|(_id, edge)| edge)
    }

    fn next(&mut self) -> Option<(EdgeId, &mut Edge)> {
        let edge_id = self.next;
        let k = self.dir as usize;
        match self.edges.get_mut(self.next.id()) {
            None => None,
            Some(edge) => {
                self.next = edge.next[k];
                Some((edge_id, edge))
            }
        }
    }
}
