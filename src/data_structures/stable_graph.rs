//! A stripped down version of petgraph's `StableUnGraph`.
//!
//! - Index types always use `u32`.
//! - Edge iteration order after serialization/deserialization is preserved.
//! - Fewer iterators and helpers, and a few new ones.

use super::graph::{index_twice, Edge, EdgeDirection, EdgeIndex, Node, NodeIndex, Pair, UnGraph};

/// A graph with undirected edges and stable indices.
///
/// Unlike [`UnGraph`], this graph *does not* invalidate any unrelated
/// node or edge indices when items are removed.  This stable order
/// is achieved by using a free list for both nodes and edges.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct StableUnGraph<N, E> {
    graph: UnGraph<Option<N>, Option<E>>,
    node_count: usize,
    edge_count: usize,

    // Node and edge free lists both work the same way.
    //
    // `free_node`, if not `NodeIndex::END`, points to a node index
    // that is vacant (after a deletion).
    //
    // The free nodes form a doubly linked list using the field's `node.next[0]`
    // for forward references and `node.next[1]` for backwards ones.
    //
    // The nodes are stored as `EdgeIndex`, and the `_into_edge()`/`_into_node()`
    // methods convert.
    //
    // `free_edge`, if not `EdgeIndex::END`, points to a free edge.
    //
    // The edges only form a singly linked list using `edge.next[0]` to store
    // the forward reference.
    free_node: NodeIndex,
    free_edge: EdgeIndex,
}

impl<N, E> Default for StableUnGraph<N, E> {
    fn default() -> Self {
        StableUnGraph {
            graph: UnGraph::default(),
            node_count: 0,
            edge_count: 0,
            free_node: NodeIndex::END,
            free_edge: EdgeIndex::END,
        }
    }
}

impl<N, E> StableUnGraph<N, E> {
    /// Creates a new [`StableUnGraph`] with estimated capacity.
    pub fn with_capacity(nodes: usize, edges: usize) -> Self {
        StableUnGraph {
            graph: UnGraph::with_capacity(nodes, edges),
            node_count: 0,
            edge_count: 0,
            free_node: NodeIndex::END,
            free_edge: EdgeIndex::END,
        }
    }

    /// Returns the number of nodes in the graph.
    ///
    /// Computes in **O(1)** time.
    pub fn node_count(&self) -> usize {
        self.node_count
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
        if self.free_node != NodeIndex::END {
            // Reuse a free node.
            let node_idx = self.free_node;
            self.occupy_vacant_node(node_idx, weight);
            node_idx
        } else {
            // Create a new node.
            let node = self.graph.add_node(Some(weight));
            self.node_count += 1;
            node
        }
    }

    /// Creates a new node using a vacant position,
    /// updating the doubly linked list of free nodes.
    fn occupy_vacant_node(&mut self, node_idx: NodeIndex, weight: N) {
        let node_slot = &mut self.graph.nodes[node_idx.index()];

        #[cfg(debug_assertions)]
        {
            let old = core::mem::replace(&mut node_slot.weight, Some(weight));
            debug_assert!(old.is_none());
        }

        let previous_node = node_slot.next[1];
        let next_node = node_slot.next[0];
        node_slot.next = [EdgeIndex::END, EdgeIndex::END];

        if previous_node != EdgeIndex::END {
            self.graph.nodes[previous_node.index()].next[0] = next_node;
        }
        if next_node != EdgeIndex::END {
            self.graph.nodes[next_node.index()].next[1] = previous_node;
        }
        if self.free_node == node_idx {
            self.free_node = NodeIndex(next_node.0);
        }

        self.node_count += 1;
    }

    /// Returns the node at index `a` if it is not vacant.
    fn get_node(&self, a: NodeIndex) -> Option<&Node<Option<N>>> {
        self.graph
            .nodes
            .get(a.index())
            .and_then(|node| node.weight.as_ref().map(|_| node))
    }

    /// Accesses the weight for node `a`.
    ///
    /// Also available with indexing syntax: `&graph[a]`.
    pub fn node_weight(&self, a: NodeIndex) -> Option<&N> {
        let node = self.graph.nodes.get(a.index())?;
        node.weight.as_ref()
    }

    /// Adds an edge from `a` to `b` to the graph, with its associated
    /// data `weight`.
    ///
    /// Returns the index of the new edge.
    ///
    /// Computes in **O(1)** time.
    ///
    /// **Note:** `UnGraph` allows adding parallel (“duplicate”) edges. If you want
    /// to avoid this, use [`.update_edge(a, b, weight)`](#method.update_edge) instead.
    ///
    /// # Panics
    ///
    /// Panics if any of the nodes don't exist or the graph is at the maximum number of edges.
    pub fn add_edge(&mut self, a: NodeIndex, b: NodeIndex, weight: E) -> EdgeIndex {
        let edge_idx;
        let mut new_edge = None::<Edge<Option<E>>>;
        {
            let edge: &mut Edge<Option<E>>;

            if self.free_edge != EdgeIndex::END {
                edge_idx = self.free_edge;
                edge = &mut self.graph.edges[edge_idx.index()];

                let _old = core::mem::replace(&mut edge.weight, Some(weight));
                debug_assert!(_old.is_none());

                self.free_edge = edge.next[0];
                edge.node = [a, b];
            } else {
                edge_idx = EdgeIndex(self.graph.edges.len() as u32);
                assert!(edge_idx != EdgeIndex::END);
                new_edge = Some(Edge {
                    weight: Some(weight),
                    node: [a, b],
                    next: [EdgeIndex::END; 2],
                });
                edge = new_edge.as_mut().unwrap();
            }

            let wrong_index = match index_twice(&mut self.graph.nodes, a.index(), b.index()) {
                Pair::None => Some(core::cmp::max(a.index(), b.index())),
                Pair::One(an) => {
                    if an.weight.is_none() {
                        Some(a.index())
                    } else {
                        edge.next = an.next;
                        an.next[0] = edge_idx;
                        an.next[1] = edge_idx;
                        None
                    }
                }
                Pair::Both(an, bn) => {
                    // a and b are different indices
                    if an.weight.is_none() {
                        Some(a.index())
                    } else if bn.weight.is_none() {
                        Some(b.index())
                    } else {
                        edge.next = [an.next[0], bn.next[1]];
                        an.next[0] = edge_idx;
                        bn.next[1] = edge_idx;
                        None
                    }
                }
            };

            assert!(
                wrong_index.is_none(),
                "Tried adding an edge to a non-existent node."
            );

            self.edge_count += 1;
        }

        if let Some(edge) = new_edge {
            self.graph.edges.push(edge);
        }

        edge_idx
    }

    /// Adds or updates an edge from `a` to `b`.
    /// If the edge already exists, its weight is updated.
    ///
    /// Returns the index of the affected edge.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of edges
    /// connected to `a` (and `b`, if the graph edges are undirected).
    ///
    /// # Panics
    ///
    /// Panics if any of the nodes doesn't exist or the graph is at the maximum number of edges.
    pub fn update_edge(&mut self, a: NodeIndex, b: NodeIndex, weight: E) -> EdgeIndex {
        if let Some(ix) = self.find_edge(a, b) {
            *self.edge_weight_mut(ix).unwrap() = weight;
            return ix;
        }
        self.add_edge(a, b, weight)
    }

    /// Accesses the weight for edge `e`.
    ///
    /// Also available with indexing syntax: `&graph[e]`.
    pub fn edge_weight(&self, e: EdgeIndex) -> Option<&E> {
        let edge = self.graph.edges.get(e.index())?;
        edge.weight.as_ref()
    }

    /// Accesses the weight for edge `e` mutably.
    ///
    /// Also available with indexing syntax: `&mut graph[e]`.
    pub fn edge_weight_mut(&mut self, e: EdgeIndex) -> Option<&mut E> {
        let edge = self.graph.edges.get_mut(e.index())?;
        edge.weight.as_mut()
    }

    /// Accesses the source and target nodes for `e`.
    pub fn edge_endpoints(&self, e: EdgeIndex) -> Option<(NodeIndex, NodeIndex)> {
        let edge = self.graph.edges.get(e.index())?;
        Some((edge.source(), edge.target()))
    }

    /// Removes `a` from the graph if it exists, calling `edge_callback` for each of its edges,
    /// and returns its weight. If it doesn't exist in the graph, returns `None`.
    ///
    /// Invalidates the node index `a`, but not any other indices.
    /// Edge indices are invalidated as they would be following
    /// the removal of each edge with an endpoint in `a`.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of affected
    /// edges, including *n* calls to [`remove_edge`](Self::remove_edge),
    /// where *n* is the number of edges with an endpoint in `a`,
    /// and including the edges with an endpoint in the displaced node.
    pub fn remove_node_with<F>(&mut self, a: NodeIndex, mut edge_callback: F) -> Option<N>
    where
        F: FnMut(&mut Self, E),
    {
        let node_weight = self.graph.nodes.get_mut(a.index())?.weight.take()?;

        for d in EdgeDirection::ALL {
            let k = d as usize;

            // Remove all edges from and to this node.
            loop {
                let next = self.graph.nodes[a.index()].next[k];
                if next == EdgeIndex::END {
                    break;
                }
                let edge = self.remove_edge(next).expect("edge not found for removal");
                edge_callback(self, edge);
            }
        }

        let node_slot = &mut self.graph.nodes[a.index()];
        node_slot.next = [EdgeIndex(self.free_node.0), EdgeIndex::END];

        if self.free_node != NodeIndex::END {
            self.graph.nodes[self.free_node.index()].next[1] = EdgeIndex(a.0);
        }

        self.free_node = a;
        self.node_count -= 1;

        Some(node_weight)
    }

    /// Removes an edge and returns its edge weight, or `None` if it didn't exist.
    ///
    /// Invalidates the edge index `e`, but not any other indices.
    ///
    /// Computes in **O(e')** time, where **e'** is the size of four particular edge lists, for
    /// the vertices of `e` and the vertices of another affected edge.
    pub fn remove_edge(&mut self, e: EdgeIndex) -> Option<E> {
        // Every edge is part of two lists, outgoing and incoming edges.
        // Remove it from both.
        let (is_edge, edge_node, edge_next) = match self.graph.edges.get(e.index()) {
            None => return None,
            Some(x) => (x.weight.is_some(), x.node, x.next),
        };

        if !is_edge {
            return None;
        }

        // Remove the edge from its in and out lists by replacing it with
        // a link to the next in the list.
        self.graph.change_edge_links(edge_node, e, edge_next);

        // Clear the edge and put it in the free list.
        let edge_slot = &mut self.graph.edges[e.index()];
        edge_slot.next = [self.free_edge, EdgeIndex::END];
        edge_slot.node = [NodeIndex::END; 2];
        self.free_edge = e;
        self.edge_count -= 1;
        edge_slot.weight.take()
    }

    /// Returns an iterator of all nodes with an edge connected to `a`.
    ///
    /// Produces an empty iterator if the node doesn't exist.
    ///
    /// The iterator element type is `NodeIndex`.
    pub fn neighbors(&self, a: NodeIndex) -> Neighbors<E> {
        Neighbors {
            skip_start: a,
            edges: &self.graph.edges,
            next: match self.get_node(a) {
                None => [EdgeIndex::END, EdgeIndex::END],
                Some(n) => n.next,
            },
        }
    }

    /// Returns an iterator of all edges connected to `a`.
    ///
    /// Produces an empty iterator if the node doesn't exist.
    ///
    /// The iterator element type is `EdgeReference<E>`.
    pub fn edges(&self, a: NodeIndex) -> Edges<E> {
        Edges {
            skip_start: a,
            edges: &self.graph.edges,
            direction: EdgeDirection::Outgoing,
            next: match self.get_node(a) {
                None => [EdgeIndex::END, EdgeIndex::END],
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
    pub fn find_edge(&self, a: NodeIndex, b: NodeIndex) -> Option<EdgeIndex> {
        self.graph.find_edge_from_node(self.get_node(a)?, b)
    }

    /// Returns an iterator yielding immutable access to edge weights for edges from or to `a`.
    pub fn edge_weights(&self, a: NodeIndex) -> EdgeWeights<E> {
        EdgeWeights {
            skip_start: a,
            edges: &self.graph.edges,
            direction: EdgeDirection::Outgoing,
            next: match self.get_node(a) {
                None => [EdgeIndex::END, EdgeIndex::END],
                Some(n) => n.next,
            },
        }
    }

    /// Returns an iterator yielding immutable access to all edge weights.
    ///
    /// The order in which weights are yielded matches the order of their
    /// edge indices.
    pub fn all_edge_weights(&self) -> impl Iterator<Item = &E> {
        self.graph
            .edges
            .iter()
            .filter_map(|edge| edge.weight.as_ref())
    }

    /// Returns an iterator yielding mutable access to all edge weights.
    ///
    /// The order in which weights are yielded matches the order of their
    /// edge indices.
    pub fn all_edge_weights_mut(&mut self) -> impl Iterator<Item = &mut E> {
        self.graph
            .edges
            .iter_mut()
            .filter_map(|edge| edge.weight.as_mut())
    }

    /// Accesses the internal edge array.
    ///
    /// Note that this also includes vacant edges.
    pub fn raw_edges(&self) -> &[Edge<Option<E>>] {
        &self.graph.edges
    }

    /// Accesses the internal edge array mutably.
    ///
    /// Note that this also includes vacant edges.
    pub fn raw_edges_mut(&mut self) -> &mut [Edge<Option<E>>] {
        &mut self.graph.edges
    }

    /// Removes all nodes and edges.
    pub fn clear(&mut self) {
        self.node_count = 0;
        self.edge_count = 0;
        self.free_node = NodeIndex::END;
        self.free_edge = EdgeIndex::END;
        self.graph.clear();
    }

    /// Removes all edges.
    pub fn clear_edges(&mut self) {
        self.edge_count = 0;
        self.free_edge = EdgeIndex::END;
        self.graph.edges.clear();

        // Clear edges without touching the free list.
        for node in &mut self.graph.nodes {
            if node.weight.is_some() {
                node.next = [EdgeIndex::END, EdgeIndex::END];
            }
        }
    }

    /// Returns the current node capacity of the graph.
    pub fn nodes_capacity(&self) -> usize {
        self.graph.nodes_capacity()
    }

    /// Returns the current edge capacity of the graph.
    pub fn edges_capacity(&self) -> usize {
        self.graph.edges_capacity()
    }
}

/// An iterator over the neighbors of a node.
///
/// The iterator element type is `NodeIndex`.
#[derive(Debug)]
pub struct Neighbors<'a, E: 'a> {
    /// The starting node to skip over.
    skip_start: NodeIndex,

    /// The edges to iterate over.
    edges: &'a [Edge<Option<E>>],

    /// The next edge to visit.
    next: [EdgeIndex; 2],
}

impl<E> Iterator for Neighbors<'_, E> {
    type Item = NodeIndex;

    fn next(&mut self) -> Option<NodeIndex> {
        // First any outgoing edges.
        match self.edges.get(self.next[0].index()) {
            None => {}
            Some(edge) => {
                debug_assert!(edge.weight.is_some());
                self.next[0] = edge.next[0];
                return Some(edge.node[1]);
            }
        }

        // Then incoming edges.
        // For an "undirected" iterator, make sure we don't double
        // count self-loops by skipping them in the incoming list.
        while let Some(edge) = self.edges.get(self.next[1].index()) {
            debug_assert!(edge.weight.is_some());
            self.next[1] = edge.next[1];
            if edge.node[0] != self.skip_start {
                return Some(edge.node[0]);
            }
        }
        None
    }
}

impl<E> Clone for Neighbors<'_, E> {
    fn clone(&self) -> Self {
        Neighbors {
            skip_start: self.skip_start,
            edges: self.edges,
            next: self.next,
        }
    }
}

/// An iterator over edges from or to a node.
pub struct Edges<'a, E: 'a> {
    /// The starting node to skip over.
    skip_start: NodeIndex,

    /// The edges to iterate over.
    edges: &'a [Edge<Option<E>>],

    /// The next edge to visit.
    next: [EdgeIndex; 2],

    /// The direction of edges.
    direction: EdgeDirection,
}

impl<'a, E> Iterator for Edges<'a, E> {
    type Item = EdgeReference<'a, E>;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.next[0].index();
        if let Some(Edge {
            weight: Some(weight),
            next,
            ..
        }) = self.edges.get(i)
        {
            self.next[0] = next[0];
            return Some(EdgeReference {
                index: EdgeIndex(i as u32),
                weight,
            });
        }

        None
    }
}

impl<E> Clone for Edges<'_, E> {
    fn clone(&self) -> Self {
        Edges {
            skip_start: self.skip_start,
            edges: self.edges,
            next: self.next,
            direction: self.direction,
        }
    }
}

// TODO: Reduce duplication between `Edges` variants.
/// An iterator over edge weights for edges from or to a node.
pub struct EdgeWeights<'a, E: 'a> {
    /// The starting node to skip over.
    skip_start: NodeIndex,

    /// The edges to iterate over.
    edges: &'a [Edge<Option<E>>],

    /// The next edge to visit.
    next: [EdgeIndex; 2],

    /// The direction of edges.
    direction: EdgeDirection,
}

impl<'a, E> Iterator for EdgeWeights<'a, E> {
    type Item = &'a E;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.next[0].index();
        if let Some(Edge {
            weight: Some(weight),
            next,
            ..
        }) = self.edges.get(i)
        {
            self.next[0] = next[0];
            return Some(weight);
        }

        while let Some(Edge {
            node,
            weight: Some(weight),
            next,
        }) = self.edges.get(self.next[1].index())
        {
            self.next[1] = next[1];

            // In any of the "both" situations, self-loops would be iterated over twice.
            // Skip them here.
            if node[0] == self.skip_start {
                continue;
            }

            return Some(weight);
        }

        None
    }
}

impl<E> Clone for EdgeWeights<'_, E> {
    fn clone(&self) -> Self {
        EdgeWeights {
            skip_start: self.skip_start,
            edges: self.edges,
            next: self.next,
            direction: self.direction,
        }
    }
}

struct EdgesWalkerMut<'a, E: 'a> {
    edges: &'a mut [Edge<E>],
    next: EdgeIndex,
    dir: EdgeDirection,
}

impl<E> EdgesWalkerMut<'_, E> {
    fn next_edge(&mut self) -> Option<&mut Edge<E>> {
        self.next().map(|t| t.1)
    }

    fn next(&mut self) -> Option<(EdgeIndex, &mut Edge<E>)> {
        let this_index = self.next;
        let k = self.dir as usize;
        match self.edges.get_mut(self.next.index()) {
            None => None,
            Some(edge) => {
                self.next = edge.next[k];
                Some((this_index, edge))
            }
        }
    }
}

/// A reference to a graph edge.
#[derive(Debug)]
pub struct EdgeReference<'a, E: 'a> {
    index: EdgeIndex,
    weight: &'a E,
}

impl<'a, E: 'a> EdgeReference<'a, E> {
    /// Returns the index of the edge.
    #[inline]
    pub fn index(&self) -> EdgeIndex {
        self.index
    }

    /// Returns the weight of the edge.
    #[inline]
    pub fn weight(&self) -> &'a E {
        self.weight
    }
}

impl<E> Clone for EdgeReference<'_, E> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<E> Copy for EdgeReference<'_, E> {}

impl<E> PartialEq for EdgeReference<'_, E>
where
    E: PartialEq,
{
    fn eq(&self, rhs: &Self) -> bool {
        self.index == rhs.index && self.weight == rhs.weight
    }
}

/// A mutable reference to a graph edge.
#[derive(Debug)]
pub struct EdgeMut<'a, E: 'a> {
    index: EdgeIndex,
    weight: &'a mut E,
}

impl<E> EdgeMut<'_, E> {
    /// Returns the index of the edge.
    #[inline]
    pub fn index(&self) -> EdgeIndex {
        self.index
    }

    /// Returns the weight of the edge.
    #[inline]
    pub fn weight(&self) -> &E {
        self.weight
    }

    /// Returns the weight of the edge mutably.
    #[inline]
    pub fn weight_mut(&mut self) -> &mut E {
        self.weight
    }
}

impl<E> PartialEq for EdgeMut<'_, E>
where
    E: PartialEq,
{
    fn eq(&self, rhs: &Self) -> bool {
        self.index == rhs.index && self.weight == rhs.weight
    }
}
