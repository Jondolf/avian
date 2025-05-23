use bevy::{
    ecs::{entity::Entity, resource::Resource},
    log::warn,
};
use parry::query::contact;

use crate::{
    collision::contact_types::{ContactEdge, ContactGraphInternal, ContactId},
    data_structures::{bit_vec::BitVec, graph::EdgeIndex},
    prelude::{ContactGraph, ContactPair, ContactPairFlags},
};

use super::contact::ContactConstraint;

/// The maximum number of colors in the constraint graph.
/// Constraints that cannot find a color are added to the overflow set,
/// which is solved on a single thread.
pub const GRAPH_COLOR_COUNT: usize = 12;

/// The index of the overflow color in the graph, used for constraints that don't fit
/// the graph color limit. This can happen when a single body is interacting with many other bodies.
pub const COLOR_OVERFLOW_INDEX: usize = GRAPH_COLOR_COUNT - 1;

// Solver using graph coloring. Islands are only used for sleep.
// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
// cause horrible cache stalls. To make this feasible I would need a way to block these writes.

/// A color in the graph. Each color is a set of bodies that can be solved together.
#[derive(Clone, Debug, Default)]
pub struct GraphColor {
    /// A bit vector representing the bodies that are part of this color.
    ///
    /// The bit vector is indexed by the body index, so it also contains static bodies.
    /// However, these bits are never iterated over, and the bit count is not used,
    /// so it should not be a problem.
    // TODO: Index by body index instead of entity index?
    pub body_set: BitVec,
    pub contacts: Vec<ContactPair>,
    pub contact_constraints: Vec<ContactConstraint>,
    // TODO: Joints
}

/// The constraint graph. This holds:
///
/// - Up to `GRAPH_COLOR_COUNT` colors
///   - Contact pairs
///   - Contact constraints
/// - Non-touching contacts
/// - Sleeping contacts
///
/// Each body can appear in a given color only once. This prevents race conditions
/// and allows constraints to be solved in parallel within each color.
///
/// Non-touching contacts and sleeping contacts are not part of the graph coloring
/// or the solver, but are still stored for the narrow phase and for users that need
/// to access them.
///
/// Sleeping contacts are stored separately to avoid iterating over them in the narrow phase.
#[derive(Resource, Clone, Debug)]
pub struct ConstraintGraph {
    /// The colors in the graph.
    pub colors: Vec<GraphColor>,
    /// All contacts for awake bodies that are not touching but have overlapping AABBs.
    pub non_touching_contacts: Vec<ContactPair>,
    /// All contacts for sleeping bodies, both touching and non-touching.
    pub sleeping_contacts: Vec<ContactPair>,
}

impl Default for ConstraintGraph {
    fn default() -> Self {
        Self::new(16)
    }
}

impl ConstraintGraph {
    /// Creates a new constraint graph with the given number of colors.
    pub fn new(body_capacity: usize) -> Self {
        let mut colors = Vec::with_capacity(GRAPH_COLOR_COUNT);

        let bit_capacity = body_capacity.max(8);

        // Initialize graph color bit vectors.
        for _ in 0..GRAPH_COLOR_COUNT {
            let mut body_set = BitVec::new(bit_capacity);
            body_set.set_bit_count_and_clear(bit_capacity);
            colors.push(GraphColor {
                body_set,
                // TODO: What's a good initial capacity for contacts and constraints?
                contacts: Vec::with_capacity(bit_capacity),
                contact_constraints: Vec::with_capacity(bit_capacity),
            });
        }

        Self {
            colors,
            // TODO: What's a good initial capacity for contacts?
            non_touching_contacts: Vec::with_capacity(bit_capacity),
            sleeping_contacts: Vec::with_capacity(bit_capacity),
        }
    }

    /// Adds a non-touching contact to the graph.
    ///
    /// Contacts are not a part of graph coloring until they are found to be touching.
    pub fn add_non_touching_contact(
        &mut self,
        contact_edge: &mut ContactEdge,
        contact_pair: ContactPair,
    ) {
        contact_edge.color_index = usize::MAX;
        contact_edge.local_index = self.non_touching_contacts.len();
        self.non_touching_contacts.push(contact_pair);
    }

    /// Adds a contact to the graph.
    pub fn add_touching_contact(
        &mut self,
        contact_edge: &mut ContactEdge,
        contact_pair: ContactPair,
    ) {
        // TODO: These shouldn't be `Option`s.
        let (Some(body1), Some(body2)) = (contact_pair.body1, contact_pair.body2) else {
            return;
        };

        let is_static1 = contact_pair.flags.contains(ContactPairFlags::STATIC1);
        let is_static2 = contact_pair.flags.contains(ContactPairFlags::STATIC2);

        debug_assert!(!contact_pair.manifolds.is_empty());
        debug_assert!(!is_static1 || !is_static2);

        let mut color_index = COLOR_OVERFLOW_INDEX;

        // TODO: We could allow forcing overflow by making this optional.
        if !is_static1 && !is_static2 {
            for i in 0..COLOR_OVERFLOW_INDEX {
                let color = &mut self.colors[i];
                if color.body_set.get(body1.index() as usize)
                    || color.body_set.get(body2.index() as usize)
                {
                    continue;
                }

                color.body_set.set_and_grow(body1.index() as usize);
                color.body_set.set_and_grow(body2.index() as usize);
                color_index = i;
                break;
            }
        } else if !is_static1 {
            // No static contacts in color 0
            for i in 1..COLOR_OVERFLOW_INDEX {
                let color = &mut self.colors[i];
                if color.body_set.get(body1.index() as usize) {
                    continue;
                }

                color.body_set.set_and_grow(body1.index() as usize);
                color_index = i;
                break;
            }
        } else if !is_static2 {
            // No static contacts in color 0
            for i in 1..COLOR_OVERFLOW_INDEX {
                let color = &mut self.colors[i];
                if color.body_set.get(body2.index() as usize) {
                    continue;
                }

                color.body_set.set_and_grow(body2.index() as usize);
                color_index = i;
                break;
            }
        }

        // Add the contact to the color.
        let color = &mut self.colors[color_index];
        contact_edge.color_index = color_index;
        contact_edge.local_index = color.contacts.len();
        color.contacts.push(contact_pair);
    }

    pub fn remove_non_touching_contact(
        &mut self,
        // We don't use the `ContactGraph` here to make this more flexible and reusable
        // for contexts where we can't provide the `ContactGraph` directly.
        contact_graph: &mut ContactGraphInternal,
        local_index: usize,
    ) -> ContactPair {
        // Remove the contact from the non-touching contacts.
        let moved_index = self.non_touching_contacts.len() - 1;
        let removed = self.non_touching_contacts.swap_remove(local_index);

        if moved_index != local_index {
            // Fix moved contact.
            let moved_contact = &mut self.non_touching_contacts[local_index];
            let moved_contact_edge = contact_graph
                .edge_weight_mut(EdgeIndex(moved_contact.contact_id.0))
                .unwrap();
            debug_assert!(moved_contact_edge.color_index == usize::MAX);
            debug_assert!(moved_contact_edge.local_index == moved_index);
            moved_contact_edge.local_index = local_index;
        }

        removed
    }

    pub fn remove_touching_contact(
        &mut self,
        // We don't use the `ContactGraph` here to make this more flexible and reusable
        // for contexts where we can't provide the `ContactGraph` directly.
        contact_graph: &mut ContactGraphInternal,
        color_index: usize,
        local_index: usize,
    ) -> ContactPair {
        debug_assert!(color_index < GRAPH_COLOR_COUNT);

        let color = &mut self.colors[color_index];

        let moved_index = color.contacts.len() - 1;
        let removed = color.contacts.swap_remove(local_index);

        let body1 = removed.body1.unwrap();
        let body2 = removed.body2.unwrap();

        if color_index != COLOR_OVERFLOW_INDEX {
            color.body_set.unset(body1.index() as usize);
            color.body_set.unset(body2.index() as usize);
        }

        if moved_index != local_index {
            // Fix moved contact.
            let moved_contact = &mut color.contacts[local_index];
            let moved_contact_edge = contact_graph
                .edge_weight_mut(EdgeIndex(moved_contact.contact_id.0))
                .unwrap();
            debug_assert!(moved_contact_edge.color_index == color_index);
            debug_assert!(moved_contact_edge.local_index == moved_index);
            moved_contact_edge.local_index = local_index;
        }

        removed
    }

    pub fn get_contact_pair(&self, color_index: usize, local_index: usize) -> Option<&ContactPair> {
        if color_index != usize::MAX {
            // The contact pair lives in a graph color.
            debug_assert!(color_index < GRAPH_COLOR_COUNT);
            let color = &self.colors[color_index];
            color.contacts.get(local_index)
        } else {
            // The contact pair lives is non-touching.
            self.non_touching_contacts.get(local_index)
        }
    }

    pub fn get_contact_pair_mut(
        &mut self,
        color_index: usize,
        local_index: usize,
    ) -> Option<&mut ContactPair> {
        if color_index != usize::MAX {
            // The contact pair lives in a graph color.
            debug_assert!(color_index < GRAPH_COLOR_COUNT);
            let color = &mut self.colors[color_index];
            color.contacts.get_mut(local_index)
        } else {
            // The contact pair lives is non-touching.
            // TODO: It could also be in the sleeping contacts!
            self.non_touching_contacts.get_mut(local_index)
        }
    }

    /// Transfers contact of a body from the sleeping contacts to the graph.
    pub fn wake_entity(&mut self, contact_graph: &mut ContactGraph, entity: Entity) {
        // TODO: Avoid this.
        let ids = contact_graph
            .collisions_with(entity)
            .map(|edge| edge.id)
            .collect::<Vec<ContactId>>();

        for contact_id in ids {
            // Find the contact edge in the contact graph.
            // TODO: Avoid this lookup.
            let contact_edge = contact_graph
                .get_mut_by_id(contact_id)
                .unwrap_or_else(|| panic!("Contact edge not found in graph: {contact_id:?}"));
            let local_index = contact_edge.local_index;

            if contact_edge.color_index != usize::MAX || local_index >= self.sleeping_contacts.len()
            {
                // The contact is already in the graph.
                // TODO: This should not happen, but it does.
                warn!("Contact edge {contact_id:?} is already in the graph");
                continue;
            }

            // Remove the contact from the sleeping contacts.
            let moved_index = self.sleeping_contacts.len() - 1;
            let transferred_pair = self.sleeping_contacts.swap_remove(local_index);

            if contact_edge.is_touching() {
                // Add the contact to the graph.
                self.add_touching_contact(contact_edge, transferred_pair);
            } else {
                // Add the contact to the non-touching contacts.
                self.add_non_touching_contact(contact_edge, transferred_pair);
            }

            if moved_index != local_index {
                // Fix moved contact.
                let moved_contact = &mut self.sleeping_contacts[local_index];
                let moved_contact_edge = contact_graph
                    .get_mut_by_id(moved_contact.contact_id)
                    .unwrap();
                debug_assert!(moved_contact_edge.color_index == usize::MAX);
                debug_assert!(moved_contact_edge.local_index == moved_index);
                moved_contact_edge.local_index = local_index;
            }
        }
    }

    /// Transfers contact of a body from the graph to the sleeping contacts.
    pub fn sleep_entity(&mut self, contact_graph: &mut ContactGraph, entity: Entity) {
        // TODO: Avoid this.
        let indices = contact_graph
            .collisions_with(entity)
            .map(|edge| (edge.is_touching(), edge.color_index, edge.local_index))
            .collect::<Vec<(bool, usize, usize)>>();

        for (is_touching, color_index, local_index) in indices {
            // Remove the contact from the graph.
            let transferred_pair = if is_touching {
                self.remove_touching_contact(&mut contact_graph.edges, color_index, local_index)
            } else {
                self.remove_non_touching_contact(&mut contact_graph.edges, local_index)
            };

            // Add the contact to the sleeping contacts.
            let moved_index = self.sleeping_contacts.len();
            let contact_id = transferred_pair.contact_id;
            self.sleeping_contacts.push(transferred_pair);
            // TODO: Avoid this lookup.
            let contact_edge = contact_graph
                .get_mut_by_id(contact_id)
                .unwrap_or_else(|| panic!("Contact edge not found in graph: {contact_id:?}"));
            contact_edge.color_index = usize::MAX;
            contact_edge.local_index = moved_index;
        }
    }
}
