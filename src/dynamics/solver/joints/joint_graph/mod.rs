//! A [`JointGraph`] for tracking how [rigid bodies] are connected by [joints].
//!
//! [rigid bodies]: crate::dynamics::RigidBody
//! [joints]: crate::dynamics::solver::joints

mod plugin;
pub use plugin::{JointComponentId, JointGraphPlugin};

use crate::data_structures::{
    graph::{EdgeIndex, NodeIndex, UnGraph},
    sparse_secondary_map::SparseSecondaryEntityMap,
};
use bevy::prelude::*;

// TODO: Once we have many-to-many relationships, we could potentially represent the joint graph in the ECS.

/// A resource for the joint graph, tracking how [rigid bodies] are connected by [joints].
///
/// [rigid bodies]: crate::dynamics::RigidBody
/// [joints]: crate::dynamics::solver::joints
#[derive(Resource, Clone, Debug, Default)]
pub struct JointGraph {
    graph: UnGraph<Entity, JointGraphEdge>,
    entity_to_body: SparseSecondaryEntityMap<NodeIndex>,
    entity_to_joint: SparseSecondaryEntityMap<EdgeIndex>,
}

/// An edge in the [`JointGraph`].
#[derive(Clone, Debug, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug)]
pub struct JointGraphEdge {
    /// The entity of the joint.
    pub entity: Entity,
    /// If `true`, collisions are disabled between the bodies connected by this joint.
    ///
    /// This is controlled by the [`JointCollisionDisabled`] component.
    ///
    /// [`JointCollisionDisabled`]: crate::dynamics::solver::joints::JointCollisionDisabled
    pub collision_disabled: bool,
}

impl JointGraph {
    /// Returns a reference to the underlying [`UnGraph`].
    #[inline]
    pub fn graph(&self) -> &UnGraph<Entity, JointGraphEdge> {
        &self.graph
    }

    /// Returns the [`NodeIndex`] of the given entity in the joint graph.
    ///
    /// If the entity is not in the graph, `None` is returned.
    #[inline]
    pub fn entity_to_body(&self, entity: Entity) -> Option<NodeIndex> {
        self.entity_to_body.get(entity).copied()
    }

    /// Returns the [`EdgeIndex`] of the joint edge for the given entity.
    ///
    /// If the entity is not in the graph, `None` is returned.
    #[inline]
    pub fn entity_to_joint(&self, entity: Entity) -> Option<EdgeIndex> {
        self.entity_to_joint.get(entity).copied()
    }

    /// Returns the [`JointGraphEdge`] for the given joint entity.
    /// If the joint entity is not in the graph, `None` is returned.
    #[inline]
    pub fn get_joint(&self, joint: Entity) -> Option<&JointGraphEdge> {
        let joint_index = self.entity_to_joint.get(joint)?;
        self.graph.edge_weight(*joint_index)
    }

    /// Returns a mutable reference to the [`JointGraphEdge`] for the given joint entity.
    /// If the joint entity is not in the graph, `None` is returned.
    #[inline]
    pub fn get_joint_mut(&mut self, joint: Entity) -> Option<&mut JointGraphEdge> {
        let joint_index = self.entity_to_joint.get(joint)?;
        self.graph.edge_weight_mut(*joint_index)
    }

    /// Returns the [`JointGraphEdge`] between two entities.
    /// If the edge does not exist, `None` is returned.
    #[inline]
    pub fn joints_between(
        &self,
        body1: Entity,
        body2: Entity,
    ) -> impl Iterator<Item = &JointGraphEdge> {
        let (Some(index1), Some(index2)) = (self.entity_to_body(body1), self.entity_to_body(body2))
        else {
            return itertools::Either::Left(core::iter::empty());
        };

        let joints = self.graph.edges_between(index1, index2).map(|e| e.weight());
        itertools::Either::Right(joints)
    }

    /// Returns an iterator yielding immutable access to all joint edges involving the given entity.
    #[inline]
    pub fn joints_of(&self, body: Entity) -> impl Iterator<Item = &JointGraphEdge> {
        let index = self.entity_to_body(body);
        if let Some(index) = index {
            itertools::Either::Left(self.graph.edge_weights(index))
        } else {
            itertools::Either::Right(core::iter::empty())
        }
    }

    /// Returns an iterator yielding mutable access to all joint edges involving the given entity.
    #[inline]
    pub fn joints_of_mut(&mut self, body: Entity) -> impl Iterator<Item = &mut JointGraphEdge> {
        let index = self.entity_to_body(body);
        if let Some(index) = index {
            itertools::Either::Left(self.graph.edge_weights_mut(index))
        } else {
            itertools::Either::Right(core::iter::empty())
        }
    }

    /// Returns the bodies that are connected by the given joint entity.
    /// If the joint entity is not in the graph, `None` is returned.
    #[inline]
    pub fn bodies_of(&self, joint: Entity) -> Option<[Entity; 2]> {
        let joint_index = self.entity_to_joint(joint)?;
        let (body1_index, body2_index) = self.graph.edge_endpoints(joint_index)?;

        Some([
            *self.graph.node_weight(body1_index)?,
            *self.graph.node_weight(body2_index)?,
        ])
    }

    /// Returns an iterator yielding immutable access to all bodies that are attached
    /// to the given entity with a joint.
    #[inline]
    pub fn bodies_attached_to(&self, body: Entity) -> impl Iterator<Item = Entity> + '_ {
        self.entity_to_body
            .get(body)
            .into_iter()
            .flat_map(move |&index| {
                self.graph
                    .neighbors(index)
                    .map(|index| *self.graph.node_weight(index).unwrap())
            })
    }

    /// Creates a [`JointGraphEdge`] between two entities if it does not already exist,
    /// and returns the [`EdgeIndex`] of the created joint edge.
    ///
    /// # Warning
    ///
    /// Creating a joint edge with this method will *not* wake up the entities involved
    /// or do any other clean-up. Only use this method if you know what you are doing.
    #[inline]
    pub fn add_joint(
        &mut self,
        body1: Entity,
        body2: Entity,
        joint_edge: JointGraphEdge,
    ) -> EdgeIndex {
        // Get the indices of the entities in the graph.
        let body1_index = self
            .entity_to_body
            .get_or_insert_with(body1, || self.graph.add_node(body1));
        let body2_index = self
            .entity_to_body
            .get_or_insert_with(body2, || self.graph.add_node(body2));

        // Add the edge to the graph.
        self.entity_to_joint
            .get_or_insert_with(joint_edge.entity, || {
                self.graph.add_edge(body1_index, body2_index, joint_edge)
            })
    }

    /// Removes a [`JointGraphEdge`] between two entites and returns its value.
    ///
    /// # Warning
    ///
    /// Removing a joint edge with this method will *not* wake up the entities involved
    /// or do any other clean-up. Only use this method if you know what you are doing.
    #[inline]
    pub fn remove_joint(&mut self, joint_entity: Entity) -> Option<JointGraphEdge> {
        let joint_index = self.entity_to_joint.remove(joint_entity)?;
        self.graph.remove_edge(joint_index)
    }

    /// Removes the body of the given entity from the joint graph, calling the given callback
    /// for each [`JointGraphEdge`] right before it is removed.
    ///
    /// # Warning
    ///
    /// Removing a body with this method will *not* wake up the entities involved
    /// or do any other clean-up. Only use this method if you know what you are doing.
    #[inline]
    pub fn remove_body_with<F>(&mut self, entity: Entity, edge_callback: F)
    where
        F: FnMut(JointGraphEdge),
    {
        // Remove the entity from the entity-to-node mapping,
        // and get the index of the node in the graph.
        let Some(index) = self.entity_to_body.remove(entity) else {
            return;
        };

        // Remove the entity from the graph.
        // TODO: Should we remove the joint from the entity-to-joint mapping as well?
        self.graph.remove_node_with(index, edge_callback);

        // Removing the node swapped the last node to its place,
        // so we need to remap the entity-to-node mapping of the swapped node.
        if let Some(swapped) = self.graph.node_weight(index).copied() {
            let swapped_index = self
                .entity_to_body
                .get_mut(swapped)
                // This should never panic.
                .expect("swapped entity has no entity-to-node mapping");
            *swapped_index = index;
        }
    }
}
