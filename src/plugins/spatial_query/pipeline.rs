use std::sync::Arc;

use crate::prelude::*;
use bevy::{prelude::*, utils::HashMap};
use parry::{
    partitioning::{Qbvh, QbvhUpdateWorkspace},
    query::{DefaultQueryDispatcher, QueryDispatcher},
    shape::{Shape, TypedSimdCompositeShape},
    utils::DefaultStorage,
};

/// A resource for the spatial query pipeline.
///
/// The pipeline maintains a quaternary bounding volume hierarchy `Qbvh` of the world's colliders
/// as an acceleration structure for spatial queries.
#[derive(Resource, Clone)]
pub struct SpatialQueryPipeline {
    pub(crate) qbvh: Qbvh<u32>,
    pub(crate) dispatcher: Arc<dyn QueryDispatcher>,
    pub(crate) workspace: QbvhUpdateWorkspace,
    pub(crate) colliders: HashMap<Entity, (Isometry<Scalar>, Collider, CollisionLayers)>,
    pub(crate) entity_generations: HashMap<u32, u32>,
}

impl Default for SpatialQueryPipeline {
    fn default() -> Self {
        Self {
            qbvh: Qbvh::new(),
            dispatcher: Arc::new(DefaultQueryDispatcher),
            workspace: QbvhUpdateWorkspace::default(),
            colliders: HashMap::default(),
            entity_generations: HashMap::default(),
        }
    }
}

impl SpatialQueryPipeline {
    pub(crate) fn as_composite_shape(
        &self,
        query_filter: SpatialQueryFilter,
    ) -> QueryPipelineAsCompositeShape {
        QueryPipelineAsCompositeShape {
            pipeline: self,
            colliders: &self.colliders,
            query_filter,
        }
    }

    pub(crate) fn update_incremental(
        &mut self,
        colliders: HashMap<Entity, (Isometry<Scalar>, Collider, CollisionLayers)>,
        added: &[Entity],
        modified: &[Entity],
        removed: &[Entity],
        refit_and_balance: bool,
    ) {
        self.colliders = colliders;

        // Insert or update generations of added entities
        for added in added {
            let index = added.index();
            if let Some(generation) = self.entity_generations.get_mut(&index) {
                *generation = added.generation();
            } else {
                self.entity_generations.insert(index, added.generation());
            }
        }

        for removed in removed {
            self.qbvh.remove(removed.index());
        }

        for modified in modified {
            if self.colliders.contains_key(modified) {
                self.qbvh.pre_update_or_insert(modified.index());
            }
        }

        if refit_and_balance {
            let _ = self.qbvh.refit(0.0, &mut self.workspace, |entity_index| {
                // Construct entity ID
                let generation = self.entity_generations.get(entity_index).map_or(0, |i| *i);
                let entity = utils::entity_from_index_and_gen(*entity_index, generation);
                // Compute and return AABB
                let (iso, shape, _) = self.colliders.get(&entity).unwrap();
                let aabb = shape.get_shape().compute_aabb(iso);
                aabb
            });
            self.qbvh.rebalance(0.0, &mut self.workspace);
        }
    }

    pub(crate) fn entity_from_index(&self, index: u32) -> Entity {
        utils::entity_from_index_and_gen(index, *self.entity_generations.get(&index).unwrap())
    }
}

pub(crate) struct QueryPipelineAsCompositeShape<'a> {
    colliders: &'a HashMap<Entity, (Isometry<Scalar>, Collider, CollisionLayers)>,
    pipeline: &'a SpatialQueryPipeline,
    query_filter: SpatialQueryFilter,
}

impl<'a> TypedSimdCompositeShape for QueryPipelineAsCompositeShape<'a> {
    type PartShape = dyn Shape;
    type PartId = u32;
    type QbvhStorage = DefaultStorage;

    fn map_typed_part_at(
        &self,
        shape_id: Self::PartId,
        mut f: impl FnMut(Option<&Isometry<Scalar>>, &Self::PartShape),
    ) {
        if let Some((entity, (iso, shape, layers))) =
            self.colliders
                .get_key_value(&utils::entity_from_index_and_gen(
                    shape_id,
                    *self.pipeline.entity_generations.get(&shape_id).unwrap(),
                ))
        {
            if self.query_filter.test(*entity, *layers) {
                f(Some(iso), &**shape.get_shape());
            }
        }
    }

    fn map_untyped_part_at(
        &self,
        shape_id: Self::PartId,
        f: impl FnMut(Option<&Isometry<Scalar>>, &dyn Shape),
    ) {
        self.map_typed_part_at(shape_id, f);
    }

    fn typed_qbvh(&self) -> &parry::partitioning::GenericQbvh<Self::PartId, Self::QbvhStorage> {
        &self.pipeline.qbvh
    }
}
