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
    pub(crate) entity_generations: HashMap<u32, u32>,
}

impl Default for SpatialQueryPipeline {
    fn default() -> Self {
        Self {
            qbvh: Qbvh::new(),
            dispatcher: Arc::new(DefaultQueryDispatcher),
            workspace: QbvhUpdateWorkspace::default(),
            entity_generations: HashMap::default(),
        }
    }
}

impl SpatialQueryPipeline {
    pub(crate) fn as_composite_shape<'a>(
        &'a self,
        colliders: &'a HashMap<Entity, (Isometry<Scalar>, &'a dyn Shape, CollisionLayers)>,
        query_filter: SpatialQueryFilter,
    ) -> QueryPipelineAsCompositeShape {
        QueryPipelineAsCompositeShape {
            pipeline: self,
            colliders,
            query_filter,
        }
    }

    pub(crate) fn update_incremental(
        &mut self,
        colliders: &HashMap<Entity, (Isometry<Scalar>, &dyn Shape)>,
        added: Vec<Entity>,
        modified: Vec<Entity>,
        removed: Vec<Entity>,
        refit_and_balance: bool,
    ) {
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
            if colliders.get(&modified).is_some() {
                self.qbvh.pre_update_or_insert(modified.index());
            }
        }

        if refit_and_balance {
            let _ = self.qbvh.refit(0.0, &mut self.workspace, |entity_index| {
                // Construct entity ID
                let generation = self.entity_generations.get(entity_index).map_or(0, |i| *i);
                let entity = Entity::from_bits((generation as u64) << 32 | *entity_index as u64);
                // Compute and return AABB
                let (iso, shape) = colliders.get(&entity).unwrap();
                shape.compute_aabb(iso)
            });
            self.qbvh.rebalance(0.0, &mut self.workspace);
        }
    }
}

pub(crate) struct QueryPipelineAsCompositeShape<'a> {
    colliders: &'a HashMap<Entity, (Isometry<Scalar>, &'a dyn Shape, CollisionLayers)>,
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
            self.colliders.get_key_value(&Entity::from_raw(shape_id))
        {
            if self.query_filter.test(*entity, *layers) {
                f(Some(iso), &**shape);
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
