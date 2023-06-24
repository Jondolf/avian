use std::sync::Arc;

use crate::prelude::*;
use bevy::{prelude::*, utils::HashMap};
use parry::{
    partitioning::{Qbvh, QbvhUpdateWorkspace},
    query::{DefaultQueryDispatcher, QueryDispatcher},
    shape::{Shape, TypedSimdCompositeShape},
    utils::DefaultStorage,
};

#[derive(Resource, Clone)]
pub struct SpatialQueryPipeline {
    pub qbvh: Qbvh<u64>,
    pub dispatcher: Arc<dyn QueryDispatcher>,
    pub workspace: QbvhUpdateWorkspace,
}

impl Default for SpatialQueryPipeline {
    fn default() -> Self {
        Self {
            qbvh: Qbvh::new(),
            dispatcher: Arc::new(DefaultQueryDispatcher),
            workspace: QbvhUpdateWorkspace::default(),
        }
    }
}

impl SpatialQueryPipeline {
    pub(crate) fn as_composite_shape<'a>(
        &'a self,
        colliders: &'a HashMap<Entity, (Isometry<Scalar>, &'a dyn Shape)>,
    ) -> QueryPipelineAsCompositeShape {
        QueryPipelineAsCompositeShape {
            pipeline: self,
            colliders,
        }
    }

    pub(crate) fn update_incremental<'a>(
        &mut self,
        colliders: &HashMap<Entity, (Isometry<Scalar>, &'a dyn Shape)>,
        modified: Vec<Entity>,
        removed: Vec<Entity>,
        refit_and_balance: bool,
    ) {
        for removed in removed {
            self.qbvh.remove(removed.to_bits());
        }

        for modified in modified {
            if colliders.get(&modified).is_some() {
                self.qbvh.pre_update_or_insert(modified.to_bits());
            }
        }

        if refit_and_balance {
            let _ = self.qbvh.refit(0.0, &mut self.workspace, |entity_bits| {
                let (iso, shape) = colliders.get(&Entity::from_bits(*entity_bits)).unwrap();
                shape.compute_aabb(iso)
            });
        }
    }
}

pub(crate) struct QueryPipelineAsCompositeShape<'a> {
    colliders: &'a HashMap<Entity, (Isometry<Scalar>, &'a dyn Shape)>,
    pipeline: &'a SpatialQueryPipeline,
}

impl<'a> TypedSimdCompositeShape for QueryPipelineAsCompositeShape<'a> {
    type PartShape = dyn Shape;
    type PartId = u64;
    type QbvhStorage = DefaultStorage;

    fn map_typed_part_at(
        &self,
        shape_id: Self::PartId,
        mut f: impl FnMut(Option<&Isometry<Scalar>>, &Self::PartShape),
    ) {
        if let Some((iso, shape)) = self.colliders.get(&Entity::from_bits(shape_id)) {
            f(Some(iso), &**shape);
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
