use std::sync::Arc;

use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*, utils::HashMap};
use parry::{
    partitioning::{Qbvh, QbvhUpdateWorkspace},
    query::{
        details::RayCompositeShapeToiAndNormalBestFirstVisitor, visitors::RayIntersectionsVisitor,
        DefaultQueryDispatcher, QueryDispatcher,
    },
    shape::{Shape, TypedSimdCompositeShape},
    utils::DefaultStorage,
};

/// A system parameter for spatial queries that require more precise control.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn print_intersections(spatial_query: SpatialQuery) {
///     let intersections = spatial_query.ray_intersections(
///         Vec3::ZERO, // Origin
///         Vec3::X,    // Direction
///         100.0,      // Maximum time of impact (travel distance)
///         20,         // Maximum number of intersections
///         true,       // Is the ray solid
///     );
///
///     println!("{:?}", intersections);
/// }
/// # #[cfg(not(all(feature = "3d", feature = "f32")))]
/// # fn print_intersections() {}
/// ```
#[derive(SystemParam)]
pub struct SpatialQuery<'w, 's> {
    colliders: Query<
        'w,
        's,
        (
            Entity,
            &'static Position,
            &'static Rotation,
            &'static Collider,
        ),
    >,
    query_pipeline: ResMut<'w, SpatialQueryPipeline>,
}

impl<'w, 's> SpatialQuery<'w, 's> {
    /// Casts a [ray](RayCaster) from `origin` in a given `direction` and computes the closest [intersection](RayIntersection)
    /// with a collider. If there are no intersections, `None` is returned.
    ///
    /// This should be used when you don't need frequent ray casts and want the result instantly.
    /// Otherwise, using [`RayCaster`] is recommended.
    pub fn ray_intersection(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = self
            .colliders
            .iter()
            .map(|(entity, position, rotation, collider)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        &**collider.get_shape(),
                    ),
                )
            })
            .collect();

        let pipeline_shape = self.query_pipeline.as_composite_shape(&colliders);
        let ray = parry::query::Ray::new(origin.into(), direction.into());
        let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
            &pipeline_shape,
            &ray,
            max_time_of_impact,
            solid,
        );

        self.query_pipeline
            .qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (entity_bits, intersection))| RayIntersection {
                entity: Entity::from_bits(entity_bits),
                time_of_impact: intersection.toi,
                normal: intersection.normal.into(),
            })
    }

    /// Casts a [ray](RayCaster) from `origin` in a given `direction` and computes all [intersections](RayIntersection)
    /// until `max_hits` is reached.
    ///
    /// Note that the order of the results is not guaranteed, and if there are more intersections than `max_hits`, some intersections will be missed.
    ///
    /// This should be used when you don't need frequent ray casts and want the result instantly.
    /// Otherwise, using [`RayCaster`] is recommended.
    pub fn ray_intersections(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: f32,
        max_hits: u32,
        solid: bool,
    ) -> Vec<RayIntersection> {
        let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = self
            .colliders
            .iter()
            .map(|(entity, position, rotation, collider)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        &**collider.get_shape(),
                    ),
                )
            })
            .collect();

        let mut intersections = Vec::with_capacity(max_hits.min(100) as usize);

        let ray = parry::query::Ray::new(origin.into(), direction.into());

        let mut leaf_callback = &mut |entity_bits: &u64| {
            let entity = Entity::from_bits(*entity_bits);
            if let Some((iso, shape)) = colliders.get(&entity) {
                if let Some(intersection) =
                    shape.cast_ray_and_get_normal(iso, &ray, max_time_of_impact, solid)
                {
                    intersections.push(RayIntersection {
                        entity,
                        time_of_impact: intersection.toi,
                        normal: intersection.normal.into(),
                    });

                    return (intersections.len() as u32) < max_hits;
                }
            }
            true
        };

        let mut visitor =
            RayIntersectionsVisitor::new(&ray, max_time_of_impact, &mut leaf_callback);
        self.query_pipeline.qbvh.traverse_depth_first(&mut visitor);

        intersections
    }

    /// Casts a [ray](RayCaster) from `origin` in a given `direction` and computes all [intersections](RayIntersection)
    /// until the given `callback` returns false or all intersections have been computed.
    ///
    /// Note that the order of the results is not guaranteed, and if there are more intersections than `max_hits`, some intersections will be missed.
    ///
    /// This should be used when you don't need frequent ray casts and want the result instantly.
    /// Otherwise, using [`RayCaster`] is recommended.
    pub fn ray_intersections_callback(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: f32,
        solid: bool,
        mut callback: impl FnMut(Entity, RayIntersection) -> bool,
    ) -> Vec<RayIntersection> {
        let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = self
            .colliders
            .iter()
            .map(|(entity, position, rotation, collider)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        &**collider.get_shape(),
                    ),
                )
            })
            .collect();

        let mut intersections = Vec::with_capacity(10);

        let ray = parry::query::Ray::new(origin.into(), direction.into());

        let mut leaf_callback = &mut |entity_bits: &u64| {
            let entity = Entity::from_bits(*entity_bits);
            if let Some((iso, shape)) = colliders.get(&entity) {
                if let Some(intersection) =
                    shape.cast_ray_and_get_normal(iso, &ray, max_time_of_impact, solid)
                {
                    let intersection = RayIntersection {
                        entity,
                        time_of_impact: intersection.toi,
                        normal: intersection.normal.into(),
                    };
                    intersections.push(intersection);

                    return callback(entity, intersection);
                }
            }
            true
        };

        let mut visitor =
            RayIntersectionsVisitor::new(&ray, max_time_of_impact, &mut leaf_callback);
        self.query_pipeline.qbvh.traverse_depth_first(&mut visitor);

        intersections
    }
}

/// A resource for the spatial query pipeline.
///
/// The pipeline maintains a quaternary bounding volume hierarchy `Qbvh` of the world's colliders
/// as an acceleration structure for spatial queries.
#[derive(Resource, Clone)]
pub struct SpatialQueryPipeline {
    pub(crate) qbvh: Qbvh<u64>,
    pub(crate) dispatcher: Arc<dyn QueryDispatcher>,
    pub(crate) workspace: QbvhUpdateWorkspace,
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

    pub(crate) fn update_incremental(
        &mut self,
        colliders: &HashMap<Entity, (Isometry<Scalar>, &dyn Shape)>,
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
