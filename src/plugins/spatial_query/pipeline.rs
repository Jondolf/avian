use std::sync::Arc;

use crate::prelude::*;
use bevy::{prelude::*, utils::HashMap};
use parry::{
    partitioning::{Qbvh, QbvhUpdateWorkspace},
    query::{
        details::{
            RayCompositeShapeToiAndNormalBestFirstVisitor, TOICompositeShapeShapeBestFirstVisitor,
        },
        point::PointCompositeShapeProjBestFirstVisitor,
        visitors::{
            BoundingVolumeIntersectionsVisitor, PointIntersectionsVisitor, RayIntersectionsVisitor,
        },
        DefaultQueryDispatcher, QueryDispatcher,
    },
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
    /// Creates a new [SpatialQueryPipeline].
    pub fn new() -> SpatialQueryPipeline {
        SpatialQueryPipeline::default()
    }

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

    /// Updates the associated acceleration structures with a new set of entities.
    pub fn update_incremental<'a>(
        &mut self,
        colliders: impl Iterator<
            Item = (
                Entity,
                &'a Position,
                &'a Rotation,
                &'a Collider,
                Option<&'a CollisionLayers>,
            ),
        >,
        added_colliders: impl Iterator<Item = Entity>,
        changed_colliders: impl Iterator<Item = Entity>,
        removed_colliders: impl Iterator<Item = Entity>,
        refit_and_balance: bool,
    ) {
        let colliders = colliders
            .map(|(entity, position, rotation, collider, layers)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        collider.clone(),
                        layers.map_or(CollisionLayers::default(), |layers| *layers),
                    ),
                )
            })
            .collect();
        let added = added_colliders.collect::<Vec<_>>();
        let modified = changed_colliders.collect::<Vec<_>>();
        let removed = removed_colliders.collect::<Vec<_>>();

        self.update_incremental_internal(colliders, &added, &modified, &removed, refit_and_balance)
    }

    fn update_incremental_internal(
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

    /// Casts a [ray](spatial_query#ray-casting) and computes the closest [hit](RayHitData) with a collider.
    /// If there are no hits, `None` is returned.
    ///
    /// ## Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `max_time_of_impact`: The maximum distance that the ray can travel.
    /// - `solid`: If true and the ray origin is inside of a collider, the hit point will be the ray origin itself.
    /// Otherwise, the collider will be treated as hollow, and the hit point will be at the collider's boundary.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// See also: [SpatialQuery::cast_ray]
    pub fn cast_ray(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: Scalar,
        solid: bool,
        query_filter: SpatialQueryFilter,
    ) -> Option<RayHitData> {
        let pipeline_shape = self.as_composite_shape(query_filter);
        let ray = parry::query::Ray::new(origin.into(), direction.into());
        let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
            &pipeline_shape,
            &ray,
            max_time_of_impact,
            solid,
        );

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (entity_index, hit))| RayHitData {
                entity: self.entity_from_index(entity_index),
                time_of_impact: hit.toi,
                normal: hit.normal.into(),
            })
    }

    /// Casts a [ray](spatial_query#ray-casting) and computes all [hits](RayHitData) until `max_hits` is reached.
    ///
    /// Note that the order of the results is not guaranteed, and if there are more hits than `max_hits`,
    /// some hits will be missed.
    ///
    /// ## Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `max_time_of_impact`: The maximum distance that the ray can travel.
    /// - `max_hits`: The maximum number of hits. Additional hits will be missed.
    /// - `solid`: If true and the ray origin is inside of a collider, the hit point will be the ray origin itself.
    /// Otherwise, the collider will be treated as hollow, and the hit point will be at the collider's boundary.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// See also: [SpatialQuery::ray_hits]
    pub fn ray_hits(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: Scalar,
        max_hits: u32,
        solid: bool,
        query_filter: SpatialQueryFilter,
    ) -> Vec<RayHitData> {
        let mut hits = 0;
        self.ray_hits_callback(
            origin,
            direction,
            max_time_of_impact,
            solid,
            query_filter,
            |_| {
                hits += 1;
                hits < max_hits
            },
        )
    }

    /// Casts a [ray](spatial_query#ray-casting) and computes all [hits](RayHitData), calling the given `callback`
    /// for each hit. The ray cast stops when `callback` returns false or all hits have been found.
    ///
    /// Note that the order of the results is not guaranteed.
    ///
    /// ## Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `max_time_of_impact`: The maximum distance that the ray can travel.
    /// - `solid`: If true and the ray origin is inside of a collider, the hit point will be the ray origin itself.
    /// Otherwise, the collider will be treated as hollow, and the hit point will be at the collider's boundary.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each hit.
    ///
    /// See also: [SpatialQuery::ray_hits_callback]
    pub fn ray_hits_callback(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: Scalar,
        solid: bool,
        query_filter: SpatialQueryFilter,
        mut callback: impl FnMut(RayHitData) -> bool,
    ) -> Vec<RayHitData> {
        let colliders = &self.colliders;

        let mut hits = Vec::with_capacity(10);
        let ray = parry::query::Ray::new(origin.into(), direction.into());

        let mut leaf_callback = &mut |entity_index: &u32| {
            let entity = self.entity_from_index(*entity_index);
            if let Some((iso, shape, layers)) = colliders.get(&entity) {
                if query_filter.test(entity, *layers) {
                    if let Some(hit) =
                        shape.cast_ray_and_get_normal(iso, &ray, max_time_of_impact, solid)
                    {
                        let hit = RayHitData {
                            entity,
                            time_of_impact: hit.toi,
                            normal: hit.normal.into(),
                        };
                        hits.push(hit);

                        return callback(hit);
                    }
                }
            }
            true
        };

        let mut visitor =
            RayIntersectionsVisitor::new(&ray, max_time_of_impact, &mut leaf_callback);
        self.qbvh.traverse_depth_first(&mut visitor);

        hits
    }

    /// Casts a [shape](spatial_query#shape-casting) with a given rotation and computes the closest [hit](ShapeHits)
    /// with a collider. If there are no hits, `None` is returned.
    ///
    /// For a more ECS-based approach, consider using the [`ShapeCaster`] component instead.
    ///
    /// ## Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `max_time_of_impact`: The maximum distance that the shape can travel.
    /// - `ignore_origin_penetration`: If true and the shape is already penetrating a collider at the
    /// shape origin, the hit will be ignored and only the next hit will be computed. Otherwise, the initial
    /// hit will be returned.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// See also: [SpatialQuery::cast_shape]
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Vector,
        max_time_of_impact: Scalar,
        ignore_origin_penetration: bool,
        query_filter: SpatialQueryFilter,
    ) -> Option<ShapeHitData> {
        let rotation: Rotation;
        #[cfg(feature = "2d")]
        {
            rotation = Rotation::from_radians(shape_rotation);
        }
        #[cfg(feature = "3d")]
        {
            rotation = Rotation::from(shape_rotation);
        }

        let shape_isometry = utils::make_isometry(origin, &rotation);
        let shape_direction = direction.into();
        let pipeline_shape = self.as_composite_shape(query_filter);
        let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
            &*self.dispatcher,
            &shape_isometry,
            &shape_direction,
            &pipeline_shape,
            &**shape.get_shape(),
            max_time_of_impact,
            !ignore_origin_penetration,
        );

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (entity_index, hit))| ShapeHitData {
                entity: self.entity_from_index(entity_index),
                time_of_impact: hit.toi,
                point1: hit.witness1.into(),
                point2: hit.witness2.into(),
                normal1: hit.normal1.into(),
                normal2: hit.normal2.into(),
            })
    }

    /// Casts a [shape](spatial_query#shape-casting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of the time of impact until `max_hits` is reached.
    ///
    /// ## Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `max_time_of_impact`: The maximum distance that the shape can travel.
    /// - `max_hits`: The maximum number of hits. Additional hits will be missed.
    /// - `ignore_origin_penetration`: If true and the shape is already penetrating a collider at the
    /// shape origin, the hit will be ignored and only the next hit will be computed. Otherwise, the initial
    /// hit will be returned.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each hit.
    ///
    /// See also: [SpatialQuery::shape_hits]
    #[allow(clippy::too_many_arguments)]
    pub fn shape_hits(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Vector,
        max_time_of_impact: Scalar,
        max_hits: u32,
        ignore_origin_penetration: bool,
        query_filter: SpatialQueryFilter,
    ) -> Vec<ShapeHitData> {
        let mut hits = 0;
        self.shape_hits_callback(
            shape,
            origin,
            shape_rotation,
            direction,
            max_time_of_impact,
            ignore_origin_penetration,
            query_filter,
            |_| {
                hits += 1;
                hits < max_hits
            },
        )
    }

    /// Casts a [shape](spatial_query#shape-casting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of the time of impact, calling the given `callback` for each hit. The shape cast stops when
    /// `callback` returns false or all hits have been found.
    ///
    /// ## Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `max_time_of_impact`: The maximum distance that the shape can travel.
    /// - `ignore_origin_penetration`: If true and the shape is already penetrating a collider at the
    /// shape origin, the hit will be ignored and only the next hit will be computed. Otherwise, the initial
    /// hit will be returned.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each hit.
    ///
    /// See also: [SpatialQuery::shape_hits_callback]
    #[allow(clippy::too_many_arguments)]
    pub fn shape_hits_callback(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Vector,
        max_time_of_impact: Scalar,
        ignore_origin_penetration: bool,
        mut query_filter: SpatialQueryFilter,
        mut callback: impl FnMut(ShapeHitData) -> bool,
    ) -> Vec<ShapeHitData> {
        let rotation: Rotation;
        #[cfg(feature = "2d")]
        {
            rotation = Rotation::from_radians(shape_rotation);
        }
        #[cfg(feature = "3d")]
        {
            rotation = Rotation::from(shape_rotation);
        }

        let shape_isometry = utils::make_isometry(origin, &rotation);
        let shape_direction = direction.into();
        let mut hits = Vec::with_capacity(10);

        loop {
            let pipeline_shape = self.as_composite_shape(query_filter.clone());
            let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
                &*self.dispatcher,
                &shape_isometry,
                &shape_direction,
                &pipeline_shape,
                &**shape.get_shape(),
                max_time_of_impact,
                !ignore_origin_penetration,
            );

            if let Some(hit) =
                self.qbvh
                    .traverse_best_first(&mut visitor)
                    .map(|(_, (entity_index, hit))| ShapeHitData {
                        entity: self.entity_from_index(entity_index),
                        time_of_impact: hit.toi,
                        point1: hit.witness1.into(),
                        point2: hit.witness2.into(),
                        normal1: hit.normal1.into(),
                        normal2: hit.normal2.into(),
                    })
            {
                hits.push(hit);
                query_filter.excluded_entities.insert(hit.entity);

                if !callback(hit) {
                    break;
                }
            } else {
                break;
            }
        }

        hits
    }

    /// Finds the [projection](spatial_query#point-projection) of a given point on the closest [collider](Collider).
    /// If one isn't found, `None` is returned.
    ///
    /// ## Arguments
    ///
    /// - `point`: The point that should be projected.
    /// - `solid`: If true and the point is inside of a collider, the projection will be at the point.
    /// Otherwise, the collider will be treated as hollow, and the projection will be at the collider's boundary.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// See also: [SpatialQuery::project_point]
    pub fn project_point(
        &self,
        point: Vector,
        solid: bool,
        query_filter: SpatialQueryFilter,
    ) -> Option<PointProjection> {
        let point = point.into();
        let pipeline_shape = self.as_composite_shape(query_filter);
        let mut visitor =
            PointCompositeShapeProjBestFirstVisitor::new(&pipeline_shape, &point, solid);

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (projection, entity_index))| PointProjection {
                entity: self.entity_from_index(entity_index),
                point: projection.point.into(),
                is_inside: projection.is_inside,
            })
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [collider](Collider)
    /// that contains the given point.
    ///
    /// ## Arguments
    ///
    /// - `point`: The point that intersections are tested against.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// See also: [SpatialQuery::point_intersections]
    pub fn point_intersections(
        &self,
        point: Vector,
        query_filter: SpatialQueryFilter,
    ) -> Vec<Entity> {
        self.point_intersections_callback(point, query_filter, |_| true)
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [collider](Collider)
    /// that contains the given point, calling the given `callback` for each intersection.
    /// The search stops when `callback` returns `false` or all intersections have been found.
    ///
    /// ## Arguments
    ///
    /// - `point`: The point that intersections are tested against.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each intersection.
    ///
    /// See also: [SpatialQuery::point_intersections_callback]
    pub fn point_intersections_callback(
        &self,
        point: Vector,
        query_filter: SpatialQueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) -> Vec<Entity> {
        let point = point.into();

        let mut intersections = vec![];

        let mut leaf_callback = &mut |entity_index: &u32| {
            let entity = self.entity_from_index(*entity_index);
            if let Some((isometry, shape, layers)) = self.colliders.get(&entity) {
                if query_filter.test(entity, *layers) && shape.contains_point(isometry, &point) {
                    intersections.push(entity);
                    return callback(entity);
                }
            }
            true
        };

        let mut visitor = PointIntersectionsVisitor::new(&point, &mut leaf_callback);
        self.qbvh.traverse_depth_first(&mut visitor);

        intersections
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`.
    ///
    /// See also: [SpatialQuery::point_intersections_callback]
    pub fn aabb_intersections_with_aabb(&self, aabb: ColliderAabb) -> Vec<Entity> {
        self.aabb_intersections_with_aabb_callback(aabb, |_| true)
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`, calling `callback` for each intersection.
    /// The search stops when `callback` returns `false` or all intersections have been found.
    ///
    /// See also: [SpatialQuery::aabb_intersections_with_aabb_callback]
    pub fn aabb_intersections_with_aabb_callback(
        &self,
        aabb: ColliderAabb,
        mut callback: impl FnMut(Entity) -> bool,
    ) -> Vec<Entity> {
        let mut intersections = vec![];
        let mut leaf_callback = |entity_index: &u32| {
            let entity = self.entity_from_index(*entity_index);
            intersections.push(entity);
            callback(entity)
        };

        let mut visitor = BoundingVolumeIntersectionsVisitor::new(&aabb, &mut leaf_callback);
        self.qbvh.traverse_depth_first(&mut visitor);

        intersections
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`Collider`]
    /// that is intersecting the given `shape` with a given position and rotation.
    ///
    /// ## Arguments
    ///
    /// - `shape`: The shape that intersections are tested against represented as a [`Collider`].
    /// - `shape_position`: The position of the shape.
    /// - `shape_rotation`: The rotation of the shape.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// See also: [SpatialQuery::shape_intersections]
    pub fn shape_intersections(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        query_filter: SpatialQueryFilter,
    ) -> Vec<Entity> {
        self.shape_intersections_callback(
            shape,
            shape_position,
            shape_rotation,
            query_filter,
            |_| true,
        )
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`Collider`]
    /// that is intersecting the given `shape` with a given position and rotation, calling `callback` for each
    /// intersection. The search stops when `callback` returns `false` or all intersections have been found.
    ///
    /// ## Arguments
    ///
    /// - `shape`: The shape that intersections are tested against represented as a [`Collider`].
    /// - `shape_position`: The position of the shape.
    /// - `shape_rotation`: The rotation of the shape.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `callback`: A callback function called for each intersection.
    ///
    /// See also: [SpatialQuery::shape_intersections_callback]
    pub fn shape_intersections_callback(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        query_filter: SpatialQueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) -> Vec<Entity> {
        let colliders = &self.colliders;
        let rotation: Rotation;
        #[cfg(feature = "2d")]
        {
            rotation = Rotation::from_radians(shape_rotation);
        }
        #[cfg(feature = "3d")]
        {
            rotation = Rotation::from(shape_rotation);
        }

        let shape_isometry = utils::make_isometry(shape_position, &rotation);
        let inverse_shape_isometry = shape_isometry.inverse();

        let dispatcher = &*self.dispatcher;
        let mut intersections = vec![];

        let mut leaf_callback = &mut |entity_index: &u32| {
            let entity = self.entity_from_index(*entity_index);

            if let Some((collider_isometry, collider, layers)) = colliders.get(&entity) {
                if query_filter.test(entity, *layers) {
                    let isometry = inverse_shape_isometry * collider_isometry;

                    if dispatcher.intersection_test(
                        &isometry,
                        &**shape.get_shape(),
                        &**collider.get_shape(),
                    ) == Ok(true)
                    {
                        intersections.push(entity);
                        return callback(entity);
                    }
                }
            }
            true
        };

        let shape_aabb = shape.get_shape().compute_aabb(&shape_isometry);
        let mut visitor = BoundingVolumeIntersectionsVisitor::new(&shape_aabb, &mut leaf_callback);
        self.qbvh.traverse_depth_first(&mut visitor);

        intersections
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

/// The result of a [point projection](spatial_query#point-projection) on a [collider](Collider).
#[derive(Debug, Clone, PartialEq)]
pub struct PointProjection {
    /// The entity of the collider that the point was projected onto.
    pub entity: Entity,
    /// The point where the point was projected.
    pub point: Vector,
    /// True if the point was inside of the collider.
    pub is_inside: bool,
}
