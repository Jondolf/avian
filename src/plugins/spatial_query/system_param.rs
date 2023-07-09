use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*, utils::HashMap};
use parry::query::{
    details::{
        RayCompositeShapeToiAndNormalBestFirstVisitor, TOICompositeShapeShapeBestFirstVisitor,
    },
    point::PointCompositeShapeProjBestFirstVisitor,
    visitors::{
        BoundingVolumeIntersectionsVisitor, PointIntersectionsVisitor, RayIntersectionsVisitor,
    },
};

/// A system parameter for performing [spatial queries](spatial_query).
///
/// ## Methods
///
/// - [Ray casting](spatial_query#ray-casting): [`cast_ray`](SpatialQuery#method.cast_ray),
/// [`ray_hits`](SpatialQuery#method.ray_hits), [`ray_hits_callback`](SpatialQuery#method.ray_hits_callback)
/// - [Shape casting](spatial_query#shape-casting): [`cast_shape`](SpatialQuery#method.cast_shape),
/// [`shape_hits`](SpatialQuery#method.shape_hits), [`shape_hits_callback`](SpatialQuery#method.shape_hits_callback)
/// - [Point projection](spatial_query#point-projection): [`project_point`](SpatialQuery#method.project_point)
/// - [Intersection tests](spatial_query#intersection-tests)
///     - Point intersections: [`point_intersections`](SpatialQuery#method.point_intersections),
/// [`point_intersections_callback`](SpatialQuery#method.point_intersections_callback)
///     - AABB intersections: [`aabb_intersections_with_aabb`](SpatialQuery#method.aabb_intersections_with_aabb),
/// [`aabb_intersections_with_aabb_callback`](SpatialQuery#method.aabb_intersections_with_aabb_callback)
///     - Shape intersections: [`shape_intersections`](SpatialQuery#method.shape_intersections)
/// [`shape_intersections_callback`](SpatialQuery#method.shape_intersections_callback)
///
/// For simple ray casts and shape casts, consider using the [`RayCaster`] and [`ShapeCaster`] components that
/// provide a more ECS-based approach and perform casts on every frame.
///
/// ## Ray casting example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn print_hits(spatial_query: SpatialQuery) {
///     // Cast ray and print first hit
///     if let Some(first_hit) = spatial_query.cast_ray(
///         Vec3::ZERO,                    // Origin
///         Vec3::X,                       // Direction
///         100.0,                         // Maximum time of impact (travel distance)
///         true,                          // Does the ray treat colliders as "solid"
///         SpatialQueryFilter::default(), // Query filter
///     ) {
///         println!("First hit: {:?}", first_hit);
///     }
///
///     // Cast ray and get up to 20 hits
///     let hits = spatial_query.ray_hits(
///         Vec3::ZERO,                    // Origin
///         Vec3::X,                       // Direction
///         100.0,                         // Maximum time of impact (travel distance)
///         20,                            // Maximum number of hits
///         true,                          // Does the ray treat colliders as "solid"
///         SpatialQueryFilter::default(), // Query filter
///     );
///
///     // Print hits
///     for hit in hits.iter() {
///         println!("Hit: {:?}", hit);
///     }
/// }
/// ```
#[derive(SystemParam)]
pub struct SpatialQuery<'w, 's> {
    pub(crate) colliders: Query<
        'w,
        's,
        (
            Entity,
            &'static Position,
            &'static Rotation,
            &'static Collider,
            Option<&'static CollisionLayers>,
        ),
    >,
    /// The [`SpatialQueryPipeline`].
    pub query_pipeline: ResMut<'w, SpatialQueryPipeline>,
}

impl<'w, 's> SpatialQuery<'w, 's> {
    pub(crate) fn get_collider_hash_map(
        &self,
    ) -> HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape, CollisionLayers)> {
        self.colliders
            .iter()
            .map(|(entity, position, rotation, collider, layers)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        &**collider.get_shape(),
                        layers.map_or(CollisionLayers::default(), |l| *l),
                    ),
                )
            })
            .collect()
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
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Cast ray and print first hit
    ///     if let Some(first_hit) = spatial_query.cast_ray(
    ///         Vec3::ZERO,                    // Origin
    ///         Vec3::X,                       // Direction
    ///         100.0,                         // Maximum time of impact (travel distance)
    ///         true,                          // Does the ray treat colliders as "solid"
    ///         SpatialQueryFilter::default(), // Query filter
    ///     ) {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    pub fn cast_ray(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: Scalar,
        solid: bool,
        query_filter: SpatialQueryFilter,
    ) -> Option<RayHitData> {
        let colliders = self.get_collider_hash_map();
        let pipeline_shape = self
            .query_pipeline
            .as_composite_shape(&colliders, query_filter);
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
            .map(|(_, (entity_index, hit))| RayHitData {
                entity: Entity::from_raw(entity_index),
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
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Cast ray and get hits
    ///     let hits = spatial_query.ray_hits(
    ///         Vec3::ZERO,                    // Origin
    ///         Vec3::X,                       // Direction
    ///         100.0,                         // Maximum time of impact (travel distance)
    ///         20,                            // Maximum number of hits
    ///         true,                          // Does the ray treat colliders as "solid"
    ///         SpatialQueryFilter::default(), // Query filter
    ///     );
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
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
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     let mut hits = vec![];
    ///
    ///     // Cast ray and get all hits
    ///     spatial_query.ray_hits_callback(
    ///         Vec3::ZERO,                    // Origin
    ///         Vec3::X,                       // Direction
    ///         100.0,                         // Maximum time of impact (travel distance)
    ///         true,                          // Does the ray treat colliders as "solid"
    ///         SpatialQueryFilter::default(), // Query filter
    ///         |hit| {                        // Callback function
    ///             hits.push(hit);
    ///             true
    ///         },
    ///     );
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    pub fn ray_hits_callback(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: Scalar,
        solid: bool,
        query_filter: SpatialQueryFilter,
        mut callback: impl FnMut(RayHitData) -> bool,
    ) -> Vec<RayHitData> {
        let colliders = self.get_collider_hash_map();

        let mut hits = Vec::with_capacity(10);
        let ray = parry::query::Ray::new(origin.into(), direction.into());

        let mut leaf_callback = &mut |entity_index: &u32| {
            let entity = Entity::from_raw(*entity_index);
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
        self.query_pipeline.qbvh.traverse_depth_first(&mut visitor);

        hits
    }

    /// Casts a [shape](spatial_query#shape-casting) with a given rotation and computes the closest [hit](ShapeHit)
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
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Cast ray and print first hit
    ///     if let Some(first_hit) = spatial_query.cast_shape(
    ///         &Collider::ball(0.5),          // Shape
    ///         Vec3::ZERO,                    // Origin
    ///         Quat::default(),               // Shape rotation
    ///         Vec3::X,                       // Direction
    ///         100.0,                         // Maximum time of impact (travel distance)
    ///         true,                          // Should initial penetration at the origin be ignored
    ///         SpatialQueryFilter::default(), // Query filter
    ///     ) {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
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
        let colliders = self.get_collider_hash_map();
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
        let pipeline_shape = self
            .query_pipeline
            .as_composite_shape(&colliders, query_filter);
        let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
            &*self.query_pipeline.dispatcher,
            &shape_isometry,
            &shape_direction,
            &pipeline_shape,
            &**shape.get_shape(),
            max_time_of_impact,
            ignore_origin_penetration,
        );

        self.query_pipeline
            .qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (entity_index, hit))| ShapeHitData {
                entity: Entity::from_raw(entity_index),
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
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     let mut hits = vec![];
    ///
    ///     // Cast shape and get all hits
    ///     spatial_query.shape_hits(
    ///         &Collider::ball(0.5),          // Shape
    ///         Vec3::ZERO,                    // Origin
    ///         Quat::default(),               // Shape rotation
    ///         Vec3::X,                       // Direction
    ///         100.0,                         // Maximum time of impact (travel distance)
    ///         20,                            // Max hits
    ///         true,                          // Should initial penetration at the origin be ignored
    ///         SpatialQueryFilter::default(), // Query filter
    ///     );
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
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
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     let mut hits = vec![];
    ///
    ///     // Cast shape and get all hits
    ///     spatial_query.shape_hits_callback(
    ///         &Collider::ball(0.5),          // Shape
    ///         Vec3::ZERO,                    // Origin
    ///         Quat::default(),               // Shape rotation
    ///         Vec3::X,                       // Direction
    ///         100.0,                         // Maximum time of impact (travel distance)
    ///         true,                          // Should initial penetration at the origin be ignored
    ///         SpatialQueryFilter::default(), // Query filter
    ///         |hit| {                        // Callback function
    ///             hits.push(hit);
    ///             true
    ///         },
    ///     );
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
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
        let colliders = self.get_collider_hash_map();
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
            let pipeline_shape = self
                .query_pipeline
                .as_composite_shape(&colliders, query_filter.clone());
            let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
                &*self.query_pipeline.dispatcher,
                &shape_isometry,
                &shape_direction,
                &pipeline_shape,
                &**shape.get_shape(),
                max_time_of_impact,
                ignore_origin_penetration,
            );

            if let Some(hit) = self
                .query_pipeline
                .qbvh
                .traverse_best_first(&mut visitor)
                .map(|(_, (entity_index, hit))| ShapeHitData {
                    entity: Entity::from_raw(entity_index),
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
    /// fn print_point_projection(spatial_query: SpatialQuery) {
    ///     // Project a point and print the result
    ///     if let Some(projection) = spatial_query.project_point(
    ///         Vec3::ZERO,                    // Point
    ///         true,                          // Are colliders treated as "solid"
    ///         SpatialQueryFilter::default(), // Query filter
    ///     ) {
    ///         println!("Projection: {:?}", projection);
    ///     }
    /// }
    /// ```
    pub fn project_point(
        &self,
        point: Vector,
        solid: bool,
        query_filter: SpatialQueryFilter,
    ) -> Option<PointProjection> {
        let point = point.into();
        let colliders = self.get_collider_hash_map();
        let pipeline_shape = self
            .query_pipeline
            .as_composite_shape(&colliders, query_filter);
        let mut visitor =
            PointCompositeShapeProjBestFirstVisitor::new(&pipeline_shape, &point, solid);

        self.query_pipeline
            .qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (projection, entity_index))| PointProjection {
                entity: Entity::from_raw(entity_index),
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
    /// fn print_point_intersections(spatial_query: SpatialQuery) {
    ///     let intersections =
    ///         spatial_query.point_intersections(Vec3::ZERO, SpatialQueryFilter::default());
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {:?}", entity);
    ///     }
    /// }
    /// ```
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
    /// fn print_point_intersections(spatial_query: SpatialQuery) {
    ///     let mut intersections = vec![];
    ///     
    ///     spatial_query.point_intersections_callback(
    ///         Vec3::ZERO,                     // Point
    ///         SpatialQueryFilter::default(),  // Query filter
    ///         |entity| {                      // Callback function
    ///             intersections.push(entity);
    ///             true
    ///         },
    ///     );
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {:?}", entity);
    ///     }
    /// }
    /// ```
    pub fn point_intersections_callback(
        &self,
        point: Vector,
        query_filter: SpatialQueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) -> Vec<Entity> {
        let point = point.into();

        let mut intersections = vec![];

        let mut leaf_callback = &mut |entity_index: &u32| {
            let entity = Entity::from_raw(*entity_index);
            if let Ok((entity, position, rotation, shape, layers)) = self.colliders.get(entity) {
                let isometry = utils::make_isometry(position.0, rotation);
                if query_filter.test(entity, layers.map_or(CollisionLayers::default(), |l| *l))
                    && shape.contains_point(&isometry, &point)
                {
                    intersections.push(entity);
                    return callback(entity);
                }
            }
            true
        };

        let mut visitor = PointIntersectionsVisitor::new(&point, &mut leaf_callback);
        self.query_pipeline.qbvh.traverse_depth_first(&mut visitor);

        intersections
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`.
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
    /// fn print_aabb_intersections(spatial_query: SpatialQuery) {
    ///     let aabb = Collider::ball(0.5).compute_aabb(Vector::ZERO, Quat::default());
    ///     let intersections = spatial_query.aabb_intersections_with_aabb(aabb);
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {:?}", entity);
    ///     }
    /// }
    /// ```
    pub fn aabb_intersections_with_aabb(&self, aabb: ColliderAabb) -> Vec<Entity> {
        self.aabb_intersections_with_aabb_callback(aabb, |_| true)
    }

    /// An [intersection test](spatial_query#intersection-tests) that finds all entities with a [`ColliderAabb`]
    /// that is intersecting the given `aabb`, calling `callback` for each intersection.
    /// The search stops when `callback` returns `false` or all intersections have been found.
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
    /// fn print_aabb_intersections(spatial_query: SpatialQuery) {
    ///     let mut intersections = vec![];
    ///
    ///     spatial_query.aabb_intersections_with_aabb_callback(
    ///         Collider::ball(0.5).compute_aabb(Vector::ZERO, Quat::default()),
    ///         |entity| {
    ///             intersections.push(entity);
    ///             true
    ///         }
    ///     );
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {:?}", entity);
    ///     }
    /// }
    /// ```
    pub fn aabb_intersections_with_aabb_callback(
        &self,
        aabb: ColliderAabb,
        mut callback: impl FnMut(Entity) -> bool,
    ) -> Vec<Entity> {
        let mut intersections = vec![];
        let mut leaf_callback = |entity_index: &u32| {
            let entity = Entity::from_raw(*entity_index);
            intersections.push(entity);
            callback(entity)
        };

        let mut visitor = BoundingVolumeIntersectionsVisitor::new(&aabb, &mut leaf_callback);
        self.query_pipeline.qbvh.traverse_depth_first(&mut visitor);

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
    /// fn print_shape_intersections(spatial_query: SpatialQuery) {
    ///     let intersections = spatial_query.shape_intersections(
    ///         &Collider::ball(0.5),          // Shape
    ///         Vec3::ZERO,                    // Shape position
    ///         Quat::default(),               // Shape rotation
    ///         SpatialQueryFilter::default(), // Query filter
    ///     );
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {:?}", entity);
    ///     }
    /// }
    /// ```
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
    /// fn print_shape_intersections(spatial_query: SpatialQuery) {
    ///     let mut intersections = vec![];
    ///
    ///     spatial_query.shape_intersections_callback(
    ///         &Collider::ball(0.5),          // Shape
    ///         Vec3::ZERO,                    // Shape position
    ///         Quat::default(),               // Shape rotation
    ///         SpatialQueryFilter::default(), // Query filter
    ///         |entity| {                     // Callback function
    ///             intersections.push(entity);
    ///             true
    ///         },
    ///     );
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {:?}", entity);
    ///     }
    /// }
    /// ```
    pub fn shape_intersections_callback(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        query_filter: SpatialQueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) -> Vec<Entity> {
        let colliders = self.get_collider_hash_map();
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

        let dispatcher = &*self.query_pipeline.dispatcher;
        let mut intersections = vec![];

        let mut leaf_callback = &mut |entity_index: &u32| {
            let entity = Entity::from_raw(*entity_index);

            if let Some((collider_isometry, collider_shape, layers)) = colliders.get(&entity) {
                if query_filter.test(entity, *layers) {
                    let isometry = inverse_shape_isometry * collider_isometry;

                    if dispatcher.intersection_test(
                        &isometry,
                        &**shape.get_shape(),
                        &**collider_shape,
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
        self.query_pipeline.qbvh.traverse_depth_first(&mut visitor);

        intersections
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
