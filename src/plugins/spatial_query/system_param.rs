use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*};

/// A system parameter for performing [spatial queries](spatial_query).
///
/// ## Methods
///
/// - [Raycasting](spatial_query#raycasting): [`cast_ray`](SpatialQuery::cast_ray),
/// [`ray_hits`](SpatialQuery::ray_hits), [`ray_hits_callback`](SpatialQuery::ray_hits_callback)
/// - [Shapecasting](spatial_query#shapecasting): [`cast_shape`](SpatialQuery::cast_shape),
/// [`shape_hits`](SpatialQuery::shape_hits), [`shape_hits_callback`](SpatialQuery::shape_hits_callback)
/// - [Point projection](spatial_query#point-projection): [`project_point`](SpatialQuery::project_point)
/// - [Intersection tests](spatial_query#intersection-tests)
///     - Point intersections: [`point_intersections`](SpatialQuery::point_intersections),
/// [`point_intersections_callback`](SpatialQuery::point_intersections_callback)
///     - AABB intersections: [`aabb_intersections_with_aabb`](SpatialQuery::aabb_intersections_with_aabb),
/// [`aabb_intersections_with_aabb_callback`](SpatialQuery::aabb_intersections_with_aabb_callback)
///     - Shape intersections: [`shape_intersections`](SpatialQuery::shape_intersections)
/// [`shape_intersections_callback`](SpatialQuery::shape_intersections_callback)
///
/// For simple raycasts and shapecasts, consider using the [`RayCaster`] and [`ShapeCaster`] components that
/// provide a more ECS-based approach and perform casts on every frame.
///
/// ## Raycasting example
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
///         Direction3d::X,                // Direction
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
///         Direction3d::X,                // Direction
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
    pub(crate) added_colliders: Query<'w, 's, Entity, Added<Collider>>,
    /// The [`SpatialQueryPipeline`].
    pub query_pipeline: ResMut<'w, SpatialQueryPipeline>,
}

impl<'w, 's> SpatialQuery<'w, 's> {
    /// Updates the colliders in the pipeline. This is done automatically once per physics frame in
    /// [`PhysicsStepSet::SpatialQuery`], but if you modify colliders or their positions before that, you can
    /// call this to make sure the data is up to date when performing spatial queries using [`SpatialQuery`].
    pub fn update_pipeline(&mut self) {
        self.query_pipeline
            .update(self.colliders.iter(), self.added_colliders.iter());
    }

    /// Casts a [ray](spatial_query#raycasting) and computes the closest [hit](RayHitData) with a collider.
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
    ///         Direction3d::X,                // Direction
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
        direction: Dir,
        max_time_of_impact: Scalar,
        solid: bool,
        query_filter: SpatialQueryFilter,
    ) -> Option<RayHitData> {
        self.query_pipeline
            .cast_ray(origin, direction, max_time_of_impact, solid, query_filter)
    }

    /// Casts a [ray](spatial_query#raycasting) and computes the closest [hit](RayHitData) with a collider.
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
    /// - `predicate`: A function with which the colliders are filtered. Given the Entity it should return false, if the
    /// entity should be ignored.
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
    /// #[derive(Component)]
    /// struct Invisible;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery, query: Query<&Invisible>) {
    ///     // Cast ray and print first hit
    ///     if let Some(first_hit) = spatial_query.cast_ray_predicate(
    ///         Vec3::ZERO,                    // Origin
    ///         Direction3d::X,                // Direction
    ///         100.0,                         // Maximum time of impact (travel distance)
    ///         true,                          // Does the ray treat colliders as "solid"
    ///         SpatialQueryFilter::default(), // Query filter
    ///         &|entity| {                    // Predicate
    ///             // Skip entities with the `Invisible` component.
    ///             !query.contains(entity)
    ///         }
    ///     ) {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    pub fn cast_ray_predicate(
        &self,
        origin: Vector,
        direction: Dir,
        max_time_of_impact: Scalar,
        solid: bool,
        query_filter: SpatialQueryFilter,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<RayHitData> {
        self.query_pipeline.cast_ray_predicate(
            origin,
            direction,
            max_time_of_impact,
            solid,
            query_filter,
            predicate,
        )
    }

    /// Casts a [ray](spatial_query#raycasting) and computes all [hits](RayHitData) until `max_hits` is reached.
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
    ///         Direction3d::X,                // Direction
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
        direction: Dir,
        max_time_of_impact: Scalar,
        max_hits: u32,
        solid: bool,
        query_filter: SpatialQueryFilter,
    ) -> Vec<RayHitData> {
        self.query_pipeline.ray_hits(
            origin,
            direction,
            max_time_of_impact,
            max_hits,
            solid,
            query_filter,
        )
    }

    /// Casts a [ray](spatial_query#raycasting) and computes all [hits](RayHitData), calling the given `callback`
    /// for each hit. The raycast stops when `callback` returns false or all hits have been found.
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
    ///         Direction3d::X,                // Direction
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
        direction: Dir,
        max_time_of_impact: Scalar,
        solid: bool,
        query_filter: SpatialQueryFilter,
        callback: impl FnMut(RayHitData) -> bool,
    ) {
        self.query_pipeline.ray_hits_callback(
            origin,
            direction,
            max_time_of_impact,
            solid,
            query_filter,
            callback,
        )
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes the closest [hit](ShapeHits)
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
    ///         &Collider::sphere(0.5),          // Shape
    ///         Vec3::ZERO,                      // Origin
    ///         Quat::default(),                 // Shape rotation
    ///         Direction3d::X,                  // Direction
    ///         100.0,                           // Maximum time of impact (travel distance)
    ///         true,                            // Should initial penetration at the origin be ignored
    ///         SpatialQueryFilter::default(),   // Query filter
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
        direction: Dir,
        max_time_of_impact: Scalar,
        ignore_origin_penetration: bool,
        query_filter: SpatialQueryFilter,
    ) -> Option<ShapeHitData> {
        self.query_pipeline.cast_shape(
            shape,
            origin,
            shape_rotation,
            direction,
            max_time_of_impact,
            ignore_origin_penetration,
            query_filter,
        )
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes computes all [hits](ShapeHitData)
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
    ///     // Cast shape and get all hits
    ///     let hits = spatial_query.shape_hits(
    ///         &Collider::sphere(0.5),          // Shape
    ///         Vec3::ZERO,                      // Origin
    ///         Quat::default(),                 // Shape rotation
    ///         Direction3d::X,                  // Direction
    ///         100.0,                           // Maximum time of impact (travel distance)
    ///         20,                              // Max hits
    ///         true,                            // Should initial penetration at the origin be ignored
    ///         SpatialQueryFilter::default(),   // Query filter
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
        direction: Dir,
        max_time_of_impact: Scalar,
        max_hits: u32,
        ignore_origin_penetration: bool,
        query_filter: SpatialQueryFilter,
    ) -> Vec<ShapeHitData> {
        self.query_pipeline.shape_hits(
            shape,
            origin,
            shape_rotation,
            direction,
            max_time_of_impact,
            max_hits,
            ignore_origin_penetration,
            query_filter,
        )
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of the time of impact, calling the given `callback` for each hit. The shapecast stops when
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
    ///         &Collider::sphere(0.5),          // Shape
    ///         Vec3::ZERO,                      // Origin
    ///         Quat::default(),                 // Shape rotation
    ///         Direction3d::X,                  // Direction
    ///         100.0,                           // Maximum time of impact (travel distance)
    ///         true,                            // Should initial penetration at the origin be ignored
    ///         SpatialQueryFilter::default(),   // Query filter
    ///         |hit| {                          // Callback function
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
        direction: Dir,
        max_time_of_impact: Scalar,
        ignore_origin_penetration: bool,
        query_filter: SpatialQueryFilter,
        callback: impl FnMut(ShapeHitData) -> bool,
    ) {
        self.query_pipeline.shape_hits_callback(
            shape,
            origin,
            shape_rotation,
            direction,
            max_time_of_impact,
            ignore_origin_penetration,
            query_filter,
            callback,
        )
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
        self.query_pipeline
            .project_point(point, solid, query_filter)
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
        self.query_pipeline.point_intersections(point, query_filter)
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
        callback: impl FnMut(Entity) -> bool,
    ) {
        self.query_pipeline
            .point_intersections_callback(point, query_filter, callback)
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
    ///     let aabb = Collider::sphere(0.5).aabb(Vec3::ZERO, Quat::default());
    ///     let intersections = spatial_query.aabb_intersections_with_aabb(aabb);
    ///
    ///     for entity in intersections.iter() {
    ///         println!("Entity: {:?}", entity);
    ///     }
    /// }
    /// ```
    pub fn aabb_intersections_with_aabb(&self, aabb: ColliderAabb) -> Vec<Entity> {
        self.query_pipeline.aabb_intersections_with_aabb(aabb)
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
    ///         Collider::sphere(0.5).aabb(Vec3::ZERO, Quat::default()),
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
        callback: impl FnMut(Entity) -> bool,
    ) {
        self.query_pipeline
            .aabb_intersections_with_aabb_callback(aabb, callback)
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
    ///         &Collider::sphere(0.5),          // Shape
    ///         Vec3::ZERO,                      // Shape position
    ///         Quat::default(),                 // Shape rotation
    ///         SpatialQueryFilter::default(),   // Query filter
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
        self.query_pipeline
            .shape_intersections(shape, shape_position, shape_rotation, query_filter)
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
    ///         &Collider::sphere(0.5),          // Shape
    ///         Vec3::ZERO,                      // Shape position
    ///         Quat::default(),                 // Shape rotation
    ///         SpatialQueryFilter::default(),   // Query filter
    ///         |entity| {                       // Callback function
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
        callback: impl FnMut(Entity) -> bool,
    ) {
        self.query_pipeline.shape_intersections_callback(
            shape,
            shape_position,
            shape_rotation,
            query_filter,
            callback,
        )
    }
}
