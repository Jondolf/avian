use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*};

/// A system parameter for performing [spatial queries](spatial_query).
///
/// ## Methods
///
/// - [Raycasting](spatial_query#raycasting): [`cast_ray`](SpatialQuery::cast_ray), [`cast_ray_predicate`](SpatialQuery::cast_ray_predicate),
///   [`ray_hits`](SpatialQuery::ray_hits), [`ray_hits_callback`](SpatialQuery::ray_hits_callback)
/// - [Shapecasting](spatial_query#shapecasting): [`cast_shape`](SpatialQuery::cast_shape), [`cast_shape_predicate`](SpatialQuery::cast_shape_predicate),
///   [`shape_hits`](SpatialQuery::shape_hits), [`shape_hits_callback`](SpatialQuery::shape_hits_callback)
/// - [Point projection](spatial_query#point-projection): [`project_point`](SpatialQuery::project_point) and [`project_point_predicate`](SpatialQuery::project_point_predicate)
/// - [Intersection tests](spatial_query#intersection-tests)
///     - Point intersections: [`point_intersections`](SpatialQuery::point_intersections),
///       [`point_intersections_callback`](SpatialQuery::point_intersections_callback)
///     - AABB intersections: [`aabb_intersections_with_aabb`](SpatialQuery::aabb_intersections_with_aabb),
///       [`aabb_intersections_with_aabb_callback`](SpatialQuery::aabb_intersections_with_aabb_callback)
///     - Shape intersections: [`shape_intersections`](SpatialQuery::shape_intersections)
///       [`shape_intersections_callback`](SpatialQuery::shape_intersections_callback)
///
/// For simple raycasts and shapecasts, consider using the [`RayCaster`] and [`ShapeCaster`] components that
/// provide a more ECS-based approach and perform casts on every frame.
///
/// ## Raycasting example
///
/// ```
/// # #[cfg(feature = "2d")]
/// # use avian2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn print_hits(spatial_query: SpatialQuery) {
///     // Configuration for the ray cast
///     let config = RayCastConfig {
///        max_distance: 100.0,
///       ..default()
///     };
///
///     // Ray origin and direction
///     let origin = Vec3::ZERO;
///     let direction = Dir3::X;
///
///     // Cast ray and print first hit
///     if let Some(first_hit) = spatial_query.cast_ray(origin, direction, &config) {
///         println!("First hit: {:?}", first_hit);
///     }
///
///     // Cast ray and get up to 20 hits
///     let hits = spatial_query.ray_hits(origin, direction, 20, &config);
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
    /// - `config`: A [`RayCastConfig`] that determines the behavior of the cast.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Configuration for the ray cast
    ///     let config = RayCastConfig {
    ///        max_distance: 100.0,
    ///       ..default()
    ///     };
    ///
    ///     // Ray origin and direction
    ///     let origin = Vec3::ZERO;
    ///     let direction = Dir3::X;
    ///
    ///     // Cast ray and print first hit
    ///     if let Some(first_hit) = spatial_query.cast_ray(origin, direction, &config) {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::cast_ray_predicate`]
    /// - [`SpatialQuery::ray_hits`]
    /// - [`SpatialQuery::ray_hits_callback`]
    pub fn cast_ray(
        &self,
        origin: Vector,
        direction: Dir,
        config: &RayCastConfig,
    ) -> Option<RayHitData> {
        self.query_pipeline.cast_ray(origin, direction, config)
    }

    /// Casts a [ray](spatial_query#raycasting) and computes the closest [hit](RayHitData) with a collider.
    /// If there are no hits, `None` is returned.
    ///
    /// ## Arguments
    ///
    /// - `origin`: Where the ray is cast from.
    /// - `direction`: What direction the ray is cast in.
    /// - `config`: A [`RayCastConfig`] that determines the behavior of the cast.
    /// - `predicate`: A function called on each entity hit by the ray. The ray keeps travelling until the predicate returns `false`.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// #[derive(Component)]
    /// struct Invisible;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery, query: Query<&Invisible>) {
    ///     // Configuration for the ray cast
    ///     let config = RayCastConfig {
    ///        max_distance: 100.0,
    ///       ..default()
    ///     };
    ///
    ///     // Ray origin and direction
    ///     let origin = Vec3::ZERO;
    ///     let direction = Dir3::X;
    ///
    ///     // Cast ray and get the first hit that matches the predicate
    ///     let hit = spatial_query.cast_ray_predicate(origin, direction, &config, &|entity| {
    ///         // Skip entities with the `Invisible` component.
    ///         !query.contains(entity)
    ///     });
    ///
    ///     // Print first hit
    ///     if let Some(first_hit) = hit {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::cast_ray`]
    /// - [`SpatialQuery::ray_hits`]
    /// - [`SpatialQuery::ray_hits_callback`]
    pub fn cast_ray_predicate(
        &self,
        origin: Vector,
        direction: Dir,
        config: &RayCastConfig,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<RayHitData> {
        self.query_pipeline
            .cast_ray_predicate(origin, direction, config, predicate)
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
    /// - `max_hits`: The maximum number of hits. Additional hits will be missed.
    /// - `config`: A [`RayCastConfig`] that determines the behavior of the cast.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Configuration for the ray cast
    ///     let config = RayCastConfig {
    ///        max_distance: 100.0,
    ///       ..default()
    ///     };
    ///
    ///     // Ray origin and direction
    ///     let origin = Vec3::ZERO;
    ///     let direction = Dir3::X;
    ///
    ///     // Cast ray and get up to 20 hits
    ///     let hits = spatial_query.ray_hits(origin, direction, 20, &config);
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::cast_ray`]
    /// - [`SpatialQuery::cast_ray_predicate`]
    /// - [`SpatialQuery::ray_hits_callback`]
    pub fn ray_hits(
        &self,
        origin: Vector,
        direction: Dir,
        max_hits: u32,
        config: &RayCastConfig,
    ) -> Vec<RayHitData> {
        self.query_pipeline
            .ray_hits(origin, direction, max_hits, config)
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
    /// - `config`: A [`RayCastConfig`] that determines the behavior of the cast.
    /// - `callback`: A callback function called for each hit.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Configuration for the ray cast
    ///     let config = RayCastConfig {
    ///        max_distance: 100.0,
    ///       ..default()
    ///     };
    ///
    ///     // Ray origin and direction
    ///     let origin = Vec3::ZERO;
    ///     let direction = Dir3::X;
    ///
    ///     // Cast ray and get all hits
    ///     let mut hits = vec![];
    ///     spatial_query.ray_hits_callback(origin, direction, 20, &config, |hit| {
    ///         hits.push(hit);
    ///         true
    ///     });
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::cast_ray`]
    /// - [`SpatialQuery::cast_ray_predicate`]
    /// - [`SpatialQuery::ray_hits`]
    pub fn ray_hits_callback(
        &self,
        origin: Vector,
        direction: Dir,
        config: &RayCastConfig,
        callback: impl FnMut(RayHitData) -> bool,
    ) {
        self.query_pipeline
            .ray_hits_callback(origin, direction, config, callback)
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes the closest [hit](ShapeHitData)
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
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Configuration for the shape cast
    ///     let config = ShapeCastConfig {
    ///        max_distance: 100.0,
    ///       ..default()
    ///     };
    ///
    ///     // Shape properties
    ///     let shape = Collider::sphere(0.5);
    ///     let origin = Vec3::ZERO;
    ///     let rotation = Quat::default();
    ///     let direction = Dir3::X;
    ///
    ///     // Cast shape and print first hit
    ///     if let Some(first_hit) = spatial_query.cast_shape(&shape, origin, rotation, direction, &config)
    ///     {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::cast_shape_predicate`]
    /// - [`SpatialQuery::shape_hits`]
    /// - [`SpatialQuery::shape_hits_callback`]
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        config: &ShapeCastConfig,
    ) -> Option<ShapeHitData> {
        self.query_pipeline
            .cast_shape(shape, origin, shape_rotation, direction, config)
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes the closest [hit](ShapeHitData)
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
    /// - `predicate`: A function called on each entity hit by the shape. The shape keeps travelling until the predicate returns `false`.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// #[derive(Component)]
    /// struct Invisible;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery, query: Query<&Invisible>) {
    ///     // Configuration for the shape cast
    ///     let config = ShapeCastConfig {
    ///        max_distance: 100.0,
    ///       ..default()
    ///     };
    ///
    ///     // Shape properties
    ///     let shape = Collider::sphere(0.5);
    ///     let origin = Vec3::ZERO;
    ///     let rotation = Quat::default();
    ///     let direction = Dir3::X;
    ///
    ///     // Cast shape and get the first hit that matches the predicate
    ///     let hit = spatial_query.cast_shape(&shape, origin, rotation, direction, &config, &|entity| {
    ///        // Skip entities with the `Invisible` component.
    ///        !query.contains(entity)
    ///     });
    ///
    ///     // Print first hit
    ///     if let Some(first_hit) = hit {
    ///         println!("First hit: {:?}", first_hit);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::cast_ray`]
    /// - [`SpatialQuery::ray_hits`]
    /// - [`SpatialQuery::ray_hits_callback`]
    pub fn cast_shape_predicate(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        config: &ShapeCastConfig,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<ShapeHitData> {
        self.query_pipeline.cast_shape_predicate(
            shape,
            origin,
            shape_rotation,
            direction,
            config,
            predicate,
        )
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of distance until `max_hits` is reached.
    ///
    /// ## Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `max_hits`: The maximum number of hits. Additional hits will be missed.
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `callback`: A callback function called for each hit.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Configuration for the shape cast
    ///     let config = ShapeCastConfig {
    ///        max_distance: 100.0,
    ///       ..default()
    ///     };
    ///
    ///     // Shape properties
    ///     let shape = Collider::sphere(0.5);
    ///     let origin = Vec3::ZERO;
    ///     let rotation = Quat::default();
    ///     let direction = Dir3::X;
    ///
    ///     // Cast shape and get up to 20 hits
    ///     let hits = spatial_query.cast_shape(&shape, origin, rotation, direction, 20, &config);
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::cast_shape`]
    /// - [`SpatialQuery::cast_shape_predicate`]
    /// - [`SpatialQuery::shape_hits_callback`]
    #[allow(clippy::too_many_arguments)]
    pub fn shape_hits(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        max_hits: u32,
        config: &ShapeCastConfig,
    ) -> Vec<ShapeHitData> {
        self.query_pipeline
            .shape_hits(shape, origin, shape_rotation, direction, max_hits, config)
    }

    /// Casts a [shape](spatial_query#shapecasting) with a given rotation and computes computes all [hits](ShapeHitData)
    /// in the order of distance, calling the given `callback` for each hit. The shapecast stops when
    /// `callback` returns false or all hits have been found.
    ///
    /// ## Arguments
    ///
    /// - `shape`: The shape being cast represented as a [`Collider`].
    /// - `origin`: Where the shape is cast from.
    /// - `shape_rotation`: The rotation of the shape being cast.
    /// - `direction`: What direction the shape is cast in.
    /// - `config`: A [`ShapeCastConfig`] that determines the behavior of the cast.
    /// - `callback`: A callback function called for each hit.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_hits(spatial_query: SpatialQuery) {
    ///     // Configuration for the shape cast
    ///     let config = ShapeCastConfig {
    ///        max_distance: 100.0,
    ///       ..default()
    ///     };
    ///
    ///     // Shape properties
    ///     let shape = Collider::sphere(0.5);
    ///     let origin = Vec3::ZERO;
    ///     let rotation = Quat::default();
    ///     let direction = Dir3::X;
    ///
    ///     // Cast shape and get up to 20 hits
    ///     let mut hits = vec![];
    ///     spatial_query.shape_hits_callback(&shape, origin, rotation, direction, 20, &config, |hit| {
    ///         hits.push(hit);
    ///         true
    ///     });
    ///
    ///     // Print hits
    ///     for hit in hits.iter() {
    ///         println!("Hit: {:?}", hit);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::cast_shape`]
    /// - [`SpatialQuery::cast_shape_predicate`]
    /// - [`SpatialQuery::shape_hits`]
    #[allow(clippy::too_many_arguments)]
    pub fn shape_hits_callback(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: RotationValue,
        direction: Dir,
        config: &ShapeCastConfig,
        callback: impl FnMut(ShapeHitData) -> bool,
    ) {
        self.query_pipeline.shape_hits_callback(
            shape,
            origin,
            shape_rotation,
            direction,
            config,
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
    ///   Otherwise, the collider will be treated as hollow, and the projection will be at the collider's boundary.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
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
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::project_point_predicate`]
    pub fn project_point(
        &self,
        point: Vector,
        solid: bool,
        query_filter: &SpatialQueryFilter,
    ) -> Option<PointProjection> {
        self.query_pipeline
            .project_point(point, solid, query_filter)
    }

    /// Finds the [projection](spatial_query#point-projection) of a given point on the closest [collider](Collider).
    /// If one isn't found, `None` is returned.
    ///
    /// ## Arguments
    ///
    /// - `point`: The point that should be projected.
    /// - `solid`: If true and the point is inside of a collider, the projection will be at the point.
    ///   Otherwise, the collider will be treated as hollow, and the projection will be at the collider's boundary.
    /// - `query_filter`: A [`SpatialQueryFilter`] that determines which colliders are taken into account in the query.
    /// - `predicate`: A function for filtering which entities are considered in the query. The projection will be on the closest collider that passes the predicate.
    ///
    /// ## Example
    ///
    /// ```
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
    ///
    /// #[derive(Component)]
    /// struct Invisible;
    ///
    /// # #[cfg(all(feature = "3d", feature = "f32"))]
    /// fn print_point_projection(spatial_query: SpatialQuery, query: Query<&Invisible>) {
    ///     // Project a point and print the result
    ///     if let Some(projection) = spatial_query.project_point_predicate(
    ///         Vec3::ZERO,                    // Point
    ///         true,                          // Are colliders treated as "solid"
    ///         SpatialQueryFilter::default(), // Query filter
    ///         &|entity| {                    // Predicate
    ///             // Skip entities with the `Invisible` component.
    ///             !query.contains(entity)
    ///         }
    ///     ) {
    ///         println!("Projection: {:?}", projection);
    ///     }
    /// }
    /// ```
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::project_point`]
    pub fn project_point_predicate(
        &self,
        point: Vector,
        solid: bool,
        query_filter: &SpatialQueryFilter,
        predicate: &dyn Fn(Entity) -> bool,
    ) -> Option<PointProjection> {
        self.query_pipeline
            .project_point_predicate(point, solid, query_filter, predicate)
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
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
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
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::point_intersections_callback`]
    pub fn point_intersections(
        &self,
        point: Vector,
        query_filter: &SpatialQueryFilter,
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
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
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
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::point_intersections`]
    pub fn point_intersections_callback(
        &self,
        point: Vector,
        query_filter: &SpatialQueryFilter,
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
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
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
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::aabb_intersections_with_aabb_callback`]
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
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
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
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::aabb_intersections_with_aabb`]
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
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
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
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::shape_intersections_callback`]
    pub fn shape_intersections(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        query_filter: &SpatialQueryFilter,
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
    /// # #[cfg(feature = "2d")]
    /// # use avian2d::prelude::*;
    /// # #[cfg(feature = "3d")]
    /// use avian3d::prelude::*;
    /// use bevy::prelude::*;
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
    ///
    /// ## Related Methods
    ///
    /// - [`SpatialQuery::shape_intersections`]
    pub fn shape_intersections_callback(
        &self,
        shape: &Collider,
        shape_position: Vector,
        shape_rotation: RotationValue,
        query_filter: &SpatialQueryFilter,
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
