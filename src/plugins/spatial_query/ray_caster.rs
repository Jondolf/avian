use crate::prelude::*;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};
use parry::query::{
    details::RayCompositeShapeToiAndNormalBestFirstVisitor, visitors::RayIntersectionsVisitor,
};

/// A component used for [raycasting](spatial_query#raycasting).
///
/// **Raycasting** is a type of [spatial query](spatial_query) that finds one or more hits
/// between a ray and a set of colliders.
///
/// Each ray is defined by a local `origin` and a `direction`. The [`RayCaster`] will find each hit
/// and add them to the [`RayHits`] component. Each hit has a `time_of_impact` property
/// which refers to how long the ray travelled, i.e. the distance between the `origin` and the point of intersection.
///
/// The [`RayCaster`] is the easiest way to handle simple raycasts. If you want more control and don't want to
/// perform raycasts every frame, consider using the [`SpatialQuery`] system parameter.
///
/// ## Hit count and order
///
/// The results of a raycast are in an arbitrary order by default. You can iterate over them in the order of
/// time of impact with the [`RayHits::iter_sorted`] method.
///
/// You can configure the maximum amount of hits for a ray using `max_hits`. By default this is unbounded,
/// so you will get all hits. When the number or complexity of colliders is large, this can be very
/// expensive computationally. Set the value to whatever works best for your case.
///
/// Note that when there are more hits than `max_hits`, **some hits will be missed**.
/// To guarantee that the closest hit is included, you should set `max_hits` to one or a value that
/// is enough to contain all hits.
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
/// fn setup(mut commands: Commands) {
///     // Spawn a ray at the center going right
///     commands.spawn(RayCaster::new(Vec3::ZERO, Vec3::X));
///     // ...spawn colliders and other things
/// }
///
/// fn print_hits(query: Query<(&RayCaster, &RayHits)>) {
///     for (ray, hits) in &query {
///         // For the faster iterator that isn't sorted, use `.iter()`
///         for hit in hits.iter_sorted() {
///             println!(
///                 "Hit entity {:?} at {} with normal {}",
///                 hit.entity,
///                 ray.origin + ray.direction * hit.time_of_impact,
///                 hit.normal,
///             );
///         }
///     }
/// }
/// ```
#[derive(Component)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct RayCaster {
    /// Controls if the ray caster is enabled.
    pub enabled: bool,
    /// The local origin of the ray relative to the [`Position`] and [`Rotation`] of the ray entity or its parent.
    ///
    /// To get the global origin, use the `global_origin` method.
    pub origin: Vector,
    /// The global origin of the ray.
    global_origin: Vector,
    /// The local direction of the ray relative to the [`Rotation`] of the ray entity or its parent.
    ///
    /// To get the global direction, use the `global_direction` method.
    pub direction: Vector,
    /// The global direction of the ray.
    global_direction: Vector,
    /// The maximum distance the ray can travel. By default this is infinite, so the ray will travel
    /// until all hits up to `max_hits` have been checked.
    pub max_time_of_impact: Scalar,
    /// The maximum number of hits allowed.
    ///
    /// When there are more hits than `max_hits`, **some hits will be missed**.
    /// To guarantee that the closest hit is included, you should set `max_hits` to one or a value that
    /// is enough to contain all hits.
    pub max_hits: u32,
    /// Controls how the ray behaves when the ray origin is inside of a [collider](Collider).
    ///
    /// If `solid` is true, the point of intersection will be the ray origin itself.\
    /// If `solid` is false, the collider will be considered to have no interior, and the point of intersection
    /// will be at the collider shape's boundary.
    pub solid: bool,
    /// If true, the ray caster ignores hits against its own [`Collider`]. This is the default.
    pub ignore_self: bool,
    /// Rules that determine which colliders are taken into account in the query.
    pub query_filter: SpatialQueryFilter,
}

impl Default for RayCaster {
    fn default() -> Self {
        Self {
            enabled: true,
            origin: Vector::ZERO,
            global_origin: Vector::ZERO,
            direction: Vector::ZERO,
            global_direction: Vector::ZERO,
            max_time_of_impact: Scalar::MAX,
            max_hits: u32::MAX,
            solid: true,
            ignore_self: true,
            query_filter: SpatialQueryFilter::default(),
        }
    }
}

impl RayCaster {
    /// Creates a new [`RayCaster`] with a given origin and direction.
    pub fn new(origin: Vector, direction: Vector) -> Self {
        Self {
            origin,
            direction,
            ..default()
        }
    }

    /// Sets the ray origin.
    pub fn with_origin(mut self, origin: Vector) -> Self {
        self.origin = origin;
        self
    }

    /// Sets the ray direction.
    pub fn with_direction(mut self, direction: Vector) -> Self {
        self.direction = direction;
        self
    }

    /// Sets if the ray treats [colliders](Collider) as solid.
    ///
    /// If `solid` is true, the point of intersection will be the ray origin itself.\
    /// If `solid` is false, the collider will be considered to have no interior, and the point of intersection
    /// will be at the collider shape's boundary.
    pub fn with_solidness(mut self, solid: bool) -> Self {
        self.solid = solid;
        self
    }

    /// Sets if the ray caster should ignore hits against its own [`Collider`].
    /// The default is true.
    pub fn with_ignore_self(mut self, ignore: bool) -> Self {
        self.ignore_self = ignore;
        self
    }

    /// Sets the maximum time of impact, i.e. the maximum distance that the ray is allowed to travel.
    pub fn with_max_time_of_impact(mut self, max_time_of_impact: Scalar) -> Self {
        self.max_time_of_impact = max_time_of_impact;
        self
    }

    /// Sets the maximum number of allowed hits.
    pub fn with_max_hits(mut self, max_hits: u32) -> Self {
        self.max_hits = max_hits;
        self
    }

    /// Sets the ray caster's [query filter](SpatialQueryFilter) that controls which colliders
    /// should be included or excluded by raycasts.
    pub fn with_query_filter(mut self, query_filter: SpatialQueryFilter) -> Self {
        self.query_filter = query_filter;
        self
    }

    /// Enables the [`RayCaster`].
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disables the [`RayCaster`].
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Returns the global origin of the ray.
    pub fn global_origin(&self) -> Vector {
        self.global_origin
    }

    /// Returns the global direction of the ray.
    pub fn global_direction(&self) -> Vector {
        self.global_direction
    }

    /// Sets the global origin of the ray.
    pub(crate) fn set_global_origin(&mut self, global_origin: Vector) {
        self.global_origin = global_origin;
    }

    /// Sets the global direction of the ray.
    pub(crate) fn set_global_direction(&mut self, global_direction: Vector) {
        self.global_direction = global_direction;
    }

    pub(crate) fn cast(
        &self,
        caster_entity: Entity,
        hits: &mut RayHits,
        query_pipeline: &SpatialQueryPipeline,
    ) {
        let mut query_filter = self.query_filter.clone();

        if self.ignore_self {
            query_filter.excluded_entities.insert(caster_entity);
        }

        hits.count = 0;

        if self.max_hits == 1 {
            let pipeline_shape = query_pipeline.as_composite_shape(query_filter);
            let ray =
                parry::query::Ray::new(self.global_origin().into(), self.global_direction().into());
            let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
                &pipeline_shape,
                &ray,
                self.max_time_of_impact,
                self.solid,
            );

            if let Some(hit) = query_pipeline.qbvh.traverse_best_first(&mut visitor).map(
                |(_, (entity_index, hit))| RayHitData {
                    entity: query_pipeline.entity_from_index(entity_index),
                    time_of_impact: hit.toi,
                    normal: hit.normal.into(),
                },
            ) {
                if (hits.vector.len() as u32) < hits.count + 1 {
                    hits.vector.push(hit);
                } else {
                    hits.vector[0] = hit;
                }
                hits.count = 1;
            }
        } else {
            let ray =
                parry::query::Ray::new(self.global_origin().into(), self.global_direction().into());

            let mut leaf_callback = &mut |entity_index: &u32| {
                let entity = query_pipeline.entity_from_index(*entity_index);
                if let Some((iso, shape, layers)) = query_pipeline.colliders.get(&entity) {
                    if query_filter.test(entity, *layers) {
                        if let Some(hit) = shape.shape_scaled().cast_ray_and_get_normal(
                            iso,
                            &ray,
                            self.max_time_of_impact,
                            self.solid,
                        ) {
                            if (hits.vector.len() as u32) < hits.count + 1 {
                                hits.vector.push(RayHitData {
                                    entity,
                                    time_of_impact: hit.toi,
                                    normal: hit.normal.into(),
                                });
                            } else {
                                hits.vector[hits.count as usize] = RayHitData {
                                    entity,
                                    time_of_impact: hit.toi,
                                    normal: hit.normal.into(),
                                };
                            }

                            hits.count += 1;

                            return hits.count < self.max_hits;
                        }
                    }
                }
                true
            };

            let mut visitor =
                RayIntersectionsVisitor::new(&ray, self.max_time_of_impact, &mut leaf_callback);
            query_pipeline.qbvh.traverse_depth_first(&mut visitor);
        }
    }
}

/// Contains the hits of a ray cast by a [`RayCaster`].
///
/// The maximum number of hits depends on the value of `max_hits` in [`RayCaster`].
///
/// ## Order
///
/// By default, the order of the hits is not guaranteed.
///
/// You can iterate the hits in the order of time of impact with `iter_sorted`.
/// Note that this will create and sort a new vector instead of the original one.
///
/// **Note**: When there are more hits than `max_hits`, **some hits
/// will be missed**. If you want to guarantee that the closest hit is included, set `max_hits` to one.
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
/// fn print_hits(query: Query<&RayHits, With<RayCaster>>) {
///     for hits in &query {
///         // For the faster iterator that isn't sorted, use `.iter()`
///         for hit in hits.iter_sorted() {
///             println!(
///                 "Hit entity {:?} with time of impact {}",
///                 hit.entity,
///                 hit.time_of_impact,
///             );
///         }
///     }
/// }
/// ```
#[derive(Component, Clone, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct RayHits {
    pub(crate) vector: Vec<RayHitData>,
    /// The number of hits.
    pub(crate) count: u32,
}

impl RayHits {
    /// Returns a slice over the ray hits.
    pub fn as_slice(&self) -> &[RayHitData] {
        &self.vector[0..self.count as usize]
    }

    /// Returns the number of hits.
    #[doc(alias = "count")]
    pub fn len(&self) -> usize {
        self.count as usize
    }

    /// Returns true if the number of hits is 0.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Clears the hits.
    pub fn clear(&mut self) {
        self.vector.clear();
        self.count = 0;
    }

    /// Returns an iterator over the hits in arbitrary order.
    ///
    /// If you want to get them sorted by time of impact, use `iter_sorted`.
    pub fn iter(&self) -> std::slice::Iter<RayHitData> {
        self.as_slice().iter()
    }

    /// Returns an iterator over the hits, sorted in ascending order according to the time of impact.
    ///
    /// Note that this creates and sorts a new vector. If you don't need the hits in order, use `iter`.
    pub fn iter_sorted(&self) -> std::vec::IntoIter<RayHitData> {
        let mut vector = self.as_slice().to_vec();
        vector.sort_by(|a, b| a.time_of_impact.partial_cmp(&b.time_of_impact).unwrap());
        vector.into_iter()
    }
}

impl MapEntities for RayHits {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        for hit in &mut self.vector {
            hit.map_entities(entity_mapper);
        }
    }
}

/// Data related to a hit during a [raycast](spatial_query#raycasting).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct RayHitData {
    /// The entity of the collider that was hit by the ray.
    pub entity: Entity,
    /// How long the ray travelled, i.e. the distance between the ray origin and the point of intersection.
    pub time_of_impact: Scalar,
    /// The normal at the point of intersection.
    pub normal: Vector,
}

impl MapEntities for RayHitData {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity = entity_mapper.map_entity(self.entity);
    }
}
