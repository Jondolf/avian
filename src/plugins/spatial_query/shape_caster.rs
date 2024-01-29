use crate::prelude::*;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};
use parry::query::details::TOICompositeShapeShapeBestFirstVisitor;

/// A component used for [shapecasting](spatial_query#shapecasting).
///
/// **Shapecasting** is a type of [spatial query](spatial_query) where a shape travels along a straight
/// line and computes hits with colliders. This is often used to determine how far an object can move
/// in a direction before it hits something.
///
/// Each shapecast is defined by a `shape` (a [`Collider`]), its local `shape_rotation`, a local `origin` and
/// a local `direction`. The [`ShapeCaster`] will find each hit and add them to the [`ShapeHits`] component in
/// the order of the time of impact.
///
/// Computing lots of hits can be expensive, especially against complex geometry, so the maximum number of hits
/// is one by default. This can be configured through the `max_hits` property.
///
/// The [`ShapeCaster`] is the easiest way to handle simple shapecasting. If you want more control and don't want
/// to perform shapecasts on every frame, consider using the [`SpatialQuery`] system parameter.
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
///     // Spawn a shape caster with a ball shape moving right starting from the origin
///     commands.spawn(ShapeCaster::new(
///         Collider::ball(0.5),
///         Vec3::ZERO,
///         Quat::default(),
///         Vec3::X
///     ));
/// }
///
/// fn print_hits(query: Query<(&ShapeCaster, &ShapeHits)>) {
///     for (shape_caster, hits) in &query {
///         for hit in hits.iter() {
///             println!("Hit entity {:?}", hit.entity);
///         }
///     }
/// }
/// ```
#[derive(Component)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ShapeCaster {
    /// Controls if the shape caster is enabled.
    pub enabled: bool,
    /// The shape being cast represented as a [`Collider`].
    pub shape: Collider,
    /// The local origin of the shape relative to the [`Position`] and [`Rotation`]
    /// of the shape caster entity or its parent.
    ///
    /// To get the global origin, use the `global_origin` method.
    pub origin: Vector,
    /// The global origin of the shape.
    global_origin: Vector,
    /// The local rotation of the shape being cast relative to the [`Rotation`]
    /// of the shape caster entity or its parent. Expressed in radians.
    ///
    /// To get the global shape rotation, use the `global_shape_rotation` method.
    #[cfg(feature = "2d")]
    pub shape_rotation: Scalar,
    /// The local rotation of the shape being cast relative to the [`Rotation`]
    /// of the shape caster entity or its parent.
    ///
    /// To get the global shape rotation, use the `global_shape_rotation` method.
    #[cfg(feature = "3d")]
    pub shape_rotation: Quaternion,
    /// The global rotation of the shape.
    #[cfg(feature = "2d")]
    global_shape_rotation: Scalar,
    /// The global rotation of the shape.
    #[cfg(feature = "3d")]
    global_shape_rotation: Quaternion,
    /// The local direction of the shapecast relative to the [`Rotation`] of the shape caster entity or its parent.
    ///
    /// To get the global direction, use the `global_direction` method.
    pub direction: Vector,
    /// The global direction of the shapecast.
    global_direction: Vector,
    /// The maximum distance the shape can travel. By default this is infinite, so the shape will travel
    /// until a hit is found.
    pub max_time_of_impact: Scalar,
    /// The maximum number of hits allowed. By default this is one and only the first hit is returned.
    pub max_hits: u32,
    /// Controls how the shapecast behaves when the shape is already penetrating a [collider](Collider)
    /// at the shape origin.
    ///
    /// If set to true **and** the shape is being cast in a direction where it will eventually stop penetrating,
    /// the shapecast will not stop immediately, and will instead continue until another hit.\
    /// If set to false, the shapecast will stop immediately and return the hit. This is the default.
    pub ignore_origin_penetration: bool,
    /// If true, the shape caster ignores hits against its own [`Collider`]. This is the default.
    pub ignore_self: bool,
    /// Rules that determine which colliders are taken into account in the query.
    pub query_filter: SpatialQueryFilter,
}

impl Default for ShapeCaster {
    fn default() -> Self {
        Self {
            enabled: true,
            shape: Collider::ball(0.0),
            origin: Vector::ZERO,
            global_origin: Vector::ZERO,
            #[cfg(feature = "2d")]
            shape_rotation: 0.0,
            #[cfg(feature = "3d")]
            shape_rotation: Quaternion::IDENTITY,
            #[cfg(feature = "2d")]
            global_shape_rotation: 0.0,
            #[cfg(feature = "3d")]
            global_shape_rotation: Quaternion::IDENTITY,
            direction: Vector::ZERO,
            global_direction: Vector::ZERO,
            max_time_of_impact: Scalar::MAX,
            max_hits: 1,
            ignore_origin_penetration: false,
            ignore_self: true,
            query_filter: SpatialQueryFilter::default(),
        }
    }
}

impl ShapeCaster {
    /// Creates a new [`ShapeCaster`] with a given shape, origin, shape rotation and direction.
    #[cfg(feature = "2d")]
    pub fn new(shape: Collider, origin: Vector, shape_rotation: Scalar, direction: Vector) -> Self {
        Self {
            shape,
            origin,
            shape_rotation,
            direction: direction.normalize(),
            ..default()
        }
    }
    #[cfg(feature = "3d")]
    /// Creates a new [`ShapeCaster`] with a given shape, origin, shape rotation and direction.
    pub fn new(
        shape: Collider,
        origin: Vector,
        shape_rotation: Quaternion,
        direction: Vector,
    ) -> Self {
        Self {
            shape,
            origin,
            shape_rotation,
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

    /// Controls how the shapecast behaves when the shape is already penetrating a [collider](Collider)
    /// at the shape origin.
    ///
    /// If set to true **and** the shape is being cast in a direction where it will eventually stop penetrating,
    /// the shapecast will not stop immediately, and will instead continue until another hit.\
    /// If set to false, the shapecast will stop immediately and return the hit. This is the default.
    pub fn with_ignore_origin_penetration(mut self, ignore: bool) -> Self {
        self.ignore_origin_penetration = ignore;
        self
    }

    /// Sets if the shape caster should ignore hits against its own [`Collider`].
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

    /// Sets the shape caster's [query filter](SpatialQueryFilter) that controls which colliders
    /// should be included or excluded by shapecasts.
    pub fn with_query_filter(mut self, query_filter: SpatialQueryFilter) -> Self {
        self.query_filter = query_filter;
        self
    }

    /// Enables the [`ShapeCaster`].
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disables the [`ShapeCaster`].
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Returns the global origin of the ray.
    pub fn global_origin(&self) -> Vector {
        self.global_origin
    }

    /// Returns the global rotation of the shape.
    #[cfg(feature = "2d")]
    pub fn global_shape_rotation(&self) -> Scalar {
        self.global_shape_rotation
    }

    /// Returns the global rotation of the shape.
    #[cfg(feature = "3d")]
    pub fn global_shape_rotation(&self) -> Quaternion {
        self.global_shape_rotation
    }

    /// Returns the global direction of the ray.
    pub fn global_direction(&self) -> Vector {
        self.global_direction
    }

    /// Sets the global origin of the ray.
    pub(crate) fn set_global_origin(&mut self, global_origin: Vector) {
        self.global_origin = global_origin;
    }

    /// Sets the global rotation of the shape.
    #[cfg(feature = "2d")]
    pub(crate) fn set_global_shape_rotation(&mut self, global_rotation: Scalar) {
        self.global_shape_rotation = global_rotation;
    }

    /// Sets the global rotation of the shape.
    #[cfg(feature = "3d")]
    pub(crate) fn set_global_shape_rotation(&mut self, global_rotation: Quaternion) {
        self.global_shape_rotation = global_rotation;
    }

    /// Sets the global direction of the ray.
    pub(crate) fn set_global_direction(&mut self, global_direction: Vector) {
        self.global_direction = global_direction;
    }

    pub(crate) fn cast(
        &self,
        caster_entity: Entity,
        hits: &mut ShapeHits,
        query_pipeline: &SpatialQueryPipeline,
    ) {
        let mut query_filter = self.query_filter.clone();

        if self.ignore_self {
            query_filter.excluded_entities.insert(caster_entity);
        }

        hits.count = 0;

        let shape_rotation: Rotation;
        #[cfg(feature = "2d")]
        {
            shape_rotation = Rotation::from_radians(self.global_shape_rotation());
        }
        #[cfg(feature = "3d")]
        {
            shape_rotation = Rotation::from(self.global_shape_rotation());
        }

        let shape_isometry = utils::make_isometry(self.global_origin(), shape_rotation);
        let shape_direction = self.global_direction().into();

        while hits.count < self.max_hits {
            let pipeline_shape = query_pipeline.as_composite_shape(query_filter.clone());
            let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
                &*query_pipeline.dispatcher,
                &shape_isometry,
                &shape_direction,
                &pipeline_shape,
                &**self.shape.shape_scaled(),
                self.max_time_of_impact,
                !self.ignore_origin_penetration,
            );

            if let Some(hit) = query_pipeline.qbvh.traverse_best_first(&mut visitor).map(
                |(_, (entity_index, hit))| ShapeHitData {
                    entity: query_pipeline.entity_from_index(entity_index),
                    time_of_impact: hit.toi,
                    point1: hit.witness1.into(),
                    point2: hit.witness2.into(),
                    normal1: hit.normal1.into(),
                    normal2: hit.normal2.into(),
                },
            ) {
                if (hits.vector.len() as u32) < hits.count + 1 {
                    hits.vector.push(hit);
                } else {
                    hits.vector[hits.count as usize] = hit;
                }

                hits.count += 1;
                query_filter.excluded_entities.insert(hit.entity);
            } else {
                return;
            }
        }
    }
}

/// Contains the hits of a shape cast by a [`ShapeCaster`]. The hits are in the order of time of impact.
///
/// The maximum number of hits depends on the value of `max_hits` in [`ShapeCaster`]. By default only
/// one hit is computed, as shapecasting for many results can be expensive.
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
/// fn print_hits(query: Query<&ShapeHits, With<ShapeCaster>>) {
///     for hits in &query {
///         for hit in hits.iter() {
///             println!(
///                 "Hit entity {:?} with time of impact {}",
///                 hit.entity,
///                 hit.time_of_impact,
///             );
///         }
///     }
/// }
/// ```
#[derive(Component, Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ShapeHits {
    pub(crate) vector: Vec<ShapeHitData>,
    pub(crate) count: u32,
}

impl ShapeHits {
    /// Returns a slice over the shapecast hits.
    pub fn as_slice(&self) -> &[ShapeHitData] {
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

    /// Returns an iterator over the hits in the order of time of impact.
    pub fn iter(&self) -> std::slice::Iter<ShapeHitData> {
        self.as_slice().iter()
    }
}

impl MapEntities for ShapeHits {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        for hit in &mut self.vector {
            hit.map_entities(entity_mapper);
        }
    }
}

/// Data related to a hit during a [shapecast](spatial_query#shapecasting).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ShapeHitData {
    /// The entity of the collider that was hit by the shape.
    pub entity: Entity,
    /// How long the shape travelled before the initial hit,
    /// i.e. the distance between the origin and the point of intersection.
    pub time_of_impact: Scalar,
    /// The closest point on the collider that was hit by the shapecast, at the time of impact,
    /// expressed in the local space of the collider shape.
    pub point1: Vector,
    /// The closest point on the cast shape, at the time of impact,
    /// expressed in the local space of the cast shape.
    pub point2: Vector,
    /// The outward normal on the collider that was hit by the shapecast, at the time of impact,
    /// expressed in the local space of the collider shape.
    pub normal1: Vector,
    /// The outward normal on the cast shape, at the time of impact,
    /// expressed in the local space of the cast shape.
    pub normal2: Vector,
}

impl MapEntities for ShapeHitData {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity = entity_mapper.map_entity(self.entity);
    }
}
