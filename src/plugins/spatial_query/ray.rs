use crate::prelude::*;
use bevy::{prelude::*, utils::HashMap};
use parry::{
    query::{
        details::RayCompositeShapeToiAndNormalBestFirstVisitor, visitors::RayIntersectionsVisitor,
    },
    shape::Shape,
};

/// A component used for ray casting.
///
/// **Ray casting** is a type of [spatial query](spatial_query) that finds one or more intersections
/// between a ray and a set of colliders.
///
/// Each ray is defined by a local `origin` and a `direction`. The [`RayCaster`] will find each intersection
/// and add them to the [`RayIntersections`] component. Each intersection has a `time_of_impact` property
/// which refers to how long the ray travelled, i.e. the distance between the `origin` and the point of intersection.
///
/// ## Intersection count and order
///
/// The results of a ray cast are in an arbitrary order by default. You can iterate over them in the order of
/// time of impact with the [`RayIntersections::iter_sorted`](RayIntersections#method.iter_sorted) method.
///
/// You can configure the maximum amount of intersections for a ray using `max_hits`. By default this is unbounded,
/// so you will get all intersections. When the number or complexity of colliders is large, this can be very
/// expensive computationally. Set the value to whatever works best for your case.
///
/// Note that when there are more intersections than `max_hits`, **some intersections will be missed**.
/// To guarantee that the closest intersection is included, you should set `max_hits` to one or a value that
/// is enough to contain all intersections.
///
///
/// ## Example
///
/// ```
/// # use bevy::prelude::*;
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
/// # #[cfg(not(feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Spawn a ray at the center going right
///     commands.spawn(RayCaster::new(Vector::ZERO, Vector::X));
///     // ...spawn colliders and other things
/// }
///
/// fn print_intersections(query: Query<(&RayCaster, &RayIntersections)>) {
///     for (ray, intersections) in &query {
///         // For the faster iterator that isn't sorted, use `.iter()`
///         for intersection in intersections.iter_sorted() {
///             println!(
///                 "Hit entity {:?} at {} with normal {}",
///                 intersection.entity,
///                 ray.origin + ray.direction * intersection.time_of_impact,
///                 intersection.normal,
///             );
///         }
///     }
/// }
/// ```
#[derive(Component)]
pub struct RayCaster {
    /// Controls if the ray caster is enabled.
    pub enabled: bool,
    /// The local origin of the ray. The global origin is computed by adding the local origin
    /// to the global [`Position`] component of the ray entity or its parent.
    pub origin: Vector,
    /// The local direction of the ray. The global direction is computed by rotating the local direction
    /// by the global [`Rotation`] component of the ray entity or its parent.
    pub direction: Vector,
    /// The maximum number of intersections allowed.
    ///
    /// When there are more intersections than `max_hits`, **some intersections will be missed**.
    /// To guarantee that the closest intersection is included, you should set `max_hits` to one or a value that
    /// is enough to contain all intersections.
    pub max_hits: u32,
}

impl Default for RayCaster {
    fn default() -> Self {
        Self {
            enabled: true,
            origin: Vector::ZERO,
            direction: Vector::ZERO,
            max_hits: u32::MAX,
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

    /// Sets the maximum amount of allowed hits.
    pub fn with_max_hits(mut self, max_hits: u32) -> Self {
        self.max_hits = max_hits;
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

    pub(crate) fn cast(
        &self,
        intersections: &mut RayIntersections,
        colliders: &HashMap<Entity, (Isometry<Scalar>, &dyn Shape)>,
        query_pipeline: &SpatialQueryPipeline,
        max_time_of_impact: Scalar,
        max_hit_count: u32,
        solid: bool,
    ) {
        intersections.count = 0;
        if max_hit_count == 1 {
            let pipeline_shape = query_pipeline.as_composite_shape(colliders);
            let ray = parry::query::Ray::new(self.origin.into(), self.direction.into());
            let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
                &pipeline_shape,
                &ray,
                max_time_of_impact,
                solid,
            );

            if let Some(intersection) = query_pipeline.qbvh.traverse_best_first(&mut visitor).map(
                |(_, (entity_bits, intersection))| RayIntersection {
                    entity: Entity::from_bits(entity_bits),
                    time_of_impact: intersection.toi,
                    normal: intersection.normal.into(),
                },
            ) {
                if (intersections.vector.len() as u32) < intersections.count + 1 {
                    intersections.vector.push(intersection);
                } else {
                    intersections.vector[0] = intersection;
                }
                intersections.count = 1;
            }
        } else {
            let mut leaf_callback = &mut |entity_bits: &u64| {
                let entity = Entity::from_bits(*entity_bits);
                if let Some((iso, shape)) = colliders.get(&entity) {
                    let ray = parry::query::Ray::new(self.origin.into(), self.direction.into());
                    if let Some(intersection) =
                        shape.cast_ray_and_get_normal(iso, &ray, max_time_of_impact, solid)
                    {
                        if (intersections.vector.len() as u32) < intersections.count + 1 {
                            intersections.vector.push(RayIntersection {
                                entity,
                                time_of_impact: intersection.toi,
                                normal: intersection.normal.into(),
                            });
                        } else {
                            intersections.vector[intersections.count as usize] = RayIntersection {
                                entity,
                                time_of_impact: intersection.toi,
                                normal: intersection.normal.into(),
                            };
                        }

                        intersections.count += 1;

                        return intersections.count < max_hit_count;
                    }
                }
                true
            };

            let ray = parry::query::Ray::new(self.origin.into(), self.direction.into());
            let mut visitor =
                RayIntersectionsVisitor::new(&ray, max_time_of_impact, &mut leaf_callback);
            query_pipeline.qbvh.traverse_depth_first(&mut visitor);
        }
    }
}

/// Contains the intersections of a ray cast by a [`RayCaster`].
///
/// The maximum number of intersections depends on the value of `max_hits` in [`RayCaster`].
///
/// ## Order
///
/// By default, the order of the intersections is not guaranteed.
///
/// You can iterate the intersections in the order of time of impact with `iter_sorted`.
/// Note that this will create and sort a new vector instead of the original one.
///
/// **Note**: When there are more intersections than `max_hits` allows, some intersections
/// will be missed. If you want to guarantee that the closest intersection is included, set `max_hits` to one.
///
/// ## Example
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn print_intersections(query: Query<&RayIntersections, With<RayCaster>>) {
///     for intersections in &query {
///         // For the faster iterator that isn't sorted, use `.iter()`
///         for intersection in intersections.iter_sorted() {
///             println!(
///                 "Hit entity {:?} with time of impact {}",
///                 intersection.entity,
///                 intersection.time_of_impact,
///             );
///         }
///     }
/// }
/// ```
#[derive(Component, Clone, Default)]
pub struct RayIntersections {
    pub(crate) vector: Vec<RayIntersection>,
    /// The number of intersections.
    pub count: u32,
}

impl RayIntersections {
    /// Returns an iterator over the intersections in arbitrary order.
    ///
    /// If you want to get them sorted by time of impact, use `iter_sorted`.
    pub fn iter(&self) -> std::slice::Iter<RayIntersection> {
        self.vector[0..self.count as usize].iter()
    }

    /// Returns an iterator over the intersections, sorted in ascending order according to the time of impact.
    ///
    /// Note that this creates and sorts a new vector. If you don't need the intersections in order, use `iter`.
    pub fn iter_sorted(&self) -> std::vec::IntoIter<RayIntersection> {
        let mut vector = self.vector[0..self.count as usize].to_vec();
        vector.sort_by(|a, b| a.time_of_impact.partial_cmp(&b.time_of_impact).unwrap());
        vector.into_iter()
    }
}

/// Data related to an intersection between a [ray](RayCaster) and a [collider](Collider).
#[derive(Clone, Copy)]
pub struct RayIntersection {
    /// The entity of the collider that was hit by the ray.
    pub entity: Entity,
    /// How long the ray travelled, i.e. the distance between the ray origin and the point of intersection.
    pub time_of_impact: Scalar,
    /// The normal at the point of intersection.
    pub normal: Vector,
}
