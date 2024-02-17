//! **Spatial queries** are a way to get information about the environment. They perform geometric queries
//! on [colliders](Collider) and retrieve data about intersections.
//!
//! There are four types of spatial queries: [raycasts](#raycasting), [shapecasts](#shapecasting),
//! [point projection](#point-projection) and [intersection tests](#intersection-tests).
//! All spatial queries can be done using the various methods provided by the [`SpatialQuery`] system parameter.
//!
//! Raycasting and shapecasting can also be done with a component-based approach using the [`RayCaster`] and
//! [`ShapeCaster`] components. They enable performing casts every frame in a way that is often more convenient
//! than the normal [`SpatialQuery`] methods. See their documentation for more information.
//!
//! ## Raycasting
//!
//! **Raycasting** is a spatial query that finds intersections between colliders and a half-line. This can be used for
//! a variety of things like getting information about the environment for character controllers and AI,
//! and even rendering using ray tracing.
//!
//! For each hit during raycasting, the hit entity, a *time of impact* and a normal will be stored in [`RayHitData`].
//! The time of impact refers to how long the ray travelled, which is essentially the distance from the ray origin to
//! the point of intersection.
//!
//! There are two ways to perform raycasts.
//!
//! 1. For simple raycasts, use the [`RayCaster`] component. It returns the results of the raycast
//! in the [`RayHits`] component every frame. It uses local coordinates, so it will automatically follow the entity
//! it's attached to or its parent.
//! 2. When you need more control or don't want to cast every frame, use the raycasting methods provided by
//! [`SpatialQuery`], like [`cast_ray`](SpatialQuery::cast_ray), [`ray_hits`](SpatialQuery::ray_hits) or
//! [`ray_hits_callback`](SpatialQuery::ray_hits_callback).
//!
//! See the documentation of the components and methods for more information.
//!
//! A simple example using the component-based method looks like this:
//!
//! ```
//! use bevy::prelude::*;
//! # #[cfg(feature = "2d")]
//! # use bevy_xpbd_2d::prelude::*;
//! # #[cfg(feature = "3d")]
//! use bevy_xpbd_3d::prelude::*;
//!
//! # #[cfg(all(feature = "3d", feature = "f32"))]
//! fn setup(mut commands: Commands) {
//!     // Spawn a ray caster at the center with the rays travelling right
//!     commands.spawn(RayCaster::new(Vec3::ZERO, Vec3::X));
//!     // ...spawn colliders and other things
//! }
//!
//! fn print_hits(query: Query<(&RayCaster, &RayHits)>) {
//!     for (ray, hits) in &query {
//!         // For the faster iterator that isn't sorted, use `.iter()`
//!         for hit in hits.iter_sorted() {
//!             println!(
//!                 "Hit entity {:?} at {} with normal {}",
//!                 hit.entity,
//!                 ray.origin + ray.direction * hit.time_of_impact,
//!                 hit.normal,
//!             );
//!         }
//!     }
//! }
//! ```
//!
//! To specify which colliders should be considered in the query, use a [spatial query filter](`SpatialQueryFilter`).
//!
//! ## Shapecasting
//!
//! **Shapecasting** or **sweep testing** is a spatial query that finds intersections between colliders and a shape
//! that is travelling along a half-line. It is very similar to [raycasting](#raycasting), but instead of a "point"
//! we have an entire shape travelling along a half-line. One use case is determining how far an object can move
//! before it hits the environment.
//!
//! For each hit during shapecasting, the hit entity, the *time of impact*, two local points of intersection and two local
//! normals will be stored in [`ShapeHitData`]. The time of impact refers to how long the shape travelled before the initial
//! hit, which is essentially the distance from the shape origin to the global point of intersection.
//!
//! There are two ways to perform shapecasts.
//!
//! 1. For simple shapecasts, use the [`ShapeCaster`] component. It returns the results of the shapecast
//! in the [`ShapeHits`] component every frame. It uses local coordinates, so it will automatically follow the entity
//! it's attached to or its parent.
//! 2. When you need more control or don't want to cast every frame, use the shapecasting methods provided by
//! [`SpatialQuery`], like [`cast_shape`](SpatialQuery::cast_shape), [`shape_hits`](SpatialQuery::shape_hits) or
//! [`shape_hits_callback`](SpatialQuery::shape_hits_callback).
//!
//! See the documentation of the components and methods for more information.
//!
//! A simple example using the component-based method looks like this:
//!
//! ```
//! use bevy::prelude::*;
//! # #[cfg(feature = "2d")]
//! # use bevy_xpbd_2d::prelude::*;
//! # #[cfg(feature = "3d")]
//! use bevy_xpbd_3d::prelude::*;
//!
//! # #[cfg(all(feature = "3d", feature = "f32"))]
//! fn setup(mut commands: Commands) {
//!     // Spawn a shape caster with a ball shape at the center travelling right
//!     commands.spawn(ShapeCaster::new(
//!         Collider::ball(0.5), // Shape
//!         Vec3::ZERO,          // Origin
//!         Quat::default(),     // Shape rotation
//!         Vec3::X              // Direction
//!     ));
//!     // ...spawn colliders and other things
//! }
//!
//! fn print_hits(query: Query<(&ShapeCaster, &ShapeHits)>) {
//!     for (shape_caster, hits) in &query {
//!         for hit in hits.iter() {
//!             println!("Hit entity {:?}", hit.entity);
//!         }
//!     }
//! }
//! ```
//!
//! To specify which colliders should be considered in the query, use a [spatial query filter](`SpatialQueryFilter`).
//!
//! ## Point projection
//!
//! **Point projection** is a spatial query that projects a point on the closest collider. It returns the collider's
//! entity, the projected point, and whether the point is inside of the collider.
//!
//! Point projection can be done with the [`project_point`](SpatialQuery::project_point) method of the [`SpatialQuery`]
//! system parameter. See its documentation for more information.
//!
//! To specify which colliders should be considered in the query, use a [spatial query filter](`SpatialQueryFilter`).
//!
//! ## Intersection tests
//!
//! **Intersection tests** are spatial queries that return the entities of colliders that are intersecting a given
//! shape or area.
//!
//! There are three types of intersection tests. They are all methods of the [`SpatialQuery`] system parameter,
//! and they all have callback variants that call a given callback on each intersection.
//!
//! - [`point_intersections`](SpatialQuery::point_intersections): Finds all entities with a collider that contains
//! the given point.
//! - [`aabb_intersections_with_aabb`](SpatialQuery::aabb_intersections_with_aabb):
//! Finds all entities with a [`ColliderAabb`] that is intersecting the given [`ColliderAabb`].
//! - [`shape_intersections`](SpatialQuery::shape_intersections): Finds all entities with a [collider](Collider)
//! that is intersecting the given shape.
//!
//! See the documentation of the components and methods for more information.
//!
//! To specify which colliders should be considered in the query, use a [spatial query filter](`SpatialQueryFilter`).

mod pipeline;
mod query_filter;
mod ray_caster;
mod shape_caster;
mod system_param;

pub use pipeline::*;
pub use query_filter::*;
pub use ray_caster::*;
pub use shape_caster::*;
pub use system_param::*;

use crate::{prelude::*, prepare::PrepareSet};
use bevy::{prelude::*, utils::intern::Interned};

/// Initializes the [`SpatialQueryPipeline`] resource and handles component-based [spatial queries](spatial_query)
/// like [raycasting](spatial_query#raycasting) and [shapecasting](spatial_query#shapecasting) with
/// [`RayCaster`] and [`ShapeCaster`].
pub struct SpatialQueryPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl SpatialQueryPlugin {
    /// Creates a [`SpatialQueryPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for SpatialQueryPlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

impl Plugin for SpatialQueryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SpatialQueryPipeline>().add_systems(
            self.schedule,
            (init_ray_hits, init_shape_hit).in_set(PrepareSet::PreInit),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule.add_systems(
            (
                update_ray_caster_positions,
                update_shape_caster_positions,
                |mut spatial_query: SpatialQuery| spatial_query.update_pipeline(),
                raycast,
                shapecast,
            )
                .chain()
                .in_set(PhysicsStepSet::SpatialQuery),
        );
    }
}

fn init_ray_hits(mut commands: Commands, rays: Query<(Entity, &RayCaster), Added<RayCaster>>) {
    for (entity, ray) in &rays {
        let max_hits = if ray.max_hits == u32::MAX {
            10
        } else {
            ray.max_hits as usize
        };
        commands.entity(entity).insert(RayHits {
            vector: Vec::with_capacity(max_hits),
            count: 0,
        });
    }
}

fn init_shape_hit(
    mut commands: Commands,
    shape_casters: Query<(Entity, &ShapeCaster), Added<ShapeCaster>>,
) {
    for (entity, shape_caster) in &shape_casters {
        commands.entity(entity).insert(ShapeHits {
            vector: Vec::with_capacity(shape_caster.max_hits.min(100_000) as usize),
            count: 0,
        });
    }
}

type RayCasterPositionQueryComponents = (
    &'static mut RayCaster,
    Option<&'static Position>,
    Option<&'static Rotation>,
    Option<&'static Parent>,
    Option<&'static GlobalTransform>,
);

#[allow(clippy::type_complexity)]
fn update_ray_caster_positions(
    mut rays: Query<RayCasterPositionQueryComponents>,
    parents: Query<
        (
            Option<&Position>,
            Option<&Rotation>,
            Option<&GlobalTransform>,
        ),
        With<Children>,
    >,
) {
    for (mut ray, position, rotation, parent, transform) in &mut rays {
        let origin = ray.origin;
        let direction = ray.direction;

        let global_position = position.copied().or(transform.map(Position::from));
        let global_rotation = rotation.copied().or(transform.map(Rotation::from));

        if let Some(global_position) = global_position {
            ray.set_global_origin(
                global_position.0 + rotation.map_or(origin, |rot| rot.rotate(origin)),
            );
        } else if parent.is_none() {
            ray.set_global_origin(origin);
        }

        if let Some(global_rotation) = global_rotation {
            let global_direction = global_rotation.rotate(ray.direction);
            ray.set_global_direction(global_direction);
        } else if parent.is_none() {
            ray.set_global_direction(direction);
        }

        if let Some(Ok((parent_position, parent_rotation, parent_transform))) =
            parent.map(|p| parents.get(p.get()))
        {
            let parent_position = parent_position
                .copied()
                .or(parent_transform.map(Position::from));
            let parent_rotation = parent_rotation
                .copied()
                .or(parent_transform.map(Rotation::from));

            // Apply parent transformations
            if global_position.is_none() {
                if let Some(position) = parent_position {
                    let rotation = global_rotation.unwrap_or(parent_rotation.unwrap_or_default());
                    ray.set_global_origin(position.0 + rotation.rotate(origin));
                }
            }
            if global_rotation.is_none() {
                if let Some(rotation) = parent_rotation {
                    let global_direction = rotation.rotate(ray.direction);
                    ray.set_global_direction(global_direction);
                }
            }
        }
    }
}

type ShapeCasterPositionQueryComponents = (
    &'static mut ShapeCaster,
    Option<&'static Position>,
    Option<&'static Rotation>,
    Option<&'static Parent>,
    Option<&'static GlobalTransform>,
);

#[allow(clippy::type_complexity)]
fn update_shape_caster_positions(
    mut shape_casters: Query<ShapeCasterPositionQueryComponents>,
    parents: Query<
        (
            Option<&Position>,
            Option<&Rotation>,
            Option<&GlobalTransform>,
        ),
        With<Children>,
    >,
) {
    for (mut shape_caster, position, rotation, parent, transform) in &mut shape_casters {
        let origin = shape_caster.origin;
        let shape_rotation = shape_caster.shape_rotation;
        let direction = shape_caster.direction;

        let global_position = position.copied().or(transform.map(Position::from));
        let global_rotation = rotation.copied().or(transform.map(Rotation::from));

        if let Some(global_position) = global_position {
            shape_caster.set_global_origin(
                global_position.0 + rotation.map_or(origin, |rot| rot.rotate(origin)),
            );
        } else if parent.is_none() {
            shape_caster.set_global_origin(origin);
        }

        if let Some(global_rotation) = global_rotation {
            let global_direction = global_rotation.rotate(shape_caster.direction);
            shape_caster.set_global_direction(global_direction);
            #[cfg(feature = "2d")]
            {
                shape_caster
                    .set_global_shape_rotation(shape_rotation + global_rotation.as_radians());
            }
            #[cfg(feature = "3d")]
            {
                shape_caster.set_global_shape_rotation(shape_rotation + global_rotation.0);
            }
        } else if parent.is_none() {
            shape_caster.set_global_direction(direction);
            #[cfg(feature = "2d")]
            {
                shape_caster.set_global_shape_rotation(shape_rotation);
            }
            #[cfg(feature = "3d")]
            {
                shape_caster.set_global_shape_rotation(shape_rotation);
            }
        }

        if let Some(Ok((parent_position, parent_rotation, parent_transform))) =
            parent.map(|p| parents.get(p.get()))
        {
            let parent_position = parent_position
                .copied()
                .or(parent_transform.map(Position::from));
            let parent_rotation = parent_rotation
                .copied()
                .or(parent_transform.map(Rotation::from));

            // Apply parent transformations
            if global_position.is_none() {
                if let Some(position) = parent_position {
                    let rotation = global_rotation.unwrap_or(parent_rotation.unwrap_or_default());
                    shape_caster.set_global_origin(position.0 + rotation.rotate(origin));
                }
            }
            if global_rotation.is_none() {
                if let Some(rotation) = parent_rotation {
                    let global_direction = rotation.rotate(shape_caster.direction);
                    shape_caster.set_global_direction(global_direction);
                    #[cfg(feature = "2d")]
                    {
                        shape_caster
                            .set_global_shape_rotation(shape_rotation + rotation.as_radians());
                    }
                    #[cfg(feature = "3d")]
                    {
                        shape_caster.set_global_shape_rotation(shape_rotation + rotation.0);
                    }
                }
            }
        }
    }
}

fn raycast(mut rays: Query<(Entity, &RayCaster, &mut RayHits)>, spatial_query: SpatialQuery) {
    for (entity, ray, mut hits) in &mut rays {
        if ray.enabled {
            ray.cast(entity, &mut hits, &spatial_query.query_pipeline);
        } else if !hits.is_empty() {
            hits.clear();
        }
    }
}

fn shapecast(
    mut shape_casters: Query<(Entity, &ShapeCaster, &mut ShapeHits)>,
    spatial_query: SpatialQuery,
) {
    for (entity, shape_caster, mut hits) in &mut shape_casters {
        if shape_caster.enabled {
            shape_caster.cast(entity, &mut hits, &spatial_query.query_pipeline);
        } else if !hits.is_empty() {
            hits.clear();
        }
    }
}
