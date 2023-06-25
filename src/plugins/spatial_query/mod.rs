//! Handles spatial queries like [ray casting](RayCaster) and shape casting.
//!
//! See [`SpatialQueryPlugin`].

mod pipeline;
mod ray_caster;

pub use pipeline::*;
pub use ray_caster::*;

use crate::prelude::*;
use bevy::{prelude::*, utils::HashMap};

/// Handles spatial queries like [ray casting](RayCaster) and shape casting.
///
/// The [`SpatialQueryPipeline`] resource is used to maintain a quaternary bounding volume hierarchy
/// as an acceleration structure for the queries.
pub struct SpatialQueryPlugin;

impl Plugin for SpatialQueryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SpatialQueryPipeline>();

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics_schedule
            .add_system(init_intersections.in_set(PhysicsSet::Prepare))
            .add_systems(
                (update_global_origins, update_query_pipeline, raycast)
                    .chain()
                    .in_set(PhysicsSet::SpatialQuery),
            );
    }
}

fn init_intersections(mut commands: Commands, rays: Query<(Entity, &RayCaster), Added<RayCaster>>) {
    for (entity, ray) in &rays {
        let max_hits = if ray.max_hits == u32::MAX {
            10
        } else {
            ray.max_hits as usize
        };
        commands.entity(entity).insert(RayIntersections {
            vector: Vec::with_capacity(max_hits),
            count: 0,
        });
    }
}

type RayCasterPositionQueryComponents = (
    &'static mut RayCaster,
    Option<&'static Position>,
    Option<&'static Rotation>,
    Option<&'static Parent>,
);

fn update_global_origins(
    mut rays: Query<RayCasterPositionQueryComponents>,
    parents: Query<(Option<&Position>, Option<&Rotation>), With<Children>>,
) {
    for (mut ray, position, rotation, parent) in &mut rays {
        let origin = ray.origin;
        let direction = ray.direction;

        if let Some(position) = position {
            ray.set_global_origin(position.0 + rotation.map_or(origin, |rot| rot.rotate(origin)));
        } else if parent.is_none() {
            ray.set_global_origin(origin);
        }

        if let Some(rotation) = rotation {
            let global_direction = rotation.rotate(ray.direction);
            ray.set_global_direction(global_direction);
        } else if parent.is_none() {
            ray.set_global_direction(direction);
        }

        if let Some(parent) = parent {
            if let Ok((parent_position, parent_rotation)) = parents.get(parent.get()) {
                if position.is_none() {
                    if let Some(position) = parent_position {
                        let rotation = rotation.map_or(
                            parent_rotation.map_or(Rotation::default(), |rot| *rot),
                            |rot| *rot,
                        );
                        ray.set_global_origin(position.0 + rotation.rotate(origin));
                    }
                }
                if rotation.is_none() {
                    if let Some(rotation) = parent_rotation {
                        let global_direction = rotation.rotate(ray.direction);
                        ray.set_global_direction(global_direction);
                    }
                }
            }
        }
    }
}

fn raycast(
    mut rays: Query<(&RayCaster, &mut RayIntersections), Without<Collider>>,
    colliders: Query<(Entity, &Position, &Rotation, &Collider)>,
    query_pipeline: ResMut<SpatialQueryPipeline>,
) {
    let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = colliders
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

    for (ray, mut intersections) in &mut rays {
        if ray.enabled {
            ray.cast(&mut intersections, &colliders, &query_pipeline);
        }
    }
}

type ColliderChangedFilter = (
    Or<(Changed<Position>, Changed<Rotation>, Changed<Collider>)>,
    With<Collider>,
);

fn update_query_pipeline(
    colliders: Query<(Entity, &Position, &Rotation, &Collider)>,
    changed_colliders: Query<Entity, ColliderChangedFilter>,
    mut removed: RemovedComponents<Collider>,
    mut query_pipeline: ResMut<SpatialQueryPipeline>,
) {
    let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = colliders
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
    let modified = changed_colliders.iter().collect::<Vec<_>>();
    let removed = removed.iter().collect::<Vec<_>>();
    query_pipeline.update_incremental(&colliders, modified, removed, true);
}
