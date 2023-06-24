//! Handles spatial queries like [ray casting](RayCaster) and shape casting.
//!
//! See [`SpatialQueryPlugin`].

mod pipeline;
mod ray;

pub use pipeline::*;
pub use ray::*;

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
                (update_query_pipeline, raycast)
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
            ray.cast(
                &mut intersections,
                &colliders,
                &query_pipeline,
                Scalar::MAX,
                ray.max_hits,
                true,
            );
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
