//! Handles spatial queries like [ray casting](RayCaster) and shape casting.
//!
//! See [`SpatialQueryPlugin`].

mod pipeline;
mod ray_caster;
mod shape_caster;
mod system_param;

pub use pipeline::*;
pub use ray_caster::*;
pub use shape_caster::*;
pub use system_param::*;

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
            .add_systems((init_ray_hits, init_shape_hit).in_set(PhysicsSet::Prepare))
            .add_systems(
                (
                    update_ray_caster_positions,
                    update_shape_caster_positions,
                    update_query_pipeline,
                    raycast,
                    shapecast,
                )
                    .chain()
                    .in_set(PhysicsSet::SpatialQuery),
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

fn init_shape_hit(mut commands: Commands, shape_casters: Query<Entity, Added<ShapeCaster>>) {
    for entity in &shape_casters {
        commands.entity(entity).insert(ShapeHit(None));
    }
}

type RayCasterPositionQueryComponents = (
    &'static mut RayCaster,
    Option<&'static Position>,
    Option<&'static Rotation>,
    Option<&'static Parent>,
);

fn update_ray_caster_positions(
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

type ShapeCasterPositionQueryComponents = (
    &'static mut ShapeCaster,
    Option<&'static Position>,
    Option<&'static Rotation>,
    Option<&'static Parent>,
);

fn update_shape_caster_positions(
    mut shape_casters: Query<ShapeCasterPositionQueryComponents>,
    parents: Query<(Option<&Position>, Option<&Rotation>), With<Children>>,
) {
    for (mut shape_caster, position, rotation, parent) in &mut shape_casters {
        let origin = shape_caster.origin;
        let shape_rotation = shape_caster.shape_rotation;
        let direction = shape_caster.direction;

        if let Some(position) = position {
            shape_caster
                .set_global_origin(position.0 + rotation.map_or(origin, |rot| rot.rotate(origin)));
        } else if parent.is_none() {
            shape_caster.set_global_origin(origin);
        }

        if let Some(rotation) = rotation {
            let global_direction = rotation.rotate(shape_caster.direction);
            shape_caster.set_global_direction(global_direction);
            #[cfg(feature = "2d")]
            {
                shape_caster.set_global_shape_rotation(shape_rotation + rotation.as_radians());
            }
            #[cfg(feature = "3d")]
            {
                shape_caster.set_global_shape_rotation(shape_rotation + rotation.0);
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

        if let Some(parent) = parent {
            if let Ok((parent_position, parent_rotation)) = parents.get(parent.get()) {
                if position.is_none() {
                    if let Some(position) = parent_position {
                        let rotation = rotation.map_or(
                            parent_rotation.map_or(Rotation::default(), |rot| *rot),
                            |rot| *rot,
                        );
                        shape_caster.set_global_origin(position.0 + rotation.rotate(origin));
                    }
                }
                if rotation.is_none() {
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
}

fn raycast(
    mut rays: Query<(&RayCaster, &mut RayHits), Without<Collider>>,
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

    for (ray, mut hits) in &mut rays {
        if ray.enabled {
            ray.cast(&mut hits, &colliders, &query_pipeline);
        }
    }
}

fn shapecast(
    mut shape_casters: Query<(&ShapeCaster, &mut ShapeHit), Without<Collider>>,
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

    for (shape_caster, mut hit) in &mut shape_casters {
        if shape_caster.enabled {
            hit.0 = shape_caster.cast(&colliders, &query_pipeline);
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
