//! A physics picking backend for `bevy_picking`.
//!
//! Add the [`PhysicsPickingPlugin`] to enable picking for [colliders](Collider).
//! By default, all colliders are pickable. Picking can be disabled for individual entities
//! by adding [`PickingBehavior::IGNORE`].
//!
//! To make physics picking entirely opt-in, set [`PhysicsPickingSettings::require_markers`]
//! to `true` and add a [`PhysicsPickable`] component to the desired camera and target entities.
//!
#![cfg_attr(
    feature = "3d",
    doc = "Note that in 3D, only the closest intersection will be reported."
)]

use crate::prelude::*;
use bevy::{
    picking::{
        backend::{ray::RayMap, HitData, PointerHits},
        PickSet,
    },
    prelude::*,
    render::view::RenderLayers,
};

/// Adds the physics picking backend to your app, enabling picking for [colliders](Collider).
#[derive(Clone, Default)]
pub struct PhysicsPickingPlugin;

impl Plugin for PhysicsPickingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsPickingSettings>()
            .add_systems(PreUpdate, update_hits.in_set(PickSet::Backend))
            .register_type::<(PhysicsPickingSettings, PhysicsPickable)>();
    }
}

/// Runtime settings for the [`PhysicsPickingPlugin`].
#[derive(Resource, Default, Reflect)]
#[reflect(Resource, Default)]
pub struct PhysicsPickingSettings {
    /// When set to `true` picking will only happen between cameras and entities marked with
    /// [`PhysicsPickable`]. `false` by default.
    ///
    /// This setting is provided to give you fine-grained control over which cameras and entities
    /// should be used by the physics picking backend at runtime.
    pub require_markers: bool,
}

/// An optional component that marks cameras and target entities that should be used in the [`PhysicsPickingPlugin`].
/// Only needed if [`PhysicsPickingSettings::require_markers`] is set to true.
#[derive(Debug, Clone, Default, Component, Reflect)]
#[reflect(Component, Default)]
pub struct PhysicsPickable;

/// Queries for collider intersections with pointers using [`PhysicsPickingSettings`] and sends [`PointerHits`] events.
pub fn update_hits(
    picking_cameras: Query<(&Camera, Option<&PhysicsPickable>, Option<&RenderLayers>)>,
    ray_map: Res<RayMap>,
    pickables: Query<&PickingBehavior>,
    marked_targets: Query<&PhysicsPickable>,
    layers: Query<&RenderLayers>,
    backend_settings: Res<PhysicsPickingSettings>,
    spatial_query: SpatialQuery,
    mut output_events: EventWriter<PointerHits>,
) {
    for (&ray_id, &ray) in ray_map.map().iter() {
        let Ok((camera, cam_pickable, cam_layers)) = picking_cameras.get(ray_id.camera) else {
            continue;
        };
        if backend_settings.require_markers && cam_pickable.is_none() || !camera.is_active {
            continue;
        }

        let cam_layers = cam_layers.unwrap_or_default();

        #[cfg(feature = "2d")]
        {
            let mut hits: Vec<(Entity, HitData)> = vec![];

            spatial_query.point_intersections_callback(
                ray.origin.truncate().adjust_precision(),
                &SpatialQueryFilter::default(),
                |entity| {
                    let marker_requirement =
                        !backend_settings.require_markers || marked_targets.get(entity).is_ok();

                    // Other entities missing render layers are on the default layer 0
                    let entity_layers = layers.get(entity).unwrap_or_default();
                    let render_layers_match = cam_layers.intersects(entity_layers);

                    let is_pickable = pickables
                        .get(entity)
                        .map(|p| *p != PickingBehavior::IGNORE)
                        .unwrap_or(true);

                    if marker_requirement && render_layers_match && is_pickable {
                        hits.push((
                            entity,
                            HitData::new(ray_id.camera, 0.0, Some(ray.origin.f32()), None),
                        ));
                    }

                    true
                },
            );

            output_events.send(PointerHits::new(ray_id.pointer, hits, camera.order as f32));
        }
        #[cfg(feature = "3d")]
        {
            if let Some((entity, hit_data)) = spatial_query
                .cast_ray_predicate(
                    ray.origin.adjust_precision(),
                    ray.direction,
                    Scalar::MAX,
                    true,
                    &SpatialQueryFilter::default(),
                    &|entity| {
                        let marker_requirement =
                            !backend_settings.require_markers || marked_targets.get(entity).is_ok();

                        // Other entities missing render layers are on the default layer 0
                        let entity_layers = layers.get(entity).unwrap_or_default();
                        let render_layers_match = cam_layers.intersects(entity_layers);

                        let is_pickable = pickables
                            .get(entity)
                            .map(|p| *p != PickingBehavior::IGNORE)
                            .unwrap_or(true);

                        marker_requirement && render_layers_match && is_pickable
                    },
                )
                .map(|ray_hit_data| {
                    #[allow(clippy::unnecessary_cast)]
                    let toi = ray_hit_data.time_of_impact as f32;
                    let hit_data = HitData::new(
                        ray_id.camera,
                        toi,
                        Some(ray.origin + *ray.direction * toi),
                        Some(ray_hit_data.normal.f32()),
                    );
                    (ray_hit_data.entity, hit_data)
                })
            {
                output_events.send(PointerHits::new(
                    ray_id.pointer,
                    vec![(entity, hit_data)],
                    camera.order as f32,
                ));
            }
        }
    }
}
