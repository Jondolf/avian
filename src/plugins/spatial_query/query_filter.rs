use bevy::{prelude::*, utils::HashSet};

use crate::prelude::*;

/// Rules that determine which colliders are taken into account in [spatial queries](crate::spatial_query).
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     let object = commands.spawn(Collider::ball(0.5)).id();
///
///     // A query filter that has three collision masks and excludes the `object` entity
///     let query_filter = SpatialQueryFilter::from_masks(0b1011).with_excluded_entities([object]);
///
///     // Spawn a ray caster with the query filter
///     commands.spawn(RayCaster::default().with_query_filter(query_filter));
/// }
/// ```
#[derive(Clone)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct SpatialQueryFilter {
    /// Specifies which [collision groups](CollisionLayers) will be included in the [spatial query](crate::spatial_query).
    pub masks: LayerMask,
    /// Entities that will not be included in [spatial queries](crate::spatial_query).
    pub excluded_entities: HashSet<Entity>,
}

impl Default for SpatialQueryFilter {
    fn default() -> Self {
        Self {
            masks: LayerMask::ALL,
            excluded_entities: default(),
        }
    }
}

impl SpatialQueryFilter {
    /// Creates a new [`SpatialQueryFilter`] with the given masks determining
    /// which [collision groups](CollisionLayers) will be included in the [spatial query](crate::spatial_query).
    pub fn from_masks(masks: impl Into<LayerMask>) -> Self {
        Self {
            masks: masks.into(),
            ..default()
        }
    }

    /// Creates a new [`SpatialQueryFilter`] with the given entities excluded from the [spatial query](crate::spatial_query).
    pub fn from_excluded_entities(entities: impl IntoIterator<Item = Entity>) -> Self {
        Self {
            excluded_entities: HashSet::from_iter(entities),
            ..default()
        }
    }

    /// Sets the masks of the filter configuration to the given layers. Only colliders with the corresponding
    /// [collision groups](CollisionLayers) will be included in the [spatial query](crate::spatial_query).
    pub fn with_masks(mut self, masks: impl Into<LayerMask>) -> Self {
        self.masks = masks.into();
        self
    }

    /// Excludes the given entities from the [spatial query](crate::spatial_query).
    pub fn with_excluded_entities(mut self, entities: impl IntoIterator<Item = Entity>) -> Self {
        self.excluded_entities = HashSet::from_iter(entities);
        self
    }

    /// Tests if an entity should be included in [spatial queries](crate::spatial_query) based on the
    /// filter configuration.
    pub fn test(&self, entity: Entity, layers: CollisionLayers) -> bool {
        !self.excluded_entities.contains(&entity)
            && CollisionLayers::new(LayerMask::ALL, self.masks)
                .interacts_with(CollisionLayers::new(layers.groups, LayerMask::ALL))
    }
}
