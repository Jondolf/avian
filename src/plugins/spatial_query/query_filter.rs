use bevy::{prelude::*, utils::HashSet};

use crate::prelude::*;

/// Rules that determine which colliders are taken into account in [spatial queries](crate::spatial_query).
///
/// ## Example
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// #
/// fn setup(mut commands: Commands) {
///     let object = commands.spawn(Collider::ball(0.5)).id();
///
///     // A query filter that has three collision masks and excludes the `object` entity
///     let query_filter = SpatialQueryFilter::new()
///         .with_masks_from_bits(0b1011)
///         .without_entities([object]);
///
///     // Spawn a ray caster with the query filter
///     commands.spawn(RayCaster::default().with_query_filter(query_filter));
/// }
/// ```
#[derive(Clone)]
pub struct SpatialQueryFilter {
    /// Specifies which [collision groups](CollisionLayers) will be included in a [spatial query](crate::spatial_query).
    pub masks: u32,
    /// Entities that will not be included in [spatial queries](crate::spatial_query).
    pub excluded_entities: HashSet<Entity>,
}

impl Default for SpatialQueryFilter {
    fn default() -> Self {
        Self {
            masks: 0xffff_ffff,
            excluded_entities: default(),
        }
    }
}

impl SpatialQueryFilter {
    /// Creates a new [`SpatialQueryFilter`] that doesn't exclude any colliders.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the masks of the filter configuration using a bitmask. Colliders with the corresponding
    /// [collision group](CollisionLayers) will be included in the [spatial query](crate::spatial_query).
    pub fn with_masks_from_bits(mut self, masks: u32) -> Self {
        self.masks = masks;
        self
    }

    /// Sets the masks of the filter configuration using a list of [layers](PhysicsLayer).
    /// Colliders with the corresponding [collision groups](CollisionLayers) will be included
    /// in the [spatial query](crate::spatial_query).
    pub fn with_masks(mut self, masks: impl IntoIterator<Item = impl PhysicsLayer>) -> Self {
        self.masks = 0;
        for mask in masks.into_iter().map(|l| l.to_bits()) {
            self.masks |= mask;
        }
        self
    }

    /// Excludes the given entities from [spatial queries](crate::spatial_query).
    #[doc(alias = "exclude_entities")]
    pub fn without_entities(mut self, entities: impl IntoIterator<Item = Entity>) -> Self {
        self.excluded_entities = HashSet::from_iter(entities);
        self
    }

    /// Tests if an entity should be included in [spatial queries](crate::spatial_query) based on the
    /// filter configuration.
    pub fn test(&self, entity: Entity, layers: CollisionLayers) -> bool {
        !self.excluded_entities.contains(&entity)
            && CollisionLayers::from_bits(0xffff_ffff, self.masks).interacts_with(
                CollisionLayers::from_bits(layers.groups_bits(), 0xffff_ffff),
            )
    }
}
