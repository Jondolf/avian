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
///     // A query filter that only includes one layer and excludes the `object` entity
///     let query_filter = SpatialQueryFilter::new()
///         .masks_from_bits(0b1011)
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

    /// Disables all masks of the filter configuration. No colliders will be included in
    /// [spatial queries](crate::spatial_query).
    pub fn no_masks(mut self) -> Self {
        self.masks = 0;
        self
    }

    /// Sets the masks of the filter configuration using a bitmask. Colliders with the corresponding
    /// [collision group](CollisionLayers) will be included in the [spatial query](crate::spatial_query).
    pub fn masks_from_bits(mut self, masks: u32) -> Self {
        self.masks = masks;
        self
    }

    /// Adds the given mask to the filter configuration. Colliders with the corresponding
    /// [collision group](CollisionLayers) will be included in the [spatial query](crate::spatial_query).
    pub fn with_mask(mut self, mask: impl PhysicsLayer) -> Self {
        self.masks |= mask.to_bits();
        self
    }

    /// Adds the given masks to the filter configuration. Colliders with the corresponding
    /// [collision groups](CollisionLayers) will be included in the [spatial query](crate::spatial_query).
    pub fn with_masks(mut self, masks: impl IntoIterator<Item = impl PhysicsLayer>) -> Self {
        for mask in masks.into_iter().map(|l| l.to_bits()) {
            self.masks |= mask;
        }
        self
    }

    /// Removes the given mask from the filter configuration. Colliders with the corresponding
    /// [collision group](CollisionLayers) will be excluded from the [spatial query](crate::spatial_query).
    pub fn without_mask(mut self, mask: impl PhysicsLayer) -> Self {
        self.masks &= !mask.to_bits();
        self
    }

    /// Removes the given masks from the filter configuration. Colliders with the corresponding
    /// [collision groups](CollisionLayers) will be excluded from the [spatial query](crate::spatial_query).
    pub fn without_masks(mut self, masks: impl IntoIterator<Item = impl PhysicsLayer>) -> Self {
        for mask in masks.into_iter().map(|l| l.to_bits()) {
            self.masks &= !mask;
        }
        self
    }

    /// Returns true if the given layer is contained in the filter's masks.
    pub fn contains_mask(self, layer: impl PhysicsLayer) -> bool {
        (self.masks & layer.to_bits()) != 0
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
