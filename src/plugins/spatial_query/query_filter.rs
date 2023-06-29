use bevy::{prelude::*, utils::HashSet};

use crate::prelude::CollisionLayers;

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
///         .with_layers(CollisionLayers::from_bits(0b1111, 0b0001))
///         .without_entities([object]);
///
///     // Spawn a ray caster with the query filter
///     commands.spawn(RayCaster::default().with_query_filter(query_filter));
/// }
/// ```
#[derive(Clone, Default)]
pub struct SpatialQueryFilter {
    /// If set, only the colliers with compatible [collision layers](CollisionLayers) will be taken into account
    /// in [spatial queries](crate::spatial_query).
    pub layers: Option<CollisionLayers>,
    /// Entities that will not be included in [spatial queries](crate::spatial_query).
    pub excluded_entities: HashSet<Entity>,
}

impl SpatialQueryFilter {
    /// Creates a new [`SpatialQueryFilter`] that doesn't exclude any colliders.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the layers that will be included in [spatial queries](crate::spatial_query).
    /// Colliders that have incompatible [collision layers](CollisionLayers) will be excluded.
    pub fn with_layers(mut self, layers: CollisionLayers) -> Self {
        self.layers = Some(layers);
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
            && (self.layers.is_none() || self.layers.is_some_and(|l| l.interacts_with(layers)))
    }
}

impl From<CollisionLayers> for SpatialQueryFilter {
    fn from(value: CollisionLayers) -> Self {
        Self {
            layers: Some(value),
            ..default()
        }
    }
}
