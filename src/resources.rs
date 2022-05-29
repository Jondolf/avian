use bevy::prelude::*;

#[derive(Debug)]
pub struct Gravity(pub Vec2);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec2::new(0.0, -9.81))
    }
}

/// Stores pairs of colliding entities.
#[derive(Debug, Default)]
pub(crate) struct CollisionPairs(pub Vec<(Entity, Entity)>);

// Stores dynamic-dynamic contacts and contact normals.
#[derive(Default, Debug)]
pub struct DynamicContacts(pub Vec<(Entity, Entity, Vec2)>);

// Stores dynamic-static contacts and contact normals.
#[derive(Default, Debug)]
pub struct StaticContacts(pub Vec<(Entity, Entity, Vec2)>);
