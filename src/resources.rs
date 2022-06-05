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

// Stores dynamic-dynamic contact data.
#[derive(Default, Debug)]
pub struct DynamicContacts(pub Vec<Contact>);

// Stores dynamic-static contact data.
#[derive(Default, Debug)]
pub struct StaticContacts(pub Vec<Contact>);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub r_a: Vec2,
    pub r_b: Vec2,
    pub normal: Vec2,
    pub penetration: f32,
}
