#![expect(missing_docs)]

use crate::prelude::*;
use bevy::{
    ecs::{query::QueryData, system::lifetimeless::Read},
    prelude::*,
};

/// A [`QueryData`] for bodies to prepare their XPBD constraints.
#[derive(QueryData)]
pub struct XpbdBodyQuery {
    pub position: Read<Position>,
    pub rotation: Read<Rotation>,
    pub center_of_mass: Option<Read<ComputedCenterOfMass>>,
    pub dominance: Option<Read<Dominance>>,
    pub is_dynamic: Has<DynamicBody>,
    pub is_sleeping: Has<Sleeping>,
}

impl XpbdBodyQueryItem<'_> {
    /// Returns the [dominance](Dominance) of the body.
    ///
    /// If it isn't specified, the default of `0` is returned for dynamic bodies.
    /// For static and kinematic bodies, `i8::MAX + 1` (`128`) is always returned instead.
    #[inline]
    pub fn dominance(&self) -> i16 {
        if !self.is_dynamic {
            i8::MAX as i16 + 1
        } else {
            self.dominance.map_or(0, |dominance| dominance.0) as i16
        }
    }

    /// Returns the local center of mass of the body.
    #[inline]
    pub fn center_of_mass(&self) -> Vector {
        self.center_of_mass.map_or(Vector::ZERO, |com| com.0)
    }
}
