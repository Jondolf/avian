//! Miscallaneous utility functions.

use crate::prelude::*;

pub(crate) use bevy::platform_support::time::Instant;

/// Computes translation of `Position` based on center of mass rotation and translation
pub(crate) fn get_pos_translation(
    com_translation: &AccumulatedTranslation,
    previous_rotation: &Rotation,
    rotation: &Rotation,
    center_of_mass: &ComputedCenterOfMass,
) -> Vector {
    com_translation.0 + (previous_rotation * center_of_mass.0) - (rotation * center_of_mass.0)
}
