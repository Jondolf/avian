//! Miscallaneous utility functions.

use crate::prelude::*;

/// Computes translation of `Position` based on center of mass rotation and translation
pub(crate) fn get_pos_translation(
    com_translation: &AccumulatedTranslation,
    previous_rotation: &Rotation,
    rotation: &Rotation,
    center_of_mass: &CenterOfMass,
) -> Vector {
    com_translation.0 + (previous_rotation * center_of_mass.0) - (rotation * center_of_mass.0)
}
