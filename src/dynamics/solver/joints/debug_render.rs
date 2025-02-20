use crate::dynamics::prelude::*;
use bevy::prelude::*;

use super::PhysicsGizmos;

/// A trait implemented by constraints that can be rendered for debugging purposes.
pub trait ConstraintDebugRender {
    /// Renders the constraint for debugging purposes.
    fn render(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        color: Color,
        gizmos: &mut Gizmos<PhysicsGizmos>,
    );
}
