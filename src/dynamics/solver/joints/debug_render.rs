use crate::dynamics::prelude::*;
use bevy::prelude::*;

use super::PhysicsGizmos;

pub trait ConstraintDebugRender {
    fn render(
        &self,
        body1: &RigidBodyQueryReadOnlyItem,
        body2: &RigidBodyQueryReadOnlyItem,
        color: Color,
        gizmos: &mut Gizmos<PhysicsGizmos>,
    );
}
