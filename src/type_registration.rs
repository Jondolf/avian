use crate::{
    ancestor_marker::AncestorMarker, physics_transform::PhysicsTransformConfig, prelude::*,
};
use bevy::prelude::*;
use dynamics::solver::{SolverConfig, schedule::SubstepCount};

/// Registers physics types to the `TypeRegistry` resource in `bevy_reflect`.
pub struct PhysicsTypeRegistrationPlugin;

impl Plugin for PhysicsTypeRegistrationPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Time<Physics>>()
            .register_type::<Time<Substeps>>()
            .register_type::<SubstepCount>()
            .register_type::<Gravity>()
            .register_type::<RigidBody>()
            .register_type::<RigidBodyDisabled>()
            .register_type::<Sleeping>()
            .register_type::<SleepingDisabled>()
            .register_type::<TimeSleeping>()
            .register_type::<Position>()
            .register_type::<Rotation>()
            .register_type::<PreSolveDeltaPosition>()
            .register_type::<PreSolveDeltaRotation>()
            .register_type::<LinearVelocity>()
            .register_type::<AngularVelocity>()
            .register_type::<MaxLinearSpeed>()
            .register_type::<MaxAngularSpeed>()
            .register_type::<Restitution>()
            .register_type::<Friction>()
            .register_type::<LinearDamping>()
            .register_type::<AngularDamping>()
            .register_type::<GravityScale>()
            .register_type::<ColliderDensity>()
            .register_type::<ColliderMassProperties>()
            .register_type::<LockedAxes>()
            .register_type::<ColliderOf>()
            .register_type::<RigidBodyColliders>()
            .register_type::<Dominance>()
            .register_type::<ColliderAabb>()
            .register_type::<CollisionLayers>()
            .register_type::<CollidingEntities>()
            .register_type::<CoefficientCombine>()
            .register_type::<Sensor>()
            .register_type::<ColliderTransform>()
            .register_type::<SpeculativeMargin>()
            .register_type::<SweptCcd>()
            .register_type::<CollisionMargin>()
            .register_type::<NarrowPhaseConfig>()
            .register_type::<SolverConfig>()
            .register_type::<PhysicsTransformConfig>()
            .register_type::<AncestorMarker<ColliderMarker>>()
            .register_type::<RayCaster>()
            .register_type::<DistanceJoint>()
            .register_type::<FixedJoint>()
            .register_type::<PrismaticJoint>()
            .register_type::<RevoluteJoint>();

        #[cfg(feature = "default-collider")]
        app.register_type::<ColliderConstructor>()
            .register_type::<ColliderConstructorHierarchy>()
            .register_type::<crate::collision::collider::ColliderConstructorHierarchyConfig>()
            .register_type::<ShapeCaster>();

        #[cfg(feature = "3d")]
        app.register_type::<SphericalJoint>();
    }
}
