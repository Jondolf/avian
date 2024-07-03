use crate::{
    prelude::*,
    sync::{PreviousGlobalTransform, SyncConfig},
};
use bevy::prelude::*;
use broad_phase::AabbIntersections;
use dynamics::solver::SolverConfig;

/// Registers physics types to the `TypeRegistry` resource in `bevy_reflect`.
pub struct PhysicsTypeRegistrationPlugin;

impl Plugin for PhysicsTypeRegistrationPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Time<Physics>>()
            .register_type::<Time<Substeps>>()
            .register_type::<SubstepCount>()
            .register_type::<BroadCollisionPairs>()
            .register_type::<AabbIntersections>()
            .register_type::<SleepingThreshold>()
            .register_type::<DeactivationTime>()
            .register_type::<Gravity>()
            .register_type::<RigidBody>()
            .register_type::<Sleeping>()
            .register_type::<SleepingDisabled>()
            .register_type::<TimeSleeping>()
            .register_type::<Position>()
            .register_type::<Rotation>()
            .register_type::<PreSolveAccumulatedTranslation>()
            .register_type::<PreviousRotation>()
            .register_type::<PreviousGlobalTransform>()
            .register_type::<AccumulatedTranslation>()
            .register_type::<LinearVelocity>()
            .register_type::<AngularVelocity>()
            .register_type::<PreSolveLinearVelocity>()
            .register_type::<PreSolveAngularVelocity>()
            .register_type::<Restitution>()
            .register_type::<Friction>()
            .register_type::<LinearDamping>()
            .register_type::<AngularDamping>()
            .register_type::<ExternalForce>()
            .register_type::<ExternalTorque>()
            .register_type::<ExternalImpulse>()
            .register_type::<ExternalAngularImpulse>()
            .register_type::<GravityScale>()
            .register_type::<Mass>()
            .register_type::<InverseMass>()
            .register_type::<Inertia>()
            .register_type::<InverseInertia>()
            .register_type::<CenterOfMass>()
            .register_type::<ColliderDensity>()
            .register_type::<ColliderMassProperties>()
            .register_type::<LockedAxes>()
            .register_type::<ColliderParent>()
            .register_type::<Dominance>()
            .register_type::<ColliderAabb>()
            .register_type::<CollisionLayers>()
            .register_type::<CollidingEntities>()
            .register_type::<CoefficientCombine>()
            .register_type::<Sensor>()
            .register_type::<ColliderTransform>()
            .register_type::<PreviousColliderTransform>()
            .register_type::<SpeculativeMargin>()
            .register_type::<SweptCcd>()
            .register_type::<CollisionMargin>()
            .register_type::<NarrowPhaseConfig>()
            .register_type::<SolverConfig>()
            .register_type::<SyncConfig>()
            .register_type::<ColliderConstructor>()
            .register_type::<ColliderConstructorHierarchy>()
            .register_type::<ColliderConstructorHierarchyConfig>()
            .register_type::<RayCaster>()
            .register_type::<ShapeCaster>()
            .register_type::<DistanceJoint>()
            .register_type::<FixedJoint>()
            .register_type::<PrismaticJoint>()
            .register_type::<RevoluteJoint>();

        #[cfg(feature = "3d")]
        app.register_type::<SphericalJoint>();
    }
}
