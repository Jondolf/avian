//! Basic diagnostics support.

mod path_macro;

#[cfg(feature = "diagnostics_ui")]
pub mod ui;

use crate::{
    collision::ColliderMarker,
    dynamics::{rigid_body::RigidBody, solver::joints::*},
    schedule::PhysicsSchedule,
    PhysicsStepSet,
};
use bevy::{
    app::{App, Plugin},
    diagnostic::{Diagnostic, DiagnosticPath, Diagnostics, RegisterDiagnostic},
    prelude::{IntoSystemConfigs, IntoSystemSetConfigs, Query, Res, ResMut, Resource, SystemSet},
    utils::Duration,
};
use bevy::{prelude::ReflectResource, reflect::Reflect};
pub(crate) use path_macro::impl_diagnostic_paths;

/// An extension trait for registering physics diagnostics.
pub trait AppDiagnosticsExt {
    /// Registers timer and counter diagnostics for a resource implementing [`PhysicsDiagnostics`].
    ///
    /// This method should be called in [`Plugin::finish`] to ensure that [`Diagnostics`]
    /// are only tracked if the [`PhysicsDiagnosticsPlugin`] was added to the app.
    fn register_physics_diagnostics<T: PhysicsDiagnostics>(&mut self);
}

impl AppDiagnosticsExt for App {
    fn register_physics_diagnostics<T: PhysicsDiagnostics>(&mut self) {
        // Initialize the diagnostics resource.
        self.init_resource::<T>();

        // If physics diagnostics are not enabled, return early.
        if !self.is_plugin_added::<PhysicsDiagnosticsPlugin>() {
            return;
        }

        // Register diagnostics for the paths returned by the diagnostics resource.
        let diagnostics = T::default();
        let timer_paths = diagnostics.timer_paths();
        let counter_paths = diagnostics.counter_paths();

        for path in timer_paths.iter().map(|(path, _)| *path) {
            // All timers are in milliseconds.
            self.register_diagnostic(Diagnostic::new(path.clone()).with_suffix("ms"));
        }

        for path in counter_paths.iter().map(|(path, _)| *path) {
            self.register_diagnostic(Diagnostic::new(path.clone()).with_smoothing_factor(0.0));
        }

        // Add systems to reset the diagnostics and write them to the `Diagnostics` resource.
        self.add_systems(
            PhysicsSchedule,
            (
                T::reset
                    .in_set(PhysicsDiagnosticsSystems::Reset)
                    .ambiguous_with_all(),
                T::write_diagnostics
                    .in_set(PhysicsDiagnosticsSystems::WriteDiagnostics)
                    .ambiguous_with_all(),
            ),
        );
    }
}

/// A system set for physics diagnostics.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsDiagnosticsSystems {
    /// Resets diagnostics to their default values.
    Reset,
    /// Writes diagnostics to the [`Diagnostics`] resource.
    WriteDiagnostics,
}

/// A trait for resources storing timers and counters for physics diagnostics.
pub trait PhysicsDiagnostics: Default + Resource {
    /// Maps diagnostic paths to their respective duration fields.
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        Vec::new()
    }

    /// Maps diagnostic paths to their respective counter fields.
    fn counter_paths(&self) -> Vec<(&'static DiagnosticPath, u32)> {
        Vec::new()
    }

    /// A system that resets the diagnostics to their default values.
    fn reset(mut physics_diagnostics: ResMut<Self>) {
        *physics_diagnostics = Self::default();
    }

    /// A system that writes diagnostics to the given [`Diagnostics`] instance.
    fn write_diagnostics(physics_diagnostics: Res<Self>, mut diagnostics: Diagnostics) {
        for (path, duration) in physics_diagnostics.timer_paths() {
            diagnostics.add_measurement(path, || duration.as_secs_f64() * 1000.0);
        }

        for (path, count) in physics_diagnostics.counter_paths() {
            diagnostics.add_measurement(path, || count as f64);
        }
    }
}

/// Diagnostics for physics entity counts.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct PhysicsEntityDiagnostics {
    /// The number of dynamic bodies.
    pub dynamic_body_count: u32,
    /// The number of kinematic bodies.
    pub kinematic_body_count: u32,
    /// The number of static bodies.
    pub static_body_count: u32,
    /// The number of colliders.
    pub collider_count: u32,
    /// The number of joint.
    pub joint_count: u32,
}

impl PhysicsDiagnostics for PhysicsEntityDiagnostics {
    fn counter_paths(&self) -> Vec<(&'static DiagnosticPath, u32)> {
        vec![
            (Self::DYNAMIC_BODY_COUNT, self.dynamic_body_count),
            (Self::KINEMATIC_BODY_COUNT, self.kinematic_body_count),
            (Self::STATIC_BODY_COUNT, self.static_body_count),
            (Self::COLLIDER_COUNT, self.collider_count),
            (Self::JOINT_COUNT, self.joint_count),
        ]
    }
}

impl_diagnostic_paths! {
    impl PhysicsEntityDiagnostics {
        DYNAMIC_BODY_COUNT: "avian/entity_count/dynamic_bodies",
        KINEMATIC_BODY_COUNT: "avian/entity_count/kinematic_bodies",
        STATIC_BODY_COUNT: "avian/entity_count/static_bodies",
        COLLIDER_COUNT: "avian/entity_count/colliders",
        JOINT_COUNT: "avian/entity_count/joints",
    }
}

/// A plugin that adds various diagnostics for debugging purposes.
/// It is not enabled by default and must be added manually.
pub struct PhysicsDiagnosticsPlugin;

impl Plugin for PhysicsDiagnosticsPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(
            PhysicsSchedule,
            (
                PhysicsDiagnosticsSystems::Reset.before(PhysicsStepSet::First),
                PhysicsDiagnosticsSystems::WriteDiagnostics.after(PhysicsStepSet::Last),
            ),
        );

        app.add_systems(
            PhysicsSchedule,
            diagnostic_entity_counts.in_set(PhysicsStepSet::First),
        );
    }

    fn finish(&self, app: &mut App) {
        // Register diagnostics for physics entity counts.
        app.register_physics_diagnostics::<PhysicsEntityDiagnostics>();
    }
}

// TODO: This is pretty inefficient.
fn diagnostic_entity_counts(
    rigid_bodies_query: Query<&RigidBody>,
    colliders_query: Query<&ColliderMarker>,
    fixed_joint_query: Query<&FixedJoint>,
    prismatic_joint_query: Query<&PrismaticJoint>,
    distance_joint_query: Query<&DistanceJoint>,
    revolute_joint_query: Query<&RevoluteJoint>,
    #[cfg(feature = "3d")] spherical_joint_query: Query<&SphericalJoint>,
    mut diagnostics: ResMut<PhysicsEntityDiagnostics>,
) {
    diagnostics.dynamic_body_count = rigid_bodies_query
        .iter()
        .filter(|rb| rb.is_dynamic())
        .count() as u32;
    diagnostics.kinematic_body_count = rigid_bodies_query
        .iter()
        .filter(|rb| rb.is_kinematic())
        .count() as u32;
    diagnostics.static_body_count = rigid_bodies_query
        .iter()
        .filter(|rb| rb.is_static())
        .count() as u32;
    diagnostics.collider_count = colliders_query.iter().count() as u32;
    diagnostics.joint_count = fixed_joint_query.iter().count() as u32
        + prismatic_joint_query.iter().count() as u32
        + distance_joint_query.iter().count() as u32
        + revolute_joint_query.iter().count() as u32;
    #[cfg(feature = "3d")]
    {
        diagnostics.joint_count += spherical_joint_query.iter().count() as u32;
    }
}
