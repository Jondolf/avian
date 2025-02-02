use std::time::Duration;

use bevy::{
    diagnostic::DiagnosticPath,
    prelude::{ReflectResource, Res, ResMut, Resource},
    reflect::Reflect,
};

use crate::diagnostics::{impl_diagnostic_paths, PhysicsDiagnostics};

/// Diagnostics for physics simulation steps.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct PhysicsStepDiagnostics {
    /// The total time taken by physics in the current frame.
    pub step_time: Duration,
}

impl PhysicsDiagnostics for PhysicsStepDiagnostics {
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![(Self::STEP_TIME, self.step_time)]
    }
}

impl_diagnostic_paths! {
    impl PhysicsStepDiagnostics {
        STEP_TIME: "avian/total_step_time",
    }
}

/// The time at which the physics step started.
#[derive(Resource, Debug, Reflect)]
#[reflect(Resource, Debug)]
pub struct PhysicsStepStart(pub bevy::utils::Instant);

pub(super) fn update_physics_step_start(mut start: bevy::prelude::ResMut<PhysicsStepStart>) {
    start.0 = bevy::utils::Instant::now();
}

pub(super) fn update_step_time(
    start: Res<PhysicsStepStart>,
    mut diagnostics: ResMut<PhysicsStepDiagnostics>,
) {
    diagnostics.step_time = start.0.elapsed();
}
