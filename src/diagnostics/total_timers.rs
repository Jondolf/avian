use bevy::{
    diagnostic::{Diagnostic, Diagnostics, DiagnosticsStore, RegisterDiagnostic},
    prelude::*,
};

use crate::diagnostics::impl_diagnostic_paths;

use super::{PhysicsSchedule, PhysicsStepSet};

/// A plugin that adds diagnostics for total physics timers.
pub struct PhysicsTotalDiagnosticsPlugin;

impl Plugin for PhysicsTotalDiagnosticsPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource::<PhysicsStepStart>(PhysicsStepStart(bevy::utils::Instant::now()));

        app.register_diagnostic(
            Diagnostic::new(PhysicsTotalDiagnostics::STEP_TIME.clone()).with_suffix("ms"),
        );
        app.register_diagnostic(
            Diagnostic::new(PhysicsTotalDiagnostics::MISCELLANEOUS.clone()).with_suffix("ms"),
        );

        app.add_systems(
            PhysicsSchedule,
            (
                update_physics_step_start.in_set(PhysicsStepSet::First),
                update_step_time.in_set(PhysicsStepSet::Last),
                update_other_physics_diagnostics.after(PhysicsStepSet::Last),
            ),
        );
    }
}

/// Diagnostics for total physics timers.
pub struct PhysicsTotalDiagnostics;

impl_diagnostic_paths! {
    impl PhysicsTotalDiagnostics {
        // The time spent on the physics step.
        STEP_TIME: "avian/total_step_time",
        // The time spent on the physics step not covered by other diagnostics.
        MISCELLANEOUS: "avian/miscellaneous",
    }
}

/// The time at which the physics step started.
#[derive(Resource, Debug, Reflect)]
#[reflect(Resource, Debug)]
pub struct PhysicsStepStart(pub bevy::utils::Instant);

pub(super) fn update_physics_step_start(mut start: ResMut<PhysicsStepStart>) {
    start.0 = bevy::utils::Instant::now();
}

pub(super) fn update_step_time(start: Res<PhysicsStepStart>, mut diagnostics: Diagnostics) {
    diagnostics.add_measurement(PhysicsTotalDiagnostics::STEP_TIME, || {
        start.0.elapsed().as_secs_f64() * 1000.0
    });
}

fn update_other_physics_diagnostics(mut diagnostics: Diagnostics, store: Res<DiagnosticsStore>) {
    let total = store
        .get(PhysicsTotalDiagnostics::STEP_TIME)
        .and_then(|d| d.measurement().map(|m| m.value))
        .unwrap_or(0.0);
    let timed: f64 = store
        .iter()
        .filter_map(|d| {
            if d.suffix != "ms"
                || d.path() == PhysicsTotalDiagnostics::STEP_TIME
                || d.path() == PhysicsTotalDiagnostics::MISCELLANEOUS
            {
                return None;
            }
            d.path()
                .components()
                .next()
                .and_then(|c| (c == "avian").then_some(d.value()))
                .flatten()
        })
        .sum();

    diagnostics.add_measurement(PhysicsTotalDiagnostics::MISCELLANEOUS, || {
        (total - timed).max(0.0)
    });
}
