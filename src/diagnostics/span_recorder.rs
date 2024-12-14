use crate::diagnostics::PhysicsDiagnosticsPlugin;
use bevy::{diagnostic::Diagnostics, log::warn, prelude::Resource};
use std::{collections::HashMap, time::Instant};

#[derive(Hash, Eq, PartialEq, Debug, Copy, Clone)]
pub enum PhysicsSpan {
    BroadPhase,
    NarrowPhase,
    Solver,
    #[allow(dead_code)]
    SolverStage(SolverStageSpan),
    ReportContacts,
    SpatialQueries,
}

#[allow(dead_code)]
#[derive(Hash, Eq, PartialEq, Debug, Copy, Clone)]
pub enum SolverStageSpan {
    Prepare,
    WarmStart,
    IntegrateVelocities,
    SolveConstraints,
    IntegratePositions,
    RelaxVelocities,
    ApplyRestitution,
    Finalize,
    StoreImpulses,
}

pub struct SpanRecord {
    started_at: Instant,
    finished_at: Option<Instant>,
}

#[derive(Resource, Default)]
pub struct PhysicsSpanRecorder {
    spans: HashMap<PhysicsSpan, SpanRecord>,
}

impl PhysicsSpanRecorder {
    pub fn reset(&mut self) {
        self.spans.clear();
    }

    pub fn finalise(&self, diagnostics: &mut Diagnostics) {
        for (span_type, span_record) in self.spans.iter() {
            let Some(finished_at) = span_record.finished_at else {
                warn!("No end instant for span_type: {:?}", span_type);
                continue;
            };

            if let Some(diagnostic_path) = match span_type {
                PhysicsSpan::BroadPhase => Some(PhysicsDiagnosticsPlugin::BROAD_TIME),
                PhysicsSpan::NarrowPhase => Some(PhysicsDiagnosticsPlugin::NARROW_TIME),
                PhysicsSpan::Solver => Some(PhysicsDiagnosticsPlugin::SOLVER_TIME),
                PhysicsSpan::ReportContacts => Some(PhysicsDiagnosticsPlugin::REPORT_TIME),
                PhysicsSpan::SpatialQueries => Some(PhysicsDiagnosticsPlugin::SPATIAL_TIME),
                PhysicsSpan::SolverStage(_) => None,
            } {
                diagnostics.add_measurement(&diagnostic_path, || {
                    finished_at
                        .duration_since(span_record.started_at)
                        .as_secs_f64()
                        * 1000.0
                });
            }
        }
    }

    pub fn start_span(&mut self, span_type: PhysicsSpan) {
        self.spans.insert(
            span_type,
            SpanRecord {
                started_at: Instant::now(),
                finished_at: None,
            },
        );
    }

    pub fn end_span(&mut self, span_type: PhysicsSpan) {
        match self.spans.get_mut(&span_type) {
            None => {
                warn!("No span found for span_type: {:?}", span_type);
            }
            Some(record) => {
                record.finished_at = Some(Instant::now());
            }
        }
    }
}
