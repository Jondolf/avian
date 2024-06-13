//! Components and plugins for rigid body dynamics, handling motion
//! and physical interactions, including collisions and joints.

// Core plugins and modules
pub mod integrator;
pub mod sleeping;
pub mod solver;

// Components
mod forces;
mod locked_axes;
mod mass_properties;
mod rigid_body;
mod world_query;

// Re-exports
pub use forces::*;
pub use integrator::{Gravity, IntegratorPlugin};
pub use locked_axes::*;
pub use mass_properties::*;
pub use rigid_body::*;
pub use sleeping::{DeactivationTime, SleepingPlugin, SleepingThreshold};
pub use solver::SolverPlugin;
pub use world_query::*;
