//! Run with:
//! `cargo run --example debugdump_2d > dump.dot  && dot -Tsvg dump.dot > dump.svg`

/*
use avian2d::prelude::*;
use bevy::prelude::*;

fn main() {
    let mut app = App::new();

    app.add_plugins((PhysicsPlugins::default(), PhysicsDebugPlugin));

    // Schedules of interest:
    // - PhysicsSchedule
    // - SubstepSchedule
    // - FixedPostUpdate
    // - Update
    bevy_mod_debugdump::print_schedule_graph(&mut app, PhysicsSchedule);
}
*/

fn main() {
    todo!("This example will be updated when bevy_mod_debugdump supports Bevy 0.17");
}
