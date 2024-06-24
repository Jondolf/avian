use avian3d::debug_render::PhysicsDebugPlugin;
use avian3d::prelude::*;
use bevy::prelude::*;

fn main() {
    let mut app = App::new();

    app.add_plugins((PhysicsPlugins::default(), PhysicsDebugPlugin::default()));
    bevy_mod_debugdump::print_schedule_graph(&mut app, PostProcessCollisions);
}
