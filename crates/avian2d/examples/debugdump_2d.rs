use bevy::prelude::*;
use bevy_xpbd_2d::plugins::PhysicsPlugins;

fn main() {
    let mut app = App::new();

    app.add_plugins((PhysicsPlugins::default(), PhysicsDebugPlugin::default()));
    bevy_mod_debugdump::print_schedule_graph(&mut app, Update);
}
