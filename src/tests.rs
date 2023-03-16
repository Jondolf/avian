use bevy::prelude::*;

use crate::XpbdPlugin;

fn create_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app
}

#[test]
fn it_loads_plugin_without_errors() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = create_app();
    app.add_plugin(XpbdPlugin);

    for _ in 0..50 {
        app.update();
    }

    Ok(())
}
