use crate::prelude::*;
use bevy::{log::LogPlugin, prelude::*};

fn create_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugin(LogPlugin::default());
    app.add_plugin(XpbdPlugin);
    app
}

#[test]
fn it_loads_plugin_without_errors() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = create_app();
    app.setup();

    for _ in 0..500 {
        app.update();
    }

    Ok(())
}
