use std::time::Duration;

use crate::prelude::*;
use approx::assert_relative_eq;
use bevy::{log::LogPlugin, prelude::*, time::TimeUpdateStrategy, utils::Instant};

fn create_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugin(LogPlugin::default());
    app.add_plugin(XpbdPlugin);
    app.insert_resource(TimeUpdateStrategy::ManualInstant(Instant::now()));
    app
}

#[test]
fn it_loads_plugin_without_errors() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = create_app();
    app.setup();

    for _ in 0..500 {
        tick_60_fps(&mut app);
    }

    Ok(())
}

#[test]
fn body_with_velocity_moves() {
    let mut app = create_app();

    app.add_startup_system(|mut commands: Commands| {
        // move right at 1 unit per second
        commands.spawn((
            SpatialBundle::default(),
            RigidBodyBundle::new_dynamic().with_lin_vel(Vector::X),
        ));
    });

    app.insert_resource(Gravity::ZERO);

    app.setup();

    const UPDATES: usize = 500;

    for _ in 0..UPDATES {
        tick_60_fps(&mut app);
    }

    let mut app_query = app.world.query::<(&Transform, &RigidBody)>();

    let (transform, _body) = app_query.single(&app.world);

    assert!(transform.translation.x > 0., "box moves right");
    assert_relative_eq!(transform.translation.y, 0.);
    assert_relative_eq!(transform.translation.z, 0.);

    // make sure we end up in the expected position
    assert_relative_eq!(
        transform.translation.x,
        1. * UPDATES as f32 * 1. / 60.,
        epsilon = 0.03 // allow some leeway, as we might be one frame off
    );
}

fn tick_60_fps(app: &mut App) {
    let mut update_strategy = app.world.resource_mut::<TimeUpdateStrategy>();
    let TimeUpdateStrategy::ManualInstant(prev_time) = *update_strategy else { unimplemented!() };
    *update_strategy =
        TimeUpdateStrategy::ManualInstant(prev_time + Duration::from_secs_f64(1. / 60.));
    app.update();
}
