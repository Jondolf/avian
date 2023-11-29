use crate::prelude::*;
use approx::assert_relative_eq;
use bevy::{
    ecs::schedule::ScheduleBuildSettings, prelude::*, time::TimeUpdateStrategy, utils::Instant,
};
#[cfg(feature = "enhanced-determinism")]
use insta::assert_debug_snapshot;
use std::time::Duration;

// rust doesn't seem to have a way to run one-time setup per test
// could consider rstest? https://github.com/la10736/rstest#use-once-fixture
// see https://insta.rs/docs/patterns/ for the pattern used here
#[cfg(feature = "enhanced-determinism")]
macro_rules! setup_insta {
    ($($expr:expr),*) => {
        let _insta_settings_guard = {
            let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
            let mut settings = insta::Settings::clone_current();
            // we need this hack, because insta doesn't like modules being outside the crate
            settings.set_snapshot_path(format!("{manifest_dir}/snapshots"));
            settings.bind_to_scope()
        };
    };
}

fn create_app() -> App {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, TransformPlugin, PhysicsPlugins::default()));
    #[cfg(feature = "async-collider")]
    {
        app.add_plugins((
            bevy::asset::AssetPlugin::default(),
            bevy::scene::ScenePlugin,
        ))
        .init_resource::<Assets<Mesh>>();
    }
    app.insert_resource(TimeUpdateStrategy::ManualInstant(Instant::now()));
    app
}

fn tick_60_fps(app: &mut App) {
    let mut update_strategy = app.world.resource_mut::<TimeUpdateStrategy>();
    let TimeUpdateStrategy::ManualInstant(prev_time) = *update_strategy else {
        unimplemented!()
    };
    *update_strategy =
        TimeUpdateStrategy::ManualInstant(prev_time + Duration::from_secs_f64(1. / 60.));
    app.update();
}

#[cfg(feature = "3d")]
fn setup_cubes_simulation(mut commands: Commands) {
    let mut next_id = 0;
    // copied from "cubes" example
    let floor_size = Vector::new(80.0, 1.0, 80.0);
    commands.spawn((
        RigidBody::Static,
        Position(Vector::NEG_Y),
        Collider::cuboid(floor_size.x, floor_size.y, floor_size.z),
    ));

    let radius = 1.0;
    let count_x = 4;
    let count_y = 4;
    let count_z = 4;
    for y in 0..count_y {
        for x in 0..count_x {
            for z in 0..count_z {
                let pos = Vector::new(
                    (x as Scalar - count_x as Scalar * 0.5) * 2.1 * radius,
                    10.0 * radius * y as Scalar,
                    (z as Scalar - count_z as Scalar * 0.5) * 2.1 * radius,
                );
                commands.spawn((
                    SpatialBundle::default(),
                    RigidBody::Dynamic,
                    Position(pos + Vector::Y * 5.0),
                    Collider::cuboid(radius * 2.0, radius * 2.0, radius * 2.0),
                    Id(next_id),
                ));
                next_id += 1;
            }
        }
    }
}

#[test]
fn it_loads_plugin_without_errors() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = create_app();

    for _ in 0..500 {
        tick_60_fps(&mut app);
    }

    Ok(())
}

#[test]
fn body_with_velocity_moves_on_first_frame() {
    let mut app = create_app();

    app.insert_resource(Gravity::ZERO);

    app.add_systems(Startup, |mut commands: Commands| {
        // move right at 1 unit per second
        commands.spawn((
            SpatialBundle::default(),
            RigidBody::Dynamic,
            LinearVelocity(Vector::X),
            MassPropertiesBundle::new_computed(&Collider::ball(0.5), 1.0),
        ));
    });

    // one tick only.
    tick_60_fps(&mut app);

    let mut app_query = app.world.query::<(&Position, &RigidBody)>();

    let (pos, _body) = app_query.single(&app.world);

    assert!(pos.x > 0.0);
}

#[test]
fn body_with_velocity_moves() {
    let mut app = create_app();

    app.insert_resource(Gravity::ZERO);

    app.add_systems(Startup, |mut commands: Commands| {
        // move right at 1 unit per second
        commands.spawn((
            SpatialBundle::default(),
            RigidBody::Dynamic,
            LinearVelocity(Vector::X),
            MassPropertiesBundle::new_computed(&Collider::ball(0.5), 1.0),
        ));
    });

    const UPDATES: usize = 500;

    for _ in 0..UPDATES {
        tick_60_fps(&mut app);
    }

    let mut app_query = app.world.query::<(&Transform, &RigidBody)>();

    let (transform, _body) = app_query.single(&app.world);

    //assert!(transform.translation.x > 0., "box moves right");
    assert_relative_eq!(transform.translation.y, 0.);
    assert_relative_eq!(transform.translation.z, 0.);

    // make sure we end up in the expected position
    assert_relative_eq!(
        transform.translation.x,
        1. * UPDATES as f32 * 1. / 60.,
        epsilon = 0.03 // allow some leeway, as we might be one frame off
    );

    #[cfg(feature = "enhanced-determinism")]
    {
        setup_insta!();
        assert_debug_snapshot!(transform);
    }
}

#[derive(Component, Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
struct Id(usize);

#[cfg(all(feature = "3d", feature = "enhanced-determinism"))]
#[test]
fn cubes_simulation_is_deterministic_across_machines() {
    setup_insta!();
    let mut app = create_app();

    app.add_systems(Startup, setup_cubes_simulation);

    const SECONDS: usize = 10;
    const UPDATES: usize = 60 * SECONDS;

    for _ in 0..UPDATES {
        tick_60_fps(&mut app);
    }

    let mut app_query = app.world.query::<(&Id, &Transform)>();

    let mut bodies: Vec<(&Id, &Transform)> = app_query.iter(&app.world).collect();
    bodies.sort_by_key(|b| b.0);

    assert_debug_snapshot!(bodies);
}

#[cfg(feature = "3d")]
#[test]
fn cubes_simulation_is_locally_deterministic() {
    use itertools::Itertools;

    fn run_cubes() -> Vec<(Id, Transform)> {
        let mut app = create_app();

        app.add_systems(Startup, setup_cubes_simulation);

        const SECONDS: usize = 5;
        const UPDATES: usize = 60 * SECONDS;

        for _ in 0..UPDATES {
            tick_60_fps(&mut app);
        }

        let mut app_query = app.world.query::<(&Id, &Transform)>();

        let mut bodies: Vec<(Id, Transform)> = app_query
            .iter(&app.world)
            .map(|(id, transform)| (*id, *transform))
            .collect();
        bodies.sort_by_key(|b| b.0);
        bodies
    }

    // run simulation and check that results are equal each time
    for (a, b) in (0..4).map(|_| run_cubes()).tuple_windows() {
        assert_eq!(a, b);
    }
}

#[test]
fn no_ambiguity_errors() {
    #[derive(ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
    struct DeterministicSchedule;

    let mut app = App::new();

    app.add_plugins((MinimalPlugins, PhysicsPlugins::new(DeterministicSchedule)));

    #[cfg(feature = "async-collider")]
    {
        app.add_plugins((
            bevy::asset::AssetPlugin::default(),
            bevy::scene::ScenePlugin,
        ))
        .init_resource::<Assets<Mesh>>();
    }

    app.edit_schedule(DeterministicSchedule, |s| {
        s.set_build_settings(ScheduleBuildSettings {
            ambiguity_detection: LogLevel::Error,
            ..default()
        });
    })
    .add_systems(Update, |w: &mut World| {
        w.run_schedule(DeterministicSchedule);
    });

    app.update();
}
