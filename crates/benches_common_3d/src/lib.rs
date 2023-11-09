use bevy::{app::PluginsState, prelude::*};
use bevy_xpbd_3d::prelude::*;
use criterion::{measurement::Measurement, BatchSize, Bencher};

pub fn bench_app<M: Measurement>(
    bencher: &mut Bencher<'_, M>,
    steps: u32,
    setup: impl Fn(&mut App),
) {
    bencher.iter_batched_ref(
        move || {
            let mut app = App::new();

            app.add_plugins((
                MinimalPlugins,
                HierarchyPlugin,
                TransformPlugin,
                PhysicsPlugins::default(),
            ));

            app.insert_resource(Time::new_with(Physics::fixed_once_hz(60.0)));

            setup(&mut app);

            while app.plugins_state() != PluginsState::Ready {
                bevy::tasks::tick_global_task_pools_on_main_thread();
            }

            app.finish();
            app.cleanup();
            app
        },
        move |app| {
            for _ in 0..steps {
                app.update();
            }
        },
        BatchSize::PerIteration,
    );
}
