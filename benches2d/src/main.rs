use avian2d::prelude::SubstepCount;
use benches_cli::Benchmark;
use bevy::{
    MinimalPlugins,
    app::{App, Plugin, PluginGroup, PluginGroupBuilder},
    asset::AssetPlugin,
    scene::ScenePlugin,
    time::{Time, TimeUpdateStrategy},
    transform::TransformPlugin,
};
use core::time::Duration;

mod large_pyramid;
mod many_pyramids;

/// All benchmarks for `avian2d`.
pub const BENCHMARKS: &[Benchmark] = &[
    Benchmark::new("Large Pyramid 2D", "pyramid2d", || {
        large_pyramid::create_bench(100)
    }),
    Benchmark::new("Many Pyramids 2D", "many_pyramids2d", || {
        many_pyramids::create_bench(10, 10, 10)
    }),
];

fn main() {
    let args = benches_cli::Args::parse();

    if args.list {
        benches_cli::list(BENCHMARKS);
        return;
    }

    if let Some(name) = &args.name {
        // Run the given benchmark with the specified number of threads.
        if let Some(benchmark) = BENCHMARKS.iter().find(|b| b.name == name) {
            benches_cli::run(benchmark, &args);
        } else if let Ok(Some(benchmark)) = name.parse::<usize>().map(|i| BENCHMARKS.get(i - 1)) {
            benches_cli::run(benchmark, &args);
        } else {
            eprintln!("Benchmark '{name}' not found. Use --list to see available benchmarks.",);
        }
    } else {
        // If no specific benchmark is requested, run all benchmarks.
        benches_cli::run_all(BENCHMARKS, &args);
    }
}

/// A plugin group that includes the minimal set of plugins used for benchmarking `avian2d`.
pub struct BenchmarkPlugins;

impl PluginGroup for BenchmarkPlugins {
    fn build(self) -> bevy::app::PluginGroupBuilder {
        PluginGroupBuilder::start::<BenchmarkPlugins>()
            .add_group(MinimalPlugins)
            .add(BenchmarkCorePlugin)
            .add(TransformPlugin)
            .add(AssetPlugin::default())
            .add(ScenePlugin)
    }
}

/// A plugin that sets up the core resources and configuration for benchmarking.
struct BenchmarkCorePlugin;

impl Plugin for BenchmarkCorePlugin {
    fn build(&self, app: &mut App) {
        // All benchmarks use a fixed time step of 60 FPS with 4 substeps.
        app.insert_resource(Time::from_hz(60.0));
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 60.0,
        )));
        app.insert_resource(SubstepCount(4));
    }
}
