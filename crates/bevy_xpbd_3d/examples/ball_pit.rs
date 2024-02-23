use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;
use examples_common_3d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(Msaa::Sample4)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands, assets: Res<AssetServer>) {
    commands.spawn(SceneBundle {
        scene: assets.load("ball_pit.gltf#Scene0"),
        ..default()
    });
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 1000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
        ..default()
    });
}
