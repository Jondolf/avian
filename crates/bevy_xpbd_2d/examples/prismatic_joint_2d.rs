use bevy::prelude::*;
use bevy_xpbd_2d::prelude::*;
use examples_common_2d::XpbdExamplePlugin;

fn setup(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());

    let square = SpriteBundle {
        sprite: Sprite {
            color: Color::rgb(0.2, 0.7, 0.9),
            custom_size: Some(Vec2::splat(50.0)),
            ..default()
        },
        ..default()
    };

    let anchor = commands
        .spawn((square.clone(), RigidBody::Kinematic, AngularVelocity(1.5)))
        .id();

    let object = commands
        .spawn((
            square,
            RigidBody::Dynamic,
            Position(Vector::X * 100.0),
            MassPropertiesBundle::new_computed(&Collider::cuboid(50.0, 50.0), 1.0),
        ))
        .id();

    commands.spawn(
        PrismaticJoint::new(anchor, object)
            .with_local_anchor_1(Vector::X * 50.0)
            .with_free_axis(Vector::X)
            .with_limits(25.0, 100.0),
    );
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(50))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_startup_system(setup)
        .run();
}
