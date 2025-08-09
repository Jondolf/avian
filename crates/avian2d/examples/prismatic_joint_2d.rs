use avian2d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(50))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands, mut time: ResMut<Time<Physics>>) {
    time.pause();

    commands.spawn(Camera2d);

    let square_sprite = Sprite {
        color: Color::srgb(0.2, 0.7, 0.9),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    let anchor = commands
        .spawn((
            square_sprite.clone(),
            RigidBody::Kinematic,
            //AngularVelocity(1.5),
            Transform::from_rotation(Quat::from_rotation_z(0.5)),
        ))
        .id();

    let object = commands
        .spawn((
            square_sprite,
            Transform::from_xyz(100.0, 0.0, 0.0),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Rectangle::from_length(50.0), 1.0),
        ))
        .id();

    commands.spawn(
        PrismaticJoint::new(anchor, object)
            .with_local_rotation1(Quat::from_rotation_z(-0.5))
            .with_local_rotation2(Quat::from_rotation_z(0.0))
            .with_free_axis(Vector::X)
            .with_limits(-75.0, 75.0),
    );
}
