use bevy::prelude::*;
use bevy_xpbd_2d::{math::*, prelude::*};
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(50))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());

    let square_sprite = Sprite {
        color: Color::rgb(0.2, 0.7, 0.9),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    let anchor = commands
        .spawn((
            SpriteBundle {
                sprite: square_sprite.clone(),
                ..default()
            },
            RigidBody::Kinematic,
        ))
        .id();

    let object = commands
        .spawn((
            SpriteBundle {
                sprite: square_sprite,
                transform: Transform::from_xyz(100.0, 0.0, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            MassPropertiesBundle::new_computed(&Collider::rectangle(50.0, 50.0), 1.0),
        ))
        .id();

    commands.spawn(
        DistanceJoint::new(anchor, object)
            .with_local_anchor_1(Vector::ZERO)
            .with_local_anchor_2(Vector::ZERO)
            .with_rest_length(100.0)
            .with_linear_velocity_damping(0.1)
            .with_angular_velocity_damping(1.0)
            .with_compliance(0.00000001),
    );
}
