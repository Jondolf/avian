//! Demonstrates Continuous Collision Detection (CCD) to prevent tunneling.
//!
//! Two forms of CCD are supported:
//!
//! 1. Speculative collision
//!     - Predicts approximate contacts before they happen.
//!     - Enabled by default for all rigid bodies.
//!     - Very efficient and relatively reliable.
//!     - Can sometimes cause ghost collisions.
//!     - Can sometimes miss collisions against objects spinning at very high speeds.
//!
//! 2. Swept CCD
//!     - Sweeps colliders from their previous positions to their current ones,
//!       and if a hit is found, moves the bodies to the time of impact.
//!     - Enabled for rigid bodies with the `SweptCcd` component.
//!     - Can prevent tunneling completely. More reliable than speculative collision.
//!     - More expensive than speculative collision.
//!     - Can cause "time loss" where bodies appear to stop for a moment
//!       because they are essentially brought back in time.
//!     - Two modes:
//!         1. Linear: Only considers translational motion.
//!         2. Non-linear: Considers both translation and rotation. More expensive.

use avian2d::{math::*, prelude::*};
use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::NEG_Y * 9.81 * 100.0))
        .add_systems(Startup, setup)
        .add_systems(Update, update_config)
        .add_systems(PhysicsSchedule, spawn_balls.in_set(PhysicsStepSet::First))
        .run();
}

#[derive(Component)]
struct SpeculativeCollisionText;

#[derive(Component)]
struct SweptCcdText;

fn setup(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());

    // Add two kinematic bodies spinning at high speeds.
    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                color: Color::srgb(0.7, 0.7, 0.8),
                custom_size: Some(Vec2::new(1.0, 400.0)),
                ..default()
            },
            transform: Transform::from_xyz(-200.0, -200.0, 0.0),
            ..default()
        },
        RigidBody::Kinematic,
        AngularVelocity(25.0),
        Collider::rectangle(1.0, 400.0),
        // Enable swept CCD for this body. Considers both translational and rotational motion by default.
        // This could also be on the ball projectiles.
        SweptCcd::default(),
    ));
    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                color: Color::srgb(0.7, 0.7, 0.8),
                custom_size: Some(Vec2::new(1.0, 400.0)),
                ..default()
            },
            transform: Transform::from_xyz(200.0, -200.0, 0.0),
            ..default()
        },
        RigidBody::Kinematic,
        AngularVelocity(-25.0),
        Collider::rectangle(1.0, 400.0),
        // Enable swept CCD for this body. Considers both translational and rotational motion by default.
        // This could also be on the ball projectiles.
        SweptCcd::default(),
    ));

    // Setup help text.
    let text_style = TextStyle {
        font_size: 20.0,
        ..default()
    };
    commands.spawn((
        SpeculativeCollisionText,
        TextBundle::from_sections([
            TextSection::new("(1) Speculative Collision: ", text_style.clone()),
            TextSection::new("On", text_style.clone()),
        ])
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(30.0),
            left: Val::Px(10.0),
            ..default()
        }),
    ));
    commands.spawn((
        SweptCcdText,
        TextBundle::from_sections([
            TextSection::new("(2) Swept CCD: ", text_style.clone()),
            TextSection::new(
                "Non-linear (considers both translation and rotation)",
                text_style,
            ),
        ])
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(50.0),
            left: Val::Px(10.0),
            ..default()
        }),
    ));
}

/// Spawns balls moving at the spinning objects at high speeds.
fn spawn_balls(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    time: Res<Time>,
) {
    let circle = Circle::new(2.0);

    // Compute the shooting direction.
    let (sin, cos) =
        (0.5 * time.elapsed_seconds_f64().adjust_precision().sin() - PI / 2.0).sin_cos();
    let direction = Vector::new(cos, sin).rotate(Vector::X);

    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(circle).into(),
            transform: Transform::from_xyz(0.0, 350.0, 0.0),
            material: materials.add(Color::srgb(0.2, 0.7, 0.9)),
            ..default()
        },
        RigidBody::Dynamic,
        LinearVelocity(2000.0 * direction),
        Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
        Collider::from(circle),
    ));
}

fn update_config(
    mut speculative_collision_text: Query<
        &mut Text,
        (With<SpeculativeCollisionText>, Without<SweptCcdText>),
    >,
    mut swept_ccd_text: Query<&mut Text, (Without<SpeculativeCollisionText>, With<SweptCcdText>)>,
    keys: Res<ButtonInput<KeyCode>>,
    mut narrow_phase_config: ResMut<NarrowPhaseConfig>,
    mut ccd_bodies: Query<&mut SweptCcd>,
) {
    // Toggle speculative collision.
    // Note: This sets the default speculative margin, but it can be overridden
    //       for individual entities with the `SpeculativeMargin` component.
    if keys.just_pressed(KeyCode::Digit1) {
        let mut text = speculative_collision_text.single_mut();
        if narrow_phase_config.default_speculative_margin == Scalar::MAX {
            narrow_phase_config.default_speculative_margin = 0.0;
            text.sections[1].value = "Off".to_string();
        } else {
            narrow_phase_config.default_speculative_margin = Scalar::MAX;
            text.sections[1].value = "On".to_string();
        }
    }

    // Change the sweep mode and whether swept CCD is enabled.
    if keys.just_pressed(KeyCode::Digit2) {
        let mut text = swept_ccd_text.single_mut();
        for mut swept_ccd in &mut ccd_bodies {
            if swept_ccd.mode == SweepMode::NonLinear && swept_ccd.include_dynamic {
                swept_ccd.mode = SweepMode::Linear;
                text.sections[1].value = "Linear (considers only translation)".to_string();
            } else if swept_ccd.mode == SweepMode::Linear {
                // Disable swept CCD for collisions against dynamic bodies.
                // To disable it completely, you should remove the component.
                swept_ccd.include_dynamic = false;

                swept_ccd.mode = SweepMode::NonLinear;
                text.sections[1].value = "Off".to_string();
            } else {
                // Enable swept CCD again.
                swept_ccd.include_dynamic = true;

                text.sections[1].value =
                    "Non-linear (considers both translation and rotation)".to_string();
            }
        }
    }
}
