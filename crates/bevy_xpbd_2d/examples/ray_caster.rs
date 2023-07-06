//! A simple ray casting example that uses the [`RayCaster`] component.
//!
//! An alternative, more controlled approach is to use the methods of
//! the [`SpatialQuery`] system parameter.

#![allow(clippy::unnecessary_cast)]

use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_xpbd_2d::prelude::*;
use examples_common_2d::{bevy_prototype_debug_lines::*, *};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdExamplePlugin)
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .add_system(render_rays)
        .add_startup_system(setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2dBundle::default());

    // Spawn a perimeter of circles that the ray will be cast against
    let radius = 16.0;
    for x in -4..=4 {
        for y in -4..=4 {
            if (-3..4).contains(&x) && (-3..4).contains(&y) {
                continue;
            }

            commands.spawn((
                MaterialMesh2dBundle {
                    mesh: meshes.add(shape::Circle::new(radius as f32).into()).into(),
                    material: materials.add(ColorMaterial::from(Color::rgb(0.2, 0.7, 0.9))),
                    ..default()
                },
                RigidBody::Kinematic,
                Position(Vector::new(
                    x as Scalar * radius * 3.0,
                    y as Scalar * radius * 3.0,
                )),
                Collider::ball(radius),
            ));
        }
    }

    // Spawn a rotating kinematic body with a ray caster
    commands.spawn((
        RigidBody::Kinematic,
        AngularVelocity(0.2),
        RayCaster::new(Vector::ZERO, Vector::X),
    ));
}

fn render_rays(mut rays: Query<(&mut RayCaster, &mut RayHits)>, mut lines: ResMut<DebugLines>) {
    for (ray, hits) in &mut rays {
        // Convert to Vec3 for lines
        let origin = ray.global_origin().extend(0.0).as_f32();
        let direction = ray.global_direction().extend(0.0).as_f32();

        for hit in hits.iter() {
            lines.line_colored(
                origin,
                origin + direction * hit.time_of_impact as f32,
                0.001,
                Color::GREEN,
            );
        }
        if hits.is_empty() {
            lines.line_colored(
                origin,
                origin + direction * 1_000_000.0,
                0.001,
                Color::ORANGE_RED,
            );
        }
    }
}
