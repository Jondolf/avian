//! A simple ray casting example that uses the [`RayCaster`] component.
//!
//! An alternative, more controlled approach is to use the methods of
//! the [`SpatialQuery`] system parameter.

#![allow(clippy::unnecessary_cast)]

use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_xpbd_2d::{math::*, prelude::*};
use examples_common_2d::*;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .add_systems(Update, render_rays)
        .add_systems(Startup, setup)
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

fn render_rays(mut rays: Query<(&mut RayCaster, &mut RayHits)>, mut gizmos: Gizmos) {
    for (ray, hits) in &mut rays {
        // Convert to Vec3 for lines
        let origin = ray.global_origin().as_f32();
        let direction = ray.global_direction().as_f32();

        for hit in hits.iter() {
            gizmos.line_2d(
                origin,
                origin + direction * hit.time_of_impact as f32,
                Color::GREEN,
            );
        }
        if hits.is_empty() {
            gizmos.line_2d(origin, origin + direction * 1_000_000.0, Color::ORANGE_RED);
        }
    }
}
