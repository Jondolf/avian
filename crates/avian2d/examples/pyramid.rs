use avian2d::{math::Scalar, prelude::*};
use bevy::{prelude::*, render::camera::ScalingMode};
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    let mut app = App::new();

    app.add_plugins((
        DefaultPlugins,
        PhysicsPlugins::default(),
        ExampleCommonPlugin,
    ));

    setup(&mut app);

    app.run();
}

fn setup(app: &mut App) {
    app.insert_resource(SubstepCount(6));
    app.add_systems(
        Startup,
        move |mut commands: Commands,
              mut meshes: ResMut<Assets<Mesh>>,
              mut materials: ResMut<Assets<ColorMaterial>>| {
            commands.spawn((
                Camera2d,
                Projection::Orthographic(OrthographicProjection {
                    scaling_mode: ScalingMode::FixedHorizontal {
                        viewport_width: 150.0,
                    },
                    ..OrthographicProjection::default_2d()
                }),
                Transform::from_xyz(0.0, 30.0, 0.0),
            ));

            // Ground
            commands.spawn((
                RigidBody::Static,
                Collider::rectangle(800.0, 40.0),
                Position::from_xy(0.0, -20.0),
                Mesh2d(meshes.add(Rectangle::new(800.0, 40.0))),
                MeshMaterial2d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
            ));

            let base_count = 50;
            let h = 0.5;
            let box_size = 2.0 * h;
            let collider = Collider::rectangle(box_size, box_size);
            let shift = h;
            for i in 0..base_count {
                let y = (2.0 * i as Scalar + 1.0) * shift * 0.99;

                for j in i..base_count {
                    let x = (i as Scalar + 1.0) * shift + 2.0 * (j - i) as Scalar * shift
                        - h * base_count as Scalar;

                    commands.spawn((
                        RigidBody::Dynamic,
                        collider.clone(),
                        Position::from_xy(x, y),
                        Mesh2d(meshes.add(Rectangle::new(box_size, box_size))),
                        MeshMaterial2d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
                    ));
                }
            }
        },
    );
}

// 0.4 ms
