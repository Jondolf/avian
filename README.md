# Bevy XPBD

Bevy XPBD is a 2D and 3D physics engine based on Extended Position Based Dynamics (XPBD) for the [Bevy game engine](https://bevyengine.org/).

XPBD is an improved variant of traditional position based dynamics. It provides unconditionally stable, time step independent and physically accurate simulations that use simple constraint projection to handle things like contacts, joints, and interactions between rigid bodies, soft bodies and fluids.

## Stability warning

Bevy XPBD is in early development, and it has not been released on [crates.io](https://crates.io) yet. There are several stability and performance issues, missing features, and a lack of proper documentation, so for the time being, I recommend using alternatives like [bevy_rapier](https://github.com/dimforge/bevy_rapier) for any serious projects.

That being said, I hope to release 0.1.0 in the not-so-distant future, and I plan to eventually reach feature parity with established physics engines. In the meantime, feel free to experiment with the engine, and consider opening issues or pull requests for any problems you may encounter.

## Usage example

> **Note**: Since the crate isn't available on [crates.io](https://crates.io) yet, you will have to clone the Git repo to test it out.

Below is a very simple example where a box with initial angular velocity falls onto a plane. This is a modified version of Bevy's [3d_scene example](https://bevyengine.org/examples/3d/3d-scene/).

```rs
use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(XpbdPlugin)
        .add_startup_system(setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Plane
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Plane::from_size(8.0))),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
            ..default()
        },
        RigidBodyBundle::new_static(),
        ColliderBundle::new(&Shape::cuboid(4.0, 0.001, 4.0), 1.0),
    ));
    // Cube
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
            ..default()
        },
        RigidBodyBundle::new_dynamic()
            .with_pos(Vec3::Y * 4.0)
            .with_ang_vel(Vec3::new(2.5, 3.4, 1.6)),
        ColliderBundle::new(&Shape::cuboid(0.5, 0.5, 0.5), 1.0),
    ));
    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-4.0, 6.5, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}
```

https://user-images.githubusercontent.com/57632562/230185604-b40441a2-48d8-4566-9b9e-be4825f4877e.mp4

To see more complete examples of the various features of Bevy XPBD, check out the 2D and 3D examples in [/crates/bevy_xpbd_2d/examples](/crates/bevy_xpbd_2d/examples) and [/crates/bevy_xpbd_3d/examples](/crates/bevy_xpbd_3d/examples) respectively.

## Current features

- 2D and 3D support
- Dynamic, kinematic and static rigid bodies
- Collision detection via [parry](https://parry.rs)
- Basic joints
  - Revolute joint (or hinge joint), optional angle limits
  - Spherical joint, optional swing and twist angle limits
  - Prismatic joint, one free translational axis with optional limits
  - Fixed joint
- Joint damping
- Gravity
- External forces
- Restitution
- Friction
- Substepping
- Configurable timesteps
- Determinism
- Choose between `f32` and `f64`

## Future features

- On-demand simulation stepping
- Linear and angular velocity damping
- Locking translational and rotational axes without joints
- Joint motors
- Spatial queries
- Continuous collision detection
- Multiple colliders per body
- Sensor colliders
- Access, filter and modify contact data
- Debug render colliders, joints, contacts etc.
- Performance optimization (sleeping, multithreading...)
- Soft bodies
  - Cloth
  - Deformable solids
- Maybe fluid simulation

## Inspirations and resources

I recommend checking out [Johan Helsing's](https://github.com/johanhelsing) amazing (but incomplete) [tutorial series on XPBD in Bevy](https://johanhelsing.studio/posts/bevy-xpbd). He inspired and helped me build this engine, and the series serves as a great practical walkthrough of how to make a basic XPBD physics engine with Bevy's ECS.

To understand the algorithm better, it's also worth checking out some of the papers:

  - Müller M, Macklin M, Chentanez N, Jeschke S, Kim T. 2020. *[Detailed Rigid Body Simulation with Extended Position Based Dynamics](https://matthias-research.github.io/pages/publications/PBDBodies.pdf)*
  - Macklin M, Müller M, Chentanez N. 2016. *[XPBD: Position-Based Simulation of Compliant Constrained Dynamics](http://mmacklin.com/xpbd.pdf)*

## License

Bevy XPBD is free and open source. All code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](/LICENSE-MIT) or http://opensource.org/licenses/MIT)
- Apache License, Version 2.0 ([LICENSE-APACHE](/LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
