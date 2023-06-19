# Bevy XPBD

[![MIT/Apache 2.0](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/Jondolf/bevy_xpbd#license)

[![2D crates.io](https://img.shields.io/crates/v/bevy_xpbd_2d)](https://crates.io/crates/bevy_xpbd_2d)
[![2D docs.rs](https://docs.rs/bevy_xpbd_2d/badge.svg)](https://docs.rs/bevy_xpbd_2d)

[![3D crates.io](https://img.shields.io/crates/v/bevy_xpbd_3d)](https://crates.io/crates/bevy_xpbd_3d)
[![3D docs.rs](https://docs.rs/bevy_xpbd_3d/badge.svg)](https://docs.rs/bevy_xpbd_3d)

**Bevy XPBD** is a 2D and 3D physics engine based on *Extended Position Based Dynamics* (XPBD)
for the [Bevy game engine](https://bevyengine.org/).

## Design

Below are some of the core design principles used in Bevy XPBD.

- Made with Bevy, for Bevy. No wrappers around existing engines.
- Provide an ergonomic and familiar API. Ergonomics is key for a good experience.
- Utilize the ECS as much as possible. The engine should feel like a part of Bevy, and it shouldn't
need to maintain a separate physics world.
- Use a highly modular plugin architecture. Users should be able to
replace parts of the engine with their own implementations.
- Have good documentation. A physics engine is pointless if you don't know how to use it.

## Features

Below are some of the current features of Bevy XPBD.

- Dynamic, kinematic and static rigid bodies
- Colliders powered by [parry](https://parry.rs)
    - Collision events: `Collision`, `CollisionStarted`, `CollisionEnded`
    - Access to colliding entities with `CollidingEntities`
    - Sensor colliders
    - Collision layers
- Material properties like restitution and friction
- External forces and torque
- Gravity
- Joints
- Built-in constraints and support for custom constraints
- Automatically deactivating bodies with `Sleeping`
- Configurable timesteps and substepping
- `f32`/`f64` precision (`f32` by default)

## Documentation

- [`bevy_xpbd_2d`](https://docs.rs/bevy_xpbd_2d)
- [`bevy_xpbd_3d`](https://docs.rs/bevy_xpbd_3d)

## Usage example

For a 2D game, add the `bevy_xpbd_2d` crate to your `Cargo.toml` like this:

```toml
[dependencies]
bevy_xpbd_2d = "0.1"
```

Similarly for a 3D game, add `bevy_xpbd_3d`:

```toml
[dependencies]
bevy_xpbd_3d = "0.1"
```

Below is a very simple example where a box with initial angular velocity falls onto a plane. This is a modified version of Bevy's [3d_scene](https://bevyengine.org/examples/3d/3d-scene/) example.

```rs
use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins)
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
        RigidBody::Static,
        Collider::cuboid(8.0, 0.002, 8.0),
    ));
    // Cube
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
            ..default()
        },
        RigidBody::Dynamic,
        Position(Vec3::Y * 4.0),
        AngularVelocity(Vec3::new(2.5, 3.4, 1.6)),
        Collider::cuboid(1.0, 1.0, 1.0),
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

## More examples

You can find lots of 2D and 3D examples in [/crates/bevy_xpbd_2d/examples](/crates/bevy_xpbd_2d/examples) and [/crates/bevy_xpbd_3d/examples](/crates/bevy_xpbd_3d/examples) respectively.

The examples support both `f32` and `f64` precisions, so the code contains some feature-dependent types like `Scalar` and `Vector`.
In actual usage these are not needed, so you can just use `f32` or `f64` types depending on the features you have chosen.

By default the examples use `f32`. To run the `f64` versions, you need to disable default features and manually choose the dimension
and precision:

```
cargo run --example cubes --no-default-features --features "3d f64"
```

## Supported Bevy versions

| Bevy | Bevy XPBD |
| ---- | --------- |
| 0.10 | 0.1       |

## Future features

- On-demand simulation stepping
- Linear and angular velocity damping
- Locking translational and rotational axes without joints
- Joint motors
- Spatial queries
- Continuous collision detection
- Multiple colliders per body
- Debug render colliders and joints
- Performance optimization (better broad phase, parallel solver...)
- Soft bodies
  - Cloth
  - Deformable solids
- Maybe fluid simulation

## Contributing

If you encounter any problems, feel free to open issues. Creating pull requests is encouraged
as well, but especially for larger changes and additions it's better to open an issue first.

You can also ask for help or ask questions on the [Bevy Discord server](https://discord.com/invite/gMUk5Ph)
where you can find me as `Jondolf`.

## License

Bevy XPBD is free and open source. All code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](/LICENSE-MIT) or http://opensource.org/licenses/MIT)
- Apache License, Version 2.0 ([LICENSE-APACHE](/LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)

at your option.
