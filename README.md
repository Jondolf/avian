# Bevy XPBD

[![MIT/Apache 2.0](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/Jondolf/bevy_xpbd#license)
[![ci](https://github.com/Jondolf/bevy_xpbd/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Jondolf/bevy_xpbd/actions/workflows/ci.yml)
[![2D crates.io](https://img.shields.io/crates/v/bevy_xpbd_2d?label=2D%20crates.io)](https://crates.io/crates/bevy_xpbd_2d)
[![2D docs.rs](https://img.shields.io/docsrs/bevy_xpbd_2d?label=2D%20docs.rs)](https://docs.rs/bevy_xpbd_2d)
[![3D crates.io](https://img.shields.io/crates/v/bevy_xpbd_3d?label=3D%20crates.io)](https://crates.io/crates/bevy_xpbd_3d)
[![3D docs.rs](https://img.shields.io/docsrs/bevy_xpbd_3d?label=3D%20docs.rs)](https://docs.rs/bevy_xpbd_3d)

**Bevy XPBD** is a 2D and 3D physics engine based on _Extended Position Based Dynamics_ (XPBD)
for the [Bevy game engine](https://bevyengine.org/).

---

## Warning ⚠️

Bevy XPBD will undergo major changes in the next release, coming out in tandem with Bevy 0.14.

You can find an in-depth explanation of the situation and plans
[here](https://github.com/Jondolf/bevy_xpbd/issues/346).

---

## Design

Below are some of the core design principles used in Bevy XPBD.

- **Made with Bevy, for Bevy.** No wrappers around existing engines.
- **Provide an ergonomic and familiar API.** Ergonomics is key for a good experience.
- **Utilize the ECS as much as possible.** The engine should feel like a part of Bevy, and it shouldn't
    need to maintain a separate physics world.
- **Use a highly modular plugin architecture.** Users should be able to replace parts of the engine
    with their own implementations.
- **Have good documentation.** A physics engine is pointless if you don't know how to use it.

## Features

Below are some of the current features of Bevy XPBD.

- Dynamic, kinematic and static rigid bodies
  - Linear and angular velocity
  - External forces, torque and impulses
  - Gravity and gravity scale
  - Linear and angular damping
  - Locking translational and rotational axes
  - Rigid body dominance
  - Automatic deactivation with sleeping
- Collision detection powered by [Parry](https://parry.rs)
  - Colliders with configurable collision layers, density, material properties and more
  - Collision events
  - Access to colliding entities
  - Filtering and modifying collisions with custom systems
  - Manual contact queries and intersection tests
- Constraints and joints
  - Flexible API for creating position-based constraints
  - Several built-in joint types: fixed, distance, prismatic, revolute, spherical
  - Support for custom joints and other constraints
- Spatial queries
  - Raycasting, shapecasting, point projection and intersection tests
  - Ergonomic component-based API for raycasts and shapecasts
  - Flexible `SpatialQuery` system parameter
  - Spatial query filters
- Debug rendering for colliders, AABBs, contacts, joints, sleeping, axes and spatial queries
- Configurable scheduling and high customizability
- Highly modular plugin architecture, freely extend and replace parts of the engine
- Support for custom collision backends
- `f32`/`f64` precision (`f32` by default)

You can find a more complete list along with documentation in the
[Table of contents](https://docs.rs/bevy_xpbd_3d/latest/bevy_xpbd_3d/#table-of-contents)
on docs.rs.

## Documentation

- [2D documentation](https://docs.rs/bevy_xpbd_2d)
- [3D documentation](https://docs.rs/bevy_xpbd_3d)

## Usage example

First, add `bevy_xpbd_2d` or `bevy_xpbd_3d` to your dependencies in `Cargo.toml`:

```toml
# For 2D applications:
[dependencies]
bevy_xpbd_2d = "0.4"

# For 3D applications:
[dependencies]
bevy_xpbd_3d = "0.4"

# If you want to use the most up-to-date version, you can follow the main branch:
[dependencies]
bevy_xpbd_3d = { git = "https://github.com/Jondolf/bevy_xpbd", branch = "main" }
```

Below is a very simple example where a box with initial angular velocity falls onto a plane. This is a modified version of Bevy's [3d_scene](https://bevyengine.org/examples/3d/3d-scene/) example.

```rust
use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Plane
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(8.0, 0.002, 8.0),
        PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh().size(8.0, 8.0)),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3)),
            ..default()
        },
    ));

    // Cube
    commands.spawn((
        RigidBody::Dynamic,
        AngularVelocity(Vec3::new(2.5, 3.4, 1.6)),
        Collider::cuboid(1.0, 1.0, 1.0),
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_xyz(0.0, 4.0, 0.0),
            ..default()
        },
    ));

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 2_000_000.0,
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

<https://user-images.githubusercontent.com/57632562/230185604-b40441a2-48d8-4566-9b9e-be4825f4877e.mp4>

## More examples

You can find lots of 2D and 3D examples in [/crates/bevy_xpbd_2d/examples](/crates/bevy_xpbd_2d/examples) and [/crates/bevy_xpbd_3d/examples](/crates/bevy_xpbd_3d/examples) respectively.

The examples support both `f32` and `f64` precisions, so the code contains some feature-dependent types like `Scalar` and `Vector`.
In actual usage these are not needed, so you can just use `f32` or `f64` types depending on the features you have chosen.

By default the examples use `f32`. To run the `f64` versions, you need to disable default features and manually choose the dimension
and precision:

```shell
# Manually specify dimension and precision. `parry-f64` enables collision detection using Parry.
cargo run --example cubes --no-default-features --features "3d f64 parry-f64"
```

## Supported Bevy versions

| Bevy | Bevy XPBD |
| ---- | --------- |
| 0.13 | 0.4       |
| 0.12 | 0.3       |
| 0.11 | 0.2       |
| 0.10 | 0.1       |

## Future features

- Continuous collision detection (CCD)
- Per-entity collision hooks or callbacks
- Flags for what types of collisions are active, like collisions against specific rigid body types, sensors or parents
- Performance optimization (better broad phase, parallel solver...)
- Joint motors
- Articulations, aka. multibody joints
- Proper cross-platform determinism
- Soft bodies (cloth and deformable solids)
- Maybe fluid simulation

## Contributing

If you encounter any problems, feel free to open issues or create pull requests.
For larger changes and additions, it's better to open an issue or ask me for input
before making a pull request.

You can also ask for help or ask questions on the [Bevy Discord](https://discord.com/invite/gMUk5Ph)
server's `bevy_xpbd` thread in `#crate-help`. My username on the Discord is `Jondolf` (`@jondolfdev`).

## Acknowledgements

Huge thanks to the entire Bevy community for the incredible support!
All of your contributions, insight and requests are a massive help
in driving the state of physics in Bevy forward, and it's what
keeps me motivated to build the best engine I can.

I would also like to give a special thanks to [Johan Helsing][johan-helsing]
for inspiring this project and helping me significantly in the early stages.
His original [tutorial series][johan-xpbd-tutorial] is the reason `bevy_xpbd`
exists in the first place, and without his support and contributions,
the project wouldn't be anywhere near where it is today.

[johan-helsing]: https://github.com/johanhelsing
[johan-xpbd-tutorial]: https://johanhelsing.studio/posts/bevy-xpbd

## License

Bevy XPBD is free and open source. All code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](/LICENSE-MIT) or <http://opensource.org/licenses/MIT>)
- Apache License, Version 2.0 ([LICENSE-APACHE](/LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)

at your option.
