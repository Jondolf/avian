use avian3d::prelude::*;
use bevy::prelude::*;
use big_space::prelude::*;
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins((
            BigSpacePlugin::default(),
            FloatingOriginDebugPlugin::default(),
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            ExampleCommonPlugin,
        ))
        .insert_resource(Gravity(Vec3::ZERO))
        .add_systems(Startup, spawn_big_space)
        .add_systems(Update, move_cube)
        .add_systems(
            PostUpdate,
            move_camera.before(TransformSystem::TransformPropagate),
        )
        .run();
}

#[derive(Component)]
struct WasdControlled;

#[derive(Component)]
struct MainCamera;

fn spawn_big_space(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn_big_space(Grid::new(5.0, 0.0), |root| {
        // Camera with no physics components
        root.spawn_spatial((
            FloatingOrigin,
            Transform::from_xyz(0.0, 0.0, 25.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
            Camera::default(),
            Camera3d::default(),
            Projection::Perspective(PerspectiveProjection::default()),
            MainCamera,
        ));

        let cube_mesh = meshes.add(Cuboid::default());

        // WASD controlled cube
        root.spawn_spatial((
            Mesh3d(cube_mesh.clone()),
            MeshMaterial3d(materials.add(Color::WHITE)),
            Transform::IDENTITY,
            RigidBody::Dynamic,
            Collider::cuboid(1.0, 1.0, 1.0),
            WasdControlled,
            LinearVelocity::default(),
            LinearDamping::default(),
            TransformInterpolation,
        ));
    });
}

fn move_cube(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut LinearVelocity, &mut LinearDamping), With<WasdControlled>>,
) {
    let mut velocity = Vec3::ZERO;
    if keyboard_input.pressed(KeyCode::KeyW) {
        velocity += Vec3::Y * 0.1;
    }
    if keyboard_input.pressed(KeyCode::KeyS) {
        velocity -= Vec3::Y * 0.1;
    }
    if keyboard_input.pressed(KeyCode::KeyA) {
        velocity -= Vec3::X * 0.1;
    }
    if keyboard_input.pressed(KeyCode::KeyD) {
        velocity += Vec3::X * 0.1;
    }

    for (mut linear_velocity, _) in &mut query {
        linear_velocity.0 += velocity;
    }

    let linear_damping_value = if keyboard_input.pressed(KeyCode::Space) {
        10.0
    } else {
        0.0
    };

    for (_, mut linear_damping) in &mut query {
        linear_damping.0 = linear_damping_value;
    }
}

fn move_camera(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut Transform, With<MainCamera>>,
    time: Res<Time>,
) {
    let camera_speed = 5.0;

    for mut transform in &mut query {
        let mut movement = Vec3::ZERO;

        if keyboard_input.pressed(KeyCode::ArrowUp) {
            movement += Vec3::Y;
        }
        if keyboard_input.pressed(KeyCode::ArrowDown) {
            movement -= Vec3::Y;
        }
        if keyboard_input.pressed(KeyCode::ArrowLeft) {
            movement -= Vec3::X;
        }
        if keyboard_input.pressed(KeyCode::ArrowRight) {
            movement += Vec3::X;
        }

        if movement != Vec3::ZERO {
            movement = movement.normalize() * camera_speed * time.delta_secs();
            transform.translation += movement;
        }
    }
}
