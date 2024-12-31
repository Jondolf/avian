use avian3d::{math::*, prelude::*};
use bevy::{
    ecs::system::{lifetimeless::Read, SystemParam},
    prelude::*,
};
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default().with_collision_hooks::<ConveyorHooks>(),
        ))
        .add_systems(Startup, setup)
        .run();
}

#[derive(SystemParam)]
struct ConveyorHooks<'w, 's> {
    conveyor_query: Query<'w, 's, (Read<ConveyorBelt>, Read<GlobalTransform>)>,
}

impl CollisionHooks for ConveyorHooks<'_, '_> {
    fn modify_contacts(&self, contacts: &mut Contacts, _commands: &mut Commands) -> bool {
        let (Ok((conveyor_belt, global_transform)), sign) = self
            .conveyor_query
            .get(contacts.entity1)
            .map_or((self.conveyor_query.get(contacts.entity2), 1.0), |q| {
                (Ok(q), -1.0)
            })
        else {
            return false;
        };

        for manifold in contacts.manifolds.iter_mut() {
            let direction = global_transform.rotation() * conveyor_belt.local_direction;
            manifold.tangent_velocity = sign * conveyor_belt.speed * direction;
        }

        true
    }
}

#[derive(Component)]
#[require(ActiveCollisionHooks(|| ActiveCollisionHooks::MODIFY_CONTACTS))]
struct ConveyorBelt {
    local_direction: Vector,
    speed: Scalar,
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let long_conveyor = Cuboid::new(18.0, 0.1, 6.0);
    let short_conveyor = Cuboid::new(14.0, 0.1, 6.0);

    let long_conveyor_mesh = meshes.add(long_conveyor);
    let short_conveyor_mesh = meshes.add(short_conveyor);

    let long_conveyor_material = materials.add(Color::srgb(0.3, 0.3, 0.3));
    let short_conveyor_material = materials.add(Color::srgb(0.2, 0.2, 0.2));

    // Spawn conveyor belts
    commands.spawn((
        RigidBody::Static,
        Collider::from(long_conveyor),
        Friction::new(1.0),
        ConveyorBelt {
            local_direction: Vec3::X,
            speed: 6.0,
        },
        Transform::from_xyz(-3.0, -0.25, 7.0)
            .with_rotation(Quat::from_rotation_z(2_f32.to_radians())),
        Mesh3d(long_conveyor_mesh.clone()),
        MeshMaterial3d(long_conveyor_material.clone()),
    ));
    commands.spawn((
        RigidBody::Static,
        Collider::from(long_conveyor),
        Friction::new(1.0),
        ConveyorBelt {
            local_direction: Vec3::NEG_X,
            speed: 6.0,
        },
        Transform::from_xyz(3.0, -0.25, -7.0)
            .with_rotation(Quat::from_rotation_z(-2_f32.to_radians())),
        Mesh3d(long_conveyor_mesh),
        MeshMaterial3d(long_conveyor_material.clone()),
    ));
    commands.spawn((
        RigidBody::Static,
        Collider::from(short_conveyor),
        Friction::new(1.0),
        ConveyorBelt {
            local_direction: Vec3::X,
            speed: 3.0,
        },
        Transform::from_xyz(9.0, -0.25, 3.0).with_rotation(
            Quat::from_rotation_y(90_f32.to_radians()) * Quat::from_rotation_z(2_f32.to_radians()),
        ),
        Mesh3d(short_conveyor_mesh.clone()),
        MeshMaterial3d(short_conveyor_material.clone()),
    ));
    commands.spawn((
        RigidBody::Static,
        Collider::from(short_conveyor),
        Friction::new(1.0),
        ConveyorBelt {
            local_direction: Vec3::NEG_X,
            speed: 3.0,
        },
        Transform::from_xyz(-9.0, -0.25, -3.0).with_rotation(
            Quat::from_rotation_y(90_f32.to_radians()) * Quat::from_rotation_z(-2_f32.to_radians()),
        ),
        Mesh3d(short_conveyor_mesh),
        MeshMaterial3d(short_conveyor_material),
    ));

    // Spawn cube stacks
    let cuboid_mesh = meshes.add(Cuboid::default());
    let cuboid_material = materials.add(Color::srgb(0.2, 0.7, 0.9));
    for x in -2..2 {
        for y in 0..3 {
            for z in -2..2 {
                let position = Vec3::new(x as f32 + 10.0, y as f32 + 0.5, z as f32);
                commands.spawn((
                    RigidBody::Dynamic,
                    // This small margin just helps prevent hitting internal edges
                    // while sliding from one conveyor to another.
                    CollisionMargin(0.01),
                    Collider::cuboid(0.98, 0.98, 0.98),
                    Transform::from_translation(position),
                    Mesh3d(cuboid_mesh.clone()),
                    MeshMaterial3d(cuboid_material.clone()),
                ));
            }
        }
    }

    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(Vec3::new(20.0, 10.0, 20.0)).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
