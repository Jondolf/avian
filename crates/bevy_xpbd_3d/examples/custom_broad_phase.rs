use bevy::prelude::*;
use bevy_xpbd_3d::{math::*, prelude::*, PhysicsSchedule, PhysicsStepSet};

fn main() {
    let mut app = App::new();

    app.add_plugins(DefaultPlugins);

    // Add PhysicsPlugins and replace default broad phase with our custom broad phase
    app.add_plugins(
        PhysicsPlugins::default()
            .build()
            .disable::<BroadPhasePlugin>()
            .add(BruteForceBroadPhasePlugin),
    );

    app.add_systems(Startup, setup).run();
}

// Modified from Bevy's 3d_scene example, a cube falling to the ground with velocity
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Plane
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh().size(8.0, 8.0)),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(8.0, 0.002, 8.0),
    ));
    // Cube
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_xyz(0.0, 4.0, 0.0),
            ..default()
        },
        RigidBody::Dynamic,
        AngularVelocity(Vector::new(2.5, 3.4, 1.6)),
        Collider::cuboid(1.0, 1.0, 1.0),
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

// Collects pairs of potentially colliding entities into the BroadCollisionPairs resource provided by the physics engine.
// A brute force algorithm is used for simplicity.
pub struct BruteForceBroadPhasePlugin;

impl Plugin for BruteForceBroadPhasePlugin {
    fn build(&self, app: &mut App) {
        // Make sure the PhysicsSchedule is available
        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        // Add the broad phase system into the broad phase set
        physics_schedule.add_systems(collect_collision_pairs.in_set(PhysicsStepSet::BroadPhase));
    }
}

fn collect_collision_pairs(
    bodies: Query<(Entity, &ColliderAabb, &RigidBody)>,
    mut broad_collision_pairs: ResMut<BroadCollisionPairs>,
) {
    // Clear old collision pairs
    broad_collision_pairs.0.clear();

    // Loop through all entity combinations and collect pairs of bodies with intersecting AABBs
    for [(ent_a, aabb_a, rb_a), (ent_b, aabb_b, rb_b)] in bodies.iter_combinations() {
        // At least one of the bodies is dynamic and their AABBs intersect
        if (rb_a.is_dynamic() || rb_b.is_dynamic()) && aabb_a.intersects(aabb_b) {
            broad_collision_pairs.0.push((ent_a, ent_b));
        }
    }
}
