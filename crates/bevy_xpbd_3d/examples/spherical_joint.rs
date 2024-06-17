use std::collections::VecDeque;

use bevy::{prelude::*, window::PrimaryWindow};
use bevy_xpbd_3d::{math::*, prelude::*};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            GrabberPlugin,
        ))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        //.insert_resource(Gravity::ZERO)
        .insert_resource(Msaa::Sample4)
        .init_resource::<Trail>()
        .add_systems(Startup, setup)
        .add_systems(Update, (record_trail, draw_trail).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let cube_mesh = meshes.add(Cuboid::default());

    // Ground
    commands.spawn((
        PbrBundle {
            mesh: cube_mesh.clone(),
            material: materials.add(Color::rgb(0.7, 0.7, 0.8)),
            transform: Transform::from_xyz(0.0, -4.0, 0.0).with_scale(Vec3::new(100.0, 1.0, 100.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(1.0, 1.0, 1.0),
    ));

    let cube_mesh = meshes.add(Cuboid::default());
    let cube_material = materials.add(Color::rgba(0.8, 0.7, 0.6, 0.85));

    // Kinematic rotating "anchor" object
    let anchor = commands
        .spawn((
            PbrBundle {
                mesh: cube_mesh.clone(),
                material: cube_material.clone(),
                transform: Transform::from_rotation(Quat::from_rotation_x(PI / 2.0)),
                ..default()
            },
            RigidBody::Kinematic,
            //AngularVelocity(Vector::new(0.0, 0.5, 1.0).normalize() * 1.0),
        ))
        .id();

    // Dynamic object rotating around anchor
    let object = commands
        .spawn((
            PbrBundle {
                mesh: cube_mesh,
                material: cube_material,
                transform: Transform::from_xyz(0.0, -2.0, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(1.0, 1.0, 1.0),
            RecordTrail,
        ))
        .id();

    // Connect anchor and dynamic object
    commands.spawn(
        SphericalJoint::new(anchor, object)
            .with_local_anchor_2(Vector::Y * 2.0)
            .with_swing_limits(-PI / 4.0, PI / 4.0),
    );

    // Directional light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 2000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
        ..default()
    });

    // Camera
    commands.spawn((Camera3dBundle {
        transform: Transform::from_xyz(3.0, 2.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    },));
}

#[derive(Default, Component)]
struct RecordTrail;
#[derive(Default, Resource)]
struct Trail {
    nodes: VecDeque<Vec3>,
}

fn record_trail(mut trail: ResMut<Trail>, body: Query<&GlobalTransform, With<RecordTrail>>) {
    let transform = body.single();

    trail.nodes.push_back(transform.translation());
    if trail.nodes.len() > 600 {
        trail.nodes.pop_front();
    }
}

fn draw_trail(trail: Res<Trail>, mut gizmos: Gizmos) {
    gizmos.linestrip(trail.nodes.iter().copied(), Color::RED);
}

#[derive(Default)]
pub struct GrabberPlugin;

impl Plugin for GrabberPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, grab);
    }
}

// Hard coded for now, could probably use a resource though
const GRAB_MIN_DISTANCE: Scalar = 100.0;
const GRAB_COMPLIANCE: Scalar = 0.01;
const GRAB_LINEAR_DAMPING: Scalar = 5.0;
const GRAB_ANGULAR_DAMPING: Scalar = 1.0;

/// A marker component for joints used by grabbers.
#[derive(Component)]
struct GrabberJoint;

/// The point that the grabbed entity should follow, positioned at the cursor position.
#[derive(Component)]
struct GrabPoint {
    /// How far away the grab point is from the camera.
    depth: Scalar,
}

#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
fn grab(
    mut commands: Commands,
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera: Query<(&Camera, &GlobalTransform)>,
    mut grabber: Query<(Entity, &mut Position, &mut GrabPoint), Without<Collider>>,
    joints: Query<(Entity, &DistanceJoint), With<GrabberJoint>>,
    bodies: Query<(&RigidBody, &Position, &Rotation), Without<GrabPoint>>,
    spatial_query: SpatialQuery,
) {
    // If grab button is pressed, spawn or update grab point and grabber joint if they don't exist
    if buttons.pressed(MouseButton::Left) {
        let window = windows.single();
        let (camera, camera_transform) = camera.single();

        if let Some(ray) = window
            .cursor_position()
            .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor))
        {
            let grabber_entity: Entity;
            let cursor_world_pos: Vector;

            // If grabber exists, update its position, otherwise spawn it
            if let Ok((entity, mut position, grabber)) = grabber.get_single_mut() {
                cursor_world_pos = ray.origin + ray.direction * grabber.depth;
                position.0 = cursor_world_pos;
                grabber_entity = entity;
            } else if let Some(ray_hit) = spatial_query.cast_ray(
                ray.origin.adjust_precision(),
                ray.direction,
                Scalar::MAX,
                true,
                SpatialQueryFilter::default(),
            ) {
                cursor_world_pos = ray.origin + ray.direction * ray_hit.time_of_impact;
                grabber_entity = commands
                    .spawn((
                        RigidBody::Kinematic,
                        Position(cursor_world_pos),
                        GrabPoint {
                            depth: ray_hit.time_of_impact,
                        },
                    ))
                    .id();
            } else {
                return;
            }

            // If grabber joint doesn't exist, spawn it
            if joints.is_empty() {
                // Use point projection to find closest point on collider
                let filter = SpatialQueryFilter::default();
                let projection = spatial_query.project_point(cursor_world_pos, true, filter);

                if let Some(projection) = projection {
                    if projection.point.distance(cursor_world_pos) <= GRAB_MIN_DISTANCE {
                        // Spawn grabber joint
                        if let Ok((_, position, rotation)) = bodies.get(projection.entity) {
                            commands.spawn((
                                DistanceJoint::new(grabber_entity, projection.entity)
                                    .with_compliance(GRAB_COMPLIANCE)
                                    .with_local_anchor_2(
                                        rotation.inverse() * (projection.point - position.0),
                                    )
                                    .with_linear_velocity_damping(GRAB_LINEAR_DAMPING)
                                    .with_angular_velocity_damping(GRAB_ANGULAR_DAMPING),
                                GrabberJoint,
                            ));
                        }
                    }
                }
            }
        }
    } else if buttons.just_released(MouseButton::Left) {
        // If grab button was released, despawn grabbers and grabber joints
        for (entity, _, _) in &grabber {
            commands.entity(entity).despawn_recursive();
        }
        for (entity, _) in &joints {
            commands.entity(entity).despawn_recursive();
        }
    }
}
