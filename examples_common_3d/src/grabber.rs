use bevy::{prelude::*, window::PrimaryWindow};
use bevy_xpbd_3d::{math::*, prelude::*};

#[derive(Default)]
pub struct GrabberPlugin;

impl Plugin for GrabberPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, grab);
    }
}

// Hard coded for now, could probably use a resource though
const GRAB_MIN_DISTANCE: Scalar = 100.0;
const GRAB_COMPLIANCE: Scalar = 0.001;
const GRAB_LINEAR_DAMPING: Scalar = 5.0;
const GRAB_ANGULAR_DAMPING: Scalar = 1.0;

/// A marker component for joints used my grabbers.
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
    buttons: Res<Input<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera: Query<(&Camera, &GlobalTransform)>,
    mut grabber: Query<(Entity, &mut Position, &mut GrabPoint), Without<Collider>>,
    joints: Query<(Entity, &DistanceJoint), With<GrabberJoint>>,
    collider_parents: Query<&ColliderParent>,
    bodies: Query<(&RigidBody, &Position, &Rotation), Without<GrabPoint>>,
    spatial_query: SpatialQuery,
) {
    // If grab button is pressed, spawn or update grab point and grabber joint if they don't exist
    if buttons.pressed(MouseButton::Left) {
        let window = windows.single();
        let (camera, camera_transform) = camera.iter().next().unwrap();

        if let Some(ray) = window
            .cursor_position()
            .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor))
        {
            let grabber_entity: Entity;
            let cursor_world_pos: Vector;

            // If grabber exists, update its position, otherwise spawn it
            if let Ok((entity, mut position, grabber)) = grabber.get_single_mut() {
                cursor_world_pos = ray.origin.adjust_precision()
                    + ray.direction.adjust_precision() * grabber.depth;
                position.0 = cursor_world_pos;
                grabber_entity = entity;
            } else if let Some(ray_hit) = spatial_query.cast_ray(
                ray.origin.adjust_precision(),
                ray.direction.adjust_precision(),
                Scalar::MAX,
                true,
                SpatialQueryFilter::default(),
            ) {
                cursor_world_pos = ray.origin.adjust_precision()
                    + ray.direction.adjust_precision() * ray_hit.time_of_impact;
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
                        let Ok(collider_parent) = collider_parents.get(projection.entity) else {
                            return;
                        };
                        // Spawn grabber joint
                        if let Ok((_, position, rotation)) = bodies.get(collider_parent.get()) {
                            commands.spawn((
                                DistanceJoint::new(grabber_entity, collider_parent.get())
                                    .with_compliance(GRAB_COMPLIANCE)
                                    .with_local_anchor_2(
                                        rotation.inverse().rotate(projection.point - position.0),
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
