//! Renders physics objects and properties for debugging purposes.
//!
//! See [`PhysicsDebugPlugin`].

mod configuration;
mod renderer;

pub use configuration::*;
pub use renderer::*;

use crate::prelude::*;
use bevy::{ecs::query::Has, prelude::*, utils::intern::Interned};

/// A plugin that renders physics objects and properties for debugging purposes.
/// It is not enabled by default and must be added manually.
///
/// Currently, the following are supported for debug rendering:
///
/// - The axes and center of mass of [rigid bodies](RigidBody)
/// - [AABBs](ColliderAabb)
/// - [Collider] wireframes
/// - Using different colors for [sleeping](Sleeping) bodies
/// - [Contacts]
/// - [Joints](joints)
/// - [`RayCaster`]
/// - [`ShapeCaster`]
/// - Changing the visibility of entities to only show debug rendering
///
/// By default, [AABBs](ColliderAabb) and [contacts](Contacts) are not debug rendered.
/// You can use the [`PhysicsDebugConfig`] resource for the global configuration and the
/// [`DebugRender`] component for entity-level configuration.
///
/// ## Example
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default(),
///             // Enables debug rendering
///             PhysicsDebugPlugin::default(),
///         ))
///         // Overwrite default debug configuration (optional)
///         .insert_resource(PhysicsDebugConfig {
///             aabb_color: Some(Color::WHITE),
///             ..default()
///         })
///         .run();
/// }
///
/// fn setup(mut commands: Commands) {
///     // This rigid body and its collider and AABB will get rendered
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::ball(0.5),
///         // Overwrite default collider color (optional)
///         DebugRender::default().with_collider_color(Color::RED),
///     ));
/// }
/// ```
pub struct PhysicsDebugPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PhysicsDebugPlugin {
    /// Creates a [`PhysicsDebugPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PhysicsDebugPlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsDebugConfig>()
            .insert_resource(GizmoConfig {
                #[cfg(feature = "2d")]
                line_width: 2.0,
                #[cfg(feature = "3d")]
                line_width: 1.5,
                ..default()
            })
            .register_type::<PhysicsDebugConfig>()
            .register_type::<DebugRender>()
            .add_systems(
                self.schedule,
                (
                    debug_render_axes,
                    debug_render_aabbs,
                    debug_render_colliders,
                    debug_render_contacts,
                    // TODO: Refactor joints to allow iterating over all of them without generics
                    debug_render_joints::<FixedJoint>,
                    debug_render_joints::<PrismaticJoint>,
                    debug_render_joints::<DistanceJoint>,
                    debug_render_joints::<RevoluteJoint>,
                    debug_render_joints::<SphericalJoint>,
                    debug_render_raycasts,
                    debug_render_shapecasts,
                )
                    .after(PhysicsSet::StepSimulation)
                    .run_if(|config: Res<PhysicsDebugConfig>| config.enabled),
            )
            .add_systems(
                self.schedule,
                change_mesh_visibility.after(PhysicsSet::StepSimulation),
            );
    }
}

#[allow(clippy::type_complexity)]
fn debug_render_axes(
    bodies: Query<(
        &Position,
        &Rotation,
        &CenterOfMass,
        Has<Sleeping>,
        Option<&DebugRender>,
    )>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    for (pos, rot, local_com, sleeping, render_config) in &bodies {
        // If the body is sleeping, the colors will be multiplied by the sleeping color multiplier
        if let Some(lengths) = render_config.map_or(config.axis_lengths, |c| c.axis_lengths) {
            let mul = if sleeping {
                render_config
                    .map_or(config.sleeping_color_multiplier, |c| {
                        c.sleeping_color_multiplier
                    })
                    .unwrap_or([1.0; 4])
            } else {
                [1.0; 4]
            };
            let [x_color, y_color, _z_color, center_color] = [
                Color::hsla(0.0, 1.0 * mul[1], 0.5 * mul[2], 1.0 * mul[3]),
                Color::hsla(120.0 * mul[0], 1.0 * mul[1], 0.4 * mul[2], 1.0 * mul[3]),
                Color::hsla(220.0 * mul[0], 1.0 * mul[1], 0.6 * mul[2], 1.0 * mul[3]),
                Color::hsla(60.0 * mul[0], 1.0 * mul[1], 0.5 * mul[2], 1.0 * mul[3]),
            ];
            let global_com = pos.0 + rot.rotate(local_com.0);

            let x = rot.rotate(Vector::X * lengths.x);
            debug_renderer.draw_line(global_com - x, global_com + x, x_color);

            let y = rot.rotate(Vector::Y * lengths.y);
            debug_renderer.draw_line(global_com - y, global_com + y, y_color);

            #[cfg(feature = "3d")]
            {
                let z = rot.rotate(Vector::Z * lengths.z);
                debug_renderer.draw_line(global_com - z, global_com + z, _z_color);
            }

            // Draw dot at the center of mass
            #[cfg(feature = "2d")]
            debug_renderer.gizmos.circle_2d(
                global_com.as_f32(),
                // Scale dot size based on axis lengths
                (lengths.x + lengths.y) / 20.0,
                center_color,
            );
            #[cfg(feature = "3d")]
            debug_renderer.gizmos.sphere(
                global_com.as_f32(),
                rot.as_f32(),
                // Scale dot size based on axis lengths
                (lengths.x + lengths.y + lengths.z) / 30.0,
                center_color,
            );
        }
    }
}

fn debug_render_aabbs(
    aabbs: Query<(
        Entity,
        &ColliderAabb,
        Option<&ColliderParent>,
        Option<&DebugRender>,
    )>,
    sleeping: Query<(), With<Sleeping>>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    #[cfg(feature = "2d")]
    for (entity, aabb, collider_parent, render_config) in &aabbs {
        if let Some(mut color) = render_config.map_or(config.aabb_color, |c| c.aabb_color) {
            let collider_parent = collider_parent.map_or(entity, |p| p.get());

            // If the body is sleeping, multiply the color by the sleeping color multiplier
            if sleeping.contains(collider_parent) {
                let [h, s, l, a] = color.as_hsla_f32();
                if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                    c.sleeping_color_multiplier
                }) {
                    color = Color::hsla(h * mul[0], s * mul[1], l * mul[2], a * mul[3]);
                }
            }

            debug_renderer.gizmos.cuboid(
                Transform::from_scale(Vector::from(aabb.extents()).extend(0.0).as_f32())
                    .with_translation(Vector::from(aabb.center()).extend(0.0).as_f32()),
                color,
            );
        }
    }

    #[cfg(feature = "3d")]
    for (entity, aabb, collider_parent, render_config) in &aabbs {
        if let Some(mut color) = render_config.map_or(config.aabb_color, |c| c.aabb_color) {
            let collider_parent = collider_parent.map_or(entity, |p| p.get());

            // If the body is sleeping, multiply the color by the sleeping color multiplier
            if sleeping.contains(collider_parent) {
                let [h, s, l, a] = color.as_hsla_f32();
                if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                    c.sleeping_color_multiplier
                }) {
                    color = Color::hsla(h * mul[0], s * mul[1], l * mul[2], a * mul[3]);
                }
            }

            debug_renderer.gizmos.cuboid(
                Transform::from_scale(Vector::from(aabb.extents()).as_f32())
                    .with_translation(Vector::from(aabb.center()).as_f32()),
                color,
            );
        }
    }
}

#[allow(clippy::type_complexity)]
fn debug_render_colliders(
    mut colliders: Query<(
        Entity,
        &Collider,
        &Position,
        &Rotation,
        Option<&ColliderParent>,
        Option<&DebugRender>,
    )>,
    sleeping: Query<(), With<Sleeping>>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    for (entity, collider, position, rotation, collider_parent, render_config) in &mut colliders {
        if let Some(mut color) = render_config.map_or(config.collider_color, |c| c.collider_color) {
            let collider_parent = collider_parent.map_or(entity, |p| p.get());

            // If the body is sleeping, multiply the color by the sleeping color multiplier
            if sleeping.contains(collider_parent) {
                let [h, s, l, a] = color.as_hsla_f32();
                if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                    c.sleeping_color_multiplier
                }) {
                    color = Color::hsla(h * mul[0], s * mul[1], l * mul[2], a * mul[3]);
                }
            }
            debug_renderer.draw_collider(collider, position, rotation, color);
        }
    }
}

fn debug_render_contacts(
    colliders: Query<(&Position, &Rotation), With<Collider>>,
    mut collisions: EventReader<Collision>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    let Some(color) = config.contact_color else {
        return;
    };
    for Collision(contacts) in collisions.read() {
        let Ok((position1, rotation1)) = colliders.get(contacts.entity1) else {
            continue;
        };
        let Ok((position2, rotation2)) = colliders.get(contacts.entity2) else {
            continue;
        };

        for manifold in contacts.manifolds.iter() {
            for contact in manifold.contacts.iter() {
                let p1 = contact.global_point1(position1, rotation1);
                let p2 = contact.global_point2(position2, rotation2);
                #[cfg(feature = "2d")]
                let len = 5.0;
                #[cfg(feature = "3d")]
                let len = 0.3;

                debug_renderer.draw_line(p1 - Vector::X * len, p1 + Vector::X * len, color);
                debug_renderer.draw_line(p1 - Vector::Y * len, p1 + Vector::Y * len, color);
                #[cfg(feature = "3d")]
                debug_renderer.draw_line(p1 - Vector::Z * len, p1 + Vector::Z * len, color);

                debug_renderer.draw_line(p2 - Vector::X * len, p2 + Vector::X * len, color);
                debug_renderer.draw_line(p2 - Vector::Y * len, p2 + Vector::Y * len, color);
                #[cfg(feature = "3d")]
                debug_renderer.draw_line(p2 - Vector::Z * len, p2 + Vector::Z * len, color);
            }
        }
    }
}

fn debug_render_joints<T: Joint>(
    bodies: Query<(&Position, &Rotation, Has<Sleeping>)>,
    joints: Query<(&T, Option<&DebugRender>)>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    for (joint, render_config) in &joints {
        if let Ok([(pos1, rot1, sleeping1), (pos2, rot2, sleeping2)]) =
            bodies.get_many(joint.entities())
        {
            if let Some(mut anchor_color) = config.joint_anchor_color {
                // If both bodies are sleeping, multiply the color by the sleeping color multiplier
                if sleeping1 && sleeping2 {
                    let [h, s, l, a] = anchor_color.as_hsla_f32();
                    if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                        c.sleeping_color_multiplier
                    }) {
                        anchor_color = Color::hsla(h * mul[0], s * mul[1], l * mul[2], a * mul[3]);
                    }
                }

                debug_renderer.draw_line(
                    pos1.0,
                    pos1.0 + rot1.rotate(joint.local_anchor_1()),
                    anchor_color,
                );
                debug_renderer.draw_line(
                    pos2.0,
                    pos2.0 + rot2.rotate(joint.local_anchor_2()),
                    anchor_color,
                );
            }
            if let Some(mut separation_color) = config.joint_separation_color {
                // If both bodies are sleeping, multiply the color by the sleeping color multiplier
                if sleeping1 && sleeping2 {
                    let [h, s, l, a] = separation_color.as_hsla_f32();
                    if let Some(mul) = render_config.map_or(config.sleeping_color_multiplier, |c| {
                        c.sleeping_color_multiplier
                    }) {
                        separation_color =
                            Color::hsla(h * mul[0], s * mul[1], l * mul[2], a * mul[3]);
                    }
                }

                debug_renderer.draw_line(
                    pos1.0 + rot1.rotate(joint.local_anchor_1()),
                    pos2.0 + rot2.rotate(joint.local_anchor_2()),
                    separation_color,
                );
            }
        }
    }
}

fn debug_render_raycasts(
    query: Query<(&RayCaster, &RayHits)>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    for (ray, hits) in &query {
        let ray_color = config
            .raycast_color
            .unwrap_or(Color::rgba(0.0, 0.0, 0.0, 0.0));
        let point_color = config
            .raycast_point_color
            .unwrap_or(Color::rgba(0.0, 0.0, 0.0, 0.0));
        let normal_color = config
            .raycast_normal_color
            .unwrap_or(Color::rgba(0.0, 0.0, 0.0, 0.0));

        debug_renderer.draw_raycast(
            ray.global_origin(),
            ray.global_direction(),
            // f32::MAX renders nothing, but this number seems to be fine :P
            ray.max_time_of_impact.min(1_000_000_000_000_000_000.0),
            hits.as_slice(),
            ray_color,
            point_color,
            normal_color,
        );
    }
}

fn debug_render_shapecasts(
    query: Query<(&ShapeCaster, &ShapeHits)>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    for (shape_caster, hits) in &query {
        let ray_color = config
            .shapecast_color
            .unwrap_or(Color::rgba(0.0, 0.0, 0.0, 0.0));
        let shape_color = config
            .shapecast_shape_color
            .unwrap_or(Color::rgba(0.0, 0.0, 0.0, 0.0));
        let point_color = config
            .shapecast_point_color
            .unwrap_or(Color::rgba(0.0, 0.0, 0.0, 0.0));
        let normal_color = config
            .shapecast_normal_color
            .unwrap_or(Color::rgba(0.0, 0.0, 0.0, 0.0));

        debug_renderer.draw_shapecast(
            &shape_caster.shape,
            shape_caster.global_origin(),
            shape_caster.global_shape_rotation(),
            shape_caster.global_direction(),
            // f32::MAX renders nothing, but this number seems to be fine :P
            shape_caster.max_time_of_impact.min(1_000_000_000_000_000.0),
            hits.as_slice(),
            ray_color,
            shape_color,
            point_color,
            normal_color,
        );
    }
}

type MeshVisibilityQueryFilter = (
    Or<(With<RigidBody>, With<Collider>)>,
    Or<(Changed<DebugRender>, Without<DebugRender>)>,
);

fn change_mesh_visibility(
    mut meshes: Query<(&mut Visibility, Option<&DebugRender>), MeshVisibilityQueryFilter>,
    config: Res<PhysicsDebugConfig>,
) {
    if config.is_changed() {
        for (mut visibility, render_config) in &mut meshes {
            let hide_mesh =
                config.enabled && render_config.map_or(config.hide_meshes, |c| c.hide_mesh);
            if hide_mesh {
                *visibility = Visibility::Hidden;
            } else {
                *visibility = Visibility::Visible;
            }
        }
    }
}
