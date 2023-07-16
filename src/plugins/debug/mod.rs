//! Renders physics objects and properties for debugging purposes. This includes [AABBs](ColliderAabb),
//! [colliders](Collider) and [contacts](Collision).
//!
//! See [`PhysicsDebugPlugin`].

mod renderer;

pub use renderer::*;

use crate::prelude::*;
use bevy::prelude::*;

/// Renders physics objects and properties for debugging purposes. This includes [AABBs](ColliderAabb),
/// [colliders](Collider) and [contacts](Collision).
///
/// You can use the [`PhysicsDebugConfig`] resource for the global configuration and the [`DebugRender`] component
/// for entity-level configuration.
pub struct PhysicsDebugPlugin {
    schedule: Box<dyn ScheduleLabel>,
}

impl PhysicsDebugPlugin {
    /// Creates a [`PhysicsDebugPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: Box::new(schedule),
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
                self.schedule.dyn_clone(),
                (
                    debug_render_axes,
                    debug_render_aabbs,
                    debug_render_colliders,
                    debug_render_contacts,
                    // Todo: Refactor joints to allow iterating over all of them without generics
                    debug_render_joints::<FixedJoint>,
                    debug_render_joints::<PrismaticJoint>,
                    debug_render_joints::<RevoluteJoint>,
                    debug_render_joints::<SphericalJoint>,
                    change_mesh_visibility,
                )
                    .after(PhysicsSet::StepSimulation),
            );
    }
}

/// Controls the global [`PhysicsDebugPlugin`] configuration.
///
/// To configure the debug rendering of specific entities, use the [`DebugRender`] component.
#[derive(Reflect, Resource)]
#[reflect(Resource)]
pub struct PhysicsDebugConfig {
    /// The lengths of the axes drawn for an entity.
    pub axis_lengths: Option<Vector>,
    /// The color of the [AABBs](ColliderAabb). If `None`, the AABBs will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframes. If `None`, the colliders will not be rendered.
    pub collider_color: Option<Color>,
    /// The color of the contact points. If `None`, the contact points will not be rendered.
    pub contact_color: Option<Color>,
    /// The color of the lines drawn from the centers of bodies to their joint anchors.
    pub joint_anchor_color: Option<Color>,
    /// The color of the lines drawn between joint anchors, indicating the separation.
    pub joint_separation_color: Option<Color>,
    /// Determines if the visibility of entities with [colliders](Collider) should be set to `Visibility::Hidden`,
    /// which will only show the debug renders.
    pub hide_meshes: bool,
}

impl Default for PhysicsDebugConfig {
    fn default() -> Self {
        Self {
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: None,
            collider_color: Some(Color::ORANGE),
            contact_color: None,
            joint_anchor_color: Some(Color::PINK),
            joint_separation_color: Some(Color::RED),
            hide_meshes: false,
        }
    }
}

/// A component for the debug render configuration of an entity.
///
/// This overwrites the global [`PhysicsDebugConfig`] for this specific entity.
#[derive(Component, Reflect, Clone, Copy, PartialEq)]
#[reflect(Component)]
pub struct DebugRender {
    /// The lengths of the axes drawn for the entity.
    pub axis_lengths: Option<Vector>,
    /// The color of the [AABB](ColliderAabb). If `None`, the AABB will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframe. If `None`, the collider will not be rendered.
    pub collider_color: Option<Color>,
    /// Determines if the entity's visibility should be set to `Visibility::Hidden`, which will only show the debug render.
    pub hide_mesh: bool,
}

impl Default for DebugRender {
    fn default() -> Self {
        Self {
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: None,
            collider_color: Some(Color::LIME_GREEN),
            hide_mesh: false,
        }
    }
}

impl DebugRender {
    /// Creates a [`DebugRender`] configuration with the given lengths for the axes
    /// that are drawn for the entity.
    pub fn axes(axis_lengths: Vector) -> Self {
        Self {
            axis_lengths: Some(axis_lengths),
            ..default()
        }
    }

    /// Creates a [`DebugRender`] configuration with a given collider color.
    pub fn collider(color: Color) -> Self {
        Self {
            collider_color: Some(color),
            ..default()
        }
    }

    /// Creates a [`DebugRender`] configuration with a given AABB color.
    pub fn aabb(color: Color) -> Self {
        Self {
            aabb_color: Some(color),
            ..default()
        }
    }

    /// Sets the lengths of the axes drawn for the entity.
    pub fn with_axes(mut self, axis_lengths: Vector) -> Self {
        self.axis_lengths = Some(axis_lengths);
        self
    }

    /// Sets the collider color.
    pub fn with_collider_color(mut self, color: Color) -> Self {
        self.collider_color = Some(color);
        self
    }

    /// Sets the AABB color.
    pub fn with_aabb_color(mut self, color: Color) -> Self {
        self.aabb_color = Some(color);
        self
    }

    /// Sets the visibility of the entity's visual mesh.
    pub fn with_mesh_visibility(mut self, is_visible: bool) -> Self {
        self.hide_mesh = !is_visible;
        self
    }

    /// Disables axis debug rendering.
    pub fn without_axes(mut self) -> Self {
        self.axis_lengths = None;
        self
    }

    /// Disables collider debug rendering.
    pub fn without_collider(mut self) -> Self {
        self.collider_color = None;
        self
    }

    /// Disables AABB debug rendering.
    pub fn without_aabb(mut self) -> Self {
        self.aabb_color = None;
        self
    }
}

fn debug_render_axes(
    bodies: Query<(&Position, &Rotation, Option<&DebugRender>)>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    for (pos, rot, render_config) in &bodies {
        if let Some(lengths) = render_config.map_or(config.axis_lengths, |c| c.axis_lengths) {
            let x = rot.rotate(Vector::X * lengths.x);
            debug_renderer.draw_line(pos.0 - x, pos.0 + x, Color::hsl(0.0, 1.0, 0.5));

            let y = rot.rotate(Vector::Y * lengths.y);
            debug_renderer.draw_line(pos.0 - y, pos.0 + y, Color::hsl(120.0, 1.0, 0.4));

            #[cfg(feature = "3d")]
            {
                let z = rot.rotate(Vector::Z * lengths.z);
                debug_renderer.draw_line(pos.0 - z, pos.0 + z, Color::hsl(220.0, 1.0, 0.6));
            }

            // Draw dot at the center of the body
            #[cfg(feature = "2d")]
            debug_renderer.gizmos.circle_2d(pos.0, 0.5, Color::YELLOW);
            #[cfg(feature = "3d")]
            debug_renderer
                .gizmos
                .sphere(pos.0, rot.0, 0.025, Color::YELLOW);
        }
    }
}

fn debug_render_aabbs(
    aabbs: Query<(&ColliderAabb, Option<&DebugRender>)>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    #[cfg(feature = "2d")]
    for (aabb, render_config) in &aabbs {
        if let Some(color) = render_config.map_or(config.aabb_color, |c| c.aabb_color) {
            debug_renderer.gizmos.cuboid(
                Transform::from_scale(Vector::from(aabb.extents()).extend(0.0).as_f32())
                    .with_translation(Vector::from(aabb.center()).extend(0.0).as_f32()),
                color,
            );
        }
    }

    #[cfg(feature = "3d")]
    for (aabb, render_config) in &aabbs {
        if let Some(color) = render_config.map_or(config.aabb_color, |c| c.aabb_color) {
            debug_renderer.gizmos.cuboid(
                Transform::from_scale(Vector::from(aabb.extents()).as_f32())
                    .with_translation(Vector::from(aabb.center()).as_f32()),
                color,
            );
        }
    }
}

fn debug_render_contacts(
    mut collisions: EventReader<Collision>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    let Some(color) = config.contact_color else { return };
    for Collision(contact) in collisions.iter() {
        let p1 = contact.point1;
        let p2 = contact.point2;
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

fn debug_render_colliders(
    mut colliders: Query<(&Collider, &Position, &Rotation, Option<&DebugRender>)>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    for (collider, position, rotation, render_config) in &mut colliders {
        if let Some(color) = render_config.map_or(config.collider_color, |c| c.collider_color) {
            debug_renderer.draw_collider(collider, position, rotation, color);
        }
    }
}

fn debug_render_joints<T: Joint>(
    bodies: Query<(&Position, &Rotation)>,
    joints: Query<&T>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    for joint in &joints {
        if let Ok([(pos1, rot1), (pos2, rot2)]) = bodies.get_many(joint.entities()) {
            if let Some(anchor_color) = config.joint_anchor_color {
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
            if let Some(separation_color) = config.joint_separation_color {
                debug_renderer.draw_line(
                    pos1.0 + rot1.rotate(joint.local_anchor_1()),
                    pos2.0 + rot2.rotate(joint.local_anchor_2()),
                    separation_color,
                );
            }
        }
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
            let hide_mesh = render_config.map_or(config.hide_meshes, |c| c.hide_mesh);
            if hide_mesh {
                *visibility = Visibility::Hidden;
            } else {
                *visibility = Visibility::Visible;
            }
        }
    }
}
