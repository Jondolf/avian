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
                    debug_render_aabbs,
                    debug_render_colliders,
                    debug_render_contacts,
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
    /// The color of the [AABBs](ColliderAabb). If `None`, the AABBs will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframes. If `None`, the colliders will not be rendered.
    pub collider_color: Option<Color>,
    /// The color of the contact points. If `None`, the contact points will not be rendered.
    pub contact_color: Option<Color>,
    /// Determines if the visibility of entities with [colliders](Collider) should be set to `Visibility::Hidden`,
    /// which will only show the debug renders.
    pub hide_meshes: bool,
}

impl Default for PhysicsDebugConfig {
    fn default() -> Self {
        Self {
            aabb_color: None,
            collider_color: Some(Color::ORANGE),
            contact_color: None,
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
            aabb_color: None,
            collider_color: Some(Color::LIME_GREEN),
            hide_mesh: false,
        }
    }
}

impl DebugRender {
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

fn debug_render_aabbs(
    aabbs: Query<(&ColliderAabb, Option<&DebugRender>)>,
    mut debug_renderer: PhysicsDebugRenderer,
    config: Res<PhysicsDebugConfig>,
) {
    #[cfg(feature = "2d")]
    for (aabb, render_config) in aabbs.iter() {
        if let Some(color) = render_config.map_or(config.aabb_color, |c| c.aabb_color) {
            debug_renderer.gizmos.cuboid(
                Transform::from_scale(Vector::from(aabb.extents()).extend(0.0).as_f32())
                    .with_translation(Vector::from(aabb.center()).extend(0.0).as_f32()),
                color,
            );
        }
    }

    #[cfg(feature = "3d")]
    for (aabb, render_config) in aabbs.iter() {
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

type MeshVisibilityQueryFilter = (
    With<Collider>,
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
