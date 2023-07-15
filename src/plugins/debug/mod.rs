//! Renders physics objects and events like [AABBs](ColliderAabb) and [contacts](Collision) for debugging purposes.
//!
//! See [`PhysicsDebugPlugin`].

mod renderer;

pub use renderer::*;

use crate::prelude::*;
use bevy::prelude::*;

/// Renders physics objects and events like [AABBs](ColliderAabb) and [contacts](Collision) for debugging purposes.
///
/// You can configure what is rendered using the [`PhysicsDebugConfig`] resource.
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
            .add_systems(
                self.schedule.dyn_clone(),
                (
                    debug_render_aabbs
                        .run_if(|config: Res<PhysicsDebugConfig>| config.render_aabbs),
                    debug_render_colliders
                        .run_if(|config: Res<PhysicsDebugConfig>| config.render_colliders),
                    debug_render_contacts
                        .run_if(|config: Res<PhysicsDebugConfig>| config.render_contacts),
                    change_mesh_visibility,
                )
                    .after(PhysicsSet::StepSimulation),
            );
    }
}

/// Controls the [`PhysicsDebugPlugin`] configuration.
#[derive(Reflect, Resource)]
#[reflect(Resource)]
pub struct PhysicsDebugConfig {
    /// Renders the Axis-Aligned Bounding Boxes of [colliders](Collider).
    pub render_aabbs: bool,
    /// Renders [colliders](Collider).
    pub render_colliders: bool,
    /// Renders contact points.
    pub render_contacts: bool,
    /// Determines if the visibility of entities with [colliders](Collider) should be set to `Visibility::Hidden`.
    pub hide_meshes: bool,
}

impl Default for PhysicsDebugConfig {
    fn default() -> Self {
        Self {
            render_aabbs: true,
            render_colliders: true,
            render_contacts: true,
            hide_meshes: true,
        }
    }
}

fn debug_render_aabbs(aabbs: Query<&ColliderAabb>, mut debug_renderer: PhysicsDebugRenderer) {
    #[cfg(feature = "2d")]
    for aabb in aabbs.iter() {
        debug_renderer.gizmos.cuboid(
            Transform::from_scale(Vector::from(aabb.extents()).extend(0.0).as_f32())
                .with_translation(Vector::from(aabb.center()).extend(0.0).as_f32()),
            Color::WHITE,
        );
    }

    #[cfg(feature = "3d")]
    for aabb in aabbs.iter() {
        debug_renderer.gizmos.cuboid(
            Transform::from_scale(Vector::from(aabb.extents()).as_f32())
                .with_translation(Vector::from(aabb.center()).as_f32()),
            Color::WHITE,
        );
    }
}

fn debug_render_contacts(
    mut collisions: EventReader<Collision>,
    mut debug_renderer: PhysicsDebugRenderer,
) {
    for Collision(contact) in collisions.iter() {
        let p1 = contact.point1;
        let p2 = contact.point2;
        #[cfg(feature = "2d")]
        let len = 5.0;
        #[cfg(feature = "3d")]
        let len = 0.3;

        debug_renderer.draw_line(p1 - Vector::X * len, p1 + Vector::X * len, Color::CYAN);
        debug_renderer.draw_line(p1 - Vector::Y * len, p1 + Vector::Y * len, Color::CYAN);
        #[cfg(feature = "3d")]
        debug_renderer.draw_line(p1 - Vector::Z * len, p1 + Vector::Z * len, Color::CYAN);

        debug_renderer.draw_line(p2 - Vector::X * len, p2 + Vector::X * len, Color::CYAN);
        debug_renderer.draw_line(p2 - Vector::Y * len, p2 + Vector::Y * len, Color::CYAN);
        #[cfg(feature = "3d")]
        debug_renderer.draw_line(p2 - Vector::Z * len, p2 + Vector::Z * len, Color::CYAN);
    }
}

fn debug_render_colliders(
    mut colliders: Query<(&Collider, &Position, &Rotation)>,
    mut debug_renderer: PhysicsDebugRenderer,
) {
    for (collider, position, rotation) in &mut colliders {
        debug_renderer.draw_collider(collider, position, rotation, Color::LIME_GREEN);
    }
}

fn change_mesh_visibility(
    config: Res<PhysicsDebugConfig>,
    mut meshes: Query<&mut Visibility, With<Collider>>,
) {
    if config.is_changed() {
        for mut visibility in &mut meshes {
            if config.hide_meshes {
                *visibility = Visibility::Hidden;
            } else {
                *visibility = Visibility::Visible;
            }
        }
    }
}
