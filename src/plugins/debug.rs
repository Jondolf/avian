//! Renders physics objects and events like [AABBs](ColliderAabb) and [contacts](Collision) for debugging purposes.
//!
//! See [`PhysicsDebugPlugin`].

use std::marker::PhantomData;

use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*};
use parry::shape::TypedShape;

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
                line_width: 1.0,
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
            render_contacts: false,
            hide_meshes: false,
        }
    }
}

// Todo: Allow custom rendering backends through generics
/// A `SystemParam` for physics debug rendering.
#[derive(SystemParam)]
pub struct PhysicsDebugRenderer<'w, 's> {
    gizmos: Gizmos<'s>,
    phantom_data: PhantomData<&'w ()>,
}

impl<'w, 's> PhysicsDebugRenderer<'w, 's> {
    /// Draws a line from `a` to `b`.
    pub fn draw_line(&mut self, a: Vector, b: Vector, color: Color) {
        #[cfg(feature = "2d")]
        self.gizmos.line_2d(a.as_f32(), b.as_f32(), color);
        #[cfg(feature = "3d")]
        self.gizmos.line(a.as_f32(), b.as_f32(), color);
    }

    /// Draws lines between a list of points.
    pub fn draw_line_strip(
        &mut self,
        points: Vec<Vector>,
        position: &Position,
        rotation: &Rotation,
        closed: bool,
        color: Color,
    ) {
        #[cfg(feature = "2d")]
        self.gizmos.linestrip_2d(
            points.iter().map(|p| position.0 + rotation.rotate(*p)),
            color,
        );
        #[cfg(feature = "3d")]
        self.gizmos.linestrip(
            points.iter().map(|p| position.0 + rotation.rotate(*p)),
            color,
        );

        if closed && points.len() > 2 {
            let a = position.0 + rotation.rotate(points[0]);
            let b = position.0 + rotation.rotate(*points.last().unwrap());
            self.draw_line(a, b, color);
        }
    }

    /// Draws a polyline based on the given vertex and index buffers.
    pub fn draw_polyline(
        &mut self,
        vertices: &[Vector],
        indices: &[[u32; 2]],
        position: &Position,
        rotation: &Rotation,
        color: Color,
    ) {
        for [i1, i2] in indices {
            let a = position.0 + rotation.rotate(vertices[*i1 as usize]);
            let b = position.0 + rotation.rotate(vertices[*i2 as usize]);
            self.draw_line(a, b, color);
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
    mut colliders: Query<(&Position, &Rotation, &Collider)>,
    mut debug_renderer: PhysicsDebugRenderer,
) {
    let nalgebra_to_glam =
        |points: Vec<_>| points.iter().map(|p| Vector::from(*p)).collect::<Vec<_>>();
    for (position, rotation, collider) in &mut colliders {
        match collider.as_typed_shape() {
            #[cfg(feature = "2d")]
            TypedShape::Ball(s) => {
                let vertices = s.to_polyline(12);
                debug_renderer.draw_line_strip(
                    nalgebra_to_glam(vertices),
                    position,
                    rotation,
                    true,
                    Color::LIME_GREEN,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Ball(s) => {
                let (vertices, indices) = s.to_outline(12);
                debug_renderer.draw_polyline(
                    &nalgebra_to_glam(vertices),
                    &indices,
                    position,
                    rotation,
                    Color::LIME_GREEN,
                );
            }
            #[cfg(feature = "2d")]
            TypedShape::Cuboid(s) => {
                debug_renderer.draw_line_strip(
                    nalgebra_to_glam(s.to_polyline()),
                    position,
                    rotation,
                    true,
                    Color::LIME_GREEN,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Cuboid(s) => {
                let (vertices, indices) = s.to_outline();
                debug_renderer.draw_polyline(
                    &nalgebra_to_glam(vertices),
                    &indices,
                    position,
                    rotation,
                    Color::LIME_GREEN,
                );
            }
            #[cfg(feature = "2d")]
            TypedShape::Capsule(s) => {
                debug_renderer.draw_line_strip(
                    nalgebra_to_glam(s.to_polyline(12)),
                    position,
                    rotation,
                    true,
                    Color::LIME_GREEN,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Capsule(s) => {
                let (vertices, indices) = s.to_outline(12);
                debug_renderer.draw_polyline(
                    &nalgebra_to_glam(vertices),
                    &indices,
                    position,
                    rotation,
                    Color::LIME_GREEN,
                );
            }
            TypedShape::Segment(s) => debug_renderer.draw_line_strip(
                vec![s.a.into(), s.b.into()],
                position,
                rotation,
                false,
                Color::LIME_GREEN,
            ),
            TypedShape::Triangle(s) => debug_renderer.draw_line_strip(
                vec![s.a.into(), s.b.into(), s.c.into()],
                position,
                rotation,
                true,
                Color::LIME_GREEN,
            ),
            _ => (),
        }
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
