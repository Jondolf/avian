//! Renders physics objects and events like [AABBs](ColliderAabb) and [contacts](Collision) for debugging purposes.
//! See [`PhysicsDebugPlugin`].

use crate::prelude::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::*;

/// Renders physics objects and events like [AABBs](ColliderAabb) and [contacts](Collision) for debugging purposes.
pub struct PhysicsDebugPlugin;

impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(DebugLinesPlugin::default())
            .init_resource::<PhysicsDebugConfig>()
            .register_type::<PhysicsDebugConfig>()
            .add_system(
                debug_render_aabbs.run_if(|config: Res<PhysicsDebugConfig>| config.render_aabbs),
            )
            .add_system(
                debug_render_contacts
                    .run_if(|config: Res<PhysicsDebugConfig>| config.render_contacts),
            );
    }
}

/// Controls the [`PhysicsDebugPlugin`] configuration.
#[derive(Reflect, Resource)]
#[reflect(Resource)]
pub struct PhysicsDebugConfig {
    /// Renders the Axis-Aligned Bounding Boxes of [colliders](`Collider`).
    pub render_aabbs: bool,
    /// Renders contact points.
    pub render_contacts: bool,
}

impl Default for PhysicsDebugConfig {
    fn default() -> Self {
        Self {
            render_aabbs: true,
            render_contacts: true,
        }
    }
}

fn debug_render_aabbs(aabbs: Query<&ColliderAabb>, mut shapes: ResMut<DebugShapes>) {
    #[cfg(feature = "2d")]
    for aabb in aabbs.iter() {
        shapes
            .rect()
            .min_max(Vec2::from(aabb.mins), Vec2::from(aabb.maxs));
    }

    #[cfg(feature = "3d")]
    for aabb in aabbs.iter() {
        shapes.cuboid().min_max(
            Vector::from(aabb.mins).as_vec3_f32(),
            Vector::from(aabb.maxs).as_vec3_f32(),
        );
    }
}

#[allow(clippy::unnecessary_cast)]
fn debug_render_contacts(mut collisions: EventReader<Collision>, mut lines: ResMut<DebugLines>) {
    #[cfg(feature = "2d")]
    for Collision(contact) in collisions.iter() {
        let p1 = contact.point1.extend(0.0).as_vec3_f32();
        let p2 = contact.point2.extend(0.0).as_vec3_f32();

        lines.line_colored(p1 - Vec3::X * 0.3, p1 + Vec3::X * 0.3, 0.01, Color::CYAN);
        lines.line_colored(p1 - Vec3::Y * 0.3, p1 + Vec3::Y * 0.3, 0.01, Color::CYAN);

        lines.line_colored(p2 - Vec3::X * 0.3, p2 + Vec3::X * 0.3, 0.01, Color::CYAN);
        lines.line_colored(p2 - Vec3::Y * 0.3, p2 + Vec3::Y * 0.3, 0.01, Color::CYAN);
    }
    #[cfg(feature = "3d")]
    for Collision(contact) in collisions.iter() {
        let p1 = contact.point1.as_vec3_f32();
        let p2 = contact.point2.as_vec3_f32();

        lines.line_colored(p1 - Vec3::X * 0.3, p1 + Vec3::X * 0.3, 0.01, Color::CYAN);
        lines.line_colored(p1 - Vec3::Y * 0.3, p1 + Vec3::Y * 0.3, 0.01, Color::CYAN);
        lines.line_colored(p1 - Vec3::Z * 0.3, p1 + Vec3::Z * 0.3, 0.01, Color::CYAN);

        lines.line_colored(p2 - Vec3::X * 0.3, p2 + Vec3::X * 0.3, 0.01, Color::CYAN);
        lines.line_colored(p2 - Vec3::Y * 0.3, p2 + Vec3::Y * 0.3, 0.01, Color::CYAN);
        lines.line_colored(p2 - Vec3::Z * 0.3, p2 + Vec3::Z * 0.3, 0.01, Color::CYAN);
    }
}
