//! Runs systems that prepare and initialize components used by physics.
//!
//! See [`PreparePlugin`].

#![allow(clippy::type_complexity)]

use crate::{prelude::*, sync::SyncConfig};
use bevy::{
    ecs::{intern::Interned, query::QueryFilter, schedule::ScheduleLabel},
    prelude::*,
};

/// Runs systems at the start of each physics frame. Initializes [rigid bodies](RigidBody)
/// and updates components.
///
/// - Adds missing rigid body components for entities with a [`RigidBody`] component
/// - Clamps restitution coefficients between 0 and 1
///
/// The [`Position`] and [`Rotation`] components will be initialized based on [`Transform`]
/// or vice versa. You can configure this synchronization using the [`PrepareConfig`] resource.
///
/// The plugin takes a collider type. This should be [`Collider`] for
/// the vast majority of applications, but for custom collisión backends
/// you may use any collider that implements the [`AnyCollider`] trait.
///
/// The systems run in [`PhysicsSet::Prepare`].
pub struct PreparePlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PreparePlugin {
    /// Creates a [`PreparePlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PreparePlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

/// Systems sets for initializing and syncing missing components.
/// You can use these to schedule your own initialization systems
/// without having to worry about implementation details.
///
/// 1. `First`: Runs at the start of the preparation step.
/// 2. `PropagateTransforms`: Responsible for propagating transforms.
/// 3. `InitTransforms`: Responsible for initializing [`Position`] and [`Rotation`] based on [`Transform`]
///    or vice versa.
/// 4. `Finalize`: Responsible for performing final updates after everything is initialized and updated.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PrepareSet {
    /// Runs at the start of the preparation step.
    First,
    /// Responsible for propagating transforms.
    PropagateTransforms,
    /// Responsible for initializing [`Position`] and [`Rotation`] based on [`Transform`]
    /// or vice versa. Parts of this system can be disabled with [`PrepareConfig`].
    /// Schedule your system with this to implement custom behavior for initializing transforms.
    InitTransforms,
    /// Responsible for performing final updates after everything is initialized.
    /// Updates mass properties and clamps collider density and restitution.
    Finalize,
}

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SyncConfig>()
            .register_type::<SyncConfig>();
        app.configure_sets(
            self.schedule,
            (
                PrepareSet::First,
                PrepareSet::PropagateTransforms,
                PrepareSet::InitTransforms,
                PrepareSet::Finalize,
            )
                .chain()
                .in_set(PhysicsSet::Prepare),
        );

        app.init_resource::<PrepareConfig>()
            .register_type::<PrepareConfig>();

        // Note: Collider logic is handled by the `ColliderBackendPlugin`
        app.add_systems(
            self.schedule,
            // Run transform propagation if new bodies have been added
            (
                crate::sync::sync_simple_transforms_physics,
                crate::sync::propagate_transforms_physics,
            )
                .chain()
                .run_if(match_any::<Added<RigidBody>>)
                .in_set(PrepareSet::PropagateTransforms),
        )
        .add_systems(
            self.schedule,
            init_transforms::<RigidBody>.in_set(PrepareSet::InitTransforms),
        )
        .add_systems(
            self.schedule,
            clamp_restitution.in_set(PrepareSet::Finalize),
        );
    }
}

/// Configures what is initialized by the [`PreparePlugin`] and how.
#[derive(Resource, Reflect, Clone, Debug, PartialEq, Eq)]
#[reflect(Resource)]
pub struct PrepareConfig {
    /// If `true`, [`Position`] and [`Rotation`] are initialized based on [`Transform`].
    ///
    /// Defaults to `true`.
    pub transform_to_position: bool,
    /// If `true`, [`Transform`] exists, and `transform_to_position` is `false`,
    /// transforms are initialized based on [`Position`] and [`Rotation`].
    ///
    /// Defaults to `true`.
    pub position_to_transform: bool,
}

impl Default for PrepareConfig {
    fn default() -> Self {
        PrepareConfig {
            position_to_transform: true,
            transform_to_position: true,
        }
    }
}

/// A run condition that returns `true` if any entity matches the given query filter.
pub(crate) fn match_any<F: QueryFilter>(query: Query<(), F>) -> bool {
    !query.is_empty()
}

/// Initializes [`Position`] and [`Rotation`] based on [`GlobalTransform`] or vice versa
/// when a component of the given type is inserted.
pub fn init_transforms<C: Component>(
    config: Res<PrepareConfig>,
    mut query: Query<
        (
            &mut Transform,
            &mut Position,
            &mut Rotation,
            Option<&mut PreviousRotation>,
            Option<&Parent>,
        ),
        Added<C>,
    >,
    parents: Query<&GlobalTransform, With<Children>>,
) {
    if !config.position_to_transform && !config.transform_to_position {
        // Nothing to do
        return;
    }

    for (mut transform, mut pos, mut rot, previous_rot, parent) in &mut query {
        let parent_transform = parent.and_then(|parent| parents.get(parent.get()).ok());

        if config.transform_to_position {
            // Compute the global transform. Here we could also use `GlobalTransform` directly,
            // but depending on the setup, transform propagation and synchronization may not have run yet.
            let transform = parent_transform.map_or(*transform, |t| {
                t.compute_transform().mul_transform(*transform)
            });

            // If `transform_to_position` is enabled, initialize `Position` and `Rotation` based on `GlobalTransform`.
            #[cfg(feature = "2d")]
            {
                pos.0 = transform.translation.truncate().adjust_precision();
            }
            #[cfg(feature = "3d")]
            {
                pos.0 = transform.translation.adjust_precision();
            }
            *rot = transform.rotation.into();
        } else if config.position_to_transform {
            // If `position_to_transform` is enabled but `transform_to_position` is not,
            // update `Transform` based on `Position` and `Rotation`.

            #[cfg(feature = "2d")]
            let mut new_translation = pos.f32().extend(transform.translation.z);
            #[cfg(feature = "3d")]
            let mut new_translation = pos.f32();
            let mut new_rotation = Quaternion::from(*rot).f32();

            // If the body is a child, subtract the parent's global translation
            // to get the local translation
            if let Some(parent_transform) = parent_transform {
                new_translation -= parent_transform.translation();
                new_rotation *= parent_transform.compute_transform().rotation.inverse();
            }

            transform.translation = new_translation;
            transform.rotation = new_rotation;
        }

        // Update the previous rotation
        if let Some(mut previous_rotation) = previous_rot {
            *previous_rotation = PreviousRotation(*rot);
        }
    }
}

/// Clamps coefficients of [restitution](Restitution) to be between 0.0 and 1.0.
fn clamp_restitution(mut query: Query<&mut Restitution, Changed<Restitution>>) {
    for mut restitution in &mut query {
        restitution.coefficient = restitution.coefficient.clamp(0.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_init_transforms_basics() {
        let mut app = App::new();

        // Add system under test
        app.add_systems(Update, init_transforms::<RigidBody>);

        app.insert_resource(PrepareConfig {
            transform_to_position: true,
            position_to_transform: true,
        });

        let e1 = app
            .world_mut()
            .spawn((RigidBody::Dynamic, Transform::from_translation(Vec3::X)))
            .id();

        let e2 = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Transform::from_translation(Vec3::X),
                Position(Vector::Y),
            ))
            .id();

        let e3 = app
            .world_mut()
            .spawn((RigidBody::Dynamic, Position(Vector::Y)))
            .id();

        app.update();

        assert_eq!(
            app.world().get::<Transform>(e1).unwrap().translation,
            Vec3::X
        );
        assert_eq!(
            app.world().get::<Transform>(e2).unwrap().translation,
            Vec3::X
        );
        assert!(app.world().get::<Transform>(e3).is_none());
        assert_eq!(app.world().get::<Position>(e1).unwrap().0, Vector::X);
        assert_eq!(app.world().get::<Position>(e2).unwrap().0, Vector::X);
        assert_eq!(app.world().get::<Position>(e3).unwrap().0, Vector::Y);

        app.insert_resource(PrepareConfig {
            transform_to_position: false,
            position_to_transform: true,
        });

        let e4 = app
            .world_mut()
            .spawn((RigidBody::Dynamic, Transform::from_translation(Vec3::X)))
            .id();

        let e5 = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Transform::from_translation(Vec3::X),
                Position(Vector::Y),
            ))
            .id();

        let e6 = app
            .world_mut()
            .spawn((RigidBody::Dynamic, Position(Vector::Y)))
            .id();

        app.update();

        assert_eq!(
            app.world().get::<Transform>(e4).unwrap().translation,
            Vec3::ZERO
        );
        assert_eq!(
            app.world().get::<Transform>(e5).unwrap().translation,
            Vec3::Y
        );
        assert!(app.world().get::<Transform>(e6).is_none());
        assert_eq!(app.world().get::<Position>(e4).unwrap().0, Vector::ZERO);
        assert_eq!(app.world().get::<Position>(e5).unwrap().0, Vector::Y);
        assert_eq!(app.world().get::<Position>(e6).unwrap().0, Vector::Y);
    }
}
