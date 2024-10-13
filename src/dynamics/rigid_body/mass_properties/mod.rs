//! Mass property functionality for rigid bodies.

#[cfg(feature = "3d")]
use crate::prepare::match_any;
use crate::{prelude::*, prepare::PrepareSet};
#[cfg(feature = "3d")]
use bevy::ecs::query::QueryFilter;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

mod components;
pub use components::*;

mod world_query;
pub use world_query::MassPropertiesQuery;

/// A plugin for managing mass properties of rigid bodies.
///
/// - Updates mass properties of rigid bodies when colliders are added or removed, or when their [`ColliderMassProperties`] are changed.
/// - Logs warnings when dynamic bodies have invalid [`Mass`] or [`AngularInertia`].
pub struct MassPropertyPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl MassPropertyPlugin {
    /// Creates a [`MassPropertyPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for MassPropertyPlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

impl Plugin for MassPropertyPlugin {
    fn build(&self, app: &mut App) {
        // Update mass properties of rigid bodies when the mass properties of attached colliders are changed.
        // This includes adding, removing, or modifying colliders.
        app.observe(
            |trigger: Trigger<OnChangeColliderMassProperties>,
             collider_parents: Query<(
                &ColliderParent,
                &PreviousColliderTransform,
                &ColliderTransform,
            )>,
             mut mass_properties: Query<MassPropertiesQuery>| {
                let Ok((collider_parent, previous_collider_transform, collider_transform)) =
                    collider_parents.get(trigger.entity())
                else {
                    return;
                };

                if let Ok(mut mass_props) = mass_properties.get_mut(collider_parent.get()) {
                    let event = trigger.event();

                    // Subtract the collider's previous mass properties, if any.
                    if let Some(previous) = event.previous {
                        mass_props -= previous.transformed_by(previous_collider_transform);
                    }

                    // Add the collider's current mass properties, if any.
                    if let Some(current) = event.current {
                        mass_props += current.transformed_by(collider_transform);
                    }
                }
            },
        );

        // Update `GlobalAngularInertia` for new rigid bodies.
        #[cfg(feature = "3d")]
        app.add_systems(
            self.schedule,
            update_global_angular_inertia::<Added<RigidBody>>
                .run_if(match_any::<Added<RigidBody>>)
                .in_set(PrepareSet::Finalize),
        );

        app.add_systems(
            self.schedule,
            (warn_missing_mass, clamp_collider_density).in_set(PrepareSet::Finalize),
        );
    }
}

/// Event triggered when the mass properties of a collider are changed.
///
/// Used to update the mass properties of rigid bodies when colliders are added, removed, or modified.
#[derive(Event)]
pub struct OnChangeColliderMassProperties {
    /// The previous mass properties of the collider.
    pub previous: Option<ColliderMassProperties>,
    /// The current mass properties of the collider.
    pub current: Option<ColliderMassProperties>,
}

/// Updates [`GlobalAngularInertia`] for entities that match the given query filter `F`.
#[cfg(feature = "3d")]
pub fn update_global_angular_inertia<F: QueryFilter>(
    mut query: Query<
        (&Rotation, &AngularInertia, &mut GlobalAngularInertia),
        (Or<(Changed<AngularInertia>, Changed<Rotation>)>, F),
    >,
) {
    query
        .par_iter_mut()
        .for_each(|(rotation, angular_inertia, mut global_angular_inertia)| {
            global_angular_inertia.update(*angular_inertia, rotation.0);
        });
}

/// Logs warnings when dynamic bodies have invalid [`Mass`] or [`AngularInertia`].
pub fn warn_missing_mass(
    mut bodies: Query<
        (Entity, &RigidBody, Ref<Mass>, Ref<AngularInertia>),
        Or<(Changed<Mass>, Changed<AngularInertia>)>,
    >,
) {
    for (entity, rb, mass, inertia) in &mut bodies {
        let is_mass_valid = mass.value().is_finite() && mass.value() >= Scalar::EPSILON;
        #[cfg(feature = "2d")]
        let is_inertia_valid = inertia.value().is_finite() && inertia.value() >= Scalar::EPSILON;
        #[cfg(feature = "3d")]
        let is_inertia_valid = inertia.value().is_finite();

        // Warn about dynamic bodies with no mass or inertia
        if rb.is_dynamic() && !(is_mass_valid && is_inertia_valid) {
            warn!(
                "Dynamic rigid body {:?} has no mass or inertia. This can cause NaN values. Consider adding a `MassPropertiesBundle` or a `Collider` with mass.",
                entity
            );
        }
    }
}

/// Clamps [`ColliderDensity`] to be above `0.0`.
fn clamp_collider_density(mut query: Query<&mut ColliderDensity, Changed<ColliderDensity>>) {
    for mut density in &mut query {
        density.0 = density.max(Scalar::EPSILON);
    }
}
