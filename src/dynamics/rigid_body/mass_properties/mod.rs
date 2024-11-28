//! Mass property functionality for [rigid bodies](RigidBody) and colliders.
//!
//! A dynamic rigid body has [`Mass`], [`AngularInertia`], and a [`CenterOfMass`].
//! These components determine how the rigid body responds to forces and torques.
//! Static and kinematic rigid bodies do not have mass or angular inertia.
//!
//! By default, the mass properties of a body are computed automatically from attached colliders
//! based on their [`ColliderDensity`] and shape. The mass properties of colliders are stored
//! in the [`ColliderMassProperties`] component. If [`Mass`], [`AngularInertia`], or [`CenterOfMass`]
//! are present for a collider, they override the computed values.
//!
//! # Initialization
//!
//! Given an entity hierarchy like this:
//!
//! - `RigidBody`, `Collider`
//!     - `Collider`
//!     - `Collider`
//!
//! The total mass properties will be the sum of the mass properties of all three colliders.
//!
//! Mass properties can be specified at spawn for individual entities:
//!
//! - `RigidBody`, `Collider`, `Mass`
//!     - `Collider`, `AngularInertia`
//!     - `Collider`
//!
//! Here, the mass of the rigid body entity and the angular inertia of the first collider are overridden.
//! However, the mass properties of the two child colliders still contribute to the total mass properties of the body.
//!
//! For full control over mass properties, automatic computation can be disabled
//! with the `NoAutoMass`, `NoAutoAngularInertia`, and `NoAutoCenterOfMass` marker components.
//!
//! # Changing Mass Properties at Runtime
//!
//! The mass properties of a rigid body can be modified at runtime in two primary ways:
//!
//! 1. Adjust [`Mass`], [`AngularInertia`], and [`CenterOfMass`] manually.
//! 2. Add, remove, transform, or modify colliders.
//!
//! If adjusting mass properties manually, care must be taken to ensure that mass doesn't become non-positive
//! if colliders are later modified or removed. For example, if the mass of a body is overridden to be `5.0`,
//! and a collider with mass `10.0` is removed, you would end up with a mass of `-5.0`.
//! With `debug_assertions` enabled, this will result in a panic.
//!
//! For full control over mass properties, automatic computation can be disabled
//! with the `NoAutoMass`, `NoAutoAngularInertia`, and `NoAutoCenterOfMass` marker components.
//!
//! # Mass Helper
//!
//! The [`MassHelper`] [system parameter](bevy::ecs::system::SystemParam) provides convenient helper methods
//! that can be used to modify or compute mass properties for individual entities and hierarchies at runtime.
//!
//! For example, [`MassHelper::mass_properties_from_colliders`] can be used to compute the mass properties
//! computed from colliders attached to the given entity. This can be useful when you want to revert to
//! automatically computed mass properties after manually adjusting mass properties.

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
        app.register_type::<(
            Mass,
            AngularInertia,
            CenterOfMass,
            ComputedMass,
            ComputedAngularInertia,
            ComputedCenterOfMass,
            ColliderDensity,
            ColliderMassProperties,
            NoAutoMass,
            NoAutoAngularInertia,
            NoAutoCenterOfMass,
        )>();

        // Update mass properties of rigid bodies when the mass properties of attached colliders are changed.
        // This includes adding, removing, or modifying colliders.
        app.add_observer(on_change_collider_mass_properties);

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
            (warn_missing_mass, clamp_collider_density)
                .chain()
                .in_set(PrepareSet::Finalize),
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

fn on_change_collider_mass_properties(
    trigger: Trigger<OnChangeColliderMassProperties>,
    collider_parents: Query<(
        &ColliderParent,
        &PreviousColliderTransform,
        &ColliderTransform,
    )>,
    mut bodies: Query<(
        Ref<RigidBody>,
        Has<NoAutoMass>,
        Has<NoAutoAngularInertia>,
        Has<NoAutoCenterOfMass>,
    )>,
    mut mass_properties: ParamSet<(
        Query<MassPropertiesQuery>,
        Query<(
            Option<&ComputedMass>,
            Option<&ComputedAngularInertia>,
            Option<&ComputedCenterOfMass>,
        )>,
    )>,
) {
    let collider_entity = trigger.entity();

    let Ok((collider_parent, previous_collider_transform, collider_transform)) =
        collider_parents.get(collider_entity)
    else {
        return;
    };

    let m1 = mass_properties.p1();
    let Ok((collider_mass, collider_angular_inertia, collider_center_of_mass)) =
        m1.get(collider_entity)
    else {
        return;
    };

    let (collider_mass, collider_angular_inertia, collider_center_of_mass) = (
        collider_mass.copied(),
        collider_angular_inertia.copied(),
        collider_center_of_mass.copied(),
    );

    let rb_entity = collider_parent.get();

    if let Ok((rb, override_mass, override_angular_inertia, override_center_of_mass)) =
        bodies.get_mut(rb_entity)
    {
        let mut m0 = mass_properties.p0();
        let Ok(mut mass_props) = m0.get_mut(rb_entity) else {
            return;
        };

        let event = trigger.event();

        // Subtract the collider's previous mass properties, if any.
        if let Some(previous) = event.previous {
            let (mut mass, mut angular_inertia, mut com) =
                &mass_props - previous.transformed_by(previous_collider_transform);

            if rb_entity == collider_entity && rb.is_added() {
                if let Some(collider_mass) = collider_mass {
                    mass = collider_mass
                };
                if let Some(collider_angular_inertia) = collider_angular_inertia {
                    angular_inertia = collider_angular_inertia
                };
                if let Some(collider_center_of_mass) = collider_center_of_mass {
                    com = collider_center_of_mass
                };
            }

            if !override_mass && !override_angular_inertia {
                // Update mass and angular inertia to the computed values.
                mass_props.mass.set(mass);
                mass_props.angular_inertia.set(angular_inertia);
            } else if override_mass && !override_angular_inertia {
                // The mass is overridden, so the computed angular inertia might not be correct.
                // We need to scale the angular inertia by the ratio of the new mass to the old mass
                // to get the angular inertia corresponding to the overridden mass.
                let mass_ratio = mass_props.mass.inverse() * mass.inverse();
                let inverse_angular_inertia =
                    mass_ratio.recip_or_zero() * angular_inertia.inverse();
                mass_props
                    .angular_inertia
                    .set(ComputedAngularInertia::from_inverse(
                        inverse_angular_inertia,
                    ));
            } else if !override_mass {
                // Angular inertia is overridden, but mass is not.
                mass_props.mass.set(mass);
            }

            if !override_center_of_mass {
                mass_props.center_of_mass.0 = com.0;
            }
        }

        // Add the collider's current mass properties, if any.
        if let Some(current) = event.current {
            mass_props += current.transformed_by(collider_transform);
        }
    }
}

/// Updates [`GlobalAngularInertia`] for entities that match the given query filter `F`.
#[cfg(feature = "3d")]
pub fn update_global_angular_inertia<F: QueryFilter>(
    mut query: Query<
        (
            &Rotation,
            &ComputedAngularInertia,
            &mut GlobalAngularInertia,
        ),
        (Or<(Changed<ComputedAngularInertia>, Changed<Rotation>)>, F),
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
        (
            Entity,
            &RigidBody,
            Ref<ComputedMass>,
            Ref<ComputedAngularInertia>,
        ),
        Or<(Changed<ComputedMass>, Changed<ComputedAngularInertia>)>,
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

#[cfg(test)]
mod tests {
    #[cfg(feature = "2d")]
    use super::*;

    #[test]
    #[cfg(feature = "2d")]
    fn mass_properties_simple() {
        let mut app = App::new();

        app.add_plugins((
            MinimalPlugins,
            PhysicsPlugins::default(),
            HierarchyPlugin,
            bevy::asset::AssetPlugin::default(),
            #[cfg(feature = "bevy_scene")]
            bevy::scene::ScenePlugin,
        ))
        .init_resource::<Assets<Mesh>>();

        let collider = Collider::circle(1.0);
        let collider_mass_properties = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, ComputedMass::new(5.0)))
            .id();

        app.world_mut().spawn(collider).set_parent(body_entity);

        app.world_mut().run_schedule(FixedPostUpdate);

        let mut query = app.world_mut().query::<MassPropertiesQuery>();
        let mass_props = query.get(app.world_mut(), body_entity).unwrap();

        assert_eq!(mass_props.mass.value(), 5.0 + collider_mass_properties.mass);
        assert_eq!(
            mass_props.angular_inertia.value(),
            collider_mass_properties.angular_inertia
        );
        assert_eq!(*mass_props.center_of_mass, ComputedCenterOfMass::default());
    }
}
