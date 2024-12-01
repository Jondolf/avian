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

use crate::{prelude::*, prepare::PrepareSet};
#[cfg(feature = "3d")]
use bevy::ecs::query::QueryFilter;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

mod components;
pub use components::*;

mod system_param;
pub use system_param::MassHelper;

#[cfg(feature = "2d")]
pub use bevy_heavy::MassProperties2d as MassProperties;
#[cfg(feature = "3d")]
pub use bevy_heavy::MassProperties3d as MassProperties;

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

        // Force mass property computation for new rigid bodies.
        app.register_required_components::<RigidBody, RecomputeMassProperties>();

        // Configure system sets for mass property computation.
        app.configure_sets(
            self.schedule,
            (
                MassPropertySystems::ColliderMassProperties,
                MassPropertySystems::QueueRecomputation,
                MassPropertySystems::ComputedMassProperties,
            )
                .chain()
                .in_set(PrepareSet::Finalize),
        );

        // Clamp collider density to be above `0.0`.
        app.add_systems(
            self.schedule,
            clamp_collider_density.before(MassPropertySystems::ColliderMassProperties),
        );

        // Queue mass property recomputation when mass properties are changed.
        app.add_systems(
            self.schedule,
            (
                queue_mass_recomputation_on_mass_change,
                queue_mass_recomputation_on_child_collider_mass_change,
            )
                .in_set(MassPropertySystems::QueueRecomputation),
        );

        // Update mass properties for entities with the `RecomputeMassProperties` component.
        app.add_systems(
            self.schedule,
            (
                update_mass_properties,
                #[cfg(feature = "3d")]
                update_global_angular_inertia::<Added<RigidBody>>,
                warn_missing_mass,
            )
                .chain()
                .in_set(MassPropertySystems::ComputedMassProperties),
        );
    }
}

/// A system set for logic related to updating mass properties.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MassPropertySystems {
    /// Update [`ColliderMassProperties`] for colliders.
    ColliderMassProperties,
    /// Adds the [`RecomputeMassProperties`] component to entities with changed mass properties.
    QueueRecomputation,
    /// Update [`ComputedMass`], [`ComputedAngularInertia`], and [`ComputedCenterOfMass`]
    /// for entities with the [`RecomputeMassProperties`] component. The component is removed after updating.
    ComputedMassProperties,
}

/// A query filter for entities with [`ComputedMass`], [`ComputedAngularInertia`], or [`ComputedCenterOfMass`].
pub type WithComputedMassProperty = Or<(
    With<ComputedMass>,
    With<ComputedAngularInertia>,
    With<ComputedCenterOfMass>,
)>;

/// A query filter for entities with changed [`Mass`], [`AngularInertia`], or [`CenterOfMass`].
pub type MassPropertyChanged = Or<(
    Changed<Mass>,
    Changed<AngularInertia>,
    Changed<CenterOfMass>,
)>;

/// Queues mass property recomputation for rigid bodies when their [`Mass`], [`AngularInertia`],
/// or [`CenterOfMass`] components are changed.
pub fn queue_mass_recomputation_on_mass_change(
    mut commands: Commands,
    mut query: Query<Entity, (WithComputedMassProperty, MassPropertyChanged)>,
) {
    for entity in &mut query {
        commands.entity(entity).insert(RecomputeMassProperties);
    }
}

/// Queues mass property recomputation for rigid bodies when the [`ColliderMassProperties`],
/// [`Mass`], [`AngularInertia`], or [`CenterOfMass`] components of their child colliders are changed.
fn queue_mass_recomputation_on_child_collider_mass_change(
    mut commands: Commands,
    mut query: Query<
        &ColliderParent,
        (
            With<RigidBody>,
            Or<(Changed<ColliderMassProperties>, MassPropertyChanged)>,
        ),
    >,
) {
    for collider_parent in &mut query {
        if let Some(mut entity_commands) = commands.get_entity(collider_parent.get()) {
            entity_commands.insert(RecomputeMassProperties);
        }
    }
}

fn update_mass_properties(
    mut commands: Commands,
    query: Query<Entity, With<RecomputeMassProperties>>,
    mut mass_helper: MassHelper,
) {
    // TODO: Parallelize mass property updates.
    for entity in query.iter() {
        mass_helper.update_mass_properties(entity);
        commands.entity(entity).remove::<RecomputeMassProperties>();
    }
}

/// An extension trait for [`MassProperties`].
pub trait MassPropertiesExt {
    /// Converts the [`MassProperties`] to a [`MassPropertiesBundle`]
    /// containing the [`Mass`], [`AngularInertia`], and [`CenterOfMass`] components.
    fn to_bundle(&self) -> MassPropertiesBundle;
}

impl MassPropertiesExt for MassProperties {
    fn to_bundle(&self) -> MassPropertiesBundle {
        #[cfg(feature = "2d")]
        let angular_inertia = AngularInertia(self.angular_inertia);
        #[cfg(feature = "3d")]
        let angular_inertia = AngularInertia::new_with_local_frame(
            self.principal_angular_inertia.f32(),
            self.local_inertial_frame.f32(),
        );

        MassPropertiesBundle {
            mass: Mass(self.mass),
            angular_inertia,
            center_of_mass: CenterOfMass(self.center_of_mass),
        }
    }
}

/// Updates [`GlobalAngularInertia`] for entities that match the given query filter `F`.
#[cfg(feature = "3d")]
pub fn update_global_angular_inertia<F: QueryFilter>(
    mut query: Populated<
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
        density.0 = density.max(f32::EPSILON);
    }
}

#[cfg(test)]
#[cfg(feature = "2d")]
#[expect(clippy::unnecessary_cast)]
mod tests {
    use super::*;

    fn create_app() -> App {
        let mut app = App::new();

        // TODO: Use minimal number of plugins
        app.add_plugins((
            MinimalPlugins,
            PhysicsPlugins::default(),
            HierarchyPlugin,
            bevy::asset::AssetPlugin::default(),
            #[cfg(feature = "bevy_scene")]
            bevy::scene::ScenePlugin,
        ))
        .init_resource::<Assets<Mesh>>();

        app
    }

    fn get_computed_mass_properties(
        world: &mut World,
        entity: Entity,
    ) -> (
        &ComputedMass,
        &ComputedAngularInertia,
        &ComputedCenterOfMass,
    ) {
        let mut query = world.query::<(
            &ComputedMass,
            &ComputedAngularInertia,
            &ComputedCenterOfMass,
        )>();
        query.get(world, entity).unwrap()
    }

    #[test]
    fn mass_properties_rb_collider() {
        // `RigidBody`, `Collider`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app.world_mut().spawn((RigidBody::Dynamic, collider)).id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, collider_mass_props.mass);
        assert_eq!(
            angular_inertia.value() as f32,
            collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider_with_set_mass() {
        // `RigidBody`, `Collider`, `Mass(5.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider, Mass(5.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(
            angular_inertia.value() as f32,
            mass.value() as f32 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider_with_set_mass_and_angular_inertia() {
        // `RigidBody`, `Collider`, `Mass(5.0)`, `AngularInertia(10.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);

        let body_entity = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                collider,
                Mass(5.0),
                AngularInertia(10.0),
            ))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(angular_inertia.value() as f32, 10.0);
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider_with_set_mass_and_child_collider() {
        // `RigidBody`, `Collider`, `Mass(5.0)`
        // - `Collider`, `ColliderDensity(2.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(2.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider.clone(), Mass(5.0)))
            .with_child((collider, ColliderDensity(2.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0 + collider_mass_props.mass);
        assert_eq!(
            angular_inertia.value() as f32,
            5.0 * collider_mass_props.unit_angular_inertia() + collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_rb_collider_with_set_mass_and_child_collider_with_set_mass() {
        // `RigidBody`, `Collider`, `Mass(5.0)`
        // - `Collider`, `ColliderDensity(2.0)`, `Mass(10.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider.clone(), Mass(5.0)))
            .with_child((collider, ColliderDensity(2.0), Mass(10.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0 + 10.0);
        assert_eq!(
            angular_inertia.value() as f32,
            5.0 * collider_mass_props.unit_angular_inertia()
                + 10.0 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_no_auto_mass() {
        // `RigidBody`, `Collider`, `Mass(5.0)`, `NoAutoMass`
        // - `Collider`, `ColliderDensity(2.0)`, `Mass(10.0)`

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider.clone(), Mass(5.0), NoAutoMass))
            .with_child((collider, ColliderDensity(2.0), Mass(10.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(
            angular_inertia.value() as f32,
            mass.value() as f32
                * (5.0 * collider_mass_props.unit_angular_inertia()
                    + 10.0 * collider_mass_props.unit_angular_inertia())
                / 15.0
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_add_remove_collider() {
        // `RigidBody`, `Collider`
        //
        // - Check mass properties
        // - Add child `Collider`
        // - Check mass properties
        // - Remove child `Collider`
        // - Check mass properties

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app.world_mut().spawn((RigidBody::Dynamic, collider)).id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, collider_mass_props.mass);
        assert_eq!(
            angular_inertia.value() as f32,
            collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());

        // Add a child collider
        let child_collider = Collider::circle(2.0);
        let child_collider_mass_props = child_collider.mass_properties(1.0);

        app.world_mut()
            .entity_mut(body_entity)
            .with_child((child_collider, ColliderDensity(1.0)));

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(
            mass.value() as f32,
            collider_mass_props.mass + child_collider_mass_props.mass
        );
        assert_eq!(
            angular_inertia.value() as f32,
            collider_mass_props.angular_inertia + child_collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());

        // Remove the child collider
        app.world_mut().entity_mut(body_entity).clear_children();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, collider_mass_props.mass);
        assert_eq!(
            angular_inertia.value() as f32,
            collider_mass_props.angular_inertia
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }

    #[test]
    fn mass_properties_change_mass() {
        // `RigidBody`, `Collider`, `Mass(5.0)`
        //
        // - Check mass properties
        // - Change mass
        // - Check mass properties

        let mut app = create_app();

        let collider = Collider::circle(1.0);
        let collider_mass_props = collider.mass_properties(1.0);

        let body_entity = app
            .world_mut()
            .spawn((RigidBody::Dynamic, collider, Mass(5.0)))
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 5.0);
        assert_eq!(
            angular_inertia.value() as f32,
            mass.value() as f32 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());

        // Change mass
        app.world_mut().entity_mut(body_entity).insert(Mass(10.0));

        app.world_mut().run_schedule(FixedPostUpdate);

        let (mass, angular_inertia, center_of_mass) =
            get_computed_mass_properties(app.world_mut(), body_entity);

        assert_eq!(mass.value() as f32, 10.0);
        assert_eq!(
            angular_inertia.value() as f32,
            mass.value() as f32 * collider_mass_props.unit_angular_inertia()
        );
        assert_eq!(*center_of_mass, ComputedCenterOfMass::default());
    }
}
