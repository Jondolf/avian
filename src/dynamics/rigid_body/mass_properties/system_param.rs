use crate::prelude::*;
use bevy::{
    ecs::system::{
        lifetimeless::{Read, Write},
        SystemParam,
    },
    prelude::*,
};

/// A [`SystemParam`] that provides helper methods for computing and updating mass properties.
#[derive(SystemParam)]
pub struct MassPropertyHelper<'w, 's> {
    query: Query<
        'w,
        's,
        (
            Option<Read<Mass>>,
            Option<Read<AngularInertia>>,
            Option<Read<CenterOfMass>>,
            Option<Read<ColliderMassProperties>>,
            Option<Read<ColliderTransform>>,
            Has<Sensor>,
        ),
        Or<(
            With<Mass>,
            With<AngularInertia>,
            With<CenterOfMass>,
            (
                With<ColliderMassProperties>,
                With<ColliderTransform>,
                Without<Sensor>,
            ),
        )>,
    >,
    computed_mass_properties: Query<
        'w,
        's,
        (
            Write<ComputedMass>,
            Write<ComputedAngularInertia>,
            Write<ComputedCenterOfMass>,
        ),
    >,
    no_auto_query: Query<
        'w,
        's,
        (
            Has<NoAutoMass>,
            Has<NoAutoAngularInertia>,
            Has<NoAutoCenterOfMass>,
        ),
        Or<(
            With<NoAutoMass>,
            With<NoAutoAngularInertia>,
            With<NoAutoCenterOfMass>,
        )>,
    >,
    children: Query<'w, 's, Read<Children>>,
}

// TODO: Override mass properties for specific entities.
impl MassPropertyHelper<'_, '_> {
    /// Updates the [`ComputedMass`], [`ComputedAngularInertia`], and [`ComputedCenterOfMass`] of the given entity.
    ///
    /// This takes into account the mass properties of descendants, unless the given entity has the [`NoAutoMass`],
    /// [`NoAutoAngularInertia`], or [`NoAutoCenterOfMass`] marker components.
    pub fn update_mass_properties(&mut self, entity: Entity) {
        let mut mass_props = self.total_mass_properties(entity);

        if let Ok((no_auto_mass, no_auto_inertia, no_auto_com)) = self.no_auto_query.get(entity) {
            let local_mass_props = self.local_mass_properties(entity).unwrap_or_default();

            if no_auto_mass {
                mass_props.set_mass(local_mass_props.mass, !no_auto_inertia);
            }

            if no_auto_inertia {
                #[cfg(feature = "2d")]
                {
                    mass_props.angular_inertia = local_mass_props.angular_inertia;
                }
                #[cfg(feature = "3d")]
                {
                    mass_props.principal_angular_inertia =
                        local_mass_props.principal_angular_inertia;
                    mass_props.local_inertial_frame = local_mass_props.local_inertial_frame;
                }
            }

            if no_auto_com {
                mass_props.center_of_mass = local_mass_props.center_of_mass;
            }
        }

        let Ok((mut computed_mass, mut computed_inertia, mut computed_com)) =
            self.computed_mass_properties.get_mut(entity)
        else {
            return;
        };

        computed_mass.set(mass_props.mass as Scalar);
        #[cfg(feature = "2d")]
        {
            computed_inertia.set(mass_props.angular_inertia as Scalar);
        }
        #[cfg(feature = "3d")]
        {
            computed_inertia.set(
                mass_props
                    .angular_inertia_tensor()
                    .as_mat3()
                    .adjust_precision(),
            );
        }
        computed_com.0 = mass_props.center_of_mass.adjust_precision();
    }

    /// Computes the total mass properties of the given entity,
    /// taking into account the mass properties of descendants.
    ///
    /// This ignores the [`NoAutoMass`], [`NoAutoAngularInertia`], and [`NoAutoCenterOfMass`] marker components.
    pub fn total_mass_properties(&self, entity: Entity) -> MassProperties {
        std::iter::once(self.local_mass_properties(entity))
            .chain(
                self.children
                    .iter_descendants(entity)
                    .map(|child| self.local_mass_properties(child)),
            )
            .flatten()
            .sum()
    }

    /// Computes the total mass properties of the descendants of the given entity.
    ///
    /// This ignores the [`NoAutoMass`], [`NoAutoAngularInertia`], and [`NoAutoCenterOfMass`] marker components.
    pub fn descendants_mass_properties(&self, entity: Entity) -> MassProperties {
        self.children
            .iter_descendants(entity)
            .filter_map(|child| self.local_mass_properties(child))
            .sum()
    }

    /// Computes the local mass properties of the given entity.
    ///
    /// This only considers the entity's own [`Mass`], [`AngularInertia`], [`CenterOfMass`],
    /// and/or [`ColliderMassProperties`] if present, not those of its children.
    ///
    /// If the entity has no mass properties or the entity does not exist, `None` is returned.
    pub fn local_mass_properties(&self, entity: Entity) -> Option<MassProperties> {
        let (mass, angular_inertia, center_of_mass, collider_mass, collider_transform, is_sensor) =
            self.query.get(entity).ok()?;

        // Initialize the mass properties with the collider's mass properties or zero.
        let mut mass_props = collider_mass
            .filter(|_| !is_sensor)
            .map_or(MassProperties::ZERO, |m| m.0);

        // Set the mass if the `Mass` component is present.
        if let Some(mass) = mass {
            // TODO: This needs to consider `NoAutoMass`.
            // Only adjust the angular inertia if it is not exlicitly set with `AngularInertia`.
            let update_angular_inertia = angular_inertia.is_none();
            mass_props.set_mass(mass.0, update_angular_inertia);
        }

        // Set the angular inertia if the `AngularInertia` component is present.
        if let Some(angular_inertia) = angular_inertia {
            #[cfg(feature = "2d")]
            {
                mass_props.angular_inertia = angular_inertia.0;
            }
            #[cfg(feature = "3d")]
            {
                mass_props.principal_angular_inertia = angular_inertia.principal;
                mass_props.local_inertial_frame = angular_inertia.local_frame;
            }
        }

        // Set the center of mass if the `CenterOfMass` component is present.
        if let Some(center_of_mass) = center_of_mass {
            mass_props.center_of_mass = center_of_mass.0;
        }

        if let Some(collider_transform) = collider_mass.and(collider_transform) {
            #[cfg(feature = "2d")]
            {
                mass_props.transform_by(Isometry2d::new(
                    collider_transform.translation.f32(),
                    Rot2::from(collider_transform.rotation),
                ));
            }
            #[cfg(feature = "3d")]
            {
                mass_props.transform_by(Isometry3d::new(
                    collider_transform.translation.f32(),
                    collider_transform.rotation.f32(),
                ));
            }
        }

        Some(mass_props)
    }
}