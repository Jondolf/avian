use std::ops::{AddAssign, SubAssign};

use crate::prelude::*;
use bevy::ecs::query::QueryData;

/// `QueryData` to make querying and modifying mass properties of [rigid bodies] more convenient.
///
/// [rigid bodies]: crate::dynamics::RigidBody
#[derive(QueryData)]
#[query_data(mutable)]
pub struct MassPropertiesQuery {
    /// The mass of the rigid body.
    pub mass: &'static mut Mass,
    /// The angular inertia of the rigid body.
    pub angular_inertia: &'static mut AngularInertia,
    /// The local center of mass of the rigid body.
    pub center_of_mass: &'static mut CenterOfMass,
}

impl<'w> MassPropertiesQueryItem<'w> {
    /// Computes the angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "2d")]
    #[inline]
    pub fn shifted_angular_inertia(&self, offset: Vector) -> Scalar {
        shifted_angular_inertia(self.angular_inertia.value(), self.mass.value(), offset)
    }

    /// Computes the angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn shifted_angular_inertia(&self, offset: Vector) -> Matrix {
        shifted_angular_inertia(self.angular_inertia.tensor(), self.mass.value(), offset)
    }

    /// Computes the inverse angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "2d")]
    #[inline]
    pub fn shifted_inverse_angular_inertia(&self, offset: Vector) -> Scalar {
        shifted_angular_inertia(self.angular_inertia.value(), self.mass.value(), offset)
            .recip_or_zero()
    }

    /// Computes the inverse angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn shifted_inverse_angular_inertia(&self, offset: Vector) -> Matrix {
        shifted_angular_inertia(self.angular_inertia.tensor(), self.mass.value(), offset)
            .inverse_or_zero()
    }
}

impl<'w> AddAssign<ColliderMassProperties> for MassPropertiesQueryItem<'w> {
    fn add_assign(&mut self, rhs: ColliderMassProperties) {
        let mass1 = self.mass.value();
        let mass2 = rhs.mass;
        let new_mass = mass1 + mass2;

        if new_mass <= 0.0 {
            return;
        }

        let com1 = self.center_of_mass.0;
        let com2 = rhs.center_of_mass;

        // Compute the combined center of mass and combined inertia tensor
        let new_com = (com1 * mass1 + com2 * mass2) / new_mass;
        let i1 = self.shifted_angular_inertia(new_com - com1);
        let i2 = rhs.shifted_angular_inertia(new_com - com2);
        let new_inertia = AngularInertia::from(i1 + i2);

        // Update mass properties
        self.mass.set(new_mass);
        self.angular_inertia.set(new_inertia);
        self.center_of_mass.0 = new_com;
    }
}

impl<'w> SubAssign<ColliderMassProperties> for MassPropertiesQueryItem<'w> {
    fn sub_assign(&mut self, rhs: ColliderMassProperties) {
        if self.mass.inverse() + rhs.mass.recip_or_zero() <= 0.0 {
            return;
        }

        let mass1 = self.mass.value();
        let mass2 = rhs.mass;

        let new_mass = (mass1 - mass2).max(0.0);
        let com1 = self.center_of_mass.0;
        let com2 = rhs.center_of_mass;

        // Compute the combined center of mass and combined inertia tensor
        let new_com = if new_mass > Scalar::EPSILON {
            (com1 * mass1 - com2 * mass2) / new_mass
        } else {
            com1
        };
        let i1 = self.shifted_angular_inertia(new_com - com1);
        let i2 = rhs.shifted_angular_inertia(new_com - com2);
        let new_inertia = AngularInertia::from(i1 - i2);

        // Update mass properties
        self.mass.set(new_mass);
        self.angular_inertia.set(new_inertia);
        self.center_of_mass.0 = new_com;
    }
}

#[cfg(test)]
mod tests {
    use crate::prelude::*;
    use approx::assert_relative_eq;
    use bevy::prelude::*;

    // TODO: Test if inertia values are correct
    #[test]
    fn mass_properties_add_assign_works() {
        // Create app
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);

        // Spawn an entity with mass properties
        app.world_mut().spawn(MassPropertiesBundle {
            mass: Mass::new(1.6),
            #[cfg(feature = "2d")]
            angular_inertia: AngularInertia::new(1.6),
            #[cfg(feature = "3d")]
            angular_inertia: AngularInertia::new(Vector::new(1.6, 2.4, 3.2)),
            center_of_mass: CenterOfMass(Vector::NEG_X * 3.8),
        });

        // Create collider mass properties that will be added to the existing mass properties
        let collider_mass_props = ColliderMassProperties {
            mass: 8.1,
            #[cfg(feature = "2d")]
            angular_inertia: 56.2,
            #[cfg(feature = "3d")]
            angular_inertia: Matrix::from_diagonal(Vector::new(56.2, 62.7, 71.4)),
            center_of_mass: Vector::X * 1.2 + Vector::Y,
        };

        // Get the mass properties and add the collider mass properties
        let mut query = app.world_mut().query::<MassPropertiesQuery>();
        let mut mass_props = query.single_mut(app.world_mut());
        mass_props += collider_mass_props;

        // Test if values are correct
        // (reference values were calculated by hand)
        assert_relative_eq!(mass_props.mass.value(), 9.7);
        assert_relative_eq!(mass_props.mass.inverse(), 1.0 / 9.7);
        assert_relative_eq!(
            mass_props.center_of_mass.0,
            Vector::X * 0.375_257 + Vector::Y * 0.835_051,
            epsilon = 0.000_001
        );
    }

    // TODO: Test if inertia values are correct
    #[test]
    fn mass_properties_sub_assign_works() {
        // Create app
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);

        // Spawn an entity with mass properties
        app.world_mut().spawn(MassPropertiesBundle {
            mass: Mass::new(8.1),
            #[cfg(feature = "2d")]
            angular_inertia: AngularInertia::new(56.2),
            #[cfg(feature = "3d")]
            angular_inertia: AngularInertia::new(Vector::new(56.2, 62.7, 71.4)),
            center_of_mass: CenterOfMass(Vector::NEG_X * 3.8),
        });

        // Create collider mass properties that will be subtracted from the existing mass properties
        let collider_mass_props = ColliderMassProperties {
            mass: 1.6,
            #[cfg(feature = "2d")]
            angular_inertia: 1.6,
            #[cfg(feature = "3d")]
            angular_inertia: Matrix::from_diagonal(Vector::new(1.6, 2.4, 3.2)),
            center_of_mass: Vector::X * 1.2 + Vector::Y,
        };

        // Get the mass properties and subtract the collider mass properties
        let mut query = app.world_mut().query::<MassPropertiesQuery>();
        let mut mass_props = query.single_mut(app.world_mut());
        mass_props -= collider_mass_props;

        // Test if values are correct.
        // The reference values were calculated by hand.
        // The center of mass is computed as: (com1 * mass1 - com2 * mass2) / (mass1 - mass2).max(0.0)
        assert_relative_eq!(mass_props.mass.value(), 6.5);
        assert_relative_eq!(mass_props.mass.inverse(), 1.0 / 6.5);
        assert_relative_eq!(
            mass_props.center_of_mass.0,
            Vector::NEG_X * 5.030_769 + Vector::NEG_Y * 0.246_153,
            epsilon = 0.000_001
        );
    }

    #[test]
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    fn mass_properties_add_sub_works() {
        // Create app
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);

        let original_mass_props =
            MassPropertiesBundle::new_computed(&Collider::capsule(0.6, 2.4), 3.9);

        // Spawn an entity with mass properties
        app.world_mut().spawn(original_mass_props.clone());

        // Create collider mass properties
        let collider_mass_props = Collider::capsule(2.1, 7.4).mass_properties(14.3);

        // Get the mass properties and then add and subtract the collider mass properties
        let mut query = app.world_mut().query::<MassPropertiesQuery>();
        let mut mass_props = query.single_mut(app.world_mut());
        mass_props += collider_mass_props;
        mass_props -= collider_mass_props;

        // Test if values are correct. They should be equal to the original values.
        // Some epsilons reduced to make test pass on apple-m1
        // see: https://github.com/Jondolf/avian/issues/137
        assert_relative_eq!(
            mass_props.mass.value(),
            original_mass_props.mass.value(),
            epsilon = 0.001
        );
        assert_relative_eq!(
            mass_props.mass.inverse(),
            original_mass_props.mass.inverse(),
            epsilon = 0.000_001
        );
        assert_relative_eq!(
            mass_props.angular_inertia.value(),
            original_mass_props.angular_inertia.value(),
            epsilon = 0.01
        );
        assert_relative_eq!(
            mass_props.angular_inertia.inverse(),
            original_mass_props.angular_inertia.inverse(),
            epsilon = 0.001
        );
        assert_relative_eq!(
            mass_props.center_of_mass.0,
            original_mass_props.center_of_mass.0,
            epsilon = 0.000_001
        );
    }
}
