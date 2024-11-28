use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;

mod collider_mass;
pub use collider_mass::*;

mod computed_mass;
pub use computed_mass::*;

/// An error returned for an invalid mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MassError {
    /// The mass is negative.
    Negative,
    /// The mass is NaN.
    NaN,
}

/// The mass of an entity. Contributes to the total [`ComputedMass`] of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn setup(mut commands: Commands) {
///     // A rigid body with a mass of `5.0`.
///     commands.spawn((RigidBody::Dynamic, Mass(5.0)));
///
///     // The totsl mass will be the mass computed for the capsule shape
///     // plus the mass of `5.0` on the child collider.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(0.5, 1.5),
///         ColliderDensity(2.0),
///     ))
#[cfg_attr(
    feature = "2d",
    doc = "    .with_child((Collider::circle(1.0), Mass(5.0)));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    .with_child((Collider::sphere(1.0), Mass(5.0)));"
)]
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Mass(pub Scalar);

// TODO: Add errors for asymmetric and non-positive definite matrices in 3D.
/// An error returned for an invalid angular inertia.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AngularInertiaError {
    /// The angular inertia is negative.
    Negative,
    /// The angular inertia is NaN.
    NaN,
}

/// The moment of inertia of an entity, representing the torque needed for a desired angular acceleration.
/// Contributes to the total [`ComputedAngularInertia`] of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AngularInertia(pub Scalar);

#[cfg(feature = "2d")]
impl AngularInertia {
    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Scalar {
        shifted_angular_inertia(self.0, mass, offset)
    }
}

/// The local moment of inertia of a dynamic [rigid body], representing the torque needed
/// for a desired angular acceleration about the XYZ axes. Contributes to the total [`ComputedAngularInertia`]
/// of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct AngularInertia {
    /// The principal angular inertia, representing the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the `local_frame`.
    pub principal: Vector,
    /// The orientation of the local inertial frame.
    pub local_frame: Quaternion,
}

#[cfg(feature = "3d")]
impl AngularInertia {
    /// Creates a new [`AngularInertia`] from the given principal angular inertia.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes.
    ///
    /// To specify the orientation of the local inertial frame, consider using [`AngularInertia::new_with_local_frame`].
    ///
    /// # Panics
    ///
    /// Panics if any component of the principal angular inertia is negative or NaN when `debug_assertions` are enabled.
    #[inline]
    #[doc(alias = "from_principal_angular_inertia")]
    pub fn new(principal_angular_inertia: Vector) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vector::ZERO).all()
                && !principal_angular_inertia.is_nan(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self {
            principal: principal_angular_inertia,
            local_frame: Quaternion::IDENTITY,
        }
    }

    /// Tries to create a new [`AngularInertia`] from the given principal angular inertia.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes. To specify the orientation of the local inertial frame,
    /// consider using [`AngularInertia::try_new_with_local_frame`].
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaError)`](AngularInertiaError) if any component of the principal angular inertia is negative or NaN.
    #[inline]
    pub fn try_new(principal_angular_inertia: Vector) -> Result<Self, AngularInertiaError> {
        if principal_angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if !principal_angular_inertia.cmpge(Vector::ZERO).all() {
            Err(AngularInertiaError::Negative)
        } else {
            Ok(Self {
                principal: principal_angular_inertia,
                local_frame: Quaternion::IDENTITY,
            })
        }
    }

    /// Creates a new [`AngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the given `local_frame`.
    ///
    /// # Panics
    ///
    /// Panics if any component of the principal angular inertia is negative or NaN when `debug_assertions` are enabled.
    #[inline]
    #[doc(alias = "from_principal_angular_inertia_with_local_frame")]
    pub fn new_with_local_frame(
        principal_angular_inertia: Vector,
        local_frame: Quaternion,
    ) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vector::ZERO).all()
                && !principal_angular_inertia.is_nan(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self {
            principal: principal_angular_inertia,
            local_frame,
        }
    }

    /// Tries to create a new [`AngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the given `local_frame`.
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaError)`](AngularInertiaError) if any component of the principal angular inertia is negative or NaN.
    #[inline]
    pub fn try_new_with_local_frame(
        principal_angular_inertia: Vector,
        local_frame: Quaternion,
    ) -> Result<Self, AngularInertiaError> {
        if principal_angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if !principal_angular_inertia.cmpge(Vector::ZERO).all() {
            Err(AngularInertiaError::Negative)
        } else {
            Ok(Self {
                principal: principal_angular_inertia,
                local_frame,
            })
        }
    }

    /// Creates a new [`AngularInertia`] from the given angular inertia tensor.
    ///
    /// The tensor should be symmetric and positive definite.
    #[inline]
    #[doc(alias = "from_mat3")]
    pub fn from_tensor(tensor: Matrix) -> Self {
        let mut eigen = crate::math::eigen3::SymmetricEigen3::new(tensor).reverse();

        if eigen.eigenvectors.determinant() < 0.0 {
            std::mem::swap(
                &mut eigen.eigenvectors.y_axis,
                &mut eigen.eigenvectors.z_axis,
            );
            std::mem::swap(&mut eigen.eigenvalues.y, &mut eigen.eigenvalues.z);
        }

        let mut local_inertial_frame = Quaternion::from_mat3(&eigen.eigenvectors).normalize();

        if !local_inertial_frame.is_finite() {
            local_inertial_frame = Quaternion::IDENTITY;
        }

        // Clamp eigenvalues to be non-negative.
        let principal_angular_inertia = eigen.eigenvalues.max(Vector::ZERO);

        Self {
            principal: principal_angular_inertia,
            local_frame: local_inertial_frame,
        }
    }

    /// Returns the angular inertia tensor.
    #[inline]
    #[doc(alias = "as_mat3")]
    pub fn tensor(self) -> Matrix {
        Matrix::from_quat(self.local_frame)
            * Matrix::from_diagonal(self.principal)
            * Matrix::from_quat(self.local_frame.inverse())
    }

    /// Computes the angular inertia tensor with the given rotation.
    ///
    /// This can be used to transform local angular inertia to world space.
    #[inline]
    pub fn rotated_tensor(self, rotation: Quaternion) -> Matrix {
        let rot_mat3 = Matrix::from_quat(rotation);
        (rot_mat3 * self.tensor()) * rot_mat3.transpose()
    }

    /// Computes the angular inertia tensor shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted_tensor(&self, mass: Scalar, offset: Vector) -> Matrix {
        shifted_angular_inertia(self.tensor(), mass, offset)
    }

    /// Returns `true` if the principal angular inertia and inertial local frame are neither infinite nor NaN.
    #[inline]
    pub fn is_finite(self) -> bool {
        self.principal.is_finite() && self.local_frame.is_finite()
    }

    /// Returns `true` if the principal angular inertia or inertial local frame is NaN.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.principal.is_nan() || self.local_frame.is_nan()
    }
}

#[cfg(feature = "3d")]
impl From<Matrix> for AngularInertia {
    fn from(tensor: Matrix) -> Self {
        Self::from_tensor(tensor)
    }
}

/// The local center of mass of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct CenterOfMass(pub Vector);

impl CenterOfMass {
    /// A center of mass set at the local origin.
    pub const ZERO: Self = Self(Vector::ZERO);
}

/// A marker component that disables automatic [`Mass`] computation
/// from attached colliders for a [rigid body].
///
/// This is useful when you want full control over mass.
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct NoAutoMass;

/// A marker component that disables automatic [`AngularInertia`] computation
/// from attached colliders for a [rigid body].
///
/// This is useful when you want full control over angular inertia.
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct NoAutoAngularInertia;

/// A marker component that disables automatic [`CenterOfMass`] computation
/// from attached colliders for a [rigid body].
///
/// This is useful when you want full control over the center of mass.
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct NoAutoCenterOfMass;

/// A bundle containing mass properties.
///
/// # Example
///
/// The easiest way to create a new bundle is to use the [`new_computed`](Self::new_computed) method
/// that computes the mass properties based on a given [`Collider`] and density.
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(
    feature = "2d",
    doc = "        MassPropertiesBundle::new_computed(&Collider::circle(0.5), 1.0),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        MassPropertiesBundle::new_computed(&Collider::sphere(0.5), 1.0),"
)]
///     ));
/// }
/// ```
#[allow(missing_docs)]
#[derive(Bundle, Debug, Default, Clone, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct MassPropertiesBundle {
    pub mass: Mass,
    pub angular_inertia: AngularInertia,
    pub center_of_mass: CenterOfMass,
}

impl MassPropertiesBundle {
    /// Computes the mass properties for a [`Collider`] based on its shape and a given density.
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    pub fn new_computed(collider: &Collider, density: Scalar) -> Self {
        let ColliderMassProperties {
            mass,
            angular_inertia,
            center_of_mass,
            ..
        } = collider.mass_properties(density);

        Self {
            mass: Mass(mass),
            #[cfg(feature = "2d")]
            angular_inertia: AngularInertia(angular_inertia),
            #[cfg(feature = "3d")]
            angular_inertia: AngularInertia::from_tensor(angular_inertia),
            center_of_mass: CenterOfMass(center_of_mass),
        }
    }
}

#[cfg(feature = "2d")]
#[inline]
pub(crate) fn shifted_angular_inertia(
    angular_inertia: Scalar,
    mass: Scalar,
    offset: Vector,
) -> Scalar {
    if mass > 0.0 && mass.is_finite() && offset != Vector::ZERO {
        angular_inertia + offset.length_squared() * mass
    } else {
        angular_inertia
    }
}

#[cfg(feature = "3d")]
#[inline]
pub(crate) fn shifted_angular_inertia(tensor: Matrix, mass: Scalar, offset: Vector) -> Matrix {
    if mass > 0.0 && mass.is_finite() && offset != Vector::ZERO {
        let diagonal_element = offset.length_squared();
        let diagonal_mat = Matrix3::from_diagonal(Vector::splat(diagonal_element));
        let offset_outer_product =
            Matrix3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
        tensor + (diagonal_mat + offset_outer_product) * mass
    } else {
        tensor
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "3d")]
    use super::*;
    #[cfg(feature = "3d")]
    use approx::assert_relative_eq;

    #[test]
    #[cfg(feature = "3d")]
    fn angular_inertia_creation() {
        let angular_inertia = AngularInertia::new(Vector::new(10.0, 20.0, 30.0));
        assert_eq!(angular_inertia.principal, Vector::new(10.0, 20.0, 30.0));
        assert_eq!(angular_inertia.local_frame, Quaternion::IDENTITY);
        assert_eq!(
            angular_inertia,
            AngularInertia::from_tensor(Matrix::from_diagonal(Vector::new(10.0, 20.0, 30.0)))
        );
        assert_relative_eq!(
            angular_inertia.tensor(),
            Matrix::from_diagonal(Vector::new(10.0, 20.0, 30.0))
        );
    }

    #[test]
    #[should_panic]
    #[cfg(feature = "3d")]
    fn negative_angular_inertia_panics() {
        AngularInertia::new(Vector::new(-1.0, 2.0, 3.0));
    }

    #[test]
    #[cfg(feature = "3d")]
    fn negative_angular_inertia_error() {
        assert_eq!(
            AngularInertia::try_new(Vector::new(-1.0, 2.0, 3.0)),
            Err(AngularInertiaError::Negative),
            "negative angular inertia should return an error"
        );
    }

    #[test]
    #[cfg(feature = "3d")]
    fn nan_angular_inertia_error() {
        assert_eq!(
            AngularInertia::try_new(Vector::new(Scalar::NAN, 2.0, 3.0)),
            Err(AngularInertiaError::NaN),
            "NaN angular inertia should return an error"
        );
    }
}
