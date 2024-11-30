use crate::prelude::*;
use bevy::prelude::*;
#[cfg(feature = "3d")]
use bevy_heavy::AngularInertiaTensor;
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
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Mass(pub f32);

impl Mass {
    /// A mass of `0.0`.
    pub const ZERO: Self = Self(0.0);
}

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
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AngularInertia(pub f32);

#[cfg(feature = "2d")]
impl AngularInertia {
    /// An angular inertia of `0.0`.
    pub const ZERO: Self = Self(0.0);

    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted(&self, mass: f32, offset: Vec2) -> f32 {
        if mass > 0.0 && mass.is_finite() && offset != Vec2::ZERO {
            self.0 + offset.length_squared() * mass
        } else {
            self.0
        }
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
    pub principal: Vec3,
    /// The orientation of the local inertial frame.
    pub local_frame: Quat,
}

#[cfg(feature = "3d")]
impl AngularInertia {
    /// An angular inertia of `0.0` for all axes, with an identity local frame.
    pub const ZERO: Self = Self {
        principal: Vec3::ZERO,
        local_frame: Quat::IDENTITY,
    };

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
    pub fn new(principal_angular_inertia: Vec3) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vec3::ZERO).all()
                && !principal_angular_inertia.is_nan(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self {
            principal: principal_angular_inertia,
            local_frame: Quat::IDENTITY,
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
    pub fn try_new(principal_angular_inertia: Vec3) -> Result<Self, AngularInertiaError> {
        if principal_angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if !principal_angular_inertia.cmpge(Vec3::ZERO).all() {
            Err(AngularInertiaError::Negative)
        } else {
            Ok(Self {
                principal: principal_angular_inertia,
                local_frame: Quat::IDENTITY,
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
    pub fn new_with_local_frame(principal_angular_inertia: Vec3, local_frame: Quat) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vec3::ZERO).all()
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
        principal_angular_inertia: Vec3,
        local_frame: Quat,
    ) -> Result<Self, AngularInertiaError> {
        if principal_angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if !principal_angular_inertia.cmpge(Vec3::ZERO).all() {
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
    pub fn from_tensor(tensor: impl Into<AngularInertiaTensor>) -> Self {
        let tensor = tensor.into();
        let (principal, local_frame) = tensor.principal_angular_inertia_with_local_frame();

        Self {
            principal,
            local_frame,
        }
    }

    /// Returns the [`AngularInertiaTensor`] represented by this principal [`AngularInertia`]
    /// and local inertial frame.
    #[inline]
    pub fn tensor(self) -> AngularInertiaTensor {
        AngularInertiaTensor::new_with_local_frame(self.principal, self.local_frame)
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
impl From<Mat3> for AngularInertia {
    fn from(tensor: Mat3) -> Self {
        Self::from_tensor(tensor)
    }
}

#[cfg(feature = "3d")]
impl From<AngularInertiaTensor> for AngularInertia {
    fn from(tensor: AngularInertiaTensor) -> Self {
        Self::from_tensor(tensor)
    }
}

#[cfg(feature = "3d")]
impl From<AngularInertia> for AngularInertiaTensor {
    fn from(inertia: AngularInertia) -> Self {
        inertia.tensor()
    }
}

/// The local center of mass of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[cfg(feature = "2d")]
pub struct CenterOfMass(pub Vec2);

/// The local center of mass of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[cfg(feature = "3d")]
pub struct CenterOfMass(pub Vec3);

impl CenterOfMass {
    /// A center of mass set at the local origin.
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(Vec2::ZERO);
    /// A center of mass set at the local origin.
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Vec3::ZERO);

    /// Creates a new [`CenterOfMass`] at the given local position.
    #[inline]
    #[cfg(feature = "2d")]
    pub const fn new(x: f32, y: f32) -> Self {
        Self(Vec2::new(x, y))
    }

    /// Creates a new [`CenterOfMass`] at the given local position.
    #[inline]
    #[cfg(feature = "3d")]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self(Vec3::new(x, y, z))
    }
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

/// A marker component that forces the recomputation of [`ComputedMass`], [`ComputedAngularInertia`]
/// and [`ComputedCenterOfMass`] for a [rigid body].
///
/// This is added automatically when the mass properties of a [rigid body] change,
/// and is removed after the recomputation.
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[component(storage = "SparseSet")]
pub struct RecomputeMassProperties;

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
        use mass_properties::MassPropertiesExt;
        collider.mass_properties(density).to_bundle()
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
        use bevy_heavy::AngularInertiaTensor;

        let angular_inertia = AngularInertia::new(Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(angular_inertia.principal, Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(angular_inertia.local_frame, Quat::IDENTITY);
        assert_relative_eq!(
            angular_inertia.tensor(),
            AngularInertiaTensor::new(Vec3::new(10.0, 20.0, 30.0))
        );
    }

    #[test]
    #[should_panic]
    #[cfg(feature = "3d")]
    fn negative_angular_inertia_panics() {
        AngularInertia::new(Vec3::new(-1.0, 2.0, 3.0));
    }

    #[test]
    #[cfg(feature = "3d")]
    fn negative_angular_inertia_error() {
        assert_eq!(
            AngularInertia::try_new(Vec3::new(-1.0, 2.0, 3.0)),
            Err(AngularInertiaError::Negative),
            "negative angular inertia should return an error"
        );
    }

    #[test]
    #[cfg(feature = "3d")]
    fn nan_angular_inertia_error() {
        assert_eq!(
            AngularInertia::try_new(Vec3::new(f32::NAN, 2.0, 3.0)),
            Err(AngularInertiaError::NaN),
            "NaN angular inertia should return an error"
        );
    }
}
