use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;

mod world_query;
pub use world_query::MassPropertiesQuery;

/// The mass of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
///
/// ## Representation
///
/// Internally, the mass is actually stored as the inverse mass `1.0 / mass`.
/// This is because most physics calculations operate on the inverse mass, and storing it directly
/// allows for fewer divisions and guards against division by zero.
///
/// When using [`RigidBodyMass`], you shouldn't need to worry about this internal representation.
/// The provided constructors and getters abstract away the implementation details.
///
/// In terms of performance, the main thing to keep in mind is that [`inverse`](Self::inverse) is a no-op
/// and [`value`](Self::value) contains a division. When dividing by the mass, it's better to use
/// `foo * mass.inverse()` than `foo / mass.value()`.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct RigidBodyMass {
    /// The inverse mass.
    ///
    /// This is stored as an inverse because most physics calculations
    /// operate on the inverse mass, and storing it directly allows for
    /// fewer divisions and guards against division by zero.
    inverse: Scalar,
}

impl RigidBodyMass {
    /// Zero mass.
    pub const ZERO: Self = Self {
        inverse: Scalar::INFINITY,
    };

    /// Infinite mass.
    pub const INFINITY: Self = Self { inverse: 0.0 };

    /// Creates a new [`RigidBodyMass`] from the given mass.
    #[inline]
    pub fn new(mass: Scalar) -> Self {
        Self::from_inverse(mass.recip_or_zero())
    }

    /// Creates a new [`RigidBodyMass`] from the given inverse mass.
    #[inline]
    pub fn from_inverse(inverse_mass: Scalar) -> Self {
        Self {
            inverse: inverse_mass,
        }
    }

    /// Returns the mass.
    ///
    /// Note that this involves a division because [`RigidBodyMass`] internally stores the inverse mass.
    /// If dividing by the mass, consider using `foo * mass.inverse()` instead of `foo / mass.value()`.
    #[inline]
    pub fn value(self) -> Scalar {
        self.inverse.recip_or_zero()
    }

    /// Returns the inverse mass.
    ///
    /// This is a no-op because [`RigidBodyMass`] internally stores the inverse mass.
    #[inline]
    pub fn inverse(self) -> Scalar {
        self.inverse
    }

    /// Sets the mass.
    #[inline]
    pub fn set(&mut self, mass: impl Into<RigidBodyMass>) {
        *self = mass.into();
    }

    /// Returns `true` if the mass is neither infinite nor NaN.
    #[inline]
    pub fn is_finite(self) -> bool {
        !self.is_infinite() && !self.is_nan()
    }

    /// Returns `true` if the mass is positive infinity or negative infinity.
    #[inline]
    pub fn is_infinite(self) -> bool {
        self == Self::INFINITY
    }

    /// Returns `true` if the mass is NaN.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.inverse.is_nan()
    }
}

impl From<Scalar> for RigidBodyMass {
    fn from(mass: Scalar) -> Self {
        Self::new(mass)
    }
}

/// The moment of inertia of a dynamic [rigid body]. This represents the torque needed for a desired angular acceleration.
///
/// [rigid body]: RigidBody
///
/// ## Representation
///
/// Internally, the angular inertia is actually stored as the inverse angular inertia `1.0 / angular_inertia`.
/// This is because most physics calculations operate on the inverse angular inertia, and storing it directly
/// allows for fewer divisions and guards against division by zero.
///
/// When using [`RigidBodyAngularInertia`], you shouldn't need to worry about this internal representation.
/// The provided constructors and getters abstract away the implementation details.
///
/// In terms of performance, the main thing to keep in mind is that [`inverse`](Self::inverse) is a no-op
/// and [`value`](Self::value) contains a division. When dividing by the angular inertia, it's better to use
/// `foo * angular_inertia.inverse()` than `foo / angular_inertia.value()`.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct RigidBodyAngularInertia {
    /// The inverse angular inertia.
    ///
    /// This is stored as an inverse to minimize the number of divisions
    /// and to guard against division by zero. Most physics calculations
    /// use the inverse angular inertia.
    inverse: Scalar,
}

#[cfg(feature = "2d")]
impl RigidBodyAngularInertia {
    /// Zero mass.
    pub const ZERO: Self = Self {
        inverse: Scalar::INFINITY,
    };

    /// Infinite mass.
    pub const INFINITY: Self = Self { inverse: 0.0 };

    /// Creates a new [`RigidBodyAngularInertia`] from the given angular inertia.
    #[inline]
    pub fn new(angular_inertia: Scalar) -> Self {
        Self::from_inverse(angular_inertia.recip_or_zero())
    }

    /// Creates a new [`RigidBodyAngularInertia`] from the given inverse angular inertia.
    #[inline]
    pub fn from_inverse(inverse_angular_inertia: Scalar) -> Self {
        Self {
            inverse: inverse_angular_inertia,
        }
    }

    /// Returns the angular inertia.
    ///
    /// Note that this involves a division because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    /// If dividing by the angular inertia, consider using `foo * angular_inertia.inverse()` instead of `foo / angular_inertia.value()`.
    #[inline]
    pub fn value(self) -> Scalar {
        self.inverse.recip_or_zero()
    }

    /// Returns the inverse angular inertia.
    ///
    /// This is a no-op because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    pub fn inverse(self) -> Scalar {
        self.inverse
    }

    /// Returns a mutable reference to the inverse of the angular inertia.
    ///
    /// Note that this is a no-op because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    pub fn inverse_mut(&mut self) -> &mut Scalar {
        &mut self.inverse
    }

    /// Sets the angular inertia.
    #[inline]
    pub fn set(&mut self, angular_inertia: impl Into<RigidBodyAngularInertia>) {
        *self = angular_inertia.into();
    }

    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Scalar {
        shifted_angular_inertia(self.value(), mass, offset)
    }

    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted_inverse(&self, mass: Scalar, offset: Vector) -> Scalar {
        shifted_angular_inertia(self.value(), mass, offset).recip_or_zero()
    }

    /// Returns `true` if the angular inertia is neither infinite nor NaN.
    #[inline]
    pub fn is_finite(self) -> bool {
        !self.is_infinite() && !self.is_nan()
    }

    /// Returns `true` if the angular inertia is positive infinity or negative infinity.
    #[inline]
    pub fn is_infinite(self) -> bool {
        self == Self::INFINITY
    }

    /// Returns `true` if the angular inertia is NaN.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.inverse.is_nan()
    }
}

#[cfg(feature = "2d")]
impl From<Scalar> for RigidBodyAngularInertia {
    fn from(angular_inertia: Scalar) -> Self {
        Self::new(angular_inertia)
    }
}

// TODO: Add errors for asymmetric and non-positive definite matrices.
/// An error returned for an invalid angular inertia tensor.
#[cfg(feature = "3d")]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AngularInertiaTensorError {
    /// Some element of the angular inertia tensor is negative.
    Negative,
    /// The angular inertia is NaN.
    Nan,
}

/// The local moment of inertia of a dynamic [rigid body] as a 3x3 tensor matrix.
/// This represents the torque needed for a desired angular acceleration about the XYZ axes.
///
/// This is computed in local space, so the object's orientation is not taken into account.
/// The world-space version is stored in [`GlobalAngularInertia`], which is automatically recomputed
/// whenever the local angular inertia or rotation is changed.
///
/// To manually compute the world-space version that takes the body's rotation into account,
/// use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
///
/// The angular inertia tensor should be symmetric and positive definite.
///
/// [rigid body]: RigidBody
///
/// ## Representation
///
/// Internally, the angular inertia is actually stored as the inverse angular inertia tensor `angular_inertia_matrix.inverse()`.
/// This is because most physics calculations operate on the inverse angular inertia, and storing it directly
/// allows for fewer inversions and guards against division by zero.
///
/// When using [`RigidBodyAngularInertia`], you shouldn't need to worry about this internal representation.
/// The provided constructors and getters abstract away the implementation details.
///
/// In terms of performance, the main thing to keep in mind is that [`inverse`](Self::inverse) is a no-op
/// and [`value`](Self::value) contains an inversion. When multiplying by the inverse angular inertia, it's better to use
/// `angular_inertia.inverse() * foo` than `angular_inertia.value().inverse() * foo`.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct RigidBodyAngularInertia {
    // TODO: The matrix should be symmetric and positive definite.
    //       We could add a custom `SymmetricMat3` type to enforce symmetricity and reduce memory usage.
    inverse: Matrix,
}

impl Default for RigidBodyAngularInertia {
    fn default() -> Self {
        Self::INFINITY
    }
}

// TODO: Add helpers for getting the principal angular inertia and local inertial frame. This requires an eigensolver.
//       `bevy_heavy` has this functionality.
#[cfg(feature = "3d")]
impl RigidBodyAngularInertia {
    /// Zero angular inertia.
    pub const ZERO: Self = Self {
        inverse: Matrix::from_diagonal(Vector::INFINITY),
    };

    /// Infinite angular inertia.
    pub const INFINITY: Self = Self {
        inverse: Matrix::ZERO,
    };

    /// Creates a new [`RigidBodyAngularInertia`] from the given principal angular inertia.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes.
    ///
    /// Note that this involves an invertion because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    ///
    /// To specify the orientation of the local inertial frame, consider using [`RigidBodyAngularInertia::new_with_local_frame`].
    ///
    /// # Panics
    ///
    /// Panics if any component of the principal angular inertia is negative when `debug_assertions` are enabled.
    #[inline]
    #[doc(alias = "from_principal_angular_inertia")]
    pub fn new(principal_angular_inertia: Vector) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vector::ZERO).all(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self::from_inverse_tensor(Matrix::from_diagonal(
            principal_angular_inertia.recip_or_zero(),
        ))
    }

    /// Tries to create a new [`RigidBodyAngularInertia`] from the given principal angular inertia.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes. To specify the orientation of the local inertial frame,
    /// consider using [`RigidBodyAngularInertia::try_new_with_local_frame`].
    ///
    /// Note that this involves an invertion because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaTensorError)`](AngularInertiaTensorError) if any component of the principal angular inertia is negative.
    #[inline]
    pub fn try_new(principal_angular_inertia: Vector) -> Result<Self, AngularInertiaTensorError> {
        if !principal_angular_inertia.cmpge(Vec3::ZERO).all() {
            Err(AngularInertiaTensorError::Negative)
        } else if principal_angular_inertia.is_nan() {
            Err(AngularInertiaTensorError::Nan)
        } else {
            Ok(Self::from_inverse_tensor(Matrix::from_diagonal(
                principal_angular_inertia.recip_or_zero(),
            )))
        }
    }

    /// Creates a new [`RigidBodyAngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the given `orientation`.
    ///
    /// Note that this involves an invertion because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    ///
    /// # Panics
    ///
    /// Panics if any component of the principal angular inertia is negative when `debug_assertions` are enabled.
    #[inline]
    #[doc(alias = "from_principal_angular_inertia_with_local_frame")]
    pub fn new_with_local_frame(
        principal_angular_inertia: Vector,
        orientation: Quaternion,
    ) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vec3::ZERO).all(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self::from_inverse_tensor(
            Mat3::from_quat(orientation)
                * Mat3::from_diagonal(principal_angular_inertia.recip_or_zero())
                * Mat3::from_quat(orientation.inverse()),
        )
    }

    /// Tries to create a new [`RigidBodyAngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the given `orientation`.
    ///
    /// Note that this involves an invertion because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaTensorError)`](AngularInertiaTensorError) if any component of the principal angular inertia is negative.
    #[inline]
    pub fn try_new_with_local_frame(
        principal_angular_inertia: Vector,
        orientation: Quaternion,
    ) -> Result<Self, AngularInertiaTensorError> {
        if !principal_angular_inertia.cmpge(Vec3::ZERO).all() {
            Err(AngularInertiaTensorError::Negative)
        } else if principal_angular_inertia.is_nan() {
            Err(AngularInertiaTensorError::Nan)
        } else {
            Ok(Self::from_inverse_tensor(
                Mat3::from_quat(orientation)
                    * Mat3::from_diagonal(principal_angular_inertia.recip_or_zero())
                    * Mat3::from_quat(orientation.inverse()),
            ))
        }
    }

    /// Creates a new [`RigidBodyAngularInertia`] from the given angular inertia tensor.
    ///
    /// The tensor should be symmetric and positive definite.
    ///
    /// Note that this involves an invertion because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    #[doc(alias = "from_mat3")]
    pub fn from_tensor(tensor: Matrix) -> Self {
        Self::from_inverse_tensor(tensor.inverse_or_zero())
    }

    /// Creates a new [`RigidBodyAngularInertia`] from the given angular inertia tensor.
    ///
    /// The tensor should be symmetric and positive definite.
    #[inline]
    #[doc(alias = "from_inverse_mat3")]
    pub fn from_inverse_tensor(inverse_tensor: Matrix) -> Self {
        Self {
            inverse: inverse_tensor,
        }
    }

    /// Returns the angular inertia tensor.
    ///
    /// Note that this involves an invertion because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    /// If multiplying by the inverse angular inertia, consider using `angular_inertia.inverse() * foo`
    /// instead of `angular_inertia.value().inverse() * foo`.
    ///
    /// Equivalent to [`AngularInertia::tensor`].
    #[inline]
    pub(crate) fn value(self) -> Matrix {
        self.tensor()
    }

    /// Returns the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    ///
    /// Equivalent to [`AngularInertia::inverse_tensor`].
    #[inline]
    pub(crate) fn inverse(self) -> Matrix {
        self.inverse_tensor()
    }

    /// Returns a mutable reference to the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    pub(crate) fn inverse_mut(&mut self) -> &mut Matrix {
        self.inverse_tensor_mut()
    }

    /// Returns the angular inertia tensor.
    ///
    /// Note that this involves an invertion because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    /// If multiplying by the inverse angular inertia, consider using `angular_inertia.inverse() * foo`
    /// instead of `angular_inertia.value().inverse() * foo`.
    #[inline]
    #[doc(alias = "as_mat3")]
    pub fn tensor(self) -> Matrix {
        self.inverse.inverse_or_zero()
    }

    /// Returns the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    #[doc(alias = "as_inverse_mat3")]
    pub fn inverse_tensor(self) -> Matrix {
        self.inverse
    }

    /// Returns a mutable reference to the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`RigidBodyAngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    #[doc(alias = "as_inverse_mat3_mut")]
    pub fn inverse_tensor_mut(&mut self) -> &mut Matrix {
        &mut self.inverse
    }

    /// Sets the angular inertia tensor.
    #[inline]
    pub fn set(&mut self, angular_inertia: impl Into<RigidBodyAngularInertia>) {
        *self = angular_inertia.into();
    }

    /// Computes the angular inertia tensor with the given rotation.
    ///
    /// This can be used to transform local angular inertia to world space.
    #[inline]
    pub fn rotated(self, rotation: Quaternion) -> Self {
        let rot_mat3 = Mat3::from_quat(rotation);
        Self::from_inverse_tensor((rot_mat3 * self.inverse) * rot_mat3.transpose())
    }

    /// Computes the angular inertia tensor shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted_tensor(&self, mass: Scalar, offset: Vector) -> Matrix3 {
        shifted_angular_inertia(self.tensor(), mass, offset)
    }

    /// Computes the inverse angular inertia tensor shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted_inverse_tensor(&self, mass: Scalar, offset: Vector) -> Matrix3 {
        shifted_angular_inertia(self.tensor(), mass, offset).inverse_or_zero()
    }

    /// Returns `true` if the angular inertia is neither infinite nor NaN.
    #[inline]
    pub fn is_finite(self) -> bool {
        !self.is_infinite() && !self.is_nan()
    }

    /// Returns `true` if the angular inertia is positive infinity or negative infinity.
    #[inline]
    pub fn is_infinite(self) -> bool {
        self == Self::INFINITY
    }

    /// Returns `true` if the angular inertia is NaN.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.inverse.is_nan()
    }
}

#[cfg(feature = "3d")]
impl From<Matrix> for RigidBodyAngularInertia {
    fn from(tensor: Matrix) -> Self {
        Self::from_tensor(tensor)
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<Scalar> for RigidBodyAngularInertia {
    type Output = Scalar;

    #[inline]
    fn mul(self, rhs: Scalar) -> Scalar {
        self.value() * rhs
    }
}

impl core::ops::Mul<Vector> for RigidBodyAngularInertia {
    type Output = Vector;

    #[inline]
    fn mul(self, rhs: Vector) -> Vector {
        self.value() * rhs
    }
}

// In 2D, the global angular inertia is the same as the local angular inertia.
#[cfg(feature = "2d")]
pub(crate) type GlobalAngularInertia = RigidBodyAngularInertia;

/// The world-space moment of inertia of a dynamic [rigid body] as a 3x3 tensor matrix.
/// This represents the torque needed for a desired angular acceleration about the XYZ axes.
///
/// This component is automatically updated whenever the local [`RigidBodyAngularInertia`] or rotation is changed.
/// To manually update it, use the associated [`update`](Self::update) method.
///
/// [rigid body]: RigidBody
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct GlobalAngularInertia(RigidBodyAngularInertia);

#[cfg(feature = "3d")]
impl GlobalAngularInertia {
    /// Creates a new [`GlobalAngularInertia`] from the given local angular inertia and rotation.
    pub fn new(
        local_angular_inertia: impl Into<RigidBodyAngularInertia>,
        rotation: impl Into<Quaternion>,
    ) -> Self {
        let local_angular_inertia: RigidBodyAngularInertia = local_angular_inertia.into();
        Self(local_angular_inertia.rotated(rotation.into()))
    }

    /// Updates the global angular inertia with the given local angular inertia and rotation.
    pub fn update(
        &mut self,
        local_angular_inertia: impl Into<RigidBodyAngularInertia>,
        rotation: impl Into<Quaternion>,
    ) {
        *self = Self::new(local_angular_inertia, rotation);
    }
}

#[cfg(feature = "3d")]
impl From<GlobalAngularInertia> for RigidBodyAngularInertia {
    fn from(inertia: GlobalAngularInertia) -> Self {
        inertia.0
    }
}

#[cfg(feature = "3d")]
impl From<Matrix> for GlobalAngularInertia {
    fn from(tensor: Matrix) -> Self {
        Self(RigidBodyAngularInertia::from_tensor(tensor))
    }
}

/// The local center of mass of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct RigidBodyCenterOfMass(pub Vector);

impl RigidBodyCenterOfMass {
    /// A center of mass set at the local origin.
    pub const ZERO: Self = Self(Vector::ZERO);
}

/// A bundle containing mass properties.
///
/// ## Example
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
    pub mass: RigidBodyMass,
    pub angular_inertia: RigidBodyAngularInertia,
    pub center_of_mass: RigidBodyCenterOfMass,
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
            mass: RigidBodyMass::new(mass),
            #[cfg(feature = "2d")]
            angular_inertia: RigidBodyAngularInertia::new(angular_inertia),
            #[cfg(feature = "3d")]
            angular_inertia: RigidBodyAngularInertia::from_tensor(angular_inertia),
            center_of_mass: RigidBodyCenterOfMass(center_of_mass),
        }
    }
}

/// The density of a [`Collider`], 1.0 by default. This is used for computing
/// the [`ColliderMassProperties`] for each collider.
///
/// ## Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Spawn a body with a collider that has a density of 2.5
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.5),")]
///         ColliderDensity(2.5),
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderDensity(pub Scalar);

impl ColliderDensity {
    /// The density of the [`Collider`] is zero. It has no mass.
    pub const ZERO: Self = Self(0.0);
}

impl Default for ColliderDensity {
    fn default() -> Self {
        Self(1.0)
    }
}

impl From<Scalar> for ColliderDensity {
    fn from(density: Scalar) -> Self {
        Self(density)
    }
}

/// An automatically added component that contains the read-only mass properties of a [`Collider`].
/// The density used for computing the mass properties can be configured using the [`ColliderDensity`]
/// component.
///
/// These mass properties will be added to the [rigid body's](RigidBody) actual [`RigidBodyMass`],
/// [`RigidBodyAngularInertia`] and [`RigidBodyCenterOfMass`] components.
///
/// ## Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         .add_systems(Startup, setup)
///         .add_systems(Update, print_collider_masses)
///         .run();
/// }
///
/// fn setup(mut commands: Commands) {
#[cfg_attr(feature = "2d", doc = "    commands.spawn(Collider::circle(0.5));")]
#[cfg_attr(feature = "3d", doc = "    commands.spawn(Collider::sphere(0.5));")]
/// }
///
/// fn print_collider_masses(query: Query<&ColliderMassProperties>) {
///     for mass_props in &query {
///         println!("{}", mass_props.mass.value());
///     }
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderMassProperties {
    /// Mass given by the collider.
    pub mass: Scalar,

    /// Angular inertia given by the collider.
    #[cfg(feature = "2d")]
    pub angular_inertia: Scalar,

    /// Angular inertia tensor given by the collider.
    #[cfg(feature = "3d")]
    pub angular_inertia: Matrix,

    /// Local center of mass given by the collider.
    pub center_of_mass: Vector,
}

impl ColliderMassProperties {
    /// The collider has no mass.
    pub const ZERO: Self = Self {
        mass: 0.0,
        #[cfg(feature = "2d")]
        angular_inertia: 0.0,
        #[cfg(feature = "3d")]
        angular_inertia: Matrix::ZERO,
        center_of_mass: Vector::ZERO,
    };

    /// The collider has infinite mass.
    pub const INFINITY: Self = Self {
        mass: Scalar::INFINITY,
        #[cfg(feature = "2d")]
        angular_inertia: Scalar::INFINITY,
        #[cfg(feature = "3d")]
        angular_inertia: Matrix::from_diagonal(Vector::INFINITY),
        center_of_mass: Vector::ZERO,
    };

    /// Computes mass properties from a given collider and density.
    ///
    /// Because [`ColliderMassProperties`] is read-only, adding this as a component manually
    /// has no effect. The mass properties will be recomputed using the [`ColliderDensity`].
    pub fn new<C: AnyCollider>(collider: &C, density: Scalar) -> Self {
        collider.mass_properties(density)
    }

    /// Transforms the center of mass by the given [`ColliderTransform`].
    #[inline]
    pub fn transformed_by(mut self, transform: &ColliderTransform) -> Self {
        self.center_of_mass = transform.transform_point(self.center_of_mass);
        self
    }

    /// Computes the angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "2d")]
    #[inline]
    pub fn shifted_angular_inertia(&self, offset: Vector) -> Scalar {
        shifted_angular_inertia(self.angular_inertia, self.mass, offset)
    }

    /// Computes the angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn shifted_angular_inertia(&self, offset: Vector) -> Matrix {
        shifted_angular_inertia(self.angular_inertia, self.mass, offset)
    }

    /// Computes the inverse angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "2d")]
    #[inline]
    pub fn shifted_inverse_angular_inertia(&self, offset: Vector) -> Scalar {
        shifted_angular_inertia(self.angular_inertia, self.mass, offset).recip_or_zero()
    }

    /// Computes the inverse angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn shifted_inverse_angular_inertia(&self, offset: Vector) -> Matrix {
        shifted_angular_inertia(self.angular_inertia, self.mass, offset).inverse_or_zero()
    }
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        Self::ZERO
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
