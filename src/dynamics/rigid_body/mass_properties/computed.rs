use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;

/// An error returned for an invalid mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MassError {
    /// The mass is negative.
    Negative,
    /// The mass is NaN.
    NaN,
}

/// The mass of a dynamic [rigid body] or collider.
///
/// # Initialization
///
/// By default, the [`Mass`] of a body is computed automatically from the attached colliders
/// based on their shape and [`ColliderDensity`]. Mass can be specified manually at spawn,
/// but masses of child entities still contribute to the total mass.
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn setup(mut commands: Commands) {
///     // The total mass of this body will be 5.0 plus the mass of the child collider.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(0.5, 1.5),
///         // Optional: Specify initial mass for this entity.
///         //           This overrides the mass of the capsule collider.
///         Mass::new(5.0),
///     ))
///     .with_child((
///         // The mass computed for this collider will be added to the total mass of the body.
#[cfg_attr(feature = "2d", doc = "        Collider::circle(1.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(1.0),")]
///         ColliderDensity(2.0),
///     ));
/// ```
///
/// Automatic mass computation can be disabled by adding the [`NoAutoMass`] marker component:
///
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn setup(mut commands: Commands) {
///     // The total mass of this body will be 5.0 plus the mass of the child collider.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(0.5, 1.5),
///         // Optional: Specify initial mass for this entity.
///         //           This overrides the mass of the capsule collider.
///         Mass::new(5.0),
///     ))
///     .with_child((
///         // The mass computed for this collider will be added to the total mass of the body.
#[cfg_attr(feature = "2d", doc = "        Collider::circle(1.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(1.0),")]
///         ColliderDensity(2.0),
///     ));
/// ```
///
/// ```
///     // The mass is set to 5.0.
///     commands.spawn((RigidBody::Dynamic, Mass::new(5.0)));
///
///     // The mass is set to 5.0 + 10.0 = 15.0.
///     commands.spawn((RigidBody::Dynamic, Mass::new(5.0)));
/// }
/// ```
///
/// Automatic mass computation can be disabled by adding the [`NoAutoMass`] marker component
/// to the rigid body entity.
///
/// Adding or removing colliders
/// or changing their transform or density also affects the rigid body.
///
/// For rigid bodies, masses of child colliders still contribute to the final value. On a [`Collider`], [`Mass`] overrides the computed mass value in [`ColliderMassProperties`].
///
/// Note that zero mass is treated as a special case, and is used to represent infinite mass.
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
///     // The mass is computed automatically from the attached colliders.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(0.5, 1.5),
///     ))
///     .with_child((
#[cfg_attr(feature = "2d", doc = "        Collider::circle(1.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(1.0),")]
///         ColliderDensity(2.0),
///     ));
///
///     // The mass is set to 5.0.
///     commands.spawn((RigidBody::Dynamic, Mass::new(5.0)));
///
///     // The mass is set to 5.0 + 10.0 = 15.0.
///     commands.spawn((RigidBody::Dynamic, Mass::new(5.0)));
/// }
/// ```
///
/// # Representation
///
/// Internally, the mass is actually stored as the inverse mass `1.0 / mass`.
/// This is because most physics calculations operate on the inverse mass, and storing it directly
/// allows for fewer divisions and guards against division by zero.
///
/// When using [`Mass`], you shouldn't need to worry about this internal representation.
/// The provided constructors and getters abstract away the implementation details.
///
/// In terms of performance, the main thing to keep in mind is that [`inverse`](Self::inverse) is a no-op
/// and [`value`](Self::value) contains a division. When dividing by the mass, it's better to use
/// `foo * mass.inverse()` than `foo / mass.value()`.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct ComputedMass {
    /// The inverse mass.
    ///
    /// This is stored as an inverse because most physics calculations
    /// operate on the inverse mass, and storing it directly allows for
    /// fewer divisions and guards against division by zero.
    inverse: Scalar,
}

impl ComputedMass {
    /// Infinite mass.
    pub const INFINITY: Self = Self { inverse: 0.0 };

    /// Creates a new [`Mass`] from the given mass.
    ///
    /// # Panics
    ///
    /// Panics if the mass is negative when `debug_assertions` are enabled.
    #[inline]
    pub fn new(mass: Scalar) -> Self {
        Self::from_inverse(mass.recip_or_zero())
    }

    /// Tries to create a new [`Mass`] from the given mass.
    ///
    /// # Errors
    ///
    /// Returns [`Err(MassError)`](MassError) if the mass is negative or NaN.
    #[inline]
    pub fn try_new(mass: Scalar) -> Result<Self, MassError> {
        if mass.is_nan() {
            Err(MassError::NaN)
        } else if mass < 0.0 {
            Err(MassError::Negative)
        } else {
            Ok(Self::from_inverse(mass.recip_or_zero()))
        }
    }

    /// Creates a new [`Mass`] from the given inverse mass.
    ///
    /// # Panics
    ///
    /// Panics if the inverse mass is negative when `debug_assertions` are enabled.
    #[inline]
    pub fn from_inverse(inverse_mass: Scalar) -> Self {
        debug_assert!(
            inverse_mass >= 0.0 && !inverse_mass.is_nan(),
            "mass must be positive or zero"
        );

        Self {
            inverse: inverse_mass,
        }
    }

    /// Tries to create a new [`Mass`] from the given inverse mass.
    ///
    /// # Errors
    ///
    /// Returns [`Err(MassError)`](MassError) if the inverse mass is negative or NaN.
    pub fn try_from_inverse(inverse_mass: Scalar) -> Result<Self, MassError> {
        if inverse_mass.is_nan() {
            Err(MassError::NaN)
        } else if inverse_mass < 0.0 {
            Err(MassError::Negative)
        } else {
            Ok(Self {
                inverse: inverse_mass,
            })
        }
    }

    /// Returns the mass. If it is infinite, returns zero.
    ///
    /// Note that this involves a division because [`Mass`] internally stores the inverse mass.
    /// If dividing by the mass, consider using `foo * mass.inverse()` instead of `foo / mass.value()`.
    #[inline]
    pub fn value(self) -> Scalar {
        self.inverse.recip_or_zero()
    }

    /// Returns the inverse mass.
    ///
    /// This is a no-op because [`Mass`] internally stores the inverse mass.
    #[inline]
    pub fn inverse(self) -> Scalar {
        self.inverse
    }

    /// Sets the mass.
    #[inline]
    pub fn set(&mut self, mass: impl Into<ComputedMass>) {
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

impl From<Scalar> for ComputedMass {
    fn from(mass: Scalar) -> Self {
        Self::new(mass)
    }
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

/// The moment of inertia of a dynamic [rigid body]. This represents the torque needed for a desired angular acceleration.
///
/// Note that zero angular inertia is treated as a special case, and is used to represent infinite angular inertia.
///
/// [rigid body]: RigidBody
///
/// ## Representation
///
/// Internally, the angular inertia is actually stored as the inverse angular inertia `1.0 / angular_inertia`.
/// This is because most physics calculations operate on the inverse angular inertia, and storing it directly
/// allows for fewer divisions and guards against division by zero.
///
/// When using [`AngularInertia`], you shouldn't need to worry about this internal representation.
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
pub struct ComputedAngularInertia {
    /// The inverse angular inertia.
    ///
    /// This is stored as an inverse to minimize the number of divisions
    /// and to guard against division by zero. Most physics calculations
    /// use the inverse angular inertia.
    inverse: Scalar,
}

#[cfg(feature = "2d")]
impl ComputedAngularInertia {
    /// Infinite angular inertia.
    pub const INFINITY: Self = Self { inverse: 0.0 };

    /// Creates a new [`AngularInertia`] from the given angular inertia.
    ///
    /// # Panics
    ///
    /// Panics if the angular inertia is negative when `debug_assertions` are enabled.
    #[inline]
    pub fn new(angular_inertia: Scalar) -> Self {
        Self::from_inverse(angular_inertia.recip_or_zero())
    }

    /// Tries to create a new [`AngularInertia`] from the given angular inertia.
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaError)`](AngularInertiaError) if the angular inertia is negative or NaN.
    #[inline]
    pub fn try_new(angular_inertia: Scalar) -> Result<Self, AngularInertiaError> {
        if angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if angular_inertia < 0.0 {
            Err(AngularInertiaError::Negative)
        } else {
            Ok(Self::from_inverse(angular_inertia.recip_or_zero()))
        }
    }

    /// Creates a new [`AngularInertia`] from the given inverse angular inertia.
    ///
    /// # Panics
    ///
    /// Panics if the inverse angular inertia is negative when `debug_assertions` are enabled.
    #[inline]
    pub fn from_inverse(inverse_angular_inertia: Scalar) -> Self {
        debug_assert!(
            inverse_angular_inertia >= 0.0 && !inverse_angular_inertia.is_nan(),
            "angular inertia must be positive or zero"
        );

        Self {
            inverse: inverse_angular_inertia,
        }
    }

    /// Tries to create a new [`AngularInertia`] from the given inverse angular inertia.
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaError)`](AngularInertiaError) if the inverse angular inertia is negative or NaN.
    #[inline]
    pub fn try_from_inverse(inverse_angular_inertia: Scalar) -> Result<Self, AngularInertiaError> {
        if inverse_angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if inverse_angular_inertia < 0.0 {
            Err(AngularInertiaError::Negative)
        } else {
            Ok(Self {
                inverse: inverse_angular_inertia,
            })
        }
    }

    /// Returns the angular inertia. If it is infinite, returns zero.
    ///
    /// Note that this involves a division because [`AngularInertia`] internally stores the inverse angular inertia.
    /// If dividing by the angular inertia, consider using `foo * angular_inertia.inverse()` instead of `foo / angular_inertia.value()`.
    #[inline]
    pub fn value(self) -> Scalar {
        self.inverse.recip_or_zero()
    }

    /// Returns the inverse angular inertia.
    ///
    /// This is a no-op because [`AngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    pub fn inverse(self) -> Scalar {
        self.inverse
    }

    /// Returns a mutable reference to the inverse of the angular inertia.
    ///
    /// Note that this is a no-op because [`AngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    pub fn inverse_mut(&mut self) -> &mut Scalar {
        &mut self.inverse
    }

    /// Sets the angular inertia.
    #[inline]
    pub fn set(&mut self, angular_inertia: impl Into<ComputedAngularInertia>) {
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
impl From<Scalar> for ComputedAngularInertia {
    fn from(angular_inertia: Scalar) -> Self {
        Self::new(angular_inertia)
    }
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
/// Note that zero angular inertia is treated as a special case, and is used to represent infinite angular inertia.
///
/// [rigid body]: RigidBody
///
/// ## Representation
///
/// Internally, the angular inertia is actually stored as the inverse angular inertia tensor `angular_inertia_matrix.inverse()`.
/// This is because most physics calculations operate on the inverse angular inertia, and storing it directly
/// allows for fewer inversions and guards against division by zero.
///
/// When using [`AngularInertia`], you shouldn't need to worry about this internal representation.
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
pub struct ComputedAngularInertia {
    // TODO: The matrix should be symmetric and positive definite.
    //       We could add a custom `SymmetricMat3` type to enforce symmetricity and reduce memory usage.
    inverse: Matrix,
}

impl Default for ComputedAngularInertia {
    fn default() -> Self {
        Self::INFINITY
    }
}

// TODO: Add helpers for getting the principal angular inertia and local inertial frame. This requires an eigensolver.
//       `bevy_heavy` has this functionality.
#[cfg(feature = "3d")]
impl ComputedAngularInertia {
    /// Infinite angular inertia.
    pub const INFINITY: Self = Self {
        inverse: Matrix::ZERO,
    };

    /// Creates a new [`AngularInertia`] from the given principal angular inertia.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes.
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
    ///
    /// To specify the orientation of the local inertial frame, consider using [`AngularInertia::new_with_local_frame`].
    ///
    /// # Panics
    ///
    /// Panics if any component of the principal angular inertia is negative when `debug_assertions` are enabled.
    #[inline]
    #[doc(alias = "from_principal_angular_inertia")]
    pub fn new(principal_angular_inertia: Vector) -> Self {
        debug_assert!(
            principal_angular_inertia.cmpge(Vector::ZERO).all()
                && !principal_angular_inertia.is_nan(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self::from_inverse_tensor(Matrix::from_diagonal(
            principal_angular_inertia.recip_or_zero(),
        ))
    }

    /// Tries to create a new [`AngularInertia`] from the given principal angular inertia.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes. To specify the orientation of the local inertial frame,
    /// consider using [`AngularInertia::try_new_with_local_frame`].
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
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
            Ok(Self::from_inverse_tensor(Matrix::from_diagonal(
                principal_angular_inertia.recip_or_zero(),
            )))
        }
    }

    /// Creates a new [`AngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the given `orientation`.
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
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
            principal_angular_inertia.cmpge(Vector::ZERO).all()
                && !principal_angular_inertia.is_nan(),
            "principal angular inertia must be positive or zero for all axes"
        );

        Self::from_inverse_tensor(
            Matrix::from_quat(orientation)
                * Matrix::from_diagonal(principal_angular_inertia.recip_or_zero())
                * Matrix::from_quat(orientation.inverse()),
        )
    }

    /// Tries to create a new [`AngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the given `orientation`.
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
    ///
    /// # Errors
    ///
    /// Returns [`Err(AngularInertiaError)`](AngularInertiaError) if any component of the principal angular inertia is negative or NaN.
    #[inline]
    pub fn try_new_with_local_frame(
        principal_angular_inertia: Vector,
        orientation: Quaternion,
    ) -> Result<Self, AngularInertiaError> {
        if principal_angular_inertia.is_nan() {
            Err(AngularInertiaError::NaN)
        } else if !principal_angular_inertia.cmpge(Vector::ZERO).all() {
            Err(AngularInertiaError::Negative)
        } else {
            Ok(Self::from_inverse_tensor(
                Matrix::from_quat(orientation)
                    * Matrix::from_diagonal(principal_angular_inertia.recip_or_zero())
                    * Matrix::from_quat(orientation.inverse()),
            ))
        }
    }

    /// Creates a new [`AngularInertia`] from the given angular inertia tensor.
    ///
    /// The tensor should be symmetric and positive definite.
    ///
    /// Equivalent to [`AngularInertia::from_inverse_tensor`].
    #[inline]
    pub(crate) fn from_inverse(inverse_tensor: Matrix) -> Self {
        Self::from_inverse_tensor(inverse_tensor)
    }

    /// Creates a new [`AngularInertia`] from the given angular inertia tensor.
    ///
    /// The tensor should be symmetric and positive definite.
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    #[doc(alias = "from_mat3")]
    pub fn from_tensor(tensor: Matrix) -> Self {
        Self::from_inverse_tensor(tensor.inverse_or_zero())
    }

    /// Creates a new [`AngularInertia`] from the given angular inertia tensor.
    ///
    /// The tensor should be symmetric and positive definite.
    #[inline]
    #[doc(alias = "from_inverse_mat3")]
    pub fn from_inverse_tensor(inverse_tensor: Matrix) -> Self {
        Self {
            inverse: inverse_tensor,
        }
    }

    /// Returns the angular inertia tensor. If it is infinite, returns zero.
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
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
    /// Note that this is a no-op because [`AngularInertia`] internally stores the inverse angular inertia.
    ///
    /// Equivalent to [`AngularInertia::inverse_tensor`].
    #[inline]
    pub(crate) fn inverse(self) -> Matrix {
        self.inverse_tensor()
    }

    /// Returns a mutable reference to the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`AngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    pub(crate) fn inverse_mut(&mut self) -> &mut Matrix {
        self.inverse_tensor_mut()
    }

    /// Returns the angular inertia tensor.
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
    /// If multiplying by the inverse angular inertia, consider using `angular_inertia.inverse() * foo`
    /// instead of `angular_inertia.value().inverse() * foo`.
    #[inline]
    #[doc(alias = "as_mat3")]
    pub fn tensor(self) -> Matrix {
        self.inverse.inverse_or_zero()
    }

    /// Returns the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`AngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    #[doc(alias = "as_inverse_mat3")]
    pub fn inverse_tensor(self) -> Matrix {
        self.inverse
    }

    /// Returns a mutable reference to the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`AngularInertia`] internally stores the inverse angular inertia.
    #[inline]
    #[doc(alias = "as_inverse_mat3_mut")]
    pub fn inverse_tensor_mut(&mut self) -> &mut Matrix {
        &mut self.inverse
    }

    /// Sets the angular inertia tensor.
    #[inline]
    pub fn set(&mut self, angular_inertia: impl Into<ComputedAngularInertia>) {
        *self = angular_inertia.into();
    }

    /// Computes the angular inertia tensor with the given rotation.
    ///
    /// This can be used to transform local angular inertia to world space.
    #[inline]
    pub fn rotated(self, rotation: Quaternion) -> Self {
        let rot_mat3 = Matrix::from_quat(rotation);
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
impl From<Matrix> for ComputedAngularInertia {
    fn from(tensor: Matrix) -> Self {
        Self::from_tensor(tensor)
    }
}

#[cfg(feature = "2d")]
impl core::ops::Mul<Scalar> for ComputedAngularInertia {
    type Output = Scalar;

    #[inline]
    fn mul(self, rhs: Scalar) -> Scalar {
        self.value() * rhs
    }
}

impl core::ops::Mul<Vector> for ComputedAngularInertia {
    type Output = Vector;

    #[inline]
    fn mul(self, rhs: Vector) -> Vector {
        self.value() * rhs
    }
}

// In 2D, the global angular inertia is the same as the local angular inertia.
#[cfg(feature = "2d")]
pub(crate) type GlobalAngularInertia = ComputedAngularInertia;

/// The world-space moment of inertia of a dynamic [rigid body] as a 3x3 tensor matrix.
/// This represents the torque needed for a desired angular acceleration about the XYZ axes.
///
/// This component is automatically updated whenever the local [`AngularInertia`] or rotation is changed.
/// To manually update it, use the associated [`update`](Self::update) method.
///
/// [rigid body]: RigidBody
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct GlobalAngularInertia(ComputedAngularInertia);

#[cfg(feature = "3d")]
impl GlobalAngularInertia {
    /// Creates a new [`GlobalAngularInertia`] from the given local angular inertia and rotation.
    pub fn new(
        local_angular_inertia: impl Into<ComputedAngularInertia>,
        rotation: impl Into<Quaternion>,
    ) -> Self {
        let local_angular_inertia: ComputedAngularInertia = local_angular_inertia.into();
        Self(local_angular_inertia.rotated(rotation.into()))
    }

    /// Updates the global angular inertia with the given local angular inertia and rotation.
    pub fn update(
        &mut self,
        local_angular_inertia: impl Into<ComputedAngularInertia>,
        rotation: impl Into<Quaternion>,
    ) {
        *self = Self::new(local_angular_inertia, rotation);
    }
}

#[cfg(feature = "3d")]
impl From<GlobalAngularInertia> for ComputedAngularInertia {
    fn from(inertia: GlobalAngularInertia) -> Self {
        inertia.0
    }
}

#[cfg(feature = "3d")]
impl From<Matrix> for GlobalAngularInertia {
    fn from(tensor: Matrix) -> Self {
        Self(ComputedAngularInertia::from_tensor(tensor))
    }
}

/// The local center of mass of a dynamic [rigid body].
///
/// [rigid body]: RigidBody
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct ComputedCenterOfMass(pub Vector);

impl ComputedCenterOfMass {
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

/// An automatically added component that contains the read-only mass properties of a [`Collider`].
/// The density used for computing the mass properties can be configured using the [`ColliderDensity`]
/// component.
///
/// These mass properties will be added to the [rigid body's](RigidBody) actual [`Mass`],
/// [`AngularInertia`] and [`CenterOfMass`] components.
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
///         println!("{}", mass_props.mass);
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

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "3d")]
    use approx::assert_relative_eq;

    #[test]
    fn mass_creation() {
        let mass = ComputedMass::new(10.0);
        assert_eq!(mass, ComputedMass::from_inverse(0.1));
        assert_eq!(mass.value(), 10.0);
        assert_eq!(mass.inverse(), 0.1);
    }

    #[test]
    fn zero_mass() {
        // Zero mass should be equivalent to infinite mass.
        let mass = ComputedMass::new(0.0);
        assert_eq!(mass, ComputedMass::new(Scalar::INFINITY));
        assert_eq!(mass, ComputedMass::from_inverse(0.0));
        assert_eq!(mass.value(), 0.0);
        assert_eq!(mass.inverse(), 0.0);
        assert!(mass.is_infinite());
        assert!(!mass.is_finite());
        assert!(!mass.is_nan());
    }

    #[test]
    fn infinite_mass() {
        let mass = ComputedMass::INFINITY;
        assert_eq!(mass, ComputedMass::new(Scalar::INFINITY));
        assert_eq!(mass, ComputedMass::from_inverse(0.0));
        assert_eq!(mass.value(), 0.0);
        assert_eq!(mass.inverse(), 0.0);
        assert!(mass.is_infinite());
        assert!(!mass.is_finite());
        assert!(!mass.is_nan());
    }

    #[test]
    #[should_panic]
    fn negative_mass_panics() {
        ComputedMass::new(-1.0);
    }

    #[test]
    fn negative_mass_error() {
        assert_eq!(
            ComputedMass::try_new(-1.0),
            Err(MassError::Negative),
            "negative mass should return an error"
        );
    }

    #[test]
    fn nan_mass_error() {
        assert_eq!(
            ComputedMass::try_new(Scalar::NAN),
            Err(MassError::NaN),
            "NaN mass should return an error"
        );
    }

    #[test]
    #[cfg(feature = "2d")]
    fn angular_inertia_creation() {
        let angular_inertia = ComputedAngularInertia::new(10.0);
        assert_eq!(angular_inertia, ComputedAngularInertia::from_inverse(0.1));
        assert_eq!(angular_inertia.value(), 10.0);
        assert_eq!(angular_inertia.inverse(), 0.1);
    }

    #[test]
    #[cfg(feature = "2d")]
    fn zero_angular_inertia() {
        // Zero angular inertia should be equivalent to infinite angular inertia.
        let angular_inertia = ComputedAngularInertia::new(0.0);
        assert_eq!(
            angular_inertia,
            ComputedAngularInertia::new(Scalar::INFINITY)
        );
        assert_eq!(angular_inertia, ComputedAngularInertia::from_inverse(0.0));
        assert_eq!(angular_inertia.value(), 0.0);
        assert_eq!(angular_inertia.inverse(), 0.0);
        assert!(angular_inertia.is_infinite());
        assert!(!angular_inertia.is_finite());
        assert!(!angular_inertia.is_nan());
    }

    #[test]
    #[cfg(feature = "2d")]
    fn infinite_angular_inertia() {
        let angular_inertia = ComputedAngularInertia::INFINITY;
        assert_eq!(
            angular_inertia,
            ComputedAngularInertia::new(Scalar::INFINITY)
        );
        assert_eq!(angular_inertia, ComputedAngularInertia::from_inverse(0.0));
        assert_eq!(angular_inertia.value(), 0.0);
        assert_eq!(angular_inertia.inverse(), 0.0);
        assert!(angular_inertia.is_infinite());
        assert!(!angular_inertia.is_finite());
        assert!(!angular_inertia.is_nan());
    }

    #[test]
    #[should_panic]
    #[cfg(feature = "2d")]
    fn negative_angular_inertia_panics() {
        ComputedAngularInertia::new(-1.0);
    }

    #[test]
    #[cfg(feature = "2d")]
    fn negative_angular_inertia_error() {
        assert_eq!(
            ComputedAngularInertia::try_new(-1.0),
            Err(AngularInertiaError::Negative),
            "negative angular inertia should return an error"
        );
    }

    #[test]
    #[cfg(feature = "2d")]
    fn nan_angular_inertia_error() {
        assert_eq!(
            ComputedAngularInertia::try_new(Scalar::NAN),
            Err(AngularInertiaError::NaN),
            "NaN angular inertia should return an error"
        );
    }

    #[test]
    #[cfg(feature = "3d")]
    fn angular_inertia_creation() {
        let angular_inertia = ComputedAngularInertia::new(Vector::new(10.0, 20.0, 30.0));
        assert_relative_eq!(
            angular_inertia.inverse_tensor(),
            ComputedAngularInertia::from_inverse_tensor(Matrix::from_diagonal(Vector::new(
                0.1,
                0.05,
                1.0 / 30.0
            )))
            .inverse_tensor()
        );
        assert_relative_eq!(
            angular_inertia.tensor(),
            Matrix::from_diagonal(Vector::new(10.0, 20.0, 30.0))
        );
        assert_relative_eq!(
            angular_inertia.inverse_tensor(),
            Matrix::from_diagonal(Vector::new(0.1, 0.05, 1.0 / 30.0))
        );
    }

    #[test]
    #[cfg(feature = "3d")]
    fn zero_angular_inertia() {
        let angular_inertia = ComputedAngularInertia::new(Vector::ZERO);
        assert_eq!(
            angular_inertia,
            ComputedAngularInertia::new(Vector::INFINITY)
        );
        assert_eq!(
            angular_inertia,
            ComputedAngularInertia::from_inverse_tensor(Matrix::from_diagonal(Vector::ZERO))
        );
        assert_relative_eq!(angular_inertia.tensor(), Matrix::ZERO);
        assert_relative_eq!(angular_inertia.inverse_tensor(), Matrix::ZERO);
        assert!(angular_inertia.is_infinite());
        assert!(!angular_inertia.is_finite());
        assert!(!angular_inertia.is_nan());
    }

    #[test]
    #[cfg(feature = "3d")]
    fn infinite_angular_inertia() {
        let angular_inertia = ComputedAngularInertia::INFINITY;
        assert_eq!(
            angular_inertia,
            ComputedAngularInertia::new(Vector::INFINITY)
        );
        assert_eq!(
            angular_inertia,
            ComputedAngularInertia::from_inverse_tensor(Matrix::ZERO)
        );
        assert_relative_eq!(angular_inertia.tensor(), Matrix::ZERO);
        assert_relative_eq!(angular_inertia.inverse_tensor(), Matrix::ZERO);
        assert!(angular_inertia.is_infinite());
        assert!(!angular_inertia.is_finite());
        assert!(!angular_inertia.is_nan());
    }

    #[test]
    #[should_panic]
    #[cfg(feature = "3d")]
    fn negative_angular_inertia_panics() {
        ComputedAngularInertia::new(Vector::new(-1.0, 2.0, 3.0));
    }

    #[test]
    #[cfg(feature = "3d")]
    fn negative_angular_inertia_error() {
        assert_eq!(
            ComputedAngularInertia::try_new(Vector::new(-1.0, 2.0, 3.0)),
            Err(AngularInertiaError::Negative),
            "negative angular inertia should return an error"
        );
    }

    #[test]
    #[cfg(feature = "3d")]
    fn nan_angular_inertia_error() {
        assert_eq!(
            ComputedAngularInertia::try_new(Vector::new(Scalar::NAN, 2.0, 3.0)),
            Err(AngularInertiaError::NaN),
            "NaN angular inertia should return an error"
        );
    }
}
