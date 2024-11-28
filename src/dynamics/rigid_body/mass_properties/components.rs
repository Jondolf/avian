use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;
use mass_properties::shifted_angular_inertia;

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
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Mass(pub Scalar);

impl Mass {
    /// Infinite mass.
    ///
    /// Zero mass is treated as a special case, and is used to represent infinite mass.
    pub const INFINITY: Self = Self(0.0);

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
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AngularInertia(pub Scalar);

#[cfg(feature = "2d")]
impl AngularInertia {
    /// Infinite angular inertia.
    ///
    /// Zero angular inertia is treated as a special case, and is used to represent infinite angular inertia.
    pub const INFINITY: Self = Self(0.0);

    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Scalar {
        shifted_angular_inertia(self.0, mass, offset)
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
pub struct AngularInertia {
    /// The principal angular inertia, representing the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the `local_frame`.
    pub principal: Vector,
    /// The orientation of the local inertial frame.
    pub local_frame: Quaternion,
}

impl Default for AngularInertia {
    fn default() -> Self {
        Self::INFINITY
    }
}

// TODO: Add helpers for getting the principal angular inertia and local inertial frame. This requires an eigensolver.
//       `bevy_heavy` has this functionality.
#[cfg(feature = "3d")]
impl AngularInertia {
    /// Infinite angular inertia with an identity local frame.
    ///
    /// Zero angular inertia is treated as a special case, and is used to represent infinite angular inertia.
    pub const INFINITY: Self = Self {
        principal: Vector::ZERO,
        local_frame: Quaternion::IDENTITY,
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
    /// Panics if any component of the principal angular inertia is negative when `debug_assertions` are enabled.
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
    /// about the local coordinate axes defined by the given `orientation`.
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

        Self {
            principal: principal_angular_inertia,
            local_frame: orientation,
        }
    }

    /// Tries to create a new [`AngularInertia`] from the given principal angular inertia
    /// and the orientation of the local inertial frame.
    ///
    /// The principal angular inertia represents the torque needed for a desired angular acceleration
    /// about the local coordinate axes defined by the given `orientation`.
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
            Ok(Self {
                principal: principal_angular_inertia,
                local_frame: orientation,
            })
        }
    }

    /// Creates a new [`AngularInertia`] from the given angular inertia tensor.
    ///
    /// The tensor should be symmetric and positive definite.
    #[inline]
    #[doc(alias = "from_mat3")]
    pub fn from_tensor(tensor: Matrix) -> Self {
        todo!()
    }

    /// Returns the angular inertia tensor.
    #[inline]
    #[doc(alias = "as_mat3")]
    pub fn tensor(self) -> Matrix {
        Matrix::from_quat(self.local_frame)
            * Matrix::from_diagonal(self.principal)
            * Matrix::from_quat(self.local_frame.inverse())
    }

    /// Computes the angular inertia tensor shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted_tensor(&self, mass: Scalar, offset: Vector) -> Matrix3 {
        shifted_angular_inertia(self.tensor(), mass, offset)
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

    /// Returns `true` if the principal angular inertia or local inertial frame is NaN.
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

#[cfg(feature = "2d")]
impl core::ops::Mul<Scalar> for AngularInertia {
    type Output = Scalar;

    #[inline]
    fn mul(self, rhs: Scalar) -> Scalar {
        self.0 * rhs
    }
}

impl core::ops::Mul<Vector> for AngularInertia {
    type Output = Vector;

    #[inline]
    fn mul(self, rhs: Vector) -> Vector {
        #[cfg(feature = "2d")]
        {
            self.0 * rhs
        }
        #[cfg(feature = "3d")]
        {
            self.tensor() * rhs
        }
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

    /// Creates a new [`CenterOfMass`] from the given local position.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new [`CenterOfMass`] from the given local position.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self(Vector::new(x, y, z))
    }
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
