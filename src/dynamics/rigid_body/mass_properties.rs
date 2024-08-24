use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;

// TODO: Add ìs_finite and other helpers
// TODO: Add debug assertions

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
pub struct Mass {
    /// The inverse mass.
    ///
    /// This is stored as an inverse because most physics calculations
    /// operate on the inverse mass, and storing it directly allows for
    /// fewer divisions and guards against division by zero.
    inverse: Scalar,
}

impl Mass {
    /// Zero mass.
    pub const ZERO: Self = Self {
        inverse: Scalar::INFINITY,
    };

    /// Infinite mass.
    pub const INFINITY: Self = Self { inverse: 0.0 };

    /// Creates a new [`Mass`] from the given mass.
    #[inline]
    pub fn new(mass: Scalar) -> Self {
        Self {
            inverse: mass.recip_or_zero(),
        }
    }

    /// Creates a new [`Mass`] from the given inverse mass.
    #[inline]
    pub fn from_inverse(inverse_mass: Scalar) -> Self {
        Self {
            inverse: inverse_mass,
        }
    }

    /// Returns the mass.
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
    pub fn set(&mut self, mass: impl Into<Mass>) {
        *self = mass.into();
    }
}

impl From<Scalar> for Mass {
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
pub struct AngularInertia {
    /// The inverse angular inertia.
    ///
    /// This is stored as an inverse to minimize the number of divisions
    /// and to guard against division by zero. Most physics calculations
    /// use the inverse angular inertia.
    inverse: Scalar,
}

#[cfg(feature = "2d")]
impl AngularInertia {
    /// Zero mass.
    pub const ZERO: Self = Self {
        inverse: Scalar::INFINITY,
    };

    /// Infinite mass.
    pub const INFINITY: Self = Self { inverse: 0.0 };

    /// Creates a new [`AngularInertia`] from the given angular inertia.
    #[inline]
    pub fn new(angular_inertia: Scalar) -> Self {
        Self {
            inverse: angular_inertia.recip_or_zero(),
        }
    }

    /// Creates a new [`AngularInertia`] from the given inverse angular inertia.
    #[inline]
    pub fn from_inverse(inverse_angular_inertia: Scalar) -> Self {
        Self {
            inverse: inverse_angular_inertia,
        }
    }

    /// Returns the angular inertia.
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

    /// Sets the angular inertia.
    #[inline]
    pub fn set(&mut self, angular_inertia: impl Into<AngularInertia>) {
        *self = angular_inertia.into();
    }

    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Scalar {
        if mass > 0.0 && mass.is_finite() && offset != Vector::ZERO {
            self.value() + offset.length_squared() * mass
        } else {
            self.value()
        }
    }

    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted_inverse(&self, mass: Scalar, offset: Vector) -> Scalar {
        if mass > 0.0 && mass.is_finite() && offset != Vector::ZERO {
            (self.value() + offset.length_squared() * mass).recip_or_zero()
        } else {
            self.inverse
        }
    }
}

#[cfg(feature = "2d")]
impl From<Scalar> for AngularInertia {
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
    inverse: Matrix3,
}

impl Default for AngularInertia {
    fn default() -> Self {
        Self::INFINITY
    }
}

#[cfg(feature = "3d")]
impl AngularInertia {
    /// Zero angular inertia.
    pub const ZERO: Self = Self {
        inverse: Matrix::from_diagonal(Vector::INFINITY),
    };

    /// Infinite angular inertia.
    pub const INFINITY: Self = Self {
        inverse: Matrix::ZERO,
    };

    /// Creates a new [`AngularInertia`] from the given angular inertia tensor.
    #[inline]
    pub fn new(tensor: Matrix) -> Self {
        let inverse = tensor.inverse_or_zero();
        Self { inverse }
    }

    /// Creates a new [`AngularInertia`] from the given inverse angular inertia tensor.
    #[inline]
    pub fn from_inverse(inverse_tensor: Matrix) -> Self {
        Self {
            inverse: inverse_tensor,
        }
    }

    /// Creates a new [`AngularInertia`] from the given principal inertia
    /// and the orientation of the local inertial frame.
    #[inline]
    pub fn from_principal(principal_inertia: Vector, orientation: Quaternion) -> Self {
        Self {
            inverse: Matrix::from_quat(orientation)
                * Matrix::from_diagonal(principal_inertia.recip_or_zero())
                * Matrix::from_quat(orientation.inverse()),
        }
    }

    /// Creates a new [`AngularInertia`] from the given inverse principal inertia
    /// and the orientation of the local inertial frame.
    #[inline]
    pub fn from_inverse_principal(
        inverse_principal_inertia: Vector,
        orientation: Quaternion,
    ) -> Self {
        Self {
            inverse: Matrix::from_quat(orientation)
                * Matrix::from_diagonal(inverse_principal_inertia)
                * Matrix::from_quat(orientation.inverse()),
        }
    }

    /// Returns the angular inertia tensor.
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
    /// If multiplying by the inverse angular inertia, consider using `angular_inertia.inverse() * foo`
    /// instead of `angular_inertia.value().inverse() * foo`.
    ///
    /// Equivalent to [`AngularInertia::tensor`].
    #[inline]
    pub fn value(self) -> Matrix {
        self.tensor()
    }

    /// Returns the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`AngularInertia`] internally stores the inverse angular inertia.
    ///
    /// Equivalent to [`AngularInertia::inverse_tensor`].
    #[inline]
    pub fn inverse(self) -> Matrix {
        self.inverse
    }

    /// Returns the angular inertia tensor.
    ///
    /// Note that this involves an invertion because [`AngularInertia`] internally stores the inverse angular inertia.
    /// If multiplying by the inverse angular inertia, consider using `angular_inertia.inverse() * foo`
    /// instead of `angular_inertia.value().inverse() * foo`.
    ///
    /// Equivalent to [`AngularInertia::value`].
    #[inline]
    pub fn tensor(self) -> Matrix {
        self.inverse.inverse_or_zero()
    }

    /// Returns the inverse of the angular inertia tensor.
    ///
    /// Note that this is a no-op because [`AngularInertia`] internally stores the inverse angular inertia.
    ///
    /// Equivalent to [`AngularInertia::inverse_tensor`].
    #[inline]
    pub fn inverse_tensor(self) -> Matrix {
        self.inverse
    }

    /// Sets the angular inertia tensor.
    #[inline]
    pub fn set(&mut self, angular_inertia: impl Into<AngularInertia>) {
        *self = angular_inertia.into();
    }

    /// Computes the angular inertia with the given rotation.
    ///
    /// This can be used to transform the local angular inertia to world space.
    #[inline]
    pub fn rotated(&self, rotation: Quaternion) -> Matrix {
        let rot_mat3 = Matrix3::from_quat(rotation);
        (rot_mat3 * self.tensor()) * rot_mat3.transpose()
    }

    /// Computes the inverse angular inertia with the given rotation.
    ///
    /// This can be used to transform the local inverse angular inertia to world space.
    #[inline]
    pub fn rotated_inverse(&self, rotation: Quaternion) -> Matrix {
        let rot_mat3 = Matrix3::from_quat(rotation);
        (rot_mat3 * self.inverse) * rot_mat3.transpose()
    }

    /// Computes the angular inertia tensor shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Matrix3 {
        if mass > 0.0 && mass.is_finite() && offset != Vector::ZERO {
            let diagonal_element = offset.length_squared();
            let diagonal_mat = Matrix3::from_diagonal(Vector::splat(diagonal_element));
            let offset_outer_product =
                Matrix3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
            self.tensor() + (diagonal_mat + offset_outer_product) * mass
        } else {
            self.tensor()
        }
    }

    /// Computes the inverse angular inertia tensor shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted_inverse(&self, mass: Scalar, offset: Vector) -> Matrix3 {
        if mass > 0.0 && mass.is_finite() && offset != Vector::ZERO {
            let diagonal_element = offset.length_squared();
            let diagonal_mat = Matrix3::from_diagonal(Vector::splat(diagonal_element));
            let offset_outer_product =
                Matrix3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
            (self.tensor() + (diagonal_mat + offset_outer_product) * mass).inverse_or_zero()
        } else {
            self.inverse
        }
    }
}

#[cfg(feature = "3d")]
impl From<Matrix> for AngularInertia {
    fn from(tensor: Matrix) -> Self {
        Self::new(tensor)
    }
}

// In 2D, the global angular inertia is the same as the local angular inertia.
#[cfg(feature = "2d")]
pub(crate) type GlobalAngularInertia = AngularInertia;

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
pub struct GlobalAngularInertia(AngularInertia);

#[cfg(feature = "3d")]
impl GlobalAngularInertia {
    /// Creates a new [`GlobalAngularInertia`] from the given local angular inertia and rotation.
    pub fn new(
        local_angular_inertia: impl Into<AngularInertia>,
        rotation: impl Into<Quaternion>,
    ) -> Self {
        let local_angular_inertia: AngularInertia = local_angular_inertia.into();
        Self(AngularInertia::from_inverse(
            local_angular_inertia.rotated_inverse(rotation.into()),
        ))
    }

    /// Updates the global angular inertia with the given local angular inertia and rotation.
    pub fn update(
        &mut self,
        local_angular_inertia: impl Into<AngularInertia>,
        rotation: impl Into<Quaternion>,
    ) {
        *self = Self::new(local_angular_inertia, rotation);
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
            mass,
            angular_inertia,
            center_of_mass,
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
///         println!("{}", mass_props.mass.0);
///     }
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderMassProperties {
    /// Mass given by collider.
    pub mass: Mass,
    /// Inertia given by collider.
    pub angular_inertia: AngularInertia,
    /// Local center of mass given by collider.
    pub center_of_mass: CenterOfMass,
}

impl ColliderMassProperties {
    /// The collider has no mass.
    pub const ZERO: Self = Self {
        mass: Mass::ZERO,
        angular_inertia: AngularInertia::ZERO,
        center_of_mass: CenterOfMass::ZERO,
    };

    /// The collider has infinite mass.
    pub const INFINITY: Self = Self {
        mass: Mass::INFINITY,
        angular_inertia: AngularInertia::INFINITY,
        center_of_mass: CenterOfMass::ZERO,
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
        self.center_of_mass.0 = transform.transform_point(self.center_of_mass.0);
        self
    }
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        Self::ZERO
    }
}
