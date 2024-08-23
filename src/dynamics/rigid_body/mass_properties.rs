use crate::prelude::*;
use bevy::prelude::*;
use derive_more::From;

// TODO: Add comments
// TODO: Add ìs_finite and other helpers
// TODO: Add debug assertions
// TODO: Improve docs

/// The mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Mass {
    pub inverse: Scalar,
}

impl Mass {
    /// Zero mass.
    pub const ZERO: Self = Self {
        inverse: Scalar::MAX,
    };

    /// Infinite mass.
    pub const INFINITY: Self = Self { inverse: 0.0 };

    #[inline]
    pub fn new(mass: Scalar) -> Self {
        Self {
            inverse: mass.recip_or_zero(),
        }
    }

    #[inline]
    pub fn from_inverse(inverse_mass: Scalar) -> Self {
        Self {
            inverse: inverse_mass,
        }
    }

    #[inline]
    pub fn value(self) -> Scalar {
        self.inverse.recip_or_zero()
    }

    #[inline]
    pub fn set(&mut self, mass: Scalar) {
        self.inverse = mass.recip_or_zero();
    }
}

#[cfg(feature = "2d")]
type AngularVector = Scalar;

#[cfg(feature = "3d")]
type AngularVector = Vector;

/// The moment of inertia of a body. This represents the torque needed for a desired angular acceleration.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AngularInertia {
    pub inverse: Scalar,
}

#[cfg(feature = "2d")]
impl AngularInertia {
    /// Zero mass.
    pub const ZERO: Self = Self {
        inverse: Scalar::MAX,
    };

    /// Infinite mass.
    pub const INFINITY: Self = Self { inverse: 0.0 };

    #[inline]
    pub fn new(angular_inertia: Scalar) -> Self {
        Self {
            inverse: angular_inertia.recip_or_zero(),
        }
    }

    #[inline]
    pub fn from_inverse(inverse_angular_inertia: Scalar) -> Self {
        Self {
            inverse: inverse_angular_inertia,
        }
    }

    #[inline]
    pub fn value(self) -> Scalar {
        self.inverse.recip_or_zero()
    }

    #[inline]
    pub fn set(&mut self, angular_inertia: Scalar) {
        self.inverse = angular_inertia.recip_or_zero();
    }

    /// Computes the inertia of a body with the given mass, shifted by the given offset.
    #[inline]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Scalar {
        if mass > 0.0 && mass.is_finite() {
            self.inverse.recip_or_zero() + offset.length_squared() * mass
        } else {
            self.inverse.recip_or_zero()
        }
    }

    /// Computes the inverse inertia of a body with the given mass, shifted by the given offset.
    #[inline]
    pub fn shifted_inverse(&self, mass: Scalar, offset: Vector) -> Scalar {
        if mass > 0.0 && mass.is_finite() {
            self.inverse + offset.length_squared() * mass
        } else {
            self.inverse
        }
    }
}

/// The local moment of inertia of the body as a 3x3 tensor matrix.
/// This represents the torque needed for a desired angular acceleration along different axes.
///
/// This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account,
/// use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct AngularInertia {
    pub inverse: Matrix3,
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
        inverse: Matrix::from_diagonal(Vector::MAX),
    };

    /// Infinite angular inertia.
    pub const INFINITY: Self = Self {
        inverse: Matrix::ZERO,
    };

    /// Creates a new [`AngularInertia`] from the inertia tensor.
    #[inline]
    pub fn new(tensor: Matrix) -> Self {
        let inverse = tensor.inverse();
        Self { inverse }
    }

    /// Creates a new [`AngularInertia`] from the inverse inertia tensor.
    #[inline]
    pub fn from_inverse(inverse_tensor: Matrix) -> Self {
        Self {
            inverse: inverse_tensor,
        }
    }

    /// Creates a new [`AngularInertia`] from the principal inertia
    /// and the orientation of the local inertial frame.
    #[inline]
    pub fn from_principal(principal_inertia: Vector, orientation: Quaternion) -> Self {
        Self {
            inverse: Matrix::from_quat(orientation)
                * Matrix::from_diagonal(principal_inertia.recip_or_zero())
                * Matrix::from_quat(orientation.inverse()),
        }
    }

    /// Creates a new [`AngularInertia`] from the inverse principal inertia
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
    /// Equivalent to [`AngularInertia::tensor`].
    #[inline]
    pub fn value(self) -> Matrix {
        self.tensor()
    }

    /// Returns the angular inertia tensor.
    ///
    /// Equivalent to [`AngularInertia::value`].
    #[inline]
    pub fn tensor(self) -> Matrix {
        self.inverse.inverse()
    }

    /// Sets the angular inertia tensor.
    #[inline]
    pub fn set(&mut self, tensor: Matrix) {
        self.inverse = tensor.inverse();
    }

    /// Computes the angular inertia with the given rotation.
    #[inline]
    pub fn rotated(&self, rotation: Quaternion) -> Matrix {
        let rot_mat3 = Matrix3::from_quat(rotation);
        (rot_mat3 * self.inverse.inverse()) * rot_mat3.transpose()
    }

    /// Computes the inverse angular inertia with the given rotation.
    #[inline]
    pub fn rotated_inverse(&self, rotation: Quaternion) -> Matrix {
        let rot_mat3 = Matrix3::from_quat(rotation);
        (rot_mat3 * self.inverse) * rot_mat3.transpose()
    }

    /// Computes the angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Matrix3 {
        if mass > 0.0 && mass.is_finite() {
            let diagonal_element = offset.length_squared();
            let diagonal_mat = Matrix3::from_diagonal(Vector::splat(diagonal_element));
            let offset_outer_product =
                Matrix3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
            self.tensor() + (diagonal_mat + offset_outer_product) * mass
        } else {
            self.tensor()
        }
    }

    /// Computes the inverse angular inertia shifted by the given offset, taking into account the given mass.
    #[inline]
    pub fn shifted_inverse(&self, mass: Scalar, offset: Vector) -> Matrix3 {
        if mass > 0.0 && mass.is_finite() {
            let diagonal_element = offset.length_squared();
            let diagonal_mat = Matrix3::from_diagonal(Vector::splat(diagonal_element));
            let offset_outer_product =
                Matrix3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
            self.inverse + (diagonal_mat + offset_outer_product) * mass
        } else {
            self.inverse
        }
    }
}

/// The local moment of inertia of the body as a 3x3 tensor matrix.
/// This represents the torque needed for a desired angular acceleration about the XYZ axes.
///
/// This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account,
/// use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct WorldAngularInertia(pub AngularInertia);

/// The local center of mass of a body.
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
/// [`Mass`], [`AngularInertia`], [`AngularInertia`] and [`CenterOfMass`] components.
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
