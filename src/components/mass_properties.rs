use crate::prelude::*;
use bevy::prelude::*;

#[cfg(feature = "3d")]
use crate::utils::get_rotated_inertia_tensor;

/// The mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Mass(pub Scalar);

impl Mass {
    /// Zero mass.
    pub const ZERO: Self = Self(0.0);
}

/// The inverse mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InverseMass(pub Scalar);

impl InverseMass {
    /// Zero inverse mass.
    pub const ZERO: Self = Self(0.0);
}

/// The moment of inertia of a body. This represents the torque needed for a desired angular acceleration.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Inertia(pub Scalar);

/// The local moment of inertia of the body as a 3x3 tensor matrix.
/// This represents the torque needed for a desired angular acceleration along different axes.
///
/// This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account, use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Inertia(pub Matrix3);

#[cfg(feature = "3d")]
impl Default for Inertia {
    fn default() -> Self {
        Self(Matrix3::ZERO)
    }
}

impl Inertia {
    /// Zero angular inertia.
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(0.0);
    /// Zero angular inertia.
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Matrix3::ZERO);

    /// In 2D this does nothing, but it is there for convenience so that you don't have to handle 2D and 3D separately.
    #[cfg(feature = "2d")]
    #[allow(dead_code)]
    pub(crate) fn rotated(&self, _rot: &Rotation) -> Self {
        *self
    }

    /// Returns the inertia tensor's world-space version that takes the body's orientation into account.
    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rotation) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    /// Returns the inverted moment of inertia.
    #[cfg(feature = "2d")]
    pub fn inverse(&self) -> InverseInertia {
        InverseInertia(1.0 / self.0)
    }

    /// Returns the inverted moment of inertia.
    #[cfg(feature = "3d")]
    pub fn inverse(&self) -> InverseInertia {
        InverseInertia(self.0.inverse())
    }

    /// Computes the inertia of a body with the given mass, shifted by the given offset.
    #[cfg(feature = "2d")]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Scalar {
        if mass > 0.0 && mass.is_finite() {
            self.0 + offset.length_squared() * mass
        } else {
            self.0
        }
    }

    /// Computes the inertia of a body with the given mass, shifted by the given offset.
    #[cfg(feature = "3d")]
    pub fn shifted(&self, mass: Scalar, offset: Vector) -> Matrix3 {
        type NaMatrix3 = parry::na::Matrix3<math::Scalar>;
        use parry::na::*;

        if mass > 0.0 && mass.is_finite() {
            let matrix = NaMatrix3::from(self.0);
            let offset = Vector::from(offset);
            let diagonal_el = offset.norm_squared();
            let diagonal_mat = NaMatrix3::from_diagonal_element(diagonal_el);
            math::Matrix3::from(matrix + (diagonal_mat + offset * offset.transpose()) * mass)
        } else {
            self.0
        }
    }
}

/// The inverse moment of inertia of the body. This represents the inverse of the torque needed for a desired angular acceleration.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InverseInertia(pub Scalar);

/// The local inverse moment of inertia of the body as a 3x3 tensor matrix.
/// This represents the inverse of the torque needed for a desired angular acceleration along different axes.
///
/// This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account, use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InverseInertia(pub Matrix3);

#[cfg(feature = "3d")]
impl Default for InverseInertia {
    fn default() -> Self {
        InverseInertia(Matrix3::ZERO)
    }
}

impl InverseInertia {
    /// Zero inverse angular inertia.
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(0.0);
    /// Zero inverse angular inertia.
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Matrix3::ZERO);

    /// In 2D this does nothing, but it is there for convenience so that you don't have to handle 2D and 3D separately.
    #[cfg(feature = "2d")]
    pub fn rotated(&self, _rot: &Rotation) -> Self {
        *self
    }

    /// Returns the inertia tensor's world-space version that takes the body's orientation into account.
    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rotation) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    /// Returns the original moment of inertia.
    #[cfg(feature = "2d")]
    pub fn inverse(&self) -> Inertia {
        Inertia(1.0 / self.0)
    }

    /// Returns the original moment of inertia.
    #[cfg(feature = "3d")]
    pub fn inverse(&self) -> Inertia {
        Inertia(self.0.inverse())
    }
}

impl From<Inertia> for InverseInertia {
    fn from(inertia: Inertia) -> Self {
        inertia.inverse()
    }
}

/// The local center of mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct CenterOfMass(pub Vector);

impl CenterOfMass {
    /// A center of mass set at the local origin.
    pub const ZERO: Self = Self(Vector::ZERO);
}

/// A bundle containing mass properties.
///
/// ## Example
///
/// The easiest way to create a new bundle is to use the [`new_computed`](#method.new_computed) method
/// that computes the mass properties based on a given [`Collider`] and density.
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         MassPropertiesBundle::new_computed(&Collider::ball(0.5), 1.0)
///     ));
/// }
/// ```
#[allow(missing_docs)]
#[derive(Bundle, Debug, Default, Clone, PartialEq)]
pub struct MassPropertiesBundle {
    pub mass: Mass,
    pub inverse_mass: InverseMass,
    pub inertia: Inertia,
    pub inverse_inertia: InverseInertia,
    pub center_of_mass: CenterOfMass,
}

impl MassPropertiesBundle {
    /// Computes the mass properties from a given [`Collider`] and density.
    pub fn new_computed(collider: &Collider, density: Scalar) -> Self {
        let ColliderMassProperties {
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
            center_of_mass,
            ..
        } = ColliderMassProperties::new_computed(collider, density);

        Self {
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
            center_of_mass,
        }
    }
}

/// The mass properties derived from a given collider shape and density.
///
/// These will be added to the body's actual [`Mass`], [`InverseMass`], [`Inertia`], [`InverseInertia`] and [`CenterOfMass`] components.
///
/// You should generally not create or modify this directly. Instead, you can generate this automatically using a given collider shape and density with the associated `from_shape_and_density` method.
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[reflect(Component)]
pub struct ColliderMassProperties {
    /// Mass given by collider.
    pub mass: Mass,
    /// Inverse mass given by collider.
    pub inverse_mass: InverseMass,
    /// Inertia given by collider.
    pub inertia: Inertia,
    /// Inverse inertia given by collider.
    pub inverse_inertia: InverseInertia,
    /// Local center of mass given by collider.
    pub center_of_mass: CenterOfMass,
    /// Density used for calculating other mass properties.
    pub density: Scalar,
}

impl ColliderMassProperties {
    /// The collider has no mass.
    pub const ZERO: Self = Self {
        mass: Mass::ZERO,
        inverse_mass: InverseMass(Scalar::INFINITY),
        inertia: Inertia::ZERO,
        inverse_inertia: InverseInertia::ZERO,
        center_of_mass: CenterOfMass::ZERO,
        density: 0.0,
    };
}

impl ColliderMassProperties {
    /// Computes mass properties from a given [`Collider`] and density.
    pub fn new_computed(collider: &Collider, density: Scalar) -> Self {
        let props = collider.shape_scaled().mass_properties(density);

        Self {
            mass: Mass(props.mass()),
            inverse_mass: InverseMass(props.inv_mass),

            #[cfg(feature = "2d")]
            inertia: Inertia(props.principal_inertia()),
            #[cfg(feature = "3d")]
            inertia: Inertia(props.reconstruct_inertia_matrix().into()),

            #[cfg(feature = "2d")]
            inverse_inertia: InverseInertia(1.0 / props.principal_inertia()),
            #[cfg(feature = "3d")]
            inverse_inertia: InverseInertia(props.reconstruct_inverse_inertia_matrix().into()),

            center_of_mass: CenterOfMass(props.local_com.into()),

            density,
        }
    }
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        Self::ZERO
    }
}

/// The previous [`ColliderMassProperties`].
#[derive(Reflect, Clone, Copy, Component, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub(crate) struct PreviousColliderMassProperties(pub ColliderMassProperties);
