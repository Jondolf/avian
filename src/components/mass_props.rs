use crate::prelude::*;
use bevy::prelude::*;

#[cfg(feature = "3d")]
use crate::utils::get_rotated_inertia_tensor;

/// The mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Mass(pub Scalar);

impl Mass {
    pub const ZERO: Self = Self(0.0);
}

/// The inverse mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InvMass(pub Scalar);

impl InvMass {
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
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(0.0);
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Matrix3::ZERO);

    /// In 2D this does nothing, but it is there for convenience so that you don't have to handle 2D and 3D separately.
    #[cfg(feature = "2d")]
    #[allow(dead_code)]
    pub(crate) fn rotated(&self, _rot: &Rot) -> Self {
        *self
    }

    /// Returns the inertia tensor's world-space version that takes the body's orientation into account.
    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rot) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    /// Returns the inverted moment of inertia.
    #[cfg(feature = "2d")]
    pub fn inverse(&self) -> InvInertia {
        InvInertia(1.0 / self.0)
    }

    /// Returns the inverted moment of inertia.
    #[cfg(feature = "3d")]
    pub fn inverse(&self) -> InvInertia {
        InvInertia(self.0.inverse())
    }
}

/// The inverse moment of inertia of the body. This represents the inverse of the torque needed for a desired angular acceleration.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InvInertia(pub Scalar);

/// The local inverse moment of inertia of the body as a 3x3 tensor matrix.
/// This represents the inverse of the torque needed for a desired angular acceleration along different axes.
///
/// This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account, use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InvInertia(pub Matrix3);

#[cfg(feature = "3d")]
impl Default for InvInertia {
    fn default() -> Self {
        InvInertia(Matrix3::ZERO)
    }
}

impl InvInertia {
    #[cfg(feature = "2d")]
    pub const ZERO: Self = Self(0.0);
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Matrix3::ZERO);

    /// In 2D this does nothing, but it is there for convenience so that you don't have to handle 2D and 3D separately.
    #[cfg(feature = "2d")]
    pub fn rotated(&self, _rot: &Rot) -> Self {
        *self
    }

    /// Returns the inertia tensor's world-space version that takes the body's orientation into account.
    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rot) -> Self {
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

impl From<Inertia> for InvInertia {
    fn from(inertia: Inertia) -> Self {
        inertia.inverse()
    }
}

/// The local center of mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct LocalCom(pub Vector);

impl LocalCom {
    pub const ZERO: Self = Self(Vector::ZERO);
}

#[derive(Bundle, Debug, Default, Clone, PartialEq)]
pub struct MassPropsBundle {
    mass: Mass,
    inv_mass: InvMass,
    inertia: Inertia,
    inv_inertia: InvInertia,
    local_center_of_mass: LocalCom,
}

impl MassPropsBundle {
    /// Computes the mass properties from a given shape and density.
    pub fn new_computed(shape: &Shape, density: Scalar) -> Self {
        let ColliderMassProperties {
            mass,
            inv_mass,
            inertia,
            inv_inertia,
            local_center_of_mass,
            ..
        } = ColliderMassProperties::from_shape_and_density(shape, density);

        Self {
            mass,
            inv_mass,
            inertia,
            inv_inertia,
            local_center_of_mass,
        }
    }
}

/// The mass properties derived from a given collider shape and density.
///
/// These will be added to the body's actual [`Mass`], [`InvMass`], [`Inertia`], [`InvInertia`] and [`LocalCom`] components.
///
/// You should generally not create or modify this directly. Instead, you can generate this automatically using a given collider shape and density with the associated `from_shape_and_density` method.
#[derive(Reflect, Clone, Copy, Component, PartialEq)]
#[reflect(Component)]
pub struct ColliderMassProperties {
    /// Mass given by collider.
    pub mass: Mass,
    /// Inverse mass given by collider.
    pub inv_mass: InvMass,
    /// Inertia given by collider.
    pub inertia: Inertia,
    /// Inverse inertia given by collider.
    pub inv_inertia: InvInertia,
    /// Local center of mass given by collider.
    pub local_center_of_mass: LocalCom,
    /// Density used for calculating other mass properties.
    pub density: Scalar,
}

impl ColliderMassProperties {
    pub const ZERO: Self = Self {
        mass: Mass::ZERO,
        inv_mass: InvMass(Scalar::INFINITY),
        inertia: Inertia::ZERO,
        inv_inertia: InvInertia::ZERO,
        local_center_of_mass: LocalCom::ZERO,
        density: 0.0,
    };
}

impl ColliderMassProperties {
    /// Computes mass properties for a given shape and density.
    pub fn from_shape_and_density(shape: &Shape, density: Scalar) -> Self {
        let props = shape.mass_properties(density);

        Self {
            mass: Mass(props.mass()),
            inv_mass: InvMass(props.inv_mass),

            #[cfg(feature = "2d")]
            inertia: Inertia(props.principal_inertia()),
            #[cfg(feature = "3d")]
            inertia: Inertia(props.reconstruct_inertia_matrix().into()),

            #[cfg(feature = "2d")]
            inv_inertia: InvInertia(1.0 / props.principal_inertia()),
            #[cfg(feature = "3d")]
            inv_inertia: InvInertia(props.reconstruct_inverse_inertia_matrix().into()),

            local_center_of_mass: LocalCom(props.local_com.into()),

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
#[derive(Clone, Copy, Component, Default, Deref, DerefMut, PartialEq)]
pub(crate) struct PrevColliderMassProperties(pub ColliderMassProperties);
