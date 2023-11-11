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
pub struct Inertia2d(pub Scalar);

/// The local moment of inertia of the body as a 3x3 tensor matrix.
/// This represents the torque needed for a desired angular acceleration along different axes.
///
/// This is computed in local-space, so the object's orientation is not taken into account.
///
/// To get the world-space version that takes the body's rotation into account,
/// use the associated `rotated` method. Note that this operation is quite expensive, so use it sparingly.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct Inertia3d(pub Matrix3);

#[cfg(feature = "3d")]
impl Default for Inertia3d {
    fn default() -> Self {
        Self(Matrix3::ZERO)
    }
}

#[cfg(feature = "2d")]
impl Inertia2d {
    /// Zero angular inertia.
    pub const ZERO: Self = Self(0.0);

    /// Returns the inverted moment of inertia.
    #[cfg(feature = "2d")]
    pub fn inverse(&self) -> InverseInertia2d {
        InverseInertia2d(1.0 / self.0)
    }

    /// Computes the inertia of a body with the given mass, shifted by the given offset.
    #[cfg(feature = "2d")]
    pub fn shifted(&self, mass: Scalar, offset: Vector2) -> Scalar {
        if mass > 0.0 && mass.is_finite() {
            self.0 + offset.length_squared() * mass
        } else {
            self.0
        }
    }
}

#[cfg(feature = "3d")]
impl Inertia3d {
    /// Zero angular inertia.
    #[cfg(feature = "3d")]
    pub const ZERO: Self = Self(Matrix3::ZERO);

    /// Returns the inertia tensor's world-space version that takes
    /// the body's orientation into account.
    #[cfg(feature = "3d")]
    pub fn rotated(&self, rot: &Rotation3d) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    /// Returns the inverted moment of inertia.
    pub fn inverse(&self) -> InverseInertia3d {
        InverseInertia3d(self.0.inverse())
    }

    /// Computes the inertia of a body with the given mass, shifted by the given offset.
    pub fn shifted(&self, mass: Scalar, offset: Vector3) -> Matrix3 {
        type NaMatrix3 = parry3d::na::Matrix3<math::Scalar>;
        use parry3d::na::*;

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

/// The inverse moment of inertia of the body. This represents the inverse of
/// the torque needed for a desired angular acceleration.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InverseInertia2d(pub Scalar);

/// The inverse moment of inertia of the body. This represents the inverse of
/// the torque needed for a desired angular acceleration.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct InverseInertia3d(pub Matrix3);

#[cfg(feature = "3d")]
impl Default for InverseInertia3d {
    fn default() -> Self {
        InverseInertia3d(Matrix3::ZERO)
    }
}

#[cfg(feature = "2d")]
impl InverseInertia2d {
    /// Zero inverse angular inertia.
    pub const ZERO: Self = Self(0.0);

    /// Returns the original moment of inertia.
    pub fn inverse(&self) -> Inertia2d {
        Inertia2d(1.0 / self.0)
    }
}

#[cfg(feature = "3d")]
impl InverseInertia3d {
    /// Zero inverse angular inertia.
    pub const ZERO: Self = Self(Matrix3::ZERO);

    /// Returns the inertia tensor's world-space version that takes the body's orientation into account.
    pub fn rotated(&self, rot: &Rotation3d) -> Self {
        Self(get_rotated_inertia_tensor(self.0, rot.0))
    }

    /// Returns the original moment of inertia.
    pub fn inverse(&self) -> Inertia3d {
        Inertia3d(self.0.inverse())
    }
}

impl From<Inertia2d> for InverseInertia2d {
    fn from(inertia: Inertia2d) -> Self {
        inertia.inverse()
    }
}

impl From<Inertia3d> for InverseInertia3d {
    fn from(inertia: Inertia3d) -> Self {
        inertia.inverse()
    }
}

/// The local center of mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct CenterOfMass2d(pub Vector2);

/// The local center of mass of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub struct CenterOfMass3d(pub Vector3);

impl CenterOfMass2d {
    /// A center of mass set at the local origin.
    pub const ZERO: Self = Self(Vector2::ZERO);
}

impl CenterOfMass3d {
    /// A center of mass set at the local origin.
    pub const ZERO: Self = Self(Vector3::ZERO);
}

/// A bundle containing mass properties.
///
/// ## Example
///
/// The easiest way to create a new bundle is to use the [`new_computed`](Self::new_computed) method
/// that computes the mass properties based on a given [`Collider`] and density.
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         MassPropertiesBundle::new_computed(&Collider::ball(0.5), 1.0)
///     ));
/// }
/// ```
#[derive(Bundle, Debug, Default, Clone, PartialEq)]
pub struct MassProperties2dBundle {
    pub mass: Mass,
    pub inverse_mass: InverseMass,
    pub inertia: Inertia2d,
    pub inverse_inertia: InverseInertia2d,
    pub center_of_mass: CenterOfMass2d,
}

#[derive(Bundle, Debug, Default, Clone, PartialEq)]
pub struct MassProperties3dBundle {
    pub mass: Mass,
    pub inverse_mass: InverseMass,
    pub inertia: Inertia3d,
    pub inverse_inertia: InverseInertia3d,
    pub center_of_mass: CenterOfMass3d,
}

impl MassProperties2dBundle {
    /// Computes the mass properties for a [`Collider`] based on its shape and a given density.
    pub fn new_computed(collider: &Collider2d, density: Scalar) -> Self {
        let ColliderMassProperties2d {
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
            center_of_mass,
            ..
        } = collider.mass_properties(density);

        Self {
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
            center_of_mass,
        }
    }
}

impl MassProperties3dBundle {
    /// Computes the mass properties for a [`Collider`] based on its shape and a given density.
    pub fn new_computed(collider: &Collider3d, density: Scalar) -> Self {
        let ColliderMassProperties3d {
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
            center_of_mass,
            ..
        } = collider.mass_properties(density);

        Self {
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
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
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// // Spawn a body with a collider that has a density of 2.5
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::ball(0.5),
///         ColliderDensity(2.5),
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq, PartialOrd)]
#[reflect(Component)]
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

/// An automatically added component that contains the read-only mass properties of a [`Collider`].
/// The density used for computing the mass properties can be configured using the [`ColliderDensity`]
/// component.
///
/// These mass properties will be added to the [rigid body's](RigidBody) actual [`Mass`],
/// [`InverseMass`], [`Inertia`], [`InverseInertia`] and [`CenterOfMass`] components.
///
/// ## Example
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
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
///     commands.spawn(Collider::ball(0.5));
/// }
///
/// fn print_collider_masses(query: Query<&ColliderMassProperties>) {
///     for mass_props in &query {
///         println!("{}", mass_props.mass());
///     }
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[reflect(Component)]
pub struct ColliderMassProperties2d {
    /// Mass given by collider.
    pub(crate) mass: Mass,
    /// Inverse mass given by collider.
    pub(crate) inverse_mass: InverseMass,
    /// Inertia given by collider.
    pub(crate) inertia: Inertia2d,
    /// Inverse inertia given by collider.
    pub(crate) inverse_inertia: InverseInertia2d,
    /// Local center of mass given by collider.
    pub(crate) center_of_mass: CenterOfMass2d,
}

#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[reflect(Component)]
pub struct ColliderMassProperties3d {
    /// Mass given by collider.
    pub(crate) mass: Mass,
    /// Inverse mass given by collider.
    pub(crate) inverse_mass: InverseMass,
    /// Inertia given by collider.
    pub(crate) inertia: Inertia3d,
    /// Inverse inertia given by collider.
    pub(crate) inverse_inertia: InverseInertia3d,
    /// Local center of mass given by collider.
    pub(crate) center_of_mass: CenterOfMass3d,
}

#[cfg(feature = "2d")]
impl ColliderMassProperties2d {
    /// The collider has no mass.
    pub const ZERO: Self = Self {
        mass: Mass::ZERO,
        inverse_mass: InverseMass(Scalar::INFINITY),
        inertia: Inertia2d::ZERO,
        inverse_inertia: InverseInertia2d::ZERO,
        center_of_mass: CenterOfMass2d::ZERO,
    };

    /// Computes mass properties from a given [`Collider`] and density.
    ///
    /// Because [`ColliderMassProperties`] is read-only, adding this as a component manually
    /// has no effect. The mass properties will be recomputed using the [`ColliderDensity`].
    pub fn new(collider: &Collider2d, density: Scalar) -> Self {
        let props = collider.shape_scaled().mass_properties(density);

        Self {
            mass: Mass(props.mass()),
            inverse_mass: InverseMass(props.inv_mass),
            inertia: Inertia2d(props.principal_inertia()),
            inverse_inertia: InverseInertia2d(1.0 / props.principal_inertia()),
            center_of_mass: CenterOfMass2d(props.local_com.into()),
        }
    }

    /// Get the [mass](Mass) of the [`Collider`].
    pub fn mass(&self) -> Scalar {
        self.mass.0
    }

    /// Get the [inverse mass](InverseMass) of the [`Collider`].
    pub fn inverse_mass(&self) -> Scalar {
        self.inverse_mass.0
    }

    /// Get the [inerta](Inertia) of the [`Collider`].
    pub fn inertia(&self) -> Scalar {
        self.inertia.0
    }

    /// Get the [inverse inertia](InverseInertia) of the [`Collider`].
    pub fn inverse_inertia(&self) -> Scalar {
        self.inverse_inertia.0
    }

    /// Get the [local center of mass](CenterOfMass) of the [`Collider`].
    pub fn center_of_mass(&self) -> Vector2 {
        self.center_of_mass.0
    }
}

#[cfg(feature = "3d")]
impl ColliderMassProperties3d {
    /// The collider has no mass.
    pub const ZERO: Self = Self {
        mass: Mass::ZERO,
        inverse_mass: InverseMass(Scalar::INFINITY),
        inertia: Inertia3d::ZERO,
        inverse_inertia: InverseInertia3d::ZERO,
        center_of_mass: CenterOfMass3d::ZERO,
    };

    /// Computes mass properties from a given [`Collider`] and density.
    ///
    /// Because [`ColliderMassProperties`] is read-only, adding this as a component manually
    /// has no effect. The mass properties will be recomputed using the [`ColliderDensity`].
    pub fn new(collider: &Collider3d, density: Scalar) -> Self {
        let props = collider.shape_scaled().mass_properties(density);

        Self {
            mass: Mass(props.mass()),
            inverse_mass: InverseMass(props.inv_mass),
            inertia: Inertia3d(props.reconstruct_inertia_matrix().into()),
            inverse_inertia: InverseInertia3d(props.reconstruct_inverse_inertia_matrix().into()),
            center_of_mass: CenterOfMass3d(props.local_com.into()),
        }
    }

    /// Get the [mass](Mass) of the [`Collider`].
    pub fn mass(&self) -> Scalar {
        self.mass.0
    }

    /// Get the [inverse mass](InverseMass) of the [`Collider`].
    pub fn inverse_mass(&self) -> Scalar {
        self.inverse_mass.0
    }

    /// Get the [inertia tensor](InverseInertia) of the [`Collider`].
    pub fn inertia(&self) -> Matrix3 {
        self.inertia.0
    }

    /// Get the [inverse inertia](InverseInertia) of the [`Collider`].
    pub fn inverse_inertia(&self) -> Matrix3 {
        self.inverse_inertia.0
    }

    /// Get the [local center of mass](CenterOfMass) of the [`Collider`].
    pub fn center_of_mass(&self) -> Vector3 {
        self.center_of_mass.0
    }
}

impl Default for ColliderMassProperties2d {
    fn default() -> Self {
        Self::ZERO
    }
}

impl Default for ColliderMassProperties3d {
    fn default() -> Self {
        Self::ZERO
    }
}
