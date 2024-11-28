use crate::prelude::*;
use bevy::prelude::*;

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
        super::shifted_angular_inertia(self.angular_inertia, self.mass, offset)
    }

    /// Computes the angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn shifted_angular_inertia(&self, offset: Vector) -> Matrix {
        super::shifted_angular_inertia(self.angular_inertia, self.mass, offset)
    }

    /// Computes the inverse angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "2d")]
    #[inline]
    pub fn shifted_inverse_angular_inertia(&self, offset: Vector) -> Scalar {
        super::shifted_angular_inertia(self.angular_inertia, self.mass, offset).recip_or_zero()
    }

    /// Computes the inverse angular inertia shifted by the given offset, taking into account mass.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn shifted_inverse_angular_inertia(&self, offset: Vector) -> Matrix {
        super::shifted_angular_inertia(self.angular_inertia, self.mass, offset).inverse_or_zero()
    }
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        Self::ZERO
    }
}
