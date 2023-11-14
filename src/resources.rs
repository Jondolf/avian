//! Resources used in the simulation.

use bevy::prelude::Resource;

use crate::prelude::*;

/// The number of substeps used in the simulation.
///
/// A higher number of substeps reduces the value of [`Time`],
/// which results in a more accurate simulation, but also reduces performance. The default
/// substep count is currently 12.
///
/// If you use a very high substep count and encounter stability issues, consider enabling the `f64`
/// feature as shown in the [getting started guide](crate#getting-started) to avoid floating point
/// precision problems.
///
/// ## Example
///
/// You can change the number of substeps by inserting the [`SubstepCount`] resource:
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         .insert_resource(SubstepCount(30))
///         .run();
/// }
/// ```
#[derive(Reflect, Resource, Clone, Copy)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Resource)]
pub struct SubstepCount(pub u32);

impl Default for SubstepCount {
    fn default() -> Self {
        Self(12)
    }
}

/// A threshold that indicates the maximum linear and angular velocity allowed for a body to be deactivated.
///
/// Setting a negative sleeping threshold disables sleeping entirely.
///
/// See [`Sleeping`] for further information about sleeping.
#[derive(Reflect, Resource, Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Resource)]
pub struct SleepingThreshold {
    /// The maximum linear velocity allowed for a body to be marked as sleeping.
    pub linear: Scalar,
    /// The maximum angular velocity allowed for a body to be marked as sleeping.
    pub angular: Scalar,
}

impl Default for SleepingThreshold {
    fn default() -> Self {
        Self {
            linear: 0.1,
            angular: 0.2,
        }
    }
}

/// How long in seconds the linear and angular velocity of a body need to be below
/// the [`SleepingThreshold`] before the body is deactivated. Defaults to 1 second.
///
/// See [`Sleeping`] for further information about sleeping.
#[derive(Reflect, Resource, Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Resource)]
pub struct DeactivationTime(pub Scalar);

impl Default for DeactivationTime {
    fn default() -> Self {
        Self(1.0)
    }
}

/// A resource for the global gravitational acceleration.
///
/// The default is an acceleration of 9.81 m/s^2 pointing down, which is approximate to the gravitational
/// acceleration near Earth's surface. Note that if you are using pixels as length units in 2D,
/// this gravity will be tiny. You should modify the gravity to fit your application.
///
/// You can also control how gravity affects a specific [rigid body](RigidBody) using the [`GravityScale`]
/// component. The magnitude of the gravity will be multiplied by this scaling factor.
///
/// ## Example
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// # #[cfg(feature = "f32")]
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
#[cfg_attr(
    feature = "2d",
    doc = "         .insert_resource(Gravity(Vec2::NEG_Y * 100.0))"
)]
#[cfg_attr(
    feature = "3d",
    doc = "         .insert_resource(Gravity(Vec3::NEG_Y * 19.6))"
)]
///         .run();
/// }
/// # #[cfg(not(feature = "f32"))]
/// # fn main() {} // Doc test needs main
/// ```
///
/// You can also modify gravity while the app is running.
#[derive(Reflect, Resource, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Resource)]
pub struct Gravity(pub Vector);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vector::Y * -9.81)
    }
}

impl Gravity {
    /// Zero gravity.
    pub const ZERO: Gravity = Gravity(Vector::ZERO);
}
