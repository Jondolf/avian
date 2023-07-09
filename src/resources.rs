//! Resources used in the simulation.

use bevy::prelude::Resource;

use crate::prelude::*;

/// Configures how many times per second the physics simulation is run.
///
/// ## Example
///
/// You can change the timestep by inserting the [`PhysicsTimestep`] resource:
///
/// ```no_run
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins(DefaultPlugins)
///         .add_plugins(PhysicsPlugins)
///         // Use a 120 Hz fixed timestep instead of the default 60 Hz
///         .insert_resource(PhysicsTimestep::Fixed(1.0 / 120.0))
///         .run();
/// }
/// ```
#[derive(Reflect, Resource, Clone, Copy, Debug, PartialEq)]
#[reflect(Resource)]
pub enum PhysicsTimestep {
    /// **Fixed timestep**: the physics simulation will be advanced by a fixed value `dt` for every `dt` seconds passed since the previous physics frame. This allows consistent behavior across different machines and framerates.
    Fixed(Scalar),
    /// **Fixed delta, once per frame**: the physics simulation will be advanced by a fixed value `dt` once every frame. This should only be used in cases where you can guarantee a fixed number of executions, like in FixedUpdate or on a server.
    FixedOnce(Scalar),
    /// **Variable timestep**: the physics simulation will be advanced by `Time::delta_seconds().min(max_dt)` seconds at each Bevy tick.
    Variable {
        /// The maximum amount of time the physics simulation can be advanced at each Bevy tick. This makes sure that the simulation doesn't break when the delta time is large.
        ///
        /// A good default is `1.0 / 60.0` (60 Hz)
        max_dt: Scalar,
    },
}

impl Default for PhysicsTimestep {
    fn default() -> Self {
        Self::Fixed(1.0 / 60.0)
    }
}

/// How much time the previous physics frame took. The timestep can be configured with the [`PhysicsTimestep`] resource.
#[derive(Reflect, Resource, Default)]
#[reflect(Resource)]
pub struct DeltaTime(pub Scalar);

/// How much time the previous physics substep took. This depends on the [`DeltaTime`] and [`SubstepCount`] resources.
#[derive(Reflect, Resource, Default)]
#[reflect(Resource)]
pub struct SubDeltaTime(pub Scalar);

/// The number of substeps used in the simulation.
///
/// A higher number of substeps reduces the value of [`SubDeltaTime`],
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
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins(DefaultPlugins)
///         .add_plugins(PhysicsPlugins)
///         .insert_resource(SubstepCount(30))
///         .run();
/// }
/// ```
#[derive(Reflect, Resource, Clone, Copy)]
#[reflect(Resource)]
pub struct SubstepCount(pub u32);

impl Default for SubstepCount {
    fn default() -> Self {
        Self(12)
    }
}

/// The number of iterations used in the position solver. It is recommended to keep this low and to increase [`SubstepCount`] instead, as substepping can provide better convergence, accuracy and energy conservation.
#[derive(Reflect, Resource)]
#[reflect(Resource)]
pub struct IterationCount(pub u32);

impl Default for IterationCount {
    fn default() -> Self {
        Self(4)
    }
}

/// A list of entity pairs for potential collisions collected during the broad phase.
#[derive(Reflect, Resource, Default, Debug)]
#[reflect(Resource)]
pub struct BroadCollisionPairs(pub Vec<(Entity, Entity)>);

/// A threshold that indicates the maximum linear and angular velocity allowed for a body to be deactivated.
///
/// Setting a negative sleeping threshold disables sleeping entirely.
///
/// See [`Sleeping`] for further information about sleeping.
#[derive(Reflect, Resource, Clone, Copy, PartialEq, PartialOrd, Debug)]
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
/// You can change gravity by simply inserting the [`Gravity`] resource:
///
/// ```no_run
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn main() {
///     App::new()
///         .add_plugins(DefaultPlugins)
///         .add_plugins(PhysicsPlugins)
///         .insert_resource(Gravity(Vec3::NEG_Y * 19.6))
///         .run();
/// }
/// # #[cfg(not(all(feature = "3d", feature = "f32")))]
/// # fn main() {} // Doc test needs main
/// ```
///
/// You can also modify gravity while the app is running.
#[derive(Reflect, Resource, Debug)]
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
