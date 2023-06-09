//! Resources used in the simulation.

use bevy::prelude::Resource;

use crate::prelude::*;

/// Configures how many times per second the physics simulation is run.
#[derive(Reflect, Resource, Clone, Copy, Debug, PartialEq)]
#[reflect(Resource)]
pub enum PhysicsTimestep {
    /// **Fixed timestep**: the physics simulation will be advanced by a fixed value `dt` for every `dt` seconds passed since the previous physics frame. This allows consistent behavior across different machines and framerates.
    Fixed(Scalar),
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

/// How much time the previous physics substep took. This depends on the [`DeltaTime`] and [`NumSubsteps`] resources.
#[derive(Reflect, Resource, Default)]
#[reflect(Resource)]
pub struct SubDeltaTime(pub Scalar);

/// The number of substeps used in XPBD simulation. A higher number of substeps reduces the value of [`SubDeltaTime`], which results in a more accurate simulation at the cost of performance.
#[derive(Reflect, Resource, Clone, Copy)]
#[reflect(Resource)]
pub struct NumSubsteps(pub u32);

impl Default for NumSubsteps {
    fn default() -> Self {
        Self(8)
    }
}

/// The number of iterations used in the position solver. It is recommended to keep this low and to increase [`NumSubsteps`] instead, as substepping can provide better convergence, accuracy and energy conservation.
#[derive(Reflect, Resource)]
#[reflect(Resource)]
pub struct NumPosIters(pub u32);

impl Default for NumPosIters {
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

/// The global gravitational acceleration. This is applied to dynamic bodies in the integration step.
///
/// The default is an acceleration of 9.81 m/s^2 pointing down, which is approximate to the gravitational acceleration near Earth's surface.
#[derive(Reflect, Resource, Debug)]
#[reflect(Resource)]
pub struct Gravity(pub Vector);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vector::Y * -9.81)
    }
}

impl Gravity {
    pub const ZERO: Gravity = Gravity(Vector::ZERO);
}
