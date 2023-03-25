use bevy::prelude::Resource;

use crate::{Scalar, Vector};

/// Configures how many times per second the physics simulation is run.
#[derive(Resource, Clone, Copy, Debug, PartialEq)]
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
#[derive(Resource, Default)]
pub(crate) struct DeltaTime(pub Scalar);

/// How much time the previous physics substep took. This depends on the [`DeltaTime`] and [`NumSubsteps`] resources.
#[derive(Resource, Default)]
pub(crate) struct SubDeltaTime(pub Scalar);

/// Number of substeps used in XPBD simulation
#[derive(Resource, Clone, Copy)]
pub struct NumSubsteps(pub u32);

impl Default for NumSubsteps {
    fn default() -> Self {
        Self(8)
    }
}

/// Number of iterations used in XPBD position solving
#[derive(Resource)]
pub struct NumPosIters(pub u32);

impl Default for NumPosIters {
    fn default() -> Self {
        Self(4)
    }
}

#[derive(Resource, Debug)]
pub struct Gravity(pub Vector);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vector::Y * -9.81)
    }
}

impl Gravity {
    pub const ZERO: Gravity = Gravity(Vector::ZERO);
}
