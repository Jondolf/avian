//! Clocks used for tracking physics simulation time.

use std::time::Duration;

use bevy::prelude::*;

use crate::prelude::*;

/// The clock representing physics time, following `Time<Virtual>`.
/// Can be configured to use a fixed or variable timestep.
///
/// The clock is automatically set as the generic `Time` resource for
/// the [`SubstepSchedule`].
///
/// By default, a fixed timestep of 60 Hz is used.
#[derive(Reflect, Clone, Copy, Debug, PartialEq)]
pub enum Physics {
    /// **Fixed timestep**: the physics simulation will be advanced by a fixed value `dt` for every `dt` seconds passed since the previous physics frame. This allows consistent behavior across different machines and framerates.
    Fixed(Duration),
    /// **Fixed delta, once per frame**: the physics simulation will be advanced by a fixed value `dt` once every frame. This should only be used in cases where you can guarantee a fixed number of executions, like in FixedUpdate or on a server.
    FixedOnce(Duration),
    /// **Variable timestep**: the physics simulation will be advanced by `Time::delta_seconds().min(max_dt)` seconds at each Bevy tick.
    Variable {
        /// The maximum amount of time the physics simulation can be advanced at each Bevy tick. This makes sure that the simulation doesn't break when the delta time is large.
        ///
        /// A good default is `1.0 / 60.0` (60 Hz)
        max_dt: Duration,
    },
}

impl Default for Physics {
    fn default() -> Self {
        // Corresponds to 60 Hz.
        // TODO: Bevy's fixed timestep is 64 Hz, but it causes physics
        //       to be run twice in a single frame every 0.25 seconds.
        //       It would be nice to have the timestep be more unified though.
        Self::Fixed(Duration::from_secs_adjusted(1.0 / 60.0))
    }
}

/// The clock representing physics substep time. It is updated based on
/// [`Time<Physics>`](Physics) and the [`SubstepCount`] resource.
///
/// The clock is automatically set as the generic `Time` resource for
/// the [`SubstepSchedule`].
#[derive(Reflect, Clone, Copy, Debug, Default, PartialEq)]
pub struct Substeps;

pub(crate) trait TimePrecisionAdjusted {
    /// Returns how much time has advanced since the last update
    /// as [`Scalar`] seconds.
    fn delta_seconds_adjusted(&self) -> Scalar;
}

pub(crate) trait DurationPrecisionAdjusted {
    /// Returns the number of seconds contained by this Duration as [`Scalar`].
    ///
    /// The returned value does include the fractional (nanosecond) part of the duration.
    fn as_secs_adjusted(&self) -> Scalar;

    /// Creates a new Duration from the specified number of seconds represented as [`Scalar`].
    fn from_secs_adjusted(secs: Scalar) -> Self;
}

impl TimePrecisionAdjusted for Time {
    /// Returns how much time has advanced since the last [`update`](#method.update)
    /// as [`Scalar`] seconds.
    fn delta_seconds_adjusted(&self) -> Scalar {
        #[cfg(feature = "f32")]
        {
            self.delta_seconds()
        }
        #[cfg(feature = "f64")]
        {
            self.delta_seconds_f64()
        }
    }
}

impl DurationPrecisionAdjusted for Duration {
    fn as_secs_adjusted(&self) -> Scalar {
        #[cfg(feature = "f32")]
        {
            self.as_secs_f32()
        }
        #[cfg(feature = "f64")]
        {
            self.as_secs_f64()
        }
    }

    fn from_secs_adjusted(secs: Scalar) -> Self {
        #[cfg(feature = "f32")]
        {
            Self::from_secs_f32(secs)
        }
        #[cfg(feature = "f64")]
        {
            Self::from_secs_f64(secs)
        }
    }
}
