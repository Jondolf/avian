//! Clocks used for tracking physics simulation time.

use std::time::Duration;

use bevy::prelude::*;

use crate::prelude::*;

/// The clock representing physics time, following `Time<Virtual>`.
/// Can be configured to use a fixed or variable timestep.
///
/// The clock is automatically set as the generic `Time` resource for
/// the [`PhysicsSchedule`].
///
/// By default, a fixed timestep of 60 Hz is used.
///
/// ## Example
///
/// ```no_run
/// use bevy::{prelude::*, utils::Duration};
#[cfg_attr(
    feature = "2d",
    doc = "use bevy_xpbd_2d::{prelude::*, PhysicsSchedule, PhysicsStepSet};"
)]
#[cfg_attr(
    feature = "3d",
    doc = "use bevy_xpbd_3d::{prelude::*, PhysicsSchedule, PhysicsStepSet};"
)]
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         // Overwrite default timestep used for physics
///         .insert_resource(Time::new_with(Physics::fixed_hz(1.0 / 144.0)))
///         // In `Update`, `Time` is `Time<Virtual>`
///         .add_systems(Update, print_delta_time)
///         // In `PhysicsSchedule`, `Time` is `Time<Physics>`
///         .add_systems(PhysicsSchedule, print_delta_time.before(PhysicsStepSet::Substeps))
///         .run();
/// }
///
/// fn print_delta_time(time: Res<Time>) {
///     println!("{}", time.delta_seconds());
/// }
/// ```
// TODO: Document when to multiply by delta time
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
        Self::fixed_hz(1.0 / 60.0)
    }
}

impl Physics {
    /// Returns a new [`Time<Physics>`](Physics) clock with a fixed timestep using
    /// the given frequency in Hertz (1/second).
    ///
    /// # Panics
    ///
    /// Panics if `hz` is zero, negative or not finite.
    pub fn fixed_hz(hz: f64) -> Self {
        assert!(hz > 0.0, "Hz less than or equal to zero");
        assert!(hz.is_finite(), "Hz is infinite");
        Self::Fixed(Duration::from_secs_f64(hz))
    }

    /// Returns a new [`Time<Physics>`](Physics) clock with a [`Physics::FixedOnce`] timestep
    /// using the given frequency in Hertz (1/second).
    ///
    /// Unlike with [`Physics::Fixed`], the [`PhysicsSchedule`] will only be run once per frame
    /// instead of accumulating time and running physics until the accumulator has been consumed.
    /// This can be useful for [server usage](crate#can-the-engine-be-used-on-servers)
    /// where the server and client must be kept in sync.
    ///
    /// # Panics
    ///
    /// Panics if `hz` is zero, negative or not finite.
    pub fn fixed_once_hz(hz: f64) -> Self {
        assert!(hz > 0.0, "Hz less than or equal to zero");
        assert!(hz.is_finite(), "Hz is infinite");
        Self::FixedOnce(Duration::from_secs_f64(hz))
    }

    /// Returns a new [`Time<Physics>`](Physics) clock with a [`Physics::Variable`] timestep using
    /// the given frequency in Hertz (1/second).
    ///
    /// # Panics
    ///
    /// Panics if `max_delta_seconds` is zero or negative.
    pub fn variable(max_delta_seconds: f64) -> Self {
        assert!(
            max_delta_seconds > 0.0,
            "max delta less than or equal to zero"
        );
        Self::Variable {
            max_dt: Duration::from_secs_f64(max_delta_seconds),
        }
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
