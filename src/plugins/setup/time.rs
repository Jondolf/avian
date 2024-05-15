//! Clocks used for tracking physics simulation time.

use std::time::Duration;

use bevy::prelude::*;

use crate::prelude::*;

/// The type of timestep used for the [`Time<Physics>`](Physics) clock.
#[derive(Reflect, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub enum TimestepMode {
    /// **Fixed timestep**: The physics simulation will be advanced by a fixed `delta`
    /// amount of time every frame until the accumulated `overstep` value has been consumed.
    /// This means that physics can run 0, 1, 2 or more times per frame based on how long the
    /// previous frames took.
    ///
    /// To avoid "death spirals" where each frame takes longer and longer
    /// to simulate, `overstep` can only be advanced by `max_delta_overstep`
    /// during a single frame.
    ///
    /// A fixed timestep allows consistent behavior across different machines and frame rates.
    Fixed {
        /// The amount of time that the simulation should be advanced by during a step.
        delta: Duration,
        /// The amount of accumulated time. The simulation will consume it in steps of `delta`
        /// to try to catch up to real time.
        overstep: Duration,
        /// The maximum amount of time that can be added to `overstep` during a single frame.
        /// Lower values help prevent "death spirals" where each frame takes longer and longer
        /// to simulate.
        ///
        /// Defaults to `1.0 / 60.0` seconds (60 Hz).
        max_delta_overstep: Duration,
    },
    /// **Fixed delta, once per frame**: The physics simulation will be advanced by
    /// a fixed `delta` amount of time once per frame. This should only be used
    /// in cases where you can guarantee a fixed number of executions,
    /// like in `FixedUpdate` or on a server.
    FixedOnce {
        /// The amount of time that the simulation should be advanced by during a step.
        delta: Duration,
    },
    /// **Variable timestep**: The physics simulation will be advanced by
    /// `Time::delta_seconds().min(max_delta)` seconds at each Bevy tick.
    /// Frame rate will affect the simulation result.
    Variable {
        /// The maximum amount of time the physics simulation can be advanced at once.
        /// This makes sure that the simulation doesn't break when the delta time is large.
        ///
        /// A good default is `1.0 / 60.0` seconds (60 Hz).
        max_delta: Duration,
    },
}

impl Default for TimestepMode {
    fn default() -> Self {
        Self::Fixed {
            delta: Duration::default(),
            overstep: Duration::default(),
            max_delta_overstep: Duration::from_secs_f64(1.0 / 60.0),
        }
    }
}

/// The clock representing physics time, following `Time<Real>`.
/// Can be configured to use a fixed or variable timestep.
///
/// The clock is automatically set as the generic `Time` resource for
/// the [`PhysicsSchedule`].
///
/// By default, a fixed timestep of 60 Hz is used.
///
/// ## Usage
///
/// The timestep used for advancing the simulation can be configured by overwriting
/// the [`Time<Physics>`](Physics) resource:
///
/// ```no_run
/// use bevy::{prelude::*, utils::Duration};
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn main() {
///     App::new()
///         .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
///         .insert_resource(Time::new_with(Physics::fixed_hz(144.0)))
///         .run();
/// }
///```
///
/// In the [`PhysicsSchedule`], the generic `Time` resource is automatically
/// replaced by [`Time<Physics>`](Physics), so time works in a unified
/// way across schedules:
///
/// ```
/// # use bevy::prelude::*;
/// #
/// // In `Update`, `Time` is `Time<Virtual>`, but in `PhysicsSchedule` it's `Time<Physics>`
/// fn print_delta_time(time: Res<Time>) {
///     println!("{}", time.delta_seconds());
/// }
/// ```
///
/// ### Physics speed
///
/// The relative speed of physics can be configured at startup using
/// [`with_relative_speed`](PhysicsTime::with_relative_speed) or when the app is running
/// using [`set_relative_speed`](PhysicsTime::set_relative_speed):
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
///         // Run physics at 0.5 speed
///         .insert_resource(Time::<Physics>::default().with_relative_speed(0.5))
///         .run();
/// }
///```
///
/// ### Pausing, resuming and stepping physics
///
/// [`Time<Physics>`](Physics) can be used to pause and resume the simulation:
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn pause(mut time: ResMut<Time<Physics>>) {
///     time.pause();
/// }
///
/// fn unpause(mut time: ResMut<Time<Physics>>) {
///     time.unpause();
/// }
/// ```
///
/// To advance the simulation by a certain amount of time instantly, you can advance the
/// [`Time<Physics>`](Physics) clock and manually run the [`PhysicsSchedule`] in an exclusive system:
///
/// ```
/// use bevy::{prelude::*, utils::Duration};
#[cfg_attr(
    feature = "2d",
    doc = "use bevy_xpbd_2d::{prelude::*, PhysicsSchedule};"
)]
#[cfg_attr(
    feature = "3d",
    doc = "use bevy_xpbd_3d::{prelude::*, PhysicsSchedule};"
)]
///
/// fn run_physics(world: &mut World) {
///     // Advance the simulation by 10 steps at 120 Hz
///     for _ in 0..10 {
///         world
///             .resource_mut::<Time<Physics>>()
///             .advance_by(Duration::from_secs_f64(1.0 / 120.0));
///         world.run_schedule(PhysicsSchedule);
///     }
/// }
/// ```
///
/// ## When to multiply by delta time?
///
/// Schedules like `Update` use a variable timestep, which can often cause frame rate dependent
/// behavior when moving bodies. One way to help address the issue is by multiplying by delta time.
///
/// In general, if you're doing a *continuous* operation, you should always multiply by delta time,
/// but for *instantaneous* operations it's not necessary.
///
/// Continuous operations move or accelerate bodies over time:
///
/// ```
/// # use bevy::math::Vec3;
/// #
/// # let mut position = Vec3::default();
/// # let mut velocity = Vec3::default();
/// # let mut acceleration = Vec3::default();
/// # let mut delta_time = 1.0 / 60.0;
/// #
/// // Move continuously over time
/// position += velocity * delta_time;
/// // Accelerate continuously
/// velocity += acceleration * delta_time;
/// ```
///
/// Instantaneous operations apply a singular sudden burst of velocity
/// or set it to a specific value:
///
/// ```
/// # use bevy::math::Vec3;
/// #
/// # let mut velocity = Vec3::default();
/// # let mut impulse = Vec3::default();
/// # let mut target_velocity = Vec3::default();
/// #
/// // Apply a burst of speed once (jumps, explosions and so on)
/// velocity += impulse;
/// // Set velocity to a specific value
/// velocity = target_velocity;
/// ```
///
/// For systems using a fixed timestep, using delta time is not necessary for frame rate
/// independence, but it's still recommended so that the physical units are more logical.

#[derive(Reflect, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Physics {
    timestep_mode: TimestepMode,
    paused: bool,
    relative_speed: f64,
}

impl Default for Physics {
    fn default() -> Self {
        // Corresponds to 60 Hz.
        // TODO: Bevy's fixed timestep is 64 Hz, but it causes physics
        //       to be run twice in a single frame every 0.25 seconds.
        //       It would be nice to have the timestep be more unified though.
        Self::fixed_hz(60.0)
    }
}

impl Physics {
    /// Creates a new [`Physics`] clock with the given type of [timestep](TimestepMode).
    pub const fn from_timestep(timestep_mode: TimestepMode) -> Self {
        Self {
            timestep_mode,
            paused: false,
            relative_speed: 1.0,
        }
    }

    /// Returns a new [`Time<Physics>`](Physics) clock with a [`TimestepMode::Fixed`]
    /// using the given frequency in Hertz (1/second).
    ///
    /// # Panics
    ///
    /// Panics if `hz` is zero, negative or not finite.
    pub fn fixed_hz(hz: f64) -> Self {
        assert!(hz > 0.0, "Hz less than or equal to zero");
        assert!(hz.is_finite(), "Hz is infinite");
        Self::from_timestep(TimestepMode::Fixed {
            delta: Duration::from_secs_f64(1.0 / hz),
            overstep: Duration::ZERO,
            max_delta_overstep: Duration::from_secs_f64(1.0 / 60.0),
        })
    }

    /// Returns a new [`Time<Physics>`](Physics) clock with a [`TimestepMode::FixedOnce`]
    /// using the given frequency in Hertz (1/second).
    ///
    /// Unlike with [`TimestepMode::Fixed`], the [`PhysicsSchedule`] will only be run once per frame
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
        Self::from_timestep(TimestepMode::FixedOnce {
            delta: Duration::from_secs_f64(1.0 / hz),
        })
    }

    /// Returns a new [`Time<Physics>`](Physics) clock with a [`TimestepMode::Variable`]
    /// using the given maximum duration that the simulation can be advanced by during
    /// a single frame in seconds.
    ///
    /// # Panics
    ///
    /// Panics if `max_delta_seconds` is zero or negative.
    pub fn variable(max_delta_seconds: f64) -> Self {
        assert!(
            max_delta_seconds > 0.0,
            "max delta less than or equal to zero"
        );
        Self::from_timestep(TimestepMode::Variable {
            max_delta: Duration::from_secs_f64(max_delta_seconds),
        })
    }
}

/// An extension trait for [`Time<Physics>`](Physics).
pub trait PhysicsTime {
    /// Creates a new [`Time<Physics>`](Physics) clock with the given type
    /// of [timestep](TimestepMode).
    fn from_timestep(timestep_mode: TimestepMode) -> Self;

    /// Gets the type of [timestep](TimestepMode) used for running physics.
    fn timestep_mode(&self) -> TimestepMode;

    /// Mutably gets the type of [timestep](TimestepMode) used for running physics.
    fn timestep_mode_mut(&mut self) -> &mut TimestepMode;

    /// Sets the type of [timestep](TimestepMode) used for running physics.
    fn set_timestep_mode(&mut self, timestep_mode: TimestepMode);

    /// Returns the speed of physics relative to your system clock as an `f32`.
    /// This is also known as "time scaling" or "time dilation" in other engines.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your [timestep](`TimestepMode`) at the cost of performance.
    fn relative_speed(&self) -> f32;

    /// Returns the speed of physics relative to your system clock as an `f64`.
    /// This is also known as "time scaling" or "time dilation" in other engines.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your [timestep](`TimestepMode`) at the cost of performance.
    fn relative_speed_f64(&self) -> f64;

    /// Sets the speed of physics relative to your system clock, given as an `f32`.
    ///
    /// For example, setting this to `2.0` will make the physics clock advance twice
    /// as fast as your system clock.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your [timestep](`TimestepMode`) at the cost of performance.
    ///
    /// # Panics
    ///
    /// Panics if `ratio` is negative or not finite.
    fn with_relative_speed(self, ratio: f32) -> Self;

    /// Sets the speed of physics relative to your system clock, given as an `f64`.
    ///
    /// For example, setting this to `2.0` will make the physics clock advance twice
    /// as fast as your system clock.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your [timestep](`TimestepMode`) at the cost of performance.
    ///
    /// # Panics
    ///
    /// Panics if `ratio` is negative or not finite.
    fn with_relative_speed_f64(self, ratio: f64) -> Self;

    /// Sets the speed of physics relative to your system clock, given as an `f32`.
    ///
    /// For example, setting this to `2.0` will make the physics clock advance twice
    /// as fast as your system clock.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your [timestep](`TimestepMode`) at the cost of performance.
    ///
    /// # Panics
    ///
    /// Panics if `ratio` is negative or not finite.
    fn set_relative_speed(&mut self, ratio: f32);

    /// Sets the speed of physics relative to your system clock, given as an `f64`.
    ///
    /// For example, setting this to `2.0` will make the physics clock advance twice
    /// as fast as your system clock.
    ///
    /// The speed impacts the accuracy of the simulation, and large values may
    /// cause jittering or missed collisions. You can improve simulation consistency
    /// by adjusting your [timestep](`TimestepMode`) at the cost of performance.
    ///
    /// # Panics
    ///
    /// Panics if `ratio` is negative or not finite.
    fn set_relative_speed_f64(&mut self, ratio: f64);

    /// Returns `true` if the physics clock is currently paused.
    fn is_paused(&self) -> bool;

    /// Stops the clock, preventing the physics simulation from advancing until resumed.
    fn pause(&mut self);

    /// Resumes the clock if paused.
    #[doc(alias = "resume")]
    fn unpause(&mut self);
}

impl PhysicsTime for Time<Physics> {
    fn from_timestep(timestep_mode: TimestepMode) -> Self {
        Self::new_with(Physics::from_timestep(timestep_mode))
    }

    fn timestep_mode(&self) -> TimestepMode {
        self.context().timestep_mode
    }

    fn timestep_mode_mut(&mut self) -> &mut TimestepMode {
        &mut self.context_mut().timestep_mode
    }

    fn set_timestep_mode(&mut self, timestep_mode: TimestepMode) {
        self.context_mut().timestep_mode = timestep_mode;
    }

    fn with_relative_speed(self, ratio: f32) -> Self {
        self.with_relative_speed_f64(ratio as f64)
    }

    fn with_relative_speed_f64(mut self, ratio: f64) -> Self {
        assert!(ratio.is_finite(), "tried to go infinitely fast");
        assert!(ratio >= 0.0, "tried to go back in time");
        self.context_mut().relative_speed = ratio;
        self
    }

    fn relative_speed(&self) -> f32 {
        self.relative_speed_f64() as f32
    }

    fn relative_speed_f64(&self) -> f64 {
        self.context().relative_speed
    }

    fn set_relative_speed(&mut self, ratio: f32) {
        self.set_relative_speed_f64(ratio as f64);
    }

    fn set_relative_speed_f64(&mut self, ratio: f64) {
        assert!(ratio.is_finite(), "tried to go infinitely fast");
        assert!(ratio >= 0.0, "tried to go back in time");
        self.context_mut().relative_speed = ratio;
    }

    fn pause(&mut self) {
        self.context_mut().paused = true;
    }

    fn unpause(&mut self) {
        self.context_mut().paused = false;
    }

    fn is_paused(&self) -> bool {
        self.context().paused
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
