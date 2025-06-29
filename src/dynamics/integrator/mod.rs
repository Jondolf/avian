//! Applies forces and velocities to bodies in order to move them according to the equations of motion
//! using numerical integration.
//!
//! See [`IntegratorPlugin`].

#[doc(alias = "symplectic_euler")]
pub mod semi_implicit_euler;

use crate::prelude::*;
use bevy::{
    ecs::{intern::Interned, query::QueryData, schedule::ScheduleLabel},
    prelude::*,
};
use dynamics::solver::SolverDiagnostics;

use super::solver::solver_body::SolverBody;

/// Integrates Newton's 2nd law of motion, applying forces and moving entities according to their velocities.
///
/// This acts as a prediction for the next positions and orientations of the bodies. The [solver](dynamics::solver)
/// corrects these predicted positions to take constraints like contacts and joints into account.
///
/// Currently, only the [semi-implicit (symplectic) Euler](semi_implicit_euler) integration scheme
/// is supported. It is the standard for game physics, being simple, efficient, and sufficiently accurate.
///
/// The plugin adds systems in the [`IntegrationSet::Velocity`] and [`IntegrationSet::Position`] system sets.
pub struct IntegratorPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl IntegratorPlugin {
    /// Creates an [`IntegratorPlugin`] with the schedule that the integration systems should run in.
    ///
    /// The default schedule is [`SubstepSchedule`].
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for IntegratorPlugin {
    fn default() -> Self {
        Self::new(SubstepSchedule)
    }
}

impl Plugin for IntegratorPlugin {
    fn build(&self, app: &mut App) {
        // Add `VelocityIntegrationData` to all `SolverBody`s.
        app.register_required_components::<SolverBody, VelocityIntegrationData>();

        // Remove `VelocityIntegrationData` when `SolverBody` is removed.
        app.add_observer(
            |trigger: Trigger<OnRemove, SolverBody>, mut commands: Commands| {
                commands
                    .entity(trigger.target())
                    .try_remove::<VelocityIntegrationData>();
            },
        );

        app.init_resource::<Gravity>();

        app.configure_sets(
            PhysicsSchedule,
            (
                IntegrationSet::UpdateVelocityIncrements
                    .in_set(SolverSet::PreSubstep)
                    .before(IntegrationSet::Velocity),
                IntegrationSet::ClearVelocityIncrements
                    .in_set(SolverSet::PostSubstep)
                    .after(IntegrationSet::Velocity),
            ),
        );

        app.add_systems(
            PhysicsSchedule,
            (
                pre_process_velocity_increments.in_set(IntegrationSet::UpdateVelocityIncrements),
                clear_velocity_increments.in_set(IntegrationSet::ClearVelocityIncrements),
            ),
        );

        app.configure_sets(
            self.schedule.intern(),
            (IntegrationSet::Velocity, IntegrationSet::Position).chain(),
        );

        app.add_systems(
            self.schedule.intern(),
            (
                (integrate_velocities, clamp_velocities)
                    .chain()
                    .in_set(IntegrationSet::Velocity),
                integrate_positions.in_set(IntegrationSet::Position),
            ),
        );
    }
}

/// System sets for position and velocity integration,
/// applying forces and moving bodies based on velocity.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum IntegrationSet {
    /// Applies gravity and locked axes to the linear and angular velocity increments of bodies,
    /// and multiplies them by the substep delta time to get the final per-substep increments.
    UpdateVelocityIncrements,
    /// Applies velocity increments to the linear and angular velocities of bodies.
    Velocity,
    /// Moves bodies based on their current velocities and the physics time step.
    Position,
    /// Clears the velocity increments of bodies after the substepping loop.
    ClearVelocityIncrements,
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
/// # Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
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
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Resource)]
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

#[cfg(feature = "2d")]
type AngularValue = Scalar;
#[cfg(feature = "3d")]
type AngularValue = Vector;
#[cfg(feature = "2d")]
type TorqueValue = Scalar;
#[cfg(feature = "3d")]
type TorqueValue = Vector;

/// Pre-computed data for speeding up velocity integration.
///
/// This includes:
///
/// - Velocity increments for gravity.
/// - Velocity increments for [`ConstantForce`] and [`ConstantTorque`].
/// - Velocity increments for forces, torques, and accelerations applied using [`RigidBodyForces`].
/// - Cached operands for applying linear and angular velocity damping.
///
/// The values are computed once per time step, and applied to the body at each substep
/// with basic addition and multiplication. This moves the more expensive operations
/// and branching out of the substepping loop.
// -----------------------
// 20 bytes in 2D with f32
// 32 bytes in 3D with f32
#[derive(Component, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct VelocityIntegrationData {
    /// The linear velocity increment to be applied to the body at each substep.
    ///
    /// **Note:** This is treated as linear acceleration until [`IntegrationSet::UpdateVelocityIncrements`].
    /// where it is multiplied by the time step to get the corresponding velocity increment.
    pub linear_increment: Vector,
    /// The angular velocity increment to be applied to the body at each substep.
    ///
    /// **Note:** This is treated as angular acceleration until [`IntegrationSet::UpdateVelocityIncrements`].
    /// where it is multiplied by the time step to get the corresponding velocity increment.
    pub angular_increment: AngularValue,
    /// The right-hand side of the linear damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub linear_damping_rhs: f32,
    /// The right-hand side of the angular damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub angular_damping_rhs: f32,
}

impl VelocityIntegrationData {
    /// Applies a given linear acceleration to the body.
    pub fn apply_linear_acceleration(&mut self, acceleration: Vector) {
        self.linear_increment += acceleration;
    }

    /// Applies a given angular acceleration to the body.
    pub fn apply_angular_acceleration(&mut self, acceleration: AngularValue) {
        self.angular_increment += acceleration;
    }

    /// Updates the cached right-hand side of the linear damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub fn update_linear_damping_rhs(&mut self, damping_coefficient: f32, delta_secs: Scalar) {
        self.linear_damping_rhs = 1.0 / (1.0 + delta_secs * damping_coefficient);
    }

    /// Updates the cached right-hand side of the angular damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub fn update_angular_damping_rhs(&mut self, damping_coefficient: f32, delta_secs: Scalar) {
        self.angular_damping_rhs = 1.0 / (1.0 + delta_secs * damping_coefficient);
    }
}

/// Applies gravity and locked axes to the linear and angular velocity increments of bodies.
pub fn pre_process_velocity_increments(
    mut bodies: Query<(
        &mut VelocityIntegrationData,
        Option<&LinearDamping>,
        Option<&AngularDamping>,
        Option<&GravityScale>,
        Option<&LockedAxes>,
    )>,
    gravity: Res<Gravity>,
    time: Res<Time<Substeps>>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_secs_f64() as Scalar;

    // TODO: Do we want to skip kinematic bodies here?
    bodies.par_iter_mut().for_each(
        |(mut integration, lin_damping, ang_damping, gravity_scale, locked_axes)| {
            let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

            // Update the cached right-hand side of the velocity damping equation,
            // `1 / (1 + dt * c)`, where `c` is the damping coefficient.
            let lin_damping = lin_damping.map_or(0.0, |damping| damping.0);
            let ang_damping = ang_damping.map_or(0.0, |damping| damping.0);
            integration.update_linear_damping_rhs(lin_damping, delta_secs);
            integration.update_angular_damping_rhs(ang_damping, delta_secs);

            // NOTE: The `ForcePlugin` handles the application of external forces and torques.
            // NOTE: The velocity increments are treated as accelerations at this point.

            // Apply gravity.
            integration.linear_increment += gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0);

            // Apply locked axes.
            integration.linear_increment = locked_axes.apply_to_vec(integration.linear_increment);
            integration.angular_increment =
                locked_axes.apply_to_angular_velocity(integration.angular_increment);

            // The velocity increments are treated as accelerations until this point.
            // Multiply by the time step to get the final velocity increments.
            integration.linear_increment *= delta_secs;
            integration.angular_increment *= delta_secs;
        },
    );

    diagnostics.update_velocity_increments += start.elapsed();
}

/// Clears the velocity increments of bodies after the substepping loop.
fn clear_velocity_increments(
    mut bodies: Query<&mut VelocityIntegrationData, With<SolverBody>>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    bodies.par_iter_mut().for_each(|mut integration| {
        integration.linear_increment = Vector::ZERO;
        integration.angular_increment = AngularValue::ZERO;
    });

    diagnostics.update_velocity_increments += start.elapsed();
}

#[derive(QueryData)]
#[query_data(mutable)]
#[doc(hidden)]
pub struct VelocityIntegrationQuery {
    solver_body: &'static mut SolverBody,
    integration: &'static mut VelocityIntegrationData,
    #[cfg(feature = "3d")]
    angular_inertia: &'static ComputedAngularInertia,
    #[cfg(feature = "3d")]
    rotation: &'static Rotation,
}

/// Integrates the velocities of bodies by applying velocity increments and damping.
pub fn integrate_velocities(
    mut bodies: Query<VelocityIntegrationQuery, RigidBodyActiveFilter>,
    mut diagnostics: ResMut<SolverDiagnostics>,
    #[cfg(feature = "3d")] time: Res<Time>,
) {
    let start = crate::utils::Instant::now();

    #[cfg(feature = "3d")]
    let delta_secs = time.delta_secs_f64() as Scalar;

    bodies.par_iter_mut().for_each(|mut body| {
        // Apply velocity damping.
        body.solver_body.linear_velocity *= body.integration.linear_damping_rhs;
        body.solver_body.angular_velocity *= body.integration.angular_damping_rhs;

        // Apply velocity increments.
        body.solver_body.linear_velocity += body.integration.linear_increment;
        body.solver_body.angular_velocity += body.integration.angular_increment;

        #[cfg(feature = "3d")]
        {
            if body.solver_body.is_gyroscopic() {
                // TODO: Should this be opt-in with a `GyroscopicMotion` component?
                // TODO: It's a bit unfortunate that this has to run in the substepping loop
                //       rather than pre-computing the velocity increments once per time step.
                //       This needs to be done because the gyroscopic torque relies on up-to-date rotations
                //       and world-space angular inertia tensors. Omitting the change in orientaton would
                //       lead to worse accuracy and angular momentum not being conserved.
                let rotation = body.solver_body.delta_rotation.0 * body.rotation.0;
                semi_implicit_euler::solve_gyroscopic_torque(
                    &mut body.solver_body.angular_velocity,
                    rotation,
                    body.angular_inertia,
                    delta_secs,
                );
            }
        }
    });

    diagnostics.integrate_velocities += start.elapsed();
}

// NOTE: If the majority of bodies have clamped velocities, it would be more efficient
//       to do this in `integrate_velocities` rather than in a separate system.
//       By doing this in a separate system, we're optimizing for the assumption
//       that only some bodies have clamped velocities.
/// Clamps the velocities of bodies to [`MaxLinearSpeed`] and [`MaxAngularSpeed`].
fn clamp_velocities(
    mut bodies: ParamSet<(
        Query<(&mut SolverBody, &MaxLinearSpeed)>,
        Query<(&mut SolverBody, &MaxAngularSpeed)>,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    // Clamp linear velocity.
    bodies.p0().iter_mut().for_each(|(mut body, max_speed)| {
        let linear_speed_squared = body.linear_velocity.length_squared();
        if linear_speed_squared > max_speed.0 * max_speed.0 {
            body.linear_velocity *= max_speed.0 / linear_speed_squared.sqrt();
        }
    });

    // Clamp angular velocity.
    bodies.p1().iter_mut().for_each(|(mut body, max_speed)| {
        #[cfg(feature = "2d")]
        if body.angular_velocity.abs() > max_speed.0 {
            body.angular_velocity = max_speed.copysign(body.angular_velocity);
        }
        #[cfg(feature = "3d")]
        {
            let angular_speed_squared = body.angular_velocity.length_squared();
            if angular_speed_squared > max_speed.0 * max_speed.0 {
                body.angular_velocity *= max_speed.0 / angular_speed_squared.sqrt();
            }
        }
    });

    diagnostics.integrate_velocities += start.elapsed();
}

/// Integrates the positions of bodies based on their velocities and the time step.
pub fn integrate_positions(
    mut solver_bodies: Query<&mut SolverBody>,
    time: Res<Time>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_seconds_adjusted();

    solver_bodies.par_iter_mut().for_each(|body| {
        let SolverBody {
            linear_velocity,
            angular_velocity,
            delta_position,
            delta_rotation,
            ..
        } = body.into_inner();

        *delta_position += *linear_velocity * delta_secs;
        #[cfg(feature = "2d")]
        {
            *delta_rotation = delta_rotation.add_angle_fast(*angular_velocity * delta_secs);
        }
        #[cfg(feature = "3d")]
        {
            delta_rotation.0 =
                Quaternion::from_scaled_axis(*angular_velocity * delta_secs) * delta_rotation.0;
        }
    });

    diagnostics.integrate_positions += start.elapsed();
}
