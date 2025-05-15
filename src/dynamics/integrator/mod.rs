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
                integrate_velocities.in_set(IntegrationSet::Velocity),
                integrate_positions.in_set(IntegrationSet::Position),
            ),
        );

        #[cfg(feature = "3d")]
        app.add_systems(
            self.schedule.intern(),
            dynamics::rigid_body::mass_properties::update_global_angular_inertia::<()>
                .in_set(IntegrationSet::Position)
                .after(integrate_positions),
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
/// - Velocity increments for forces, torques, and accelerations applied using the [`ForceHelper`].
/// - Cached operands for applying linear and angular velocity damping.
///
/// The values are computed once per time step, and applied to the body at each substep
/// with basic addition and multiplication. This moves the more expensive operations
/// and branching out of the substepping loop.
// -----------------------
// 18 bytes in 2D with f32
// 32 bytes in 3D with f32
// TODO: We could make the 2D version 16 bytes if we stored the damping rhs values
//       in a separate `VelocityDampingRhs` component. Not sure if it's worth it though.
// TODO: We could technically make the semantics such that the increments are actually
//       acceleration until `IntegrationSet::UpdateVelocityIncrements`. This way,
//       we would multiply by `delta_time` only once, rather than for each acceleration.
#[derive(Component, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct VelocityIntegrationData {
    /// The linear velocity increment to be applied to the body at each substep.
    pub linear_increment: Vector,
    /// The angular velocity increment to be applied to the body at each substep.
    pub angular_increment: AngularValue,
    /// The right-hand side of the linear damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub linear_damping_rhs: f32,
    /// The right-hand side of the angular damping equation,
    /// `1 / (1 + dt * c)`, where `c` is the damping coefficient.
    pub angular_damping_rhs: f32,
}

impl VelocityIntegrationData {
    /// Applies a given linear acceleration to the body, taking into account locked axes.
    pub fn apply_linear_acceleration(&mut self, acceleration: Vector, delta_secs: Scalar) {
        self.linear_increment += acceleration * delta_secs;
    }

    /// Applies a given angular acceleration to the body, taking into account locked axes.
    pub fn apply_angular_acceleration(&mut self, acceleration: AngularValue, delta_secs: Scalar) {
        self.angular_increment += acceleration * delta_secs;
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
fn pre_process_velocity_increments(
    mut bodies: Query<
        (
            &mut VelocityIntegrationData,
            Option<&LinearDamping>,
            Option<&AngularDamping>,
            Option<&GravityScale>,
            Option<&LockedAxes>,
        ),
        With<SolverBody>,
    >,
    gravity: Res<Gravity>,
    time: Res<Time<Substeps>>,
    mut solver_diagnostics: ResMut<SolverDiagnostics>,
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

            // Apply gravity.
            integration.linear_increment +=
                gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0) * delta_secs;

            // Apply locked axes.
            integration.linear_increment = locked_axes.apply_to_vec(integration.linear_increment);
            integration.angular_increment =
                locked_axes.apply_to_angular_velocity(integration.angular_increment);
        },
    );

    solver_diagnostics.update_velocity_increments += start.elapsed();
}

fn clear_velocity_increments(
    mut bodies: Query<&mut VelocityIntegrationData, With<SolverBody>>,
    mut solver_diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    bodies.par_iter_mut().for_each(|mut integration| {
        integration.linear_increment = Vector::ZERO;
        integration.angular_increment = AngularValue::ZERO;
    });

    solver_diagnostics.update_velocity_increments += start.elapsed();
}

#[derive(QueryData)]
#[query_data(mutable)]
struct VelocityIntegrationQuery {
    solver_body: &'static mut SolverBody,
    integration: &'static mut VelocityIntegrationData,
    #[cfg(feature = "3d")]
    ang_inertia: &'static ComputedAngularInertia,
    #[cfg(feature = "3d")]
    rotation: &'static Rotation,
    #[cfg(feature = "3d")]
    locked_axes: Option<&'static LockedAxes>,
    max_linear_speed: Option<&'static MaxLinearSpeed>,
    max_angular_speed: Option<&'static MaxAngularSpeed>,
}

#[allow(clippy::type_complexity)]
fn integrate_velocities(
    mut bodies: Query<VelocityIntegrationQuery, RigidBodyActiveFilter>,
    mut diagnostics: ResMut<SolverDiagnostics>,
    #[cfg(feature = "3d")] time: Res<Time>,
) {
    let start = crate::utils::Instant::now();

    #[cfg(feature = "3d")]
    let delta_secs = time.delta_secs_f64() as Scalar;

    bodies.par_iter_mut().for_each(|body| {
        #[cfg(feature = "3d")]
        let is_gyroscopic = body.solver_body.is_gyroscopic();

        let SolverBody {
            linear_velocity: lin_vel,
            angular_velocity: ang_vel,
            #[cfg(feature = "3d")]
            delta_rotation,
            ..
        } = body.solver_body.into_inner();

        // Apply velocity damping.
        *lin_vel *= body.integration.linear_damping_rhs;
        *ang_vel *= body.integration.angular_damping_rhs;

        // Apply velocity increments.
        *lin_vel += body.integration.linear_increment;
        *ang_vel += body.integration.angular_increment;

        #[cfg(feature = "3d")]
        {
            if is_gyroscopic {
                let locked_axes = body
                    .locked_axes
                    .map_or(LockedAxes::default(), |locked_axes| *locked_axes);

                // TODO: Should this be opt-in with a `GyroscopicMotion` component?
                // TODO: It's a bit unfortunate that this has to run in the substepping loop
                //       rather than pre-computing the velocity increments once per time step.
                //       This needs to be done because the gyroscopic torque relies on up-to-date
                //       rotations and world-space angular inertia tensors. Omitting the change
                //       in orientaton would lead to angular momentum not being conserved.
                semi_implicit_euler::solve_gyroscopic_torque(
                    ang_vel,
                    delta_rotation.0 * body.rotation.0,
                    body.ang_inertia,
                    delta_secs,
                );
                *ang_vel = locked_axes.apply_to_angular_velocity(*ang_vel);
            }
        }

        // Clamp velocities.
        // TODO: Should we do this in a separate system?
        if let Some(max_linear_speed) = body.max_linear_speed {
            let linear_speed_squared = lin_vel.length_squared();
            if linear_speed_squared > max_linear_speed.0.powi(2) {
                *lin_vel *= max_linear_speed.0 / linear_speed_squared.sqrt();
            }
        }
        if let Some(max_angular_speed) = body.max_angular_speed {
            #[cfg(feature = "2d")]
            if ang_vel.abs() > max_angular_speed.0 {
                *ang_vel = max_angular_speed.copysign(*ang_vel);
            }
            #[cfg(feature = "3d")]
            {
                let angular_speed_squared = ang_vel.length_squared();
                if angular_speed_squared > max_angular_speed.0.powi(2) {
                    *ang_vel *= max_angular_speed.0 / angular_speed_squared.sqrt();
                }
            }
        }
    });

    diagnostics.integrate_velocities += start.elapsed();
}

#[allow(clippy::type_complexity)]
fn integrate_positions(
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
            delta_rotation.0 *= Quaternion::from_scaled_axis(*angular_velocity * delta_secs);
        }
    });

    diagnostics.integrate_positions += start.elapsed();
}
