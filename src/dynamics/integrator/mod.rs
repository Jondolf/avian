//! Applies forces and velocities to bodies in order to move them according to the equations of motion
//! using numerical integration.
//!
//! See [`IntegratorPlugin`].

#[doc(alias = "symplectic_euler")]
pub mod semi_implicit_euler;

use crate::prelude::*;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

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
    /// Creates a [`IntegratorPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
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
        app.init_resource::<Gravity>();

        app.configure_sets(
            self.schedule.intern(),
            (IntegrationSet::Velocity, IntegrationSet::Position).chain(),
        );

        app.get_schedule_mut(self.schedule.intern())
            .expect("add SubstepSchedule first")
            .add_systems((
                integrate_velocities.in_set(IntegrationSet::Velocity),
                integrate_positions.in_set(IntegrationSet::Position),
            ));

        app.get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first")
            .add_systems(
                (
                    apply_impulses.before(SolverSet::Substep),
                    clear_forces_and_impulses.after(SolverSet::Substep),
                )
                    .in_set(PhysicsStepSet::Solver),
            );
    }
}

/// System sets for position and velocity integration,
/// applying forces and moving bodies based on velocity.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum IntegrationSet {
    /// Applies gravity and external forces to bodies, updating their velocities.
    Velocity,
    /// Moves bodies based on their current velocities and the physics time step.
    Position,
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

#[allow(clippy::type_complexity)]
fn integrate_velocities(
    mut bodies: Query<
        (
            &RigidBody,
            &Position,
            Option<&mut PreSolveAccumulatedTranslation>,
            &Rotation,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &ExternalForce,
            &ExternalTorque,
            &InverseMass,
            &InverseInertia,
            Option<&LinearDamping>,
            Option<&AngularDamping>,
            Option<&GravityScale>,
            Option<&LockedAxes>,
        ),
        Without<Sleeping>,
    >,
    gravity: Res<Gravity>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    bodies.par_iter_mut().for_each(
        |(
            rb,
            pos,
            prev_pos,
            rot,
            mut lin_vel,
            mut ang_vel,
            force,
            torque,
            inv_mass,
            inv_inertia,
            lin_damping,
            ang_damping,
            gravity_scale,
            locked_axes,
        )| {
            if let Some(mut previous_position) = prev_pos {
                previous_position.0 = pos.0;
            }

            if rb.is_static() {
                if *lin_vel != LinearVelocity::ZERO {
                    *lin_vel = LinearVelocity::ZERO;
                }
                if *ang_vel != AngularVelocity::ZERO {
                    *ang_vel = AngularVelocity::ZERO;
                }
                return;
            }

            if rb.is_kinematic() {
                return;
            }

            let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

            // Apply damping
            if rb.is_dynamic() {
                if let Some(lin_damping) = lin_damping {
                    if lin_vel.0 != Vector::ZERO && lin_damping.0 != 0.0 {
                        lin_vel.0 *= 1.0 / (1.0 + delta_secs * lin_damping.0);
                    }
                }
                if let Some(ang_damping) = ang_damping {
                    if ang_vel.0 != AngularVelocity::ZERO.0 && ang_damping.0 != 0.0 {
                        ang_vel.0 *= 1.0 / (1.0 + delta_secs * ang_damping.0);
                    }
                }
            }

            let external_force = force.force();
            let external_torque = torque.torque() + force.torque();
            let gravity = gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0);

            semi_implicit_euler::integrate_velocity(
                &mut lin_vel.0,
                &mut ang_vel.0,
                external_force,
                external_torque,
                inv_mass.0,
                *inv_inertia,
                *rot,
                locked_axes,
                gravity,
                delta_secs,
            );
        },
    );
}

#[allow(clippy::type_complexity)]
fn integrate_positions(
    mut bodies: Query<
        (
            &RigidBody,
            &Position,
            Option<&mut PreSolveAccumulatedTranslation>,
            &mut AccumulatedTranslation,
            &mut Rotation,
            &LinearVelocity,
            &AngularVelocity,
            Option<&LockedAxes>,
        ),
        Without<Sleeping>,
    >,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    bodies.par_iter_mut().for_each(
        |(
            rb,
            pos,
            pre_solve_accumulated_translation,
            mut accumulated_translation,
            mut rot,
            lin_vel,
            ang_vel,
            locked_axes,
        )| {
            if let Some(mut previous_position) = pre_solve_accumulated_translation {
                previous_position.0 = pos.0;
            }

            if rb.is_static() || (lin_vel.0 == Vector::ZERO && *ang_vel == AngularVelocity::ZERO) {
                return;
            }

            let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

            semi_implicit_euler::integrate_position(
                &mut accumulated_translation.0,
                &mut rot,
                lin_vel.0,
                ang_vel.0,
                locked_axes,
                delta_secs,
            );
        },
    );
}

#[cfg(feature = "2d")]
type AngularValue = Scalar;
#[cfg(feature = "3d")]
type AngularValue = Vector;
#[cfg(feature = "2d")]
type TorqueValue = Scalar;
#[cfg(feature = "3d")]
type TorqueValue = Vector;
#[cfg(feature = "2d")]
type InertiaValue = Scalar;
#[cfg(feature = "3d")]
type InertiaValue = Matrix3;

type ImpulseQueryComponents = (
    &'static RigidBody,
    &'static mut ExternalImpulse,
    &'static mut ExternalAngularImpulse,
    &'static mut LinearVelocity,
    &'static mut AngularVelocity,
    &'static Rotation,
    &'static InverseMass,
    &'static InverseInertia,
    Option<&'static LockedAxes>,
);

fn apply_impulses(mut bodies: Query<ImpulseQueryComponents, Without<Sleeping>>) {
    for (
        rb,
        impulse,
        ang_impulse,
        mut lin_vel,
        mut ang_vel,
        rotation,
        inv_mass,
        inv_inertia,
        locked_axes,
    ) in &mut bodies
    {
        if !rb.is_dynamic() {
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        let effective_inv_mass = locked_axes.apply_to_vec(Vector::splat(inv_mass.0));
        let effective_inv_inertia = locked_axes.apply_to_rotation(inv_inertia.rotated(rotation).0);

        // Avoid triggering bevy's change detection unnecessarily.
        let delta_lin_vel = impulse.impulse() * effective_inv_mass;
        let delta_ang_vel =
            effective_inv_inertia * (ang_impulse.impulse() + impulse.angular_impulse());

        if delta_lin_vel != Vector::ZERO {
            lin_vel.0 += delta_lin_vel;
        }
        if delta_ang_vel != AngularVelocity::ZERO.0 {
            ang_vel.0 += delta_ang_vel;
        }
    }
}

type ForceComponents = (
    &'static mut ExternalForce,
    &'static mut ExternalTorque,
    &'static mut ExternalImpulse,
    &'static mut ExternalAngularImpulse,
);
type ForceComponentsChanged = Or<(
    Changed<ExternalForce>,
    Changed<ExternalTorque>,
    Changed<ExternalImpulse>,
    Changed<ExternalAngularImpulse>,
)>;

/// Responsible for clearing forces and impulses on bodies.
///
/// Runs in [`PhysicsSchedule`], after [`PhysicsStepSet::SpatialQuery`].
pub fn clear_forces_and_impulses(mut forces: Query<ForceComponents, ForceComponentsChanged>) {
    for (mut force, mut torque, mut impulse, mut angular_ímpulse) in &mut forces {
        if !force.persistent {
            force.clear();
        }
        if !torque.persistent {
            torque.clear();
        }
        if !impulse.persistent {
            impulse.clear();
        }
        if !angular_ímpulse.persistent {
            angular_ímpulse.clear();
        }
    }
}
