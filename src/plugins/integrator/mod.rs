//! This module contains an [`IntegratorPlugin`] that handles applying forces
//! and velocities to bodies in order to move them according to the equations of motion.
//! This is done using *numerical integration*.
//!
//! Several integration schemes are provided in sub-modules:
//!
//! - [`semi_implicit_euler`] (default)
//! - [`velocity_verlet`]
//!
//! Each module contains its own `integrate_forces` and `integrate_velocities` functions.
//! See the documentation of the modules for more information.
//!
//! The default integration scheme can be changed using the [`IntegrationScheme`] resource:
//!
//! ```no_run
//! # use bevy::prelude::*;
#![cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::dynamics::integrator::*;")]
#![cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::dynamics::integrator::*;")]
//! let mut app = App::new();
//! app.insert_resource(IntegrationScheme::VelocityVerlet);
//! ```

#[doc(alias = "symplectic_euler")]
pub mod semi_implicit_euler;
pub mod velocity_verlet;

use crate::prelude::*;
use bevy::prelude::*;

/// Integrates Newton's 2nd law of motion, applying forces and moving entities according to their velocities.
///
/// This acts as a prediction for the next positions and orientations of the bodies. The [solver] corrects these predicted
/// positions to follow the rules set by the [constraints].
///
/// The integration scheme used is very closely related to implicit Euler integration.
///
/// The integration systems run in [`SubstepSet::Integrate`].
pub struct IntegratorPlugin;

impl Plugin for IntegratorPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<IntegrationScheme>();

        app.get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first")
            .add_systems(
                (
                    // Schemes that first integrate forces and then velocity
                    (integrate_forces, integrate_velocity).chain().run_if(
                        |scheme: Res<IntegrationScheme>| {
                            *scheme == IntegrationScheme::SemiImplicitEuler
                        },
                    ),
                    // Schemes that first integrate velocity and then forces
                    (integrate_velocity, integrate_forces).chain().run_if(
                        |scheme: Res<IntegrationScheme>| {
                            *scheme == IntegrationScheme::VelocityVerlet
                        },
                    ),
                )
                    .in_set(SubstepSet::Integrate)
                    .chain(),
            );

        app.get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first")
            .add_systems(
                apply_impulses
                    .after(PhysicsStepSet::BroadPhase)
                    .before(PhysicsStepSet::Substeps),
            )
            .add_systems(clear_forces_and_impulses.after(PhysicsStepSet::SpatialQuery));
    }
}

/// A resource controlling the default integration scheme
/// used for integrating the equations of motion.
#[derive(Resource, Debug, Default, PartialEq, Eq, Reflect)]
pub enum IntegrationScheme {
    /// The *semi-implicit* or *symplectic* Euler integration method.
    ///
    /// See [`semi_implicit_euler`] for more details.
    #[doc(alias = "SymplecticEuler")]
    #[default]
    SemiImplicitEuler,
    /// The velocity Verlet integration method.
    ///
    /// See [`velocity_verlet`] for more details.
    VelocityVerlet,
}

#[allow(clippy::type_complexity)]
fn integrate_forces(
    mut bodies: Query<
        (
            &RigidBody,
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
    integration_scheme: Res<IntegrationScheme>,
    gravity: Res<Gravity>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (
        rb,
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
    ) in &mut bodies
    {
        if !rb.is_dynamic() {
            if *lin_vel != LinearVelocity::ZERO {
                *lin_vel = LinearVelocity::ZERO;
            }
            if *ang_vel != AngularVelocity::ZERO {
                *ang_vel = AngularVelocity::ZERO;
            }
            continue;
        }

        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        // Apply damping
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

        let inv_inertia = inv_inertia.rotated(rot).0;
        let torque = torque.torque() + force.torque();

        match *integration_scheme {
            IntegrationScheme::SemiImplicitEuler => {
                semi_implicit_euler::integrate_forces(
                    &mut lin_vel.0,
                    &mut ang_vel.0,
                    force.force(),
                    torque,
                    inv_mass.0,
                    inv_inertia,
                    locked_axes,
                    gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0),
                    delta_secs,
                );
            }
            IntegrationScheme::VelocityVerlet => {
                let lin_acc =
                    linear_acceleration(force.force(), inv_mass.0, locked_axes, gravity.0);
                let ang_acc = angular_acceleration(ang_vel.0, torque, inv_inertia, locked_axes);
                velocity_verlet::integrate_forces(
                    &mut lin_vel.0,
                    &mut ang_vel.0,
                    lin_acc,
                    ang_acc,
                    force.force(),
                    torque,
                    inv_mass.0,
                    inv_inertia,
                    locked_axes,
                    gravity.0 * gravity_scale.map_or(1.0, |scale| scale.0),
                    delta_secs,
                );
            }
        }
    }
}

#[allow(clippy::type_complexity)]
fn integrate_velocity(
    mut bodies: Query<
        (
            &RigidBody,
            &Position,
            &mut AccumulatedTranslation,
            &mut Rotation,
            Option<&mut PreviousPosition>,
            Option<&mut PreviousRotation>,
            &LinearVelocity,
            &AngularVelocity,
            &ExternalForce,
            &ExternalTorque,
            &InverseMass,
            &InverseInertia,
            Option<&LockedAxes>,
        ),
        Without<Sleeping>,
    >,
    integration_scheme: Res<IntegrationScheme>,
    gravity: Res<Gravity>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (
        rb,
        pos,
        mut accumulated_translation,
        mut rot,
        prev_pos,
        prev_rot,
        lin_vel,
        ang_vel,
        force,
        torque,
        inv_mass,
        inv_inertia,
        locked_axes,
    ) in &mut bodies
    {
        if let Some(mut prev_pos) = prev_pos {
            prev_pos.0 = pos.0;
        }
        if let Some(mut prev_rot) = prev_rot {
            prev_rot.0 = *rot;
        }

        if rb.is_static() {
            continue;
        }

        let inv_inertia = inv_inertia.rotated(&rot).0;
        let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

        match *integration_scheme {
            IntegrationScheme::SemiImplicitEuler => {
                semi_implicit_euler::integrate_velocity(
                    &mut accumulated_translation.0,
                    &mut rot,
                    lin_vel.0,
                    ang_vel.0,
                    locked_axes,
                    delta_secs,
                );
            }
            IntegrationScheme::VelocityVerlet => {
                let torque = torque.torque() + force.torque();
                velocity_verlet::integrate_velocity(
                    &mut accumulated_translation.0,
                    &mut rot,
                    lin_vel.0,
                    ang_vel.0,
                    linear_acceleration(force.force(), inv_mass.0, locked_axes, gravity.0),
                    angular_acceleration(ang_vel.0, torque, inv_inertia, locked_axes),
                    locked_axes,
                    delta_secs,
                );
            }
        }
    }
}

/// Computes linear acceleration based on the given forces and mass.
pub fn linear_acceleration(
    force: Vector,
    inv_mass: Scalar,
    locked_axes: LockedAxes,
    gravity: Vector,
) -> Vector {
    // Effective inverse mass along each axis
    let inv_mass = locked_axes.apply_to_vec(Vector::splat(inv_mass));

    if inv_mass != Vector::ZERO && inv_mass.is_finite() {
        // Newton's 2nd law for translational movement:
        //
        // F = m * a
        // a = F / m
        //
        // where a is the acceleration, F is the force, and m is the mass.
        //
        // `gravity` below is the gravitational acceleration,
        // so it doesn't need to be divided by mass.
        force * inv_mass + gravity
    } else {
        Vector::ZERO
    }
}

/// Computes angular acceleration based on the current angular velocity, torque, and inertia.
#[allow(unused_variables)]
pub fn angular_acceleration(
    ang_vel: AngularValue,
    torque: TorqueValue,
    inv_inertia: InertiaValue,
    locked_axes: LockedAxes,
) -> AngularValue {
    // Effective inverse inertia along each axis
    let inv_inertia = locked_axes.apply_to_rotation(inv_inertia);

    if inv_inertia != InverseInertia::ZERO.0 && inv_inertia.is_finite() {
        // Newton's 2nd law for rotational movement:
        //
        // τ = I * α
        // α = τ / I
        //
        // where α (alpha) is the angular acceleration,
        // τ (tau) is the torque, and I is the moment of inertia.
        #[cfg(feature = "2d")]
        {
            inv_inertia * torque
        }
        #[cfg(feature = "3d")]
        {
            inv_inertia * (torque - ang_vel.cross(inv_inertia.inverse() * ang_vel))
        }
    } else {
        AngularValue::ZERO
    }
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

        // avoid triggering bevy's change detection unnecessarily
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
