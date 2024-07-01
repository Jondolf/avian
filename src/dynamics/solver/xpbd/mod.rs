//! Extended Position-Based Dynamics (XPBD) constraint functionality.
//!
//! XPBD is a simulation method that solves constraints at the position-level.
//! Avian currently uses it for [joints](dynamics::solver::joints),
//! while contacts use an impulse-based approach.
//!
//! This module contains traits and systems for XPBD functionality.
//! The actual joint implementations are in [`dynamics::solver::joints`],
//! but it is also possible to create your own constraints.
//!
//! The following section has an overview of what exactly constraints are,
//! how they work with XPBD, and how you can define your own constraints.
//!
//! # Constraints
//!
//! **Constraints** are a way to model physical relationships between entities.
//! They are an integral part of physics simulation, and they can be used for things
//! like contact resolution, [joints](super::joints), soft bodies, and much more.
//!
//! At its core, a constraint is just a rule that is enforced by moving the participating entities in a way that satisfies that rule.
//! For example, a distance constraint is satisfied when the distance between two entities is equal to the desired distance.
//!
//! Most constraints in Avian are modeled as seperate entities with a component that implements [`XpbdConstraint`].
//! They contain a `solve` method that receives the states of the participating entities as parameters.
//! You can find more details on how to use each constraint by taking a look at their documentation.
//!
//! Below are the currently implemented XPBD-based constraints.
//!
//! - [Joints](super::joints)
//!     - [`FixedJoint`]
//!     - [`DistanceJoint`]
#![cfg_attr(feature = "3d", doc = "    - [`SphericalJoint`]")]
//!     - [`RevoluteJoint`]
//!     - [`PrismaticJoint`]
//!
//! Avian's [`ContactConstraint`](dynamics::solver::contact::ContactConstraint)
//! is impulse-based instead.
//!
//! ## Custom constraints
//!
//! In Avian, you can easily create your own XPBD constraints using the same APIs that the engine uses for its own constraints.
//!
//! First, create a struct and implement the [`XpbdConstraint`] trait, giving the number of participating entities using generics.
//! It should look similar to this:
//!
//! ```
#![cfg_attr(
    feature = "2d",
    doc = "use avian2d::{prelude::*, dynamics::solver::xpbd::XpbdConstraint};"
)]
#![cfg_attr(
    feature = "3d",
    doc = "use avian3d::{prelude::*, dynamics::solver::xpbd::XpbdConstraint};"
)]
//! use bevy::{ecs::entity::{EntityMapper, MapEntities}, prelude::*};
//!
//! struct CustomConstraint {
//!     entity1: Entity,
//!     entity2: Entity,
//!     lagrange: f32,
//! }
//!
//! #[cfg(feature = "f32")]
//! impl XpbdConstraint<2> for CustomConstraint {
//!     fn entities(&self) -> [Entity; 2] {
//!         [self.entity1, self.entity2]
//!     }
//!     fn clear_lagrange_multipliers(&mut self) {
//!         self.lagrange = 0.0;
//!     }
//!     fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: f32) {
//!         // Constraint solving logic goes here
//!     }
//! }
//!
//! impl MapEntities for CustomConstraint {
//!     fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
//!        self.entity1 = entity_mapper.map_entity(self.entity1);
//!        self.entity2 = entity_mapper.map_entity(self.entity2);
//!     }
//! }
//! ```
//!
//! Take a look at [`XpbdConstraint::solve`] and the constraint [theory](#theory) to learn more about what to put in `solve`.
//!
//! Next, we need to add a system that solves the constraint during each run of the [solver](dynamics::solver).
//! If your constraint is a component like Avian's joints, you can use the generic [`solve_constraint`]
//! system that handles some of the background work for you.
//!
//! Add the `solve_constraint::<YourConstraint, ENTITY_COUNT>` system to the [substepping schedule's](SubstepSchedule)
//! [`SubstepSolverSet::SolveUserConstraints`](dynamics::solver::SubstepSolverSet::SolveUserConstraints) system set.
//! It should look like this:
//!
//! ```ignore
//! // Get substep schedule
//! let substeps = app
//!     .get_schedule_mut(SubstepSchedule)
//!     .expect("add SubstepSchedule first");
//!
//! // Add custom constraint
//! substeps.add_systems(
//!     solve_constraint::<CustomConstraint, 2>.in_set(SubstepSet::SolveUserConstraints),
//! );
//! ```
//!
//! Now, just spawn an instance of the constraint, give it the participating entities, and the constraint should be getting
//! solved automatically according to the `solve` method!
//!
//! You can find a working example of a custom constraint
//! [here](https://github.com/Jondolf/avian/blob/main/crates/avian3d/examples/custom_constraint.rs).
//!
//! ## Theory
//!
//! In this section, you can learn some of the theory behind how position-based constraints work. Understanding the theory and maths isn't
//! important for using constraints, but it can be useful if you want to [create your own constraints](#custom-constraints).
//!
//! **Note**: In the following theory, primarily the word "particle" is used, but the same logic applies to normal
//! [rigid bodies](RigidBody) as well. However, unlike particles, rigid bodies can also have angular quantities such as
//! [rotation](Rotation) and [angular inertia](Inertia), so constraints can also affect their orientation. This is explained
//! in more detail [at the end](#rigid-body-constraints).
//!
//! ### Constraint functions
//!
//! At the mathematical level, each constraint has a *constraint function* `C(x)` that takes the state
//! of the particles as parameters and outputs a scalar value. The goal of the constraint is to move the particles
//! in a way that the output *satisfies* a constraint equation.
//!
//! For *equality constraints* the equation takes the form `C(x) = 0`. In other words, the constraint tries to
//! *minimize* the value of `C(x)` to be as close to zero as possible. When the equation is true, the constraint is *satisfied*.
//!
//! For a distance constraint, the constraint function would be `C(x) = distance - rest_distance`,
//! because this would be zero when the distance is equal to the desired rest distance.
//!
//! For *inequality constraints* the equation instead takes the form `C(x) >= 0`. These constraints are only applied
//! when `C(x) < 0`, which is useful for things like static friction and [joint limits](super::joints#joint-limits).
//!
//! ### Constraint gradients
//!
//! To know what directions the particles should be moved towards, constraints compute a *constraint gradient* `▽C(x)`
//! for each particle. It is a vector that points in the direction in which the constraint function value `C` increases the most.
//! The length of the gradient indicates how much `C` changes when moving the particle by one unit. This is often equal to one.
//!
//! In a case where two particles are being constrained by a distance constraint, and the particles are outside of the
//! rest distance, the gradient vector would point away from the other particle, because it would increase the distance
//! even further.
//!
//! ### Lagrange multipliers
//!
//! In the context of constraints, a Lagrange multiplier `λ` corresponds to the signed magnitude of the constraint force.
//! It is a scalar value that is the same for all of the constraint's participating particles, and it is used for computing
//! the correction that the constraint should apply to the particles along the gradients.
//!
//! In XPBD, the Lagrange multiplier update `Δλ` during a substep is computed by dividing the opposite of `C`
//! by the sum of the products of the inverse masses and squared gradient lengths plus an additional compliance term:
//!
//! ```text
//! Δλ = -C / (sum(w_i * |▽C_i|^2) + α / h^2)
//! ```
//!
//! where `w_i` is the inverse mass of particle `i`, `|▽C_i|` is the length of the gradient vector for particle `i`,
//! `α` is the constraint's compliance (inverse of stiffness) and `h` is the substep size. Using `α = 0`
//! corresponds to infinite stiffness.
//!
//! The minus sign is there because the gradients point in the direction in which `C` increases the most,
//! and we instead want to minimize `C`.
//!
//! Note that if the gradients are normalized, as is often the case, the squared gradient lengths can be omitted from the
//! calculation.
//!
//! ### Solving constraints
//!
//! Once we have computed the Lagrange multiplier `λ`, we can compute the positional correction for a given particle
//! as the product of the Lagrange multiplier and the particle's inverse mass and gradient vector:
//!
//! ```text
//! Δx_i = Δλ * w_i * ▽C_i
//! ```
//!
//! In other words, we typically move the particle along the gradient by `Δλ` proportional to the particle's inverse mass.
//!
//! ### Rigid body constraints
//!
//! Unlike particles, [rigid bodies](RigidBody) also have angular quantities like [rotation](Rotation),
//! [angular velocity](AngularVelocity) and [angular inertia](Inertia). In addition, constraints can be applied at specific
//! points in the body, like contact positions or joint attachment positions, which also affects the orientation.
//!
//! When the constraint is not applied at the center of mass, the inverse mass in the computation of `Δλ` must
//! be replaced with a *generalized inverse mass* that is essentially the effective mass when applying the constraint
//! at some specified position.
//!
//! For a positional constraint applied at position `r_i`, the generalized inverse mass computation for body `i` looks like this:
//!
//! ```text
//! w_i = 1 / m_i + (r_i x ▽C_i)^T * I_i^-1 * (r_i x ▽C_i)
//! ```
//!
//! where `m_i` is the [mass](Mass) of body `i`, `I_i^-1` is the [inverse inertia tensor](InverseInertia), and `^T` refers to the
//! transpose of a vector. Note that the value of the inertia tensor depends on the orientation of the body, so it should be
//! recomputed each time the constraint is solved.
//!
//! For an angular constraint where the gradient vector is the rotation axis, the generalized inverse mass computation instead
//! looks like this:
//!
//! ```text
//! w_i = ▽C_i^T * I_i^-1 * ▽C_i
//! ```
//!
//! Once we have computed the Lagrange multiplier update, we can apply the positional correction as shown in the
//! [previous section](#solving-constraints).
//!
//! However, angular constraints are handled differently. If the constraint function's value is the rotation angle and
//! the gradient vector is the rotation axis, we can compute the angular correction for a given body like this:
//!
//! ```text
//! Δq_i = 0.5 * [I_i^-1 * (r_i x (Δλ * ▽C_i)), 0] * q_i
//! ```
//!
//! where `q_i` is the [rotation](Rotation) of body `i` and `r_i` is a vector pointing from the body's center of mass to some
//! attachment position.

mod angular_constraint;
mod positional_constraint;

pub use angular_constraint::AngularConstraint;
pub use positional_constraint::PositionConstraint;

use crate::prelude::*;
use bevy::{ecs::entity::MapEntities, prelude::*};

/// A trait for all XPBD [constraints](self#constraints).
pub trait XpbdConstraint<const ENTITY_COUNT: usize>: MapEntities {
    /// The entities participating in the constraint.
    fn entities(&self) -> [Entity; ENTITY_COUNT];

    /// Solves the constraint.
    ///
    /// There are two main steps to solving a constraint:
    ///
    /// 1. Compute the generalized inverse masses, [gradients](self#constraint-gradients)
    /// and the [Lagrange multiplier](self#lagrange-multipliers) update.
    /// 2. Apply corrections along the gradients using the Lagrange multiplier update.
    ///
    /// [`XpbdConstraint`] provides the [`compute_lagrange_update`](XpbdConstraint::compute_lagrange_update)
    /// method for all constraints. It requires the gradients and inverse masses of the participating entities.
    ///
    /// For constraints between two bodies, you can implement [`PositionConstraint`]. and [`AngularConstraint`]
    /// to get the associated `compute_generalized_inverse_mass`, `apply_positional_correction` and
    /// `apply_angular_correction` methods. Otherwise you must implement the generalized inverse mass
    /// computations and correction applying logic yourself.
    ///
    /// You can find a working example of a custom constraint
    /// [here](https://github.com/Jondolf/avian/blob/main/crates/avian3d/examples/custom_constraint.rs).
    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; ENTITY_COUNT], dt: Scalar);

    /// Computes how much a constraint's [Lagrange multiplier](self#lagrange-multipliers) changes when projecting
    /// the constraint for all participating particles.
    ///
    /// `c` is a scalar value returned by the [constraint function](self#constraint-functions).
    /// When it is zero, the constraint is satisfied.
    ///
    /// Each particle should have a corresponding [gradient](self#constraint-gradients) in `gradients`.
    /// A gradient is a vector that refers to the direction in which `c` increases the most.
    ///
    /// See the [constraint theory](#theory) for more information.
    fn compute_lagrange_update_with_gradients(
        &self,
        lagrange: Scalar,
        c: Scalar,
        gradients: &[Vector],
        inverse_masses: &[Scalar],
        compliance: Scalar,
        dt: Scalar,
    ) -> Scalar {
        // Compute the sum of all inverse masses multiplied by the squared lengths of the corresponding gradients.
        let w_sum = inverse_masses
            .iter()
            .enumerate()
            .fold(0.0, |acc, (i, w)| acc + *w * gradients[i].length_squared());

        // Avoid division by zero
        if w_sum <= Scalar::EPSILON {
            return 0.0;
        }

        // tilde_a = a/h^2
        let tilde_compliance = compliance / dt.powi(2);

        (-c - tilde_compliance * lagrange) / (w_sum + tilde_compliance)
    }

    /// Computes how much a constraint's [Lagrange multiplier](self#lagrange-multipliers) changes when projecting
    /// the constraint for all participating particles. The constraint gradients are assumed to be unit-length.
    ///
    /// `c` is a scalar value returned by the [constraint function](self#constraint-functions).
    /// When it is zero, the constraint is satisfied.
    ///
    /// See the [constraint theory](#theory) for more information.
    fn compute_lagrange_update(
        &self,
        lagrange: Scalar,
        c: Scalar,
        inverse_masses: &[Scalar],
        compliance: Scalar,
        dt: Scalar,
    ) -> Scalar {
        // Compute the sum of all inverse masses.
        // The gradients are unit length, so they don't need to be considered.
        let w_sum: Scalar = inverse_masses.iter().copied().sum();

        // Avoid division by zero
        if w_sum <= Scalar::EPSILON {
            return 0.0;
        }

        // tilde_a = a/h^2
        let tilde_compliance = compliance / dt.powi(2);

        (-c - tilde_compliance * lagrange) / (w_sum + tilde_compliance)
    }

    /// Sets the constraint's [Lagrange multipliers](self#lagrange-multipliers) to 0.
    fn clear_lagrange_multipliers(&mut self);
}

// TODO: Make a joint-optimized version
/// Iterates through the XPBD constraints of a given type and solves them. Sleeping bodies are woken up when
/// active bodies interact with them in a constraint.
///
/// Note that this system only works for constraints that are modeled as entities.
/// If you store constraints in a resource, you must create your own system for solving them.
///
/// ## User constraints
///
/// To create a new constraint, implement [`XpbdConstraint`] for a component, get the [`SubstepSchedule`] and add this system into
/// the [`SubstepSolverSet::SolveUserConstraints`](super::SubstepSolverSet::SolveUserConstraints) set.
/// You must provide the number of entities in the constraint using generics.
///
/// It should look something like this:
///
/// ```ignore
/// let substeps = app
///     .get_schedule_mut(SubstepSchedule)
///     .expect("add SubstepSchedule first");
///
/// substeps.add_systems(
///     solve_constraint::<YourConstraint, ENTITY_COUNT>
///         .in_set(SubstepSolverSet::SolveUserConstraints),
/// );
/// ```
pub fn solve_constraint<C: XpbdConstraint<ENTITY_COUNT> + Component, const ENTITY_COUNT: usize>(
    mut commands: Commands,
    mut bodies: Query<RigidBodyQuery>,
    mut constraints: Query<&mut C, Without<RigidBody>>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    // Clear Lagrange multipliers
    constraints
        .iter_mut()
        .for_each(|mut c| c.clear_lagrange_multipliers());

    for mut constraint in &mut constraints {
        // Get components for entities
        if let Ok(mut bodies) = bodies.get_many_mut(constraint.entities()) {
            let none_dynamic = bodies.iter().all(|body| !body.rb.is_dynamic());
            let all_inactive = bodies
                .iter()
                .all(|body| body.rb.is_static() || body.is_sleeping);

            // No constraint solving if none of the bodies is dynamic,
            // or if all of the bodies are either static or sleeping
            if none_dynamic || all_inactive {
                continue;
            }

            // At least one of the participating bodies is active, so wake up any sleeping bodies
            for body in &mut bodies {
                body.time_sleeping.0 = 0.0;

                if body.is_sleeping {
                    commands.entity(body.entity).remove::<Sleeping>();
                }
            }

            // Get the bodies as an array and solve the constraint
            if let Ok(bodies) = bodies
                .iter_mut()
                .collect::<Vec<&mut RigidBodyQueryItem>>()
                .try_into()
            {
                constraint.solve(bodies, delta_secs);
            }
        }
    }
}

/// Updates the linear velocity of all dynamic bodies based on the change in position from the XPBD solver.
pub(super) fn project_linear_velocity(
    mut bodies: Query<
        (
            &RigidBody,
            &PreSolveAccumulatedTranslation,
            &AccumulatedTranslation,
            &mut LinearVelocity,
        ),
        Without<Sleeping>,
    >,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (rb, prev_pos, translation, mut lin_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && lin_vel.0 != Vector::ZERO {
            lin_vel.0 = Vector::ZERO;
        }

        if rb.is_dynamic() {
            // v = (x - x_prev) / h
            let new_lin_vel = (translation.0 - prev_pos.0) / delta_secs;
            // Avoid triggering bevy's change detection unnecessarily.
            if new_lin_vel != Vector::ZERO && new_lin_vel.is_finite() {
                lin_vel.0 += new_lin_vel;
            }
        }
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the XPBD solver.
#[cfg(feature = "2d")]
pub(super) fn project_angular_velocity(
    mut bodies: Query<
        (
            &RigidBody,
            &Rotation,
            &PreSolveRotation,
            &mut AngularVelocity,
        ),
        Without<Sleeping>,
    >,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (rb, rot, prev_rot, mut ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && ang_vel.0 != 0.0 {
            ang_vel.0 = 0.0;
        }

        if rb.is_dynamic() {
            let new_ang_vel = prev_rot.angle_between(*rot) / delta_secs;
            // Avoid triggering bevy's change detection unnecessarily.
            if new_ang_vel != ang_vel.0 && new_ang_vel.is_finite() {
                ang_vel.0 += new_ang_vel;
            }
        }
    }
}

/// Updates the angular velocity of all dynamic bodies based on the change in rotation from the XPBD solver.
#[cfg(feature = "3d")]
pub(super) fn project_angular_velocity(
    mut bodies: Query<
        (
            &RigidBody,
            &Rotation,
            &PreSolveRotation,
            &mut AngularVelocity,
        ),
        Without<Sleeping>,
    >,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();

    for (rb, rot, prev_rot, mut ang_vel) in &mut bodies {
        // Static bodies have no velocity
        if rb.is_static() && ang_vel.0 != Vector::ZERO {
            ang_vel.0 = Vector::ZERO;
        }

        if rb.is_dynamic() {
            let delta_rot = rot.mul_quat(prev_rot.inverse().0);
            let mut new_ang_vel = 2.0 * delta_rot.xyz() / delta_secs;
            if delta_rot.w < 0.0 {
                new_ang_vel = -new_ang_vel;
            }
            // Avoid triggering bevy's change detection unnecessarily.
            if new_ang_vel != ang_vel.0 && new_ang_vel.is_finite() {
                ang_vel.0 += new_ang_vel;
            }
        }
    }
}
