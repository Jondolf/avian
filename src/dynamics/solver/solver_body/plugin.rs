use bevy::prelude::*;

use crate::{
    dynamics::solver::SolverDiagnostics, AngularVelocity, LinearVelocity, PhysicsSchedule,
    Position, RigidBody, RigidBodyActiveFilter, RigidBodyDisabled, Rotation, Sleeping, SolverSet,
    Vector,
};

use super::{SolverBodies, SolverBody, SolverBodyIndex};

// TODO: Add tests for this.
/// A plugin for managing [`SolverBodies`].
///
/// A [`SolverBody`] is created for each dynamic and kinematic rigid body when:
///
/// 1. The rigid body is created.
/// 2. The rigid body is enabled by removing `RigidBodyDisabled`.
/// 3. The rigid body is woken up.
/// 4. The rigid body is set to be dynamic or kinematic.
///
/// A [`SolverBody`] is removed when:
///
/// 1. The rigid body is removed.
/// 2. The rigid body is disabled by adding `RigidBodyDisabled`.
/// 3. The rigid body is put to sleep.
/// 4. The rigid body is set to be static.
pub struct SolverBodyPlugin;

impl Plugin for SolverBodyPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SolverBodies>();

        // Automatically insert `SolverBodyIndex` for each rigid body, initialized as `SolverBodyIndex::INVALID`.
        // This is later updated by observers.
        app.register_required_components_with::<RigidBody, SolverBodyIndex>(|| {
            SolverBodyIndex::INVALID
        });

        // Add a solver body for each dynamic and kinematic rigid body when the rigid body is created.
        app.add_observer(
            |trigger: Trigger<OnAdd, RigidBody>,
             rb_query: Query<(&RigidBody, &mut SolverBodyIndex), RigidBodyActiveFilter>,
             solver_bodies: ResMut<SolverBodies>| {
                add_solver_body(In(trigger.entity()), rb_query, solver_bodies);
            },
        );

        // Add a solver body for each dynamic and kinematic rigid body
        // when the associated rigid body is enabled or woken up.
        app.add_observer(
            |trigger: Trigger<OnRemove, (RigidBodyDisabled, Sleeping)>,
             rb_query: Query<(&RigidBody, &mut SolverBodyIndex), RigidBodyActiveFilter>,
             solver_bodies: ResMut<SolverBodies>| {
                add_solver_body(In(trigger.entity()), rb_query, solver_bodies);
            },
        );

        // Remove-swap solver bodies when their associated rigid body is removed.
        app.add_observer(
            |trigger: Trigger<OnRemove, RigidBody>,
             rb_query: Query<(&RigidBody, &mut SolverBodyIndex)>,
             solver_bodies: ResMut<SolverBodies>| {
                remove_swap_solver_body(In(trigger.entity()), rb_query, solver_bodies);
            },
        );

        // Remove-swap solver bodies when their associated rigid body is disabled or put to sleep.
        app.add_observer(
            |trigger: Trigger<OnAdd, (RigidBodyDisabled, Sleeping)>,
             rb_query: Query<(&RigidBody, &mut SolverBodyIndex)>,
             solver_bodies: ResMut<SolverBodies>| {
                remove_swap_solver_body(In(trigger.entity()), rb_query, solver_bodies);
            },
        );

        // Prepare solver bodies before the substepping loop.
        app.add_systems(
            PhysicsSchedule,
            (on_change_rigid_body_type, prepare_solver_bodies)
                .chain()
                .in_set(SolverSet::PrepareSolverBodies),
        );

        // Write back solver body data to rigid bodies after the substepping loop.
        app.add_systems(
            PhysicsSchedule,
            writeback_solver_bodies.in_set(SolverSet::Finalize),
        );
    }
}

fn on_change_rigid_body_type(
    mut bodies: ResMut<SolverBodies>,
    query: Query<(Entity, Ref<RigidBody>, &mut SolverBodyIndex), Changed<RigidBody>>,
) {
    for (entity, rb, &index) in &mut query.iter() {
        // Only handle modifications to the rigid body type here.
        if rb.is_added() {
            continue;
        }

        if rb.is_static() {
            // Swap-remove the solver body.
            bodies.swap_remove(index);

            // Update the `SolverBodyIndex` of the entity whose solver body was swapped.
            if let Some(last_entity) = bodies.get_entity(index) {
                // SAFETY: The entity is guaranteed to exist.
                if let Ok((_, _, mut last_index)) = unsafe { query.get_unchecked(last_entity) } {
                    *last_index = index;
                }
            }
        } else if !bodies.contains_index(index) {
            // Create a new solver body if the rigid body is dynamic or kinematic.
            bodies.push(entity, SolverBody::default());
        }
    }
}

fn add_solver_body(
    In(entity): In<Entity>,
    mut rb_query: Query<(&RigidBody, &mut SolverBodyIndex), RigidBodyActiveFilter>,
    mut solver_bodies: ResMut<SolverBodies>,
) {
    if let Ok((rb, mut index)) = rb_query.get_mut(entity) {
        if rb.is_static() {
            return;
        }

        // Create a new solver body if the rigid body is dynamic or kinematic.
        index.0 = solver_bodies.len();
        solver_bodies.push(entity, SolverBody::default());
    }
}

fn remove_swap_solver_body(
    In(entity): In<Entity>,
    mut rb_query: Query<(&RigidBody, &mut SolverBodyIndex)>,
    mut solver_bodies: ResMut<SolverBodies>,
) {
    if let Ok((rb, &index)) = rb_query.get(entity) {
        if rb.is_static() {
            return;
        }

        // Swap-remove the solver body.
        solver_bodies.swap_remove(index);

        // Update the `SolverBodyIndex` of the entity whose solver body was swapped.
        if let Some(last_entity) = solver_bodies.get_entity(index) {
            if let Ok((_, mut last_index)) = rb_query.get_mut(last_entity) {
                *last_index = index;
            }
        }
    }
}

fn prepare_solver_bodies(
    mut bodies: ResMut<SolverBodies>,
    query: Query<(&SolverBodyIndex, &LinearVelocity, &AngularVelocity)>,
) {
    for (index, linear_velocity, angular_velocity) in &query {
        if !index.is_valid() {
            continue;
        }

        // SAFETY: The index is guaranteed to be valid.
        let solver_body = unsafe { bodies.get_unchecked_mut(index.0) };
        solver_body.linear_velocity = linear_velocity.0;
        solver_body.angular_velocity = angular_velocity.0;
        solver_body.delta_position = Vector::ZERO;
        solver_body.delta_rotation = Rotation::IDENTITY;
    }
}

/// Writes back solver body data to rigid bodies.
#[allow(clippy::type_complexity)]
fn writeback_solver_bodies(
    mut bodies: ResMut<SolverBodies>,
    mut query: Query<(
        &mut Position,
        &mut Rotation,
        &mut LinearVelocity,
        &mut AngularVelocity,
        &SolverBodyIndex,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = bevy::utils::Instant::now();

    for (mut pos, mut rot, mut lin_vel, mut ang_vel, index) in &mut query {
        if !index.is_valid() {
            continue;
        }

        let solver_body = unsafe { bodies.get_unchecked_mut(index.0) };

        // TODO: Make sure rotation about the center of mass is handled correctly.
        pos.0 += solver_body.delta_position;
        *rot = (solver_body.delta_rotation * *rot).fast_renormalize();
        lin_vel.0 = solver_body.linear_velocity;
        ang_vel.0 = solver_body.angular_velocity;
    }

    diagnostics.finalize += start.elapsed();
}
