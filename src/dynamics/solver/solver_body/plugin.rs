use bevy::{ecs::query::QueryFilter, prelude::*};

use crate::{
    dynamics::solver::SolverDiagnostics, AngularVelocity, LinearVelocity, PhysicsSchedule,
    Position, RigidBody, RigidBodyActiveFilter, RigidBodyDisabled, Rotation, Sleeping, SolverSet,
    Vector,
};

use super::{SolverBodies, SolverBody, SolverBodyIndex};

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
                add_solver_body(In(trigger.target()), rb_query, solver_bodies);
            },
        );

        // Add a solver body for each dynamic and kinematic rigid body
        // when the associated rigid body is enabled or woken up.
        app.add_observer(
            |trigger: Trigger<OnRemove, RigidBodyDisabled>,
             rb_query: Query<(&RigidBody, &mut SolverBodyIndex), Without<Sleeping>>,
             solver_bodies: ResMut<SolverBodies>| {
                add_solver_body::<Without<Sleeping>>(In(trigger.target()), rb_query, solver_bodies);
            },
        );
        app.add_observer(
            |trigger: Trigger<OnRemove, Sleeping>,
             rb_query: Query<(&RigidBody, &mut SolverBodyIndex), Without<RigidBodyDisabled>>,
             solver_bodies: ResMut<SolverBodies>| {
                add_solver_body::<Without<RigidBodyDisabled>>(
                    In(trigger.target()),
                    rb_query,
                    solver_bodies,
                );
            },
        );

        // Remove-swap solver bodies when their associated rigid body is removed.
        app.add_observer(
            |trigger: Trigger<OnRemove, RigidBody>,
             rb_query: Query<&RigidBody, With<SolverBodyIndex>>,
             index_query: Query<&mut SolverBodyIndex>,
             solver_bodies: ResMut<SolverBodies>| {
                remove_solver_body(In(trigger.target()), rb_query, index_query, solver_bodies);
            },
        );

        // Remove-swap solver bodies when their associated rigid body is disabled or put to sleep.
        app.add_observer(
            |trigger: Trigger<OnAdd, (RigidBodyDisabled, Sleeping)>,
             rb_query: Query<&RigidBody, With<SolverBodyIndex>>,
             index_query: Query<&mut SolverBodyIndex>,
             solver_bodies: ResMut<SolverBodies>| {
                remove_solver_body(In(trigger.target()), rb_query, index_query, solver_bodies);
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
    mut solver_bodies: ResMut<SolverBodies>,
    rb_query: Query<(Entity, Ref<RigidBody>), (With<SolverBodyIndex>, Changed<RigidBody>)>,
    mut index_query: Query<&mut SolverBodyIndex>,
) {
    for (entity, rb) in &rb_query {
        // Only handle modifications to the rigid body type here.
        if rb.is_added() {
            continue;
        }

        let mut index = index_query.get_mut(entity).unwrap();

        if rb.is_static() {
            // Reset the `SolverBodyIndex`.
            *index = SolverBodyIndex::INVALID;

            // Swap-remove the solver body.
            solver_bodies.swap_remove(*index);

            // Update the `SolverBodyIndex` of the entity whose solver body was swapped.
            if let Some(last_entity) = solver_bodies.get_entity(*index) {
                let index = *index;
                if let Ok(mut last_index) = index_query.get_mut(last_entity) {
                    *last_index = index;
                }
            }
        } else if !solver_bodies.contains_index(*index) {
            // Create a new solver body if the rigid body is dynamic or kinematic.
            solver_bodies.push(entity, SolverBody::default());
        }
    }
}

fn add_solver_body<F: QueryFilter>(
    In(entity): In<Entity>,
    mut rb_query: Query<(&RigidBody, &mut SolverBodyIndex), F>,
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

fn remove_solver_body(
    In(entity): In<Entity>,
    rb_query: Query<&RigidBody, With<SolverBodyIndex>>,
    mut index_query: Query<&mut SolverBodyIndex>,
    mut solver_bodies: ResMut<SolverBodies>,
) {
    if let Ok(rb) = rb_query.get(entity) {
        if rb.is_static() {
            return;
        }

        let mut index_mut = index_query.get_mut(entity).unwrap();
        let index = *index_mut;

        // Swap-remove the solver body and reset the `SolverBodyIndex`.
        solver_bodies.swap_remove(index);
        *index_mut = SolverBodyIndex::INVALID;

        // Update the `SolverBodyIndex` of the entity whose solver body was swapped.
        if let Some(swapped_entity) = solver_bodies.get_entity(index) {
            if let Ok(mut swapped_index) = index_query.get_mut(swapped_entity) {
                *swapped_index = index;
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

        if let Some(solver_body) = bodies.get_mut(*index) {
            solver_body.linear_velocity = linear_velocity.0;
            solver_body.angular_velocity = angular_velocity.0;
            solver_body.delta_position = Vector::ZERO;
            solver_body.delta_rotation = Rotation::IDENTITY;
        }
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
    let start = bevy::platform::time::Instant::now();

    for (mut pos, mut rot, mut lin_vel, mut ang_vel, index) in &mut query {
        if !index.is_valid() {
            continue;
        }

        if let Some(solver_body) = bodies.get_mut(*index) {
            // TODO: Make sure rotation about the center of mass is handled correctly.
            pos.0 += solver_body.delta_position;
            *rot = (solver_body.delta_rotation * *rot).fast_renormalize();
            lin_vel.0 = solver_body.linear_velocity;
            ang_vel.0 = solver_body.angular_velocity;
        }
    }

    diagnostics.finalize += start.elapsed();
}

// TODO: Change rigid body type to static and immediately despawn it.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{dynamics::solver::SolverBodyIndex, PhysicsSchedulePlugin, SolverSchedulePlugin};

    fn create_app() -> App {
        let mut app = App::new();

        app.add_plugins((
            PhysicsSchedulePlugin::default(),
            SolverSchedulePlugin,
            SolverBodyPlugin,
        ));

        app
    }

    fn has_solver_body(app: &App, entity: Entity) -> bool {
        app.world()
            .resource::<SolverBodies>()
            .contains_entity(entity)
    }

    fn get_solver_body_entity(app: &App, index: usize) -> Entity {
        app.world()
            .resource::<SolverBodies>()
            .get_entity(SolverBodyIndex(index))
            .unwrap()
    }

    #[test]
    fn add_remove_solver_bodies() {
        let mut app = create_app();

        // Create a dynamic, kinematic, and static rigid body.
        let entity1 = app.world_mut().spawn(RigidBody::Dynamic).id();
        let entity2 = app.world_mut().spawn(RigidBody::Kinematic).id();
        let entity3 = app.world_mut().spawn(RigidBody::Static).id();

        // The dynamic and kinematic rigid bodies should have solver bodies.
        assert!(has_solver_body(&app, entity1));
        assert!(has_solver_body(&app, entity2));
        assert!(!has_solver_body(&app, entity3));

        // Disable the dynamic rigid body.
        app.world_mut()
            .entity_mut(entity1)
            .insert(RigidBodyDisabled);

        // The entity should no longer have a solver body.
        // Its index should be `SolverBodyIndex::INVALID`.
        assert!(!has_solver_body(&app, entity1));

        // The index of the kinematic rigid body should be `0`.
        assert_eq!(get_solver_body_entity(&app, 0), entity2);

        // Enable the dynamic rigid body.
        app.world_mut()
            .entity_mut(entity1)
            .remove::<RigidBodyDisabled>();

        // The dynamic rigid body should have a solver body again.
        assert!(has_solver_body(&app, entity1));

        // The index of the dynamic rigid body should be `1`.
        assert_eq!(get_solver_body_entity(&app, 1), entity1);
    }
}
