use bevy::{
    ecs::{entity_disabling::Disabled, query::QueryFilter, world::DeferredWorld},
    prelude::*,
};

use super::{SolverBody, SolverBodyInertia};
use crate::{
    AngularVelocity, LinearVelocity, PhysicsSchedule, Position, RigidBody, RigidBodyActiveFilter,
    RigidBodyDisabled, Rotation, Sleeping, SolverSet, Vector,
    dynamics::solver::{SolverDiagnostics, solver_body::SolverBodyFlags},
    prelude::{
        ComputedAngularInertia, ComputedCenterOfMass, ComputedMass, DynamicBody, KinematicBody,
        LockedAxes,
    },
};
#[cfg(feature = "3d")]
use crate::{
    MatExt,
    dynamics::integrator::{IntegrationSet, integrate_positions},
    prelude::SubstepSchedule,
};

/// A plugin for managing solver bodies.
///
/// A [`SolverBody`] is created for each dynamic and kinematic rigid body when:
///
/// 1. The rigid body is created.
/// 2. The rigid body is enabled by removing [`Disabled`]/[`RigidBodyDisabled`].
/// 3. The rigid body is woken up.
/// 4. The rigid body is set to be dynamic or kinematic.
///
/// A [`SolverBody`] is removed when:
///
/// 1. The rigid body is removed.
/// 2. The rigid body is disabled by adding [`Disabled`]/[`RigidBodyDisabled`].
/// 3. The rigid body is put to sleep.
/// 4. The rigid body is set to be static.
pub struct SolverBodyPlugin;

impl Plugin for SolverBodyPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<(SolverBody, SolverBodyInertia)>();

        // Add a solver body for each dynamic and kinematic rigid body when the rigid body is created.
        app.add_observer(
            |trigger: Trigger<OnAdd, (DynamicBody, KinematicBody)>,
             rb_query: Query<(), RigidBodyActiveFilter>,
             commands: Commands| {
                add_solver_body(In(trigger.target()), rb_query, commands);
            },
        );

        // Add a solver body for each dynamic and kinematic rigid body
        // when the associated rigid body is enabled or woken up.
        app.add_observer(
            |trigger: Trigger<OnRemove, RigidBodyDisabled>,
             rb_query: Query<
                (),
                (
                    Or<(With<DynamicBody>, With<KinematicBody>)>,
                    Without<Sleeping>,
                ),
            >,
             commands: Commands| {
                add_solver_body(In(trigger.target()), rb_query, commands);
            },
        );
        app.add_observer(
            |trigger: Trigger<OnRemove, Disabled>,
             rb_query: Query<
                (),
                (
                    Or<(With<DynamicBody>, With<KinematicBody>)>,
                    // The body still has `Disabled` at this point,
                    // and we need to include in the query to match against the entity.
                    With<Disabled>,
                    Without<RigidBodyDisabled>,
                    Without<Sleeping>,
                ),
            >,
             commands: Commands| {
                add_solver_body(In(trigger.target()), rb_query, commands);
            },
        );
        app.add_observer(
            |trigger: Trigger<OnRemove, Sleeping>,
             rb_query: Query<
                (),
                (
                    Or<(With<DynamicBody>, With<KinematicBody>)>,
                    Without<RigidBodyDisabled>,
                ),
            >,
             commands: Commands| {
                add_solver_body(In(trigger.target()), rb_query, commands);
            },
        );

        // Remove solver bodies when their associated rigid body is removed.
        app.add_observer(
            |trigger: Trigger<OnRemove, RigidBody>, deferred_world: DeferredWorld| {
                remove_solver_body(In(trigger.target()), deferred_world);
            },
        );

        // Remove solver bodies when their associated rigid body is disabled or put to sleep.
        app.add_observer(
            |trigger: Trigger<OnAdd, (Disabled, RigidBodyDisabled, Sleeping)>,
             deferred_world: DeferredWorld| {
                remove_solver_body(In(trigger.target()), deferred_world);
            },
        );

        // Prepare solver bodies before the substepping loop.
        app.add_systems(
            PhysicsSchedule,
            (
                prepare_dynamic_solver_bodies,
                prepare_kinematic_solver_bodies,
            )
                .chain()
                .in_set(SolverSet::PrepareSolverBodies),
        );

        // Write back solver body data to rigid bodies after the substepping loop.
        app.add_systems(
            PhysicsSchedule,
            writeback_solver_bodies.in_set(SolverSet::Finalize),
        );

        // Update the world-space angular inertia of solver bodies right after position integration
        // in the substepping loop.
        #[cfg(feature = "3d")]
        app.add_systems(
            SubstepSchedule,
            update_solver_body_angular_inertia
                .in_set(IntegrationSet::Position)
                .after(integrate_positions),
        );
    }
}

fn add_solver_body<F: QueryFilter>(
    In(entity): In<Entity>,
    query: Query<(), F>,
    mut commands: Commands,
) {
    if query.contains(entity) {
        // Create a new solver body if the rigid body is dynamic or kinematic.
        commands
            .entity(entity)
            .try_insert((SolverBody::default(), SolverBodyInertia::default()));
    }
}

fn remove_solver_body(In(entity): In<Entity>, mut deferred_world: DeferredWorld) {
    let entity_ref = deferred_world.entity(entity);
    if entity_ref.contains::<SolverBody>() {
        deferred_world
            .commands()
            .entity(entity)
            .try_remove::<(SolverBody, SolverBodyInertia)>();
    }
}

fn prepare_dynamic_solver_bodies(
    mut query: Query<
        (
            &mut SolverBody,
            &mut SolverBodyInertia,
            &LinearVelocity,
            &AngularVelocity,
            &Rotation,
            &ComputedMass,
            &ComputedAngularInertia,
            Option<&LockedAxes>,
        ),
        With<DynamicBody>,
    >,
) {
    #[allow(unused_variables)]
    query.par_iter_mut().for_each(
        |(
            mut solver_body,
            mut inertial_properties,
            linear_velocity,
            angular_velocity,
            rotation,
            mass,
            angular_inertia,
            locked_axes,
        )| {
            solver_body.linear_velocity = linear_velocity.0;
            solver_body.angular_velocity = angular_velocity.0;
            solver_body.delta_position = Vector::ZERO;
            solver_body.delta_rotation = Rotation::IDENTITY;

            let locked_axes = locked_axes.copied().unwrap_or_default();
            *inertial_properties = SolverBodyInertia::new(
                mass.inverse(),
                #[cfg(feature = "2d")]
                angular_inertia.inverse(),
                #[cfg(feature = "3d")]
                angular_inertia.rotated(rotation.0).inverse(),
                locked_axes,
            );
            solver_body.flags = SolverBodyFlags(locked_axes.to_bits() as u32);
            solver_body.flags.set(SolverBodyFlags::IS_KINEMATIC, false);

            #[cfg(feature = "3d")]
            {
                // Only compute gyroscopic motion if the following conditions are met:
                //
                // 1. Rotation is unlocked on at least one axis.
                // 2. The inertia tensor is not isotropic.
                //
                // If the inertia tensor is isotropic, it is invariant under all rotations,
                // and the same around any axis passing through the object's center of mass.
                // This is the case for shapes like spheres, as well as cubes and other regular solids.
                //
                // From Moments of inertia of Archimedean solids by Frédéric Perrier:
                // "The condition of two axes of threefold or higher rotational symmetry
                // crossing at the center of gravity is sufficient to produce an isotropic tensor of inertia.
                // The MI around any axis passing by the center of gravity are then identical."

                // TODO: Kinematic bodies should not have gyroscopic motion.
                // TODO: Should we scale the epsilon based on the `PhysicsLengthUnit`?
                // TODO: We should only do this when the body is added or the local inertia tensor is changed.
                let epsilon = 1e-6;
                let is_gyroscopic = !locked_axes.is_rotation_locked()
                    && !angular_inertia.inverse().is_isotropic(epsilon);

                solver_body
                    .flags
                    .set(SolverBodyFlags::GYROSCOPIC_MOTION, is_gyroscopic);
            }
        },
    );
}

fn prepare_kinematic_solver_bodies(
    mut query: Query<
        (
            &mut SolverBody,
            &LinearVelocity,
            &AngularVelocity,
            Option<&LockedAxes>,
        ),
        With<KinematicBody>,
    >,
) {
    #[allow(unused_variables)]
    query.par_iter_mut().for_each(
        |(mut solver_body, linear_velocity, angular_velocity, locked_axes)| {
            solver_body.linear_velocity = linear_velocity.0;
            solver_body.angular_velocity = angular_velocity.0;
            solver_body.delta_position = Vector::ZERO;
            solver_body.delta_rotation = Rotation::IDENTITY;

            let locked_axes = locked_axes.copied().unwrap_or_default();
            solver_body.flags = SolverBodyFlags(locked_axes.to_bits() as u32);
            solver_body.flags.set(SolverBodyFlags::IS_KINEMATIC, true);
        },
    );
}

/// Writes back solver body data to rigid bodies.
#[allow(clippy::type_complexity)]
fn writeback_solver_bodies(
    mut query: Query<(
        &SolverBody,
        &mut Position,
        &mut Rotation,
        Option<&ComputedCenterOfMass>,
        &mut LinearVelocity,
        &mut AngularVelocity,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = bevy::platform::time::Instant::now();

    query.par_iter_mut().for_each(
        |(solver_body, mut pos, mut rot, com, mut lin_vel, mut ang_vel)| {
            // Write back the position and rotation deltas,
            // rotating the body around its center of mass.
            let com = com.copied().unwrap_or_default();
            let old_world_com = *rot * com.0;
            *rot = (solver_body.delta_rotation * *rot).fast_renormalize();
            let new_world_com = *rot * com.0;
            pos.0 += solver_body.delta_position + old_world_com - new_world_com;

            // Write back velocities.
            lin_vel.0 = solver_body.linear_velocity;
            ang_vel.0 = solver_body.angular_velocity;
        },
    );

    diagnostics.finalize += start.elapsed();
}

#[cfg(feature = "3d")]
pub(crate) fn update_solver_body_angular_inertia(
    mut query: Query<(&mut SolverBodyInertia, &ComputedAngularInertia, &Rotation)>,
) {
    query
        .par_iter_mut()
        .for_each(|(mut inertia, angular_inertia, rotation)| {
            inertia.update_effective_inv_angular_inertia(angular_inertia, rotation.0);
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PhysicsSchedulePlugin, SolverSchedulePlugin, prelude::StaticBody};

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
        app.world().get::<SolverBody>(entity).is_some()
    }

    #[test]
    fn add_remove_solver_bodies() {
        let mut app = create_app();

        // Create a dynamic, kinematic, and static rigid body.
        let entity1 = app.world_mut().spawn(DynamicBody).id();
        let entity2 = app.world_mut().spawn(KinematicBody).id();
        let entity3 = app.world_mut().spawn(StaticBody).id();

        app.update();

        // The dynamic and kinematic rigid bodies should have solver bodies.
        assert!(has_solver_body(&app, entity1));
        assert!(has_solver_body(&app, entity2));
        assert!(!has_solver_body(&app, entity3));

        // Disable the dynamic rigid body.
        app.world_mut()
            .entity_mut(entity1)
            .insert(RigidBodyDisabled);

        app.update();

        // The entity should no longer have a solver body.
        assert!(!has_solver_body(&app, entity1));

        // Enable the dynamic rigid body.
        app.world_mut()
            .entity_mut(entity1)
            .remove::<RigidBodyDisabled>();

        app.update();

        // The dynamic rigid body should have a solver body again.
        assert!(has_solver_body(&app, entity1));
    }
}
