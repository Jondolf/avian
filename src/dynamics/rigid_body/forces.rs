//! Forces, torques, linear impulses, and angular impulses
//! that can be applied to dynamic rigid bodies.

#![allow(missing_docs)]

use crate::{
    dynamics::{
        integrator::{self, IntegrationSet, VelocityIntegrationData},
        solver::{
            solver_body::{SolverBody, SolverBodyInertia},
            SolverDiagnostics,
        },
    },
    prelude::*,
};
use bevy::{
    ecs::{
        entity::EntityHashMap,
        query::{QueryData, QueryEntityError},
        system::{
            lifetimeless::{Read, Write},
            SystemParam,
        },
    },
    prelude::*,
};

#[cfg(feature = "2d")]
pub(crate) type Torque = Scalar;

#[cfg(feature = "3d")]
pub(crate) type Torque = Vector;

#[cfg(feature = "2d")]
pub(crate) trait FloatZero {
    const ZERO: Self;
}

#[cfg(feature = "2d")]
impl FloatZero for Scalar {
    const ZERO: Self = 0.0;
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct RigidBodyForceQuery {
    // TODO: If we apply forces using an offset from the center of mass,
    //       we don't need `Position` or `ComputedCenterOfMass` here.
    position: Read<Position>,
    rotation: Read<Rotation>,
    linear_velocity: Write<LinearVelocity>,
    angular_velocity: Write<AngularVelocity>,
    mass_props: Read<SolverBodyInertia>,
    center_of_mass: Read<ComputedCenterOfMass>,
    // TODO: Use `SolverBodyInertia` for the locked axes.
    locked_axes: Option<Read<LockedAxes>>,
    integration: Write<VelocityIntegrationData>,
    accumulated_world_forces: Option<Write<AccumulatedWorldForces>>,
    accumulated_local_forces: Option<Write<AccumulatedLocalForces>>,
    accumulated_local_acceleration: Option<Write<AccumulatedLocalAcceleration>>,
}

#[derive(SystemParam)]
pub struct ForceHelper<'w, 's> {
    query: Query<'w, 's, RigidBodyForceQuery>,
    commands: Commands<'w, 's>,
    insertion_buffers: Local<'s, ForceInsertionBuffers>,
}

pub struct EntityForces<'w> {
    entity: Entity,
    body: RigidBodyForceQueryItem<'w>,
    insertion_buffers: &'w mut ForceInsertionBuffers,
}

#[derive(Default)]
struct ForceInsertionBuffers {
    world_forces: EntityHashMap<AccumulatedWorldForces>,
    local_forces: EntityHashMap<AccumulatedLocalForces>,
    local_acceleration: EntityHashMap<AccumulatedLocalAcceleration>,
}

impl Drop for ForceHelper<'_, '_> {
    fn drop(&mut self) {
        // Batch insert all the new forces and accelerations that were applied.
        if !self.insertion_buffers.world_forces.is_empty() {
            let forces = self.insertion_buffers.world_forces.drain();
            self.commands.try_insert_batch(forces.collect::<Vec<_>>());
        }
        if !self.insertion_buffers.local_forces.is_empty() {
            let forces = self.insertion_buffers.local_forces.drain();
            self.commands.try_insert_batch(forces.collect::<Vec<_>>());
        }
        if !self.insertion_buffers.local_acceleration.is_empty() {
            let accelerations = self.insertion_buffers.local_acceleration.drain();
            self.commands
                .try_insert_batch(accelerations.collect::<Vec<_>>());
        }
    }
}

// TODO: Iterators for efficiently querying entities without calling `Query::get_mut`
// TODO: Helpers for triggering explosions
impl ForceHelper<'_, '_> {
    /// Gets [`EntityForces`] for the given rigid body entity.
    ///
    /// # Panics
    ///
    /// Panics if the entity does not exist or is not a dynamic rigid body.
    pub fn entity(&mut self, entity: Entity) -> EntityForces {
        self.get_entity(entity).unwrap_or_else(|_| {
            panic!("Entity {entity} does not exist or is not a dynamic rigid body.")
        })
    }

    /// Tries to get [`EntityForces`] for the given rigid body entity.
    ///
    /// # Errors
    ///
    /// Returns a [`QueryEntityError`] if the entity does not exist or is not a dynamic rigid body.
    pub fn get_entity(&mut self, entity: Entity) -> Result<EntityForces, QueryEntityError> {
        self.query.get_mut(entity).map(|body| EntityForces {
            entity,
            body,
            insertion_buffers: &mut self.insertion_buffers,
        })
    }
}

// TODO: Helpers for velocity
impl EntityForces<'_> {
    /// Applies a linear impulse in local space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    pub fn apply_local_linear_impulse(&mut self, impulse: Vector) {
        let world_impulse = *self.body.rotation * impulse;
        self.apply_linear_impulse(world_impulse);
    }

    /// Applies a linear impulse at the center of mass in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    pub fn apply_linear_impulse(&mut self, impulse: Vector) {
        self.body.linear_velocity.0 += self.body.mass_props.effective_inv_mass() * impulse;
    }

    /// Applies a linear impulse at the given point in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// If the point is not at the center of mass, the impulse will also generate an angular impulse.
    ///
    /// The impulse modifies the [`LinearVelocity`] and [`AngularVelocity`] of the body immediately.
    pub fn apply_linear_impulse_at_point(&mut self, impulse: Vector, world_point: Vector) {
        let world_center_of_mass =
            self.body.position.0 + *self.body.rotation * self.body.center_of_mass.0;
        self.apply_linear_impulse(impulse);
        self.apply_angular_impulse(cross(world_point - world_center_of_mass, impulse));
    }

    /// Applies an angular impulse in world space. The unit is typically N⋅m⋅s or kg⋅m^2/s.
    ///
    /// The impulse modifies the [`AngularVelocity`] of the body immediately.
    pub fn apply_angular_impulse(&mut self, impulse: Torque) {
        self.body.angular_velocity.0 +=
            self.body.mass_props.effective_inv_angular_inertia() * impulse;
    }

    /// Applies a force at the center of mass in local space. The unit is typically N or kg⋅m/s^2.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    // TODO: Do we also want `apply_local_force`? It's not clear what the point should be relative to.
    pub fn apply_local_force(&mut self, force: Vector) {
        if let Some(ref mut accumulated) = self.body.accumulated_local_forces {
            accumulated.force += force;
        } else {
            self.insertion_buffers
                .local_forces
                .entry(self.entity)
                .or_default()
                .force += force;
        }
    }

    /// Applies a force at the center of mass in world space. The unit is typically N or kg⋅m/s^2.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    pub fn apply_force(&mut self, force: Vector) {
        if let Some(ref mut accumulated) = self.body.accumulated_world_forces {
            accumulated.force += force;
        } else {
            self.insertion_buffers
                .world_forces
                .entry(self.entity)
                .or_default()
                .force += force;
        }
    }

    /// Applies a force at the given point in world space. The unit is typically N or kg⋅m/s^2.
    ///
    /// If the point is not at the center of mass, the force will also generate a torque.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    pub fn apply_force_at_point(&mut self, force: Vector, world_point: Vector) {
        // Note: This does not consider the rotation of the body during substeps,
        //       so the torque may not be accurate if the body is rotating quickly.
        let world_center_of_mass =
            self.body.position.0 + *self.body.rotation * self.body.center_of_mass.0;
        self.apply_force(force);
        self.apply_torque(cross(world_point - world_center_of_mass, force));
    }

    /// Applies a torque in local space. The unit is typically N⋅m or kg⋅m^2/s^2.
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    ///
    /// **Note:** This does not consider the rotation of the body during substeps,
    ///           so the torque may not be accurate if the body is rotating quickly.
    #[cfg(feature = "3d")]
    pub fn apply_local_torque(&mut self, torque: Torque) {
        if let Some(ref mut accumulated) = self.body.accumulated_local_forces {
            accumulated.torque += torque;
        } else {
            self.insertion_buffers
                .local_forces
                .entry(self.entity)
                .or_default()
                .torque += torque;
        }
    }

    /// Applies a torque in world space. The unit is typically N⋅m or kg⋅m^2/s^2.
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    pub fn apply_torque(&mut self, torque: Torque) {
        if let Some(ref mut forces) = self.body.accumulated_world_forces {
            forces.torque += torque;
        } else {
            self.insertion_buffers
                .world_forces
                .entry(self.entity)
                .or_default()
                .torque += torque;
        }
    }

    /// Applies a linear acceleration in local space, ignoring mass. The unit is typically m/s^2.
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    pub fn apply_local_linear_acceleration(&mut self, acceleration: Vector) {
        if let Some(ref mut accumulated) = self.body.accumulated_local_acceleration {
            accumulated.linear += acceleration;
        } else {
            self.insertion_buffers
                .local_acceleration
                .entry(self.entity)
                .or_default()
                .linear += acceleration;
        }
    }

    /// Applies a linear acceleration, ignoring mass. The unit is typically m/s^2.
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    pub fn apply_linear_acceleration(&mut self, acceleration: Vector) {
        // TODO: Technically we don't need to apply locked axes here since it's applied in the solver.
        let locked_axes = self.body.locked_axes.copied().unwrap_or_default();
        self.body
            .integration
            .apply_linear_acceleration(locked_axes.apply_to_vec(acceleration));
    }

    /// Applies an angular acceleration in local space, ignoring angular inertia. The unit is rad/s^2.
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[cfg(feature = "3d")]
    pub fn apply_local_angular_acceleration(
        &mut self,
        #[cfg(feature = "2d")] acceleration: f32,
        #[cfg(feature = "3d")] acceleration: Vector,
    ) {
        if let Some(ref mut accumulated) = self.body.accumulated_local_acceleration {
            accumulated.angular += acceleration;
        } else {
            self.insertion_buffers
                .local_acceleration
                .entry(self.entity)
                .or_default()
                .angular += acceleration;
        }
    }

    /// Applies an angular acceleration, ignoring angular inertia. The unit is rad/s^2.
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    pub fn apply_angular_acceleration(
        &mut self,
        #[cfg(feature = "2d")] acceleration: f32,
        #[cfg(feature = "3d")] acceleration: Vector,
    ) {
        // TODO: Technically we don't need to apply locked axes here since it's applied in the solver.
        let locked_axes = self.body.locked_axes.copied().unwrap_or_default();
        self.body
            .integration
            .apply_angular_acceleration(locked_axes.apply_to_angular_velocity(acceleration));
    }

    /// Returns the external forces that the body has accumulated
    /// before the physics step in world space.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only forces applied through [`ForceHelper`] are included.
    pub fn accumulated_force(&self) -> Vector {
        let world_force = if let Some(ref forces) = self.body.accumulated_world_forces {
            forces.force
        } else {
            self.insertion_buffers
                .world_forces
                .get(&self.entity)
                .map_or(Vector::ZERO, |forces| forces.force)
        };
        let local_force = if let Some(ref forces) = self.body.accumulated_local_forces {
            forces.force
        } else {
            self.insertion_buffers
                .local_forces
                .get(&self.entity)
                .map_or(Vector::ZERO, |forces| forces.force)
        };

        // Return the total world-space force.
        world_force + self.body.rotation * local_force
    }

    /// Returns the external torque that the body has accumulated
    /// before the physics step.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only torques applied through [`ForceHelper`] are included.
    #[cfg(feature = "2d")]
    pub fn accumulated_torque(&self) -> Torque {
        if let Some(ref forces) = self.body.accumulated_world_forces {
            forces.torque
        } else {
            self.insertion_buffers
                .world_forces
                .get(&self.entity)
                .map_or(Torque::ZERO, |forces| forces.torque)
        }
    }

    /// Returns the external torque that the body has accumulated
    /// before the physics step in world space.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only torques applied through [`ForceHelper`] are included.
    #[cfg(feature = "3d")]
    pub fn accumulated_torque(&self) -> Torque {
        let world_torque = if let Some(ref forces) = self.body.accumulated_world_forces {
            forces.torque
        } else {
            self.insertion_buffers
                .world_forces
                .get(&self.entity)
                .map_or(Torque::ZERO, |forces| forces.torque)
        };
        let local_torque = if let Some(ref forces) = self.body.accumulated_local_forces {
            forces.torque
        } else {
            self.insertion_buffers
                .local_forces
                .get(&self.entity)
                .map_or(Torque::ZERO, |forces| forces.torque)
        };

        // Return the total world-space torque.
        world_torque + self.body.rotation * local_torque
    }

    /// Returns the linear acceleration that the body has accumulated
    /// before the physics step in world space.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only accelerations applied through [`ForceHelper`] are included.
    pub fn accumulated_linear_acceleration(&self) -> Vector {
        // The linear increment is treated as linear acceleration until the integration step.
        let world_linear_acceleration = self.body.integration.linear_increment;
        let local_linear_acceleration =
            if let Some(ref forces) = self.body.accumulated_local_acceleration {
                forces.linear
            } else {
                self.insertion_buffers
                    .local_acceleration
                    .get(&self.entity)
                    .map_or(Vector::ZERO, |forces| forces.linear)
            };

        // Return the total world-space linear acceleration.
        world_linear_acceleration + self.body.rotation * local_linear_acceleration
    }

    /// Returns the angular acceleration that the body has accumulated
    /// before the physics step.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only accelerations applied through [`ForceHelper`] are included.
    #[cfg(feature = "2d")]
    pub fn accumulated_angular_acceleration(&self) -> Torque {
        // The angular increment is treated as angular acceleration until the integration step.
        self.body.integration.angular_increment
    }

    /// Returns the angular acceleration that the body has accumulated
    /// before the physics step in world space.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only accelerations applied through [`ForceHelper`] are included.
    #[cfg(feature = "3d")]
    pub fn accumulated_angular_acceleration(&self) -> Torque {
        // The angular increment is treated as angular acceleration until the integration step.
        let world_angular_acceleration = self.body.integration.angular_increment;
        let local_angular_acceleration =
            if let Some(ref forces) = self.body.accumulated_local_acceleration {
                forces.angular
            } else {
                self.insertion_buffers
                    .local_acceleration
                    .get(&self.entity)
                    .map_or(Vector::ZERO, |forces| forces.angular)
            };

        // Return the total world-space angular acceleration.
        world_angular_acceleration + self.body.rotation * local_angular_acceleration
    }

    /// Resets the accumulated forces to zero.
    pub fn reset_accumulated_force(&mut self) {
        if let Some(ref mut forces) = self.body.accumulated_world_forces {
            forces.force = Vector::ZERO;
        } else {
            self.insertion_buffers
                .world_forces
                .entry(self.entity)
                .or_default()
                .force = Vector::ZERO;
        }
        if let Some(ref mut forces) = self.body.accumulated_local_forces {
            forces.force = Vector::ZERO;
        } else {
            self.insertion_buffers
                .local_forces
                .entry(self.entity)
                .or_default()
                .force = Vector::ZERO;
        }
    }

    /// Resets the accumulated torque to zero.
    pub fn reset_accumulated_torque(&mut self) {
        if let Some(ref mut forces) = self.body.accumulated_world_forces {
            forces.torque = Torque::ZERO;
        } else {
            self.insertion_buffers
                .world_forces
                .entry(self.entity)
                .or_default()
                .torque = Torque::ZERO;
        }
        #[cfg(feature = "3d")]
        {
            if let Some(ref mut forces) = self.body.accumulated_local_forces {
                forces.torque = Torque::ZERO;
            } else {
                self.insertion_buffers
                    .local_forces
                    .entry(self.entity)
                    .or_default()
                    .torque = Torque::ZERO;
            }
        }
    }

    /// Resets the accumulated linear acceleration to zero.
    pub fn reset_accumulated_linear_acceleration(&mut self) {
        self.body.integration.linear_increment = Vector::ZERO;
        if let Some(ref mut forces) = self.body.accumulated_local_acceleration {
            forces.linear = Vector::ZERO;
        } else {
            self.insertion_buffers
                .local_acceleration
                .entry(self.entity)
                .or_default()
                .linear = Vector::ZERO;
        }
    }

    /// Resets the accumulated angular acceleration to zero.
    pub fn reset_accumulated_angular_acceleration(&mut self) {
        self.body.integration.angular_increment = Torque::ZERO;
        #[cfg(feature = "3d")]
        {
            if let Some(ref mut forces) = self.body.accumulated_local_acceleration {
                forces.angular = Vector::ZERO;
            } else {
                self.insertion_buffers
                    .local_acceleration
                    .entry(self.entity)
                    .or_default()
                    .angular = Vector::ZERO;
            }
        }
    }
}

// TODO: Should accumulated forces and accelerations be `SparseSet` components?

/// A component with the user-applied world forces and torques
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero world forces have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedWorldForces {
    pub force: Vector,
    pub torque: Torque,
}

/// A component with the user-applied local forces and torques
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero local forces have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedLocalForces {
    pub force: Vector,
    pub torque: Torque,
}

/// A component with the user-applied local acceleration
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero local acceleration have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedLocalAcceleration {
    pub linear: Vector,
    #[cfg(feature = "3d")]
    pub angular: Vector,
}

pub struct ForcePlugin;

impl Plugin for ForcePlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<(
            AccumulatedWorldForces,
            AccumulatedLocalForces,
            AccumulatedLocalAcceleration,
        )>();

        app.configure_sets(
            PhysicsSchedule,
            (
                ForceSet::ApplyWorldForces
                    .in_set(IntegrationSet::UpdateVelocityIncrements)
                    .before(integrator::pre_process_velocity_increments),
                ForceSet::Clear.in_set(SolverSet::PostSubstep),
            ),
        );

        app.configure_sets(
            SubstepSchedule,
            ForceSet::ApplyLocalForces
                .in_set(IntegrationSet::Velocity)
                .before(integrator::integrate_velocities),
        );

        app.add_systems(
            PhysicsSchedule,
            apply_world_forces.in_set(ForceSet::ApplyWorldForces),
        );
        app.add_systems(
            SubstepSchedule,
            (apply_local_forces, apply_local_acceleration)
                .chain()
                .in_set(ForceSet::ApplyLocalForces),
        );

        app.add_systems(
            PhysicsSchedule,
            (
                clear_accumulated_world_forces,
                clear_accumulated_local_forces,
                clear_accumulated_local_acceleration,
            )
                .in_set(ForceSet::Clear),
        );
    }
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ForceSet {
    ApplyWorldForces,
    ApplyLocalForces,
    Clear,
}

/// Clears [`AccumulatedWorldForces`] for all rigid bodies.
///
/// Continuously applied forces and torques are only reset to zero,
/// while forces and torques that were already zero for an entire time step
/// are removed from the entities.
fn clear_accumulated_world_forces(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AccumulatedWorldForces)>,
) {
    // Initialize a buffer for entities to remove the component from.
    let mut entity_buffer = Vec::new();

    for (entity, mut forces) in &mut query {
        if forces.force != Vector::ZERO || forces.torque != Torque::ZERO {
            // The force or torque was not zero, so these may be continuously applied forces.
            // Just reset the forces and keep the component.
            forces.force = Vector::ZERO;
            forces.torque = Torque::ZERO;
        } else {
            // No forces or torques were applied for an entire time step, so we can remove the component.
            entity_buffer.push(entity);
        }
    }

    if entity_buffer.is_empty() {
        return;
    }

    // Remove the component from all entities that had no forces or torques applied.
    commands.queue(|world: &mut World| {
        entity_buffer.into_iter().for_each(|entity| {
            world.entity_mut(entity).remove::<AccumulatedWorldForces>();
        });
    });
}

/// Clears [`AccumulatedLocalForces`] for all rigid bodies.
///
/// Continuously applied forces and torques are only reset to zero,
/// while forces and torques that were already zero for an entire time step
/// are removed from the entities.
fn clear_accumulated_local_forces(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AccumulatedLocalForces)>,
) {
    // Initialize a buffer for entities to remove the component from.
    let mut entity_buffer = Vec::new();

    for (entity, mut forces) in &mut query {
        if forces.force != Vector::ZERO || forces.torque != Torque::ZERO {
            // The force or torque was not zero, so these may be continuously applied forces.
            // Just reset the forces and keep the component.
            forces.force = Vector::ZERO;
            forces.torque = Torque::ZERO;
        } else {
            // No forces or torques were applied for an entire time step, so we can remove the component.
            entity_buffer.push(entity);
        }
    }

    if entity_buffer.is_empty() {
        return;
    }

    // Remove the component from all entities that had no forces or torques applied.
    commands.queue(|world: &mut World| {
        entity_buffer.into_iter().for_each(|entity| {
            world.entity_mut(entity).remove::<AccumulatedLocalForces>();
        });
    });
}

fn clear_accumulated_local_acceleration(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AccumulatedLocalAcceleration)>,
) {
    // Initialize a buffer for entities to remove the component from.
    let mut entity_buffer = Vec::new();

    for (entity, mut acceleration) in &mut query {
        #[cfg(feature = "2d")]
        let non_zero_acceleration = acceleration.linear != Vector::ZERO;
        #[cfg(feature = "3d")]
        let non_zero_acceleration =
            acceleration.linear != Vector::ZERO || acceleration.angular != Vector::ZERO;
        if non_zero_acceleration {
            // The acceleration was not zero, so this may be a continuously applied acceleration.
            // Just reset the acceleration and keep the component.
            acceleration.linear = Vector::ZERO;
            #[cfg(feature = "3d")]
            {
                acceleration.angular = Vector::ZERO;
            }
        } else {
            // No acceleration was applied for an entire time step, so we can remove the component.
            entity_buffer.push(entity);
        }
    }

    if entity_buffer.is_empty() {
        return;
    }

    // Remove the component from all entities that had no acceleration applied.
    commands.queue(|world: &mut World| {
        entity_buffer.into_iter().for_each(|entity| {
            world
                .entity_mut(entity)
                .remove::<AccumulatedLocalAcceleration>();
        });
    });
}

/// Applies gravity and locked axes to the linear and angular velocity increments of bodies.
fn apply_world_forces(
    mut bodies: Query<(
        &mut VelocityIntegrationData,
        &AccumulatedWorldForces,
        &SolverBodyInertia,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    // TODO: Do we want to skip kinematic bodies here?
    bodies
        .par_iter_mut()
        .for_each(|(mut integration, forces, mass_props)| {
            // NOTE: The velocity increments are treated as accelerations at this point.

            // Apply external forces and torques.
            // NOTE: We ignore changes in the inertia tensor, keeping angular acceleration constant across substeps.
            //       This may not be entirely correct, but is generally acceptable for world-space torque.
            integration.linear_increment += mass_props.effective_inv_mass() * forces.force;
            integration.angular_increment +=
                mass_props.effective_inv_angular_inertia() * forces.torque;

            // The `IntegrationPlugin` will take care of applying the time step
            // and locked axes to the velocity increments.
        });

    diagnostics.update_velocity_increments += start.elapsed();
}

/// Applies [`AccumulatedLocalForces`] to the linear and angular velocity of bodies.
fn apply_local_forces(
    mut bodies: Query<
        (
            &mut SolverBody,
            &AccumulatedLocalForces,
            &Rotation,
            &SolverBodyInertia,
            Option<&LockedAxes>,
        ),
        With<SolverBody>,
    >,
    time: Res<Time<Substeps>>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_secs_f64() as Scalar;

    bodies
        .par_iter_mut()
        .for_each(|(mut body, forces, rotation, mass_props, locked_axes)| {
            let rotation = body.delta_rotation * *rotation;
            let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

            // NOTE:
            //
            // We could have a `LocalVelocityIncrements` component and apply these forces and torques
            // to that once per time step rather than once per substep. However, that would have two caveats:
            //
            // 1. We need to store and manage both `AccumulatedLocalForces` and `LocalVelocityIncrements`
            //    when local forces are applied.
            // 2. Changes in the inertia tensor during substeps would not be considered.
            //    (though we currently accept this for world-space torque)

            // Compute the world-space accelerations with locked axes applied.
            let linear_acceleration = locked_axes
                .apply_to_vec(mass_props.effective_inv_mass() * (rotation * forces.force));
            #[cfg(feature = "3d")]
            let angular_acceleration = locked_axes.apply_to_angular_velocity(
                mass_props.effective_inv_angular_inertia() * (rotation * forces.torque),
            );

            // Apply external forces and torques.
            body.linear_velocity += linear_acceleration * delta_secs;
            #[cfg(feature = "3d")]
            {
                body.angular_velocity += angular_acceleration * delta_secs;
            }
        });

    diagnostics.integrate_velocities += start.elapsed();
}

/// Applies [`AccumulatedLocalAcceleration`] to the linear and angular velocity of bodies.
///
/// This should run in the substepping loop, just before [`IntegrationSet::Velocity`].
fn apply_local_acceleration(
    mut bodies: Query<(
        &mut SolverBody,
        &AccumulatedLocalAcceleration,
        &Rotation,
        Option<&LockedAxes>,
    )>,
    mut diagnostics: ResMut<SolverDiagnostics>,
    time: Res<Time<Substeps>>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_secs_f64() as Scalar;

    bodies
        .par_iter_mut()
        .for_each(|(mut body, acceleration, rotation, locked_axes)| {
            let rotation = body.delta_rotation * *rotation;
            let locked_axes = locked_axes.map_or(LockedAxes::default(), |locked_axes| *locked_axes);

            // Compute the world space velocity increments with locked axes applied.
            let world_linear_acceleration =
                locked_axes.apply_to_vec(rotation * acceleration.linear);
            #[cfg(feature = "3d")]
            let world_angular_acceleration =
                locked_axes.apply_to_vec(rotation * acceleration.angular);

            // Apply acceleration.
            body.linear_velocity += world_linear_acceleration * delta_secs;
            #[cfg(feature = "3d")]
            {
                body.angular_velocity += world_angular_acceleration * delta_secs;
            }
        });

    diagnostics.integrate_velocities += start.elapsed();
}
