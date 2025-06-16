use crate::{
    dynamics::{integrator::VelocityIntegrationData, solver::solver_body::SolverBodyInertia},
    prelude::{mass_properties::components::GlobalCenterOfMass, *},
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

use super::{AccumulatedLocalAcceleration, AccumulatedLocalForces, AccumulatedWorldForces};

#[derive(QueryData)]
#[query_data(mutable)]
pub struct RigidBodyForceQuery {
    // TODO: This is currently an `Option`, because `Rotation` may not be present until physics has run.
    //       We should properly initialize `Rotation` on spawn.
    rotation: Option<Read<Rotation>>,
    linear_velocity: Write<LinearVelocity>,
    angular_velocity: Write<AngularVelocity>,
    mass_props: Read<SolverBodyInertia>,
    global_center_of_mass: Read<GlobalCenterOfMass>,
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
        let world_impulse = self.body.rotation.copied().unwrap_or_default() * impulse;
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
        self.apply_linear_impulse(impulse);
        self.apply_angular_impulse(cross(
            world_point - self.body.global_center_of_mass.get(),
            impulse,
        ));
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
        self.apply_force(force);
        self.apply_torque(cross(
            world_point - self.body.global_center_of_mass.get(),
            force,
        ));
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
        self.body
            .integration
            .apply_linear_acceleration(acceleration);
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
        self.body
            .integration
            .apply_angular_acceleration(acceleration);
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
        world_force + self.body.rotation.copied().unwrap_or_default() * local_force
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
        world_torque + self.body.rotation.copied().unwrap_or_default() * local_torque
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
        world_linear_acceleration
            + self.body.rotation.copied().unwrap_or_default() * local_linear_acceleration
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
        world_angular_acceleration
            + self.body.rotation.copied().unwrap_or_default() * local_angular_acceleration
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
