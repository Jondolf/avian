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

/// A [`SystemParam`] for applying forces, torques, impulses, and accelerations to dynamic [rigid bodies](RigidBody).
///
/// For constant forces that persist across time steps, consider using components like [`ConstantForce`] instead.
///
/// See the [module-level documentation](crate::dynamics::rigid_body::forces) for more general information about forces in Avian.
///
/// # Usage
///
/// To use the [`ForceHelper`], add it as a system parameter to your system, and use the [`entity`](ForceHelper::entity)
/// method to get access to [`EntityForces`] for applying forces, impulses, and acceleration to that entity.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
#[cfg_attr(feature = "serialize", doc = "# use bevy::prelude::*;")]
/// #
/// # #[cfg_attr(feature = "f32")]
/// fn apply_forces(query: Query<Entity, With<RigidBody>>, forces: ForceHelper) {
///     for entity in &mut query {
///         // Apply a force of 10 N in the positive Y direction to `entity`.
#[cfg_attr(
    feature = "2d",
    doc = "        forces.entity(entity).apply_force(Vec2::new(0.0, 10.0));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        forces.entity(entity).apply_force(Vec3::new(0.0, 10.0, 0.0));"
)]
///     }
/// }
/// ```
///
/// The force is applied continuously during the physics step, and cleared after the step is complete.
/// The [`ForceHelper`] manages everything for you, so there is no need to add or remove any components manually.
///
/// The [`ForceHelper`] can also apply forces and impulses at a specific point in the world.
/// If the point is not aligned with the [`GlobalCenterOfMass`], it will also apply a torque to the body.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
#[cfg_attr(feature = "serialize", doc = "# use bevy::prelude::*;")]
/// #
/// # fn apply_impulses(query: Query<Entity, With<RigidBody>>, forces: ForceHelper) {
/// #     for entity in &mut query {
/// #         let force = Vector::default();
/// #         let point = Vector::default();
/// // Apply an impulse at a specific point in the world.
/// // Unlike forces, impulses are applied immediately to the velocity,
/// forces.entity(entity).apply_linear_impulse_at_point(force, point);
/// #     }
/// # }
/// ```
///
/// As an example, you could implement radial gravity that pulls rigid bodies towards the world origin
/// with a system like the following:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # #[cfg(feature = "f32")]
/// fn radial_gravity(query: Query<(Entity, &GlobalTransform), With<RigidBody>>, forces: ForceHelper) {
///     for (entity, global_transform) in &mut query {
///         // Compute the direction towards the center of the world.
///         let direction = -global_transform.translation().normalize_or_zero();
///         // Apply a linear acceleration of 9.81 m/s² towards the center of the world.
#[cfg_attr(
    feature = "2d",
    doc = "        forces.entity(entity).apply_linear_acceleration(direction.truncate() * 9.81);"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        forces.entity(entity).apply_linear_acceleration(direction * 9.81);"
)]
///     }
/// }
/// ```
#[derive(SystemParam)]
pub struct ForceHelper<'w, 's> {
    query: Query<'w, 's, RigidBodyForceQuery>,
    commands: Commands<'w, 's>,
    insertion_buffers: Local<'s, ForceInsertionBuffers>,
}

#[derive(QueryData)]
#[query_data(mutable)]
struct RigidBodyForceQuery {
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

/// Provides APIs for applying forces, impulses, and accelerations to a [rigid body](RigidBody) entity.
///
/// This is returned by [`ForceHelper::entity`] and [`ForceHelper::get_entity`].
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
    /// Returns [`EntityForces`] for the given rigid body entity.
    ///
    /// # Panics
    ///
    /// Panics if the entity does not exist or is not a dynamic rigid body.
    #[inline]
    pub fn entity(&mut self, entity: Entity) -> EntityForces<'_> {
        self.get_entity(entity).unwrap_or_else(|_| {
            panic!("Entity {entity} does not exist or is not a dynamic rigid body.")
        })
    }

    /// Tries to return [`EntityForces`] for the given rigid body entity.
    ///
    /// # Errors
    ///
    /// Returns a [`QueryEntityError`] if the entity does not exist or is not a dynamic rigid body.
    #[inline]
    pub fn get_entity(&mut self, entity: Entity) -> Result<EntityForces<'_>, QueryEntityError> {
        self.query.get_mut(entity).map(|body| EntityForces {
            entity,
            body,
            insertion_buffers: &mut self.insertion_buffers,
        })
    }
}

impl EntityForces<'_> {
    /// Applies a force at the center of mass in world space. The unit is typically N or kg⋅m/s².
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    #[inline]
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

    /// Applies a force at the given point in world space. The unit is typically N or kg⋅m/s².
    ///
    /// If the point is not at the center of mass, the force will also generate a torque.
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_force_at_point(&mut self, force: Vector, world_point: Vector) {
        // Note: This does not consider the rotation of the body during substeps,
        //       so the torque may not be accurate if the body is rotating quickly.
        self.apply_force(force);
        self.apply_torque(cross(
            world_point - self.body.global_center_of_mass.get(),
            force,
        ));
    }

    /// Applies a force at the center of mass in local space. The unit is typically N or kg⋅m/s².
    ///
    /// The force is applied continuously over the physics step and cleared afterwards.
    #[inline]
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

    /// Applies a torque in world space. The unit is typically N⋅m or kg⋅m²/s².
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_torque(&mut self, torque: AngularVector) {
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

    /// Applies a torque in local space. The unit is typically N⋅m or kg⋅m²/s².
    ///
    /// The torque is applied continuously over the physics step and cleared afterwards.
    ///
    /// **Note:** This does not consider the rotation of the body during substeps,
    ///           so the torque may not be accurate if the body is rotating quickly.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn apply_local_torque(&mut self, torque: AngularVector) {
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

    /// Applies a linear impulse at the center of mass in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    #[inline]
    pub fn apply_linear_impulse(&mut self, impulse: Vector) {
        self.body.linear_velocity.0 += self.body.mass_props.effective_inv_mass() * impulse;
    }

    /// Applies a linear impulse at the given point in world space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// If the point is not at the center of mass, the impulse will also generate an angular impulse.
    ///
    /// The impulse modifies the [`LinearVelocity`] and [`AngularVelocity`] of the body immediately.
    #[inline]
    pub fn apply_linear_impulse_at_point(&mut self, impulse: Vector, world_point: Vector) {
        self.apply_linear_impulse(impulse);
        self.apply_angular_impulse(cross(
            world_point - self.body.global_center_of_mass.get(),
            impulse,
        ));
    }

    /// Applies a linear impulse in local space. The unit is typically N⋅s or kg⋅m/s.
    ///
    /// The impulse modifies the [`LinearVelocity`] of the body immediately.
    #[inline]
    pub fn apply_local_linear_impulse(&mut self, impulse: Vector) {
        let world_impulse = self.body.rotation.copied().unwrap_or_default() * impulse;
        self.apply_linear_impulse(world_impulse);
    }

    /// Applies an angular impulse in world space. The unit is typically N⋅m⋅s or kg⋅m²/s.
    ///
    /// The impulse modifies the [`AngularVelocity`] of the body immediately.
    #[inline]
    pub fn apply_angular_impulse(&mut self, impulse: AngularVector) {
        self.body.angular_velocity.0 +=
            self.body.mass_props.effective_inv_angular_inertia() * impulse;
    }

    /// Applies an angular impulse in local space. The unit is typically N⋅m⋅s or kg⋅m²/s.
    ///
    /// The impulse modifies the [`AngularVelocity`] of the body immediately.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn apply_local_angular_impulse(&mut self, impulse: AngularVector) {
        let world_impulse = self.body.rotation.copied().unwrap_or_default() * impulse;
        self.apply_angular_impulse(world_impulse);
    }

    /// Applies a linear acceleration, ignoring mass. The unit is typically m/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_linear_acceleration(&mut self, acceleration: Vector) {
        self.body
            .integration
            .apply_linear_acceleration(acceleration);
    }

    /// Applies a linear acceleration in local space, ignoring mass. The unit is typically m/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[inline]
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

    /// Applies an angular acceleration, ignoring angular inertia. The unit is rad/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[inline]
    pub fn apply_angular_acceleration(
        &mut self,
        #[cfg(feature = "2d")] acceleration: Scalar,
        #[cfg(feature = "3d")] acceleration: Vector,
    ) {
        self.body
            .integration
            .apply_angular_acceleration(acceleration);
    }

    /// Applies an angular acceleration in local space, ignoring angular inertia. The unit is rad/s².
    ///
    /// The acceleration is applied continuously over the physics step and cleared afterwards.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn apply_local_angular_acceleration(
        &mut self,
        #[cfg(feature = "2d")] acceleration: Scalar,
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

    /// Returns the external forces that the body has accumulated
    /// before the physics step in world space.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only forces applied through [`ForceHelper`] are included.
    #[inline]
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
    #[inline]
    pub fn accumulated_torque(&self) -> AngularVector {
        if let Some(ref forces) = self.body.accumulated_world_forces {
            forces.torque
        } else {
            self.insertion_buffers
                .world_forces
                .get(&self.entity)
                .map_or(AngularVector::ZERO, |forces| forces.torque)
        }
    }

    /// Returns the external torque that the body has accumulated
    /// before the physics step in world space.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only torques applied through [`ForceHelper`] are included.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn accumulated_torque(&self) -> AngularVector {
        let world_torque = if let Some(ref forces) = self.body.accumulated_world_forces {
            forces.torque
        } else {
            self.insertion_buffers
                .world_forces
                .get(&self.entity)
                .map_or(AngularVector::ZERO, |forces| forces.torque)
        };
        let local_torque = if let Some(ref forces) = self.body.accumulated_local_forces {
            forces.torque
        } else {
            self.insertion_buffers
                .local_forces
                .get(&self.entity)
                .map_or(AngularVector::ZERO, |forces| forces.torque)
        };

        // Return the total world-space torque.
        world_torque + self.body.rotation.copied().unwrap_or_default() * local_torque
    }

    /// Returns the linear acceleration that the body has accumulated
    /// before the physics step in world space.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only accelerations applied through [`ForceHelper`] are included.
    #[inline]
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
    #[inline]
    pub fn accumulated_angular_acceleration(&self) -> AngularVector {
        // The angular increment is treated as angular acceleration until the integration step.
        self.body.integration.angular_increment
    }

    /// Returns the angular acceleration that the body has accumulated
    /// before the physics step in world space.
    ///
    /// This does not include gravity, contact forces, or joint forces.
    /// Only accelerations applied through [`ForceHelper`] are included.
    #[cfg(feature = "3d")]
    #[inline]
    pub fn accumulated_angular_acceleration(&self) -> AngularVector {
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
    #[inline]
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
    #[inline]
    pub fn reset_accumulated_torque(&mut self) {
        if let Some(ref mut forces) = self.body.accumulated_world_forces {
            forces.torque = AngularVector::ZERO;
        } else {
            self.insertion_buffers
                .world_forces
                .entry(self.entity)
                .or_default()
                .torque = AngularVector::ZERO;
        }
        #[cfg(feature = "3d")]
        {
            if let Some(ref mut forces) = self.body.accumulated_local_forces {
                forces.torque = AngularVector::ZERO;
            } else {
                self.insertion_buffers
                    .local_forces
                    .entry(self.entity)
                    .or_default()
                    .torque = AngularVector::ZERO;
            }
        }
    }

    /// Resets the accumulated linear acceleration to zero.
    #[inline]
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
    #[inline]
    pub fn reset_accumulated_angular_acceleration(&mut self) {
        self.body.integration.angular_increment = AngularVector::ZERO;
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

    /// Returns the [`LinearVelocity`] of the body in world space.
    #[inline]
    pub fn linear_velocity(&self) -> Vector {
        self.body.linear_velocity.0
    }

    /// Returns a mutable reference to the [`LinearVelocity`] of the body in world space.
    #[inline]
    pub fn linear_velocity_mut(&mut self) -> &mut Vector {
        &mut self.body.linear_velocity.0
    }

    /// Returns the [`AngularVelocity`] of the body in world space.
    #[inline]
    pub fn angular_velocity(&self) -> AngularVector {
        self.body.angular_velocity.0
    }

    /// Returns a mutable reference to the [`AngularVelocity`] of the body in world space.
    #[inline]
    pub fn angular_velocity_mut(&mut self) -> &mut AngularVector {
        &mut self.body.angular_velocity.0
    }
}
