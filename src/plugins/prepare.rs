//! Runs systems at the start of each physics frame; initializes [rigid bodies](RigidBody)
//! and [colliders](Collider) and updates components.
//!
//! See [`PreparePlugin`].

#![allow(clippy::type_complexity)]

use crate::prelude::*;
use bevy::prelude::*;

/// Runs systems at the start of each physics frame; initializes [rigid bodies](RigidBody)
/// and [colliders](Collider) and updates components.
///
/// - Adds missing rigid body components for entities with a [`RigidBody`] component
/// - Adds missing collider components for entities with a [`Collider`] component
/// - Adds missing mass properties for entities with a [`RigidBody`] or [`Collider`] component
/// - Updates mass properties and adds [`ColliderMassProperties`] on top of the existing mass properties
/// - Clamps restitution coefficients between 0 and 1
///
/// The systems run in [`PhysicsSet::Prepare`].
pub struct PreparePlugin {
    schedule: Box<dyn ScheduleLabel>,
}

impl PreparePlugin {
    /// Creates a [`PreparePlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: Box::new(schedule),
        }
    }
}

impl Default for PreparePlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            self.schedule.dyn_clone(),
            (
                apply_deferred,
                // Run transform propagation if new bodies or colliders have been added
                (
                    bevy::transform::systems::sync_simple_transforms,
                    bevy::transform::systems::propagate_transforms,
                )
                    .chain()
                    .run_if(any_new_physics_entities),
                init_transforms,
                init_rigid_bodies,
                init_mass_properties,
                init_colliders,
                apply_deferred,
                update_collider_parents,
                (
                    sync::update_collider_offset,
                    sync::update_child_collider_position,
                )
                    .chain()
                    .run_if(any_new_physics_entities),
                update_mass_properties,
                clamp_restitution,
            )
                .chain()
                .in_set(PhysicsSet::Prepare),
        );
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
struct PreviousColliderOffset(Vector);

/// A run condition that returns `true` if new [rigid bodies](RigidBody) or [colliders](Collider)
/// have been added. Used for avoiding unnecessary transform propagation.
fn any_new_physics_entities(query: Query<(), Or<(Added<RigidBody>, Added<Collider>)>>) -> bool {
    !query.is_empty()
}

/// Initializes [`Transform`] based on [`Position`] and [`Rotation`] or vice versa.
fn init_transforms(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            Option<&mut Transform>,
            Option<&GlobalTransform>,
            Option<&Position>,
            Option<&PreviousPosition>,
            Option<&Rotation>,
            Option<&PreviousRotation>,
            Option<&Parent>,
        ),
        Or<(Added<RigidBody>, Added<Collider>)>,
    >,
    parents: Query<&GlobalTransform, With<Children>>,
) {
    for (entity, mut transform, global_transform, pos, previous_pos, rot, previous_rot, parent) in
        &mut query
    {
        let position: Position;
        let rotation: Rotation;

        // Compute Transform based on Position or vice versa
        if let Some(pos) = pos {
            position = *pos;

            if let Some(ref mut transform) = transform {
                // Initialize new translation as global position
                #[cfg(feature = "2d")]
                let mut new_translation = pos.as_f32().extend(transform.translation.z);
                #[cfg(feature = "3d")]
                let mut new_translation = pos.as_f32();

                // If the body is a child, subtract the parent's global translation
                // to get the local translation
                if let Some(parent) = parent {
                    if let Ok(parent_transform) = parents.get(**parent) {
                        new_translation -= parent_transform.translation();
                    }
                }

                transform.translation = new_translation;
            }
        } else {
            #[cfg(feature = "2d")]
            {
                position = Position(global_transform.as_ref().map_or(Vector::ZERO, |t| {
                    Vector::new(t.translation().x as Scalar, t.translation().y as Scalar)
                }));
            }
            #[cfg(feature = "3d")]
            {
                position = Position(global_transform.as_ref().map_or(Vector::ZERO, |t| {
                    Vector::new(
                        t.translation().x as Scalar,
                        t.translation().y as Scalar,
                        t.translation().z as Scalar,
                    )
                }));
            }
        }

        // Compute Transform based on Rotation or vice versa
        if let Some(rot) = rot {
            rotation = *rot;

            if let Some(mut transform) = transform {
                // Initialize new rotation as global rotation
                let mut new_rotation = Quaternion::from(*rot).as_f32();

                // If the body is a child, subtract the parent's global rotation
                // to get the local rotation
                if let Some(parent) = parent {
                    if let Ok(parent_transform) = parents.get(**parent) {
                        new_rotation = new_rotation - parent_transform.compute_transform().rotation;
                    }
                }

                transform.rotation = new_rotation;
            }
        } else {
            rotation = global_transform.map_or(Rotation::default(), |t| {
                t.compute_transform().rotation.into()
            });
        }

        // Insert the position and rotation.
        // The values are either unchanged (Position and Rotation already exist) or computed based on the GlobalTransform.
        commands.entity(entity).insert((
            position,
            *previous_pos.unwrap_or(&PreviousPosition(position.0)),
            rotation,
            *previous_rot.unwrap_or(&PreviousRotation(rotation)),
        ));
    }
}

/// Initializes missing components for [rigid bodies](RigidBody).
fn init_rigid_bodies(
    mut commands: Commands,
    mut bodies: Query<
        (
            Entity,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
            Option<&ExternalForce>,
            Option<&ExternalTorque>,
            Option<&ExternalImpulse>,
            Option<&ExternalAngularImpulse>,
            Option<&Restitution>,
            Option<&Friction>,
            Option<&TimeSleeping>,
        ),
        Added<RigidBody>,
    >,
) {
    for (
        entity,
        lin_vel,
        ang_vel,
        force,
        torque,
        impulse,
        angular_impulse,
        restitution,
        friction,
        time_sleeping,
    ) in &mut bodies
    {
        commands.entity(entity).insert((
            AccumulatedTranslation(Vector::ZERO),
            *lin_vel.unwrap_or(&LinearVelocity::default()),
            *ang_vel.unwrap_or(&AngularVelocity::default()),
            PreSolveLinearVelocity::default(),
            PreSolveAngularVelocity::default(),
            *force.unwrap_or(&ExternalForce::default()),
            *torque.unwrap_or(&ExternalTorque::default()),
            *impulse.unwrap_or(&ExternalImpulse::default()),
            *angular_impulse.unwrap_or(&ExternalAngularImpulse::default()),
            *restitution.unwrap_or(&Restitution::default()),
            *friction.unwrap_or(&Friction::default()),
            *time_sleeping.unwrap_or(&TimeSleeping::default()),
        ));
    }
}

/// Initializes missing mass properties for [rigid bodies](RigidBody) and [colliders](Collider).
fn init_mass_properties(
    mut commands: Commands,
    mass_properties: Query<
        (
            Entity,
            Option<&Mass>,
            Option<&InverseMass>,
            Option<&Inertia>,
            Option<&InverseInertia>,
            Option<&CenterOfMass>,
        ),
        Or<(Added<RigidBody>, Added<Collider>)>,
    >,
) {
    for (entity, mass, inverse_mass, inertia, inverse_inertia, center_of_mass) in &mass_properties {
        commands.entity(entity).insert((
            *mass.unwrap_or(&Mass(
                inverse_mass.map_or(0.0, |inverse_mass| 1.0 / inverse_mass.0),
            )),
            *inverse_mass.unwrap_or(&InverseMass(mass.map_or(0.0, |mass| 1.0 / mass.0))),
            *inertia.unwrap_or(
                &inverse_inertia.map_or(Inertia::ZERO, |inverse_inertia| inverse_inertia.inverse()),
            ),
            *inverse_inertia
                .unwrap_or(&inertia.map_or(InverseInertia::ZERO, |inertia| inertia.inverse())),
            *center_of_mass.unwrap_or(&CenterOfMass::default()),
        ));
    }
}

/// Initializes missing components for [colliders](Collider).
fn init_colliders(
    mut commands: Commands,
    mut colliders: Query<
        (
            Entity,
            &Collider,
            Option<&ColliderAabb>,
            Option<&ColliderOffset>,
            Option<&ColliderMassProperties>,
            Option<&PreviousColliderMassProperties>,
        ),
        Added<Collider>,
    >,
) {
    for (entity, collider, aabb, offset, mass_properties, previous_mass_properties) in
        &mut colliders
    {
        commands.entity(entity).insert((
            *aabb.unwrap_or(&ColliderAabb::from_shape(collider.get_shape())),
            *offset.unwrap_or(&default()),
            PreviousColliderOffset(Vector::ZERO),
            *mass_properties.unwrap_or(&ColliderMassProperties::new_computed(collider, 1.0)),
            *previous_mass_properties.unwrap_or(&PreviousColliderMassProperties(
                ColliderMassProperties::ZERO,
            )),
            CollidingEntities::default(),
            ColliderParent(entity),
        ));
    }
}

fn update_collider_parents(
    mut bodies: Query<(Entity, Option<&mut ColliderParent>), With<RigidBody>>,
    body_children: Query<&Children>,
    mut child_colliders: Query<
        &mut ColliderParent,
        (With<Collider>, Without<RigidBody>, Changed<Parent>),
    >,
) {
    for (entity, collider_parent) in &mut bodies {
        if let Some(mut collider_parent) = collider_parent {
            collider_parent.0 = entity;
        }
        for child in body_children.iter_descendants(entity) {
            if let Ok(mut collider_parent) = child_colliders.get_mut(child) {
                collider_parent.0 = entity;
            }
        }
    }
}

/// Updates each body's mass properties whenever their dependant mass properties or the body's [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
fn update_mass_properties(
    mut bodies: Query<(Entity, &RigidBody, MassPropertiesQuery)>,
    mut colliders: Query<
        (
            &ColliderOffset,
            &mut PreviousColliderOffset,
            &ColliderParent,
            &Collider,
            &mut ColliderMassProperties,
            &mut PreviousColliderMassProperties,
        ),
        Or<(
            Changed<Collider>,
            Changed<ColliderOffset>,
            Changed<ColliderMassProperties>,
        )>,
    >,
) {
    for (
        collider_offset,
        mut previous_collider_offset,
        collider_parent,
        collider,
        mut collider_mass_properties,
        mut previous_collider_mass_properties,
    ) in &mut colliders
    {
        if let Ok((_, _, mut mass_properties)) = bodies.get_mut(collider_parent.0) {
            // Subtract previous collider mass props from the body's mass props
            mass_properties -= *PreviousColliderMassProperties(ColliderMassProperties {
                center_of_mass: CenterOfMass(
                    previous_collider_offset.0 + previous_collider_mass_properties.center_of_mass.0,
                ),
                ..previous_collider_mass_properties.0
            });

            previous_collider_offset.0 = collider_offset.0;

            // Update previous and current collider mass props
            previous_collider_mass_properties.0 = *collider_mass_properties;
            *collider_mass_properties =
                ColliderMassProperties::new_computed(collider, collider_mass_properties.density);

            // Add new collider mass props to the body's mass props
            mass_properties += ColliderMassProperties {
                center_of_mass: CenterOfMass(
                    collider_offset.0 + collider_mass_properties.center_of_mass.0,
                ),
                ..*collider_mass_properties
            };
        }
    }

    for (entity, rb, mut mass_properties) in &mut bodies {
        let is_mass_valid =
            mass_properties.mass.is_finite() && mass_properties.mass.0 >= Scalar::EPSILON;
        #[cfg(feature = "2d")]
        let is_inertia_valid =
            mass_properties.inertia.is_finite() && mass_properties.inertia.0 >= Scalar::EPSILON;
        #[cfg(feature = "3d")]
        let is_inertia_valid =
            mass_properties.inertia.is_finite() && *mass_properties.inertia != Inertia::ZERO;

        if mass_properties.mass.is_changed() && is_mass_valid {
            mass_properties.inverse_mass.0 = 1.0 / mass_properties.mass.0;
        }
        if mass_properties.inertia.is_changed() && is_inertia_valid {
            mass_properties.inverse_inertia.0 = mass_properties.inertia.inverse().0;
        }

        // Warn about dynamic bodies with no mass or inertia
        if rb.is_dynamic() && !(is_mass_valid && is_inertia_valid) {
            warn!(
                "Dynamic rigid body {:?} has no mass or inertia. This can cause NaN values. Consider adding a `MassPropertiesBundle` or a `Collider` with mass.",
                entity
            );
        }
    }
}

/// Clamps coefficients of [restitution](Restitution) to be between 0.0 and 1.0.
fn clamp_restitution(mut query: Query<&mut Restitution, Changed<Restitution>>) {
    for mut restitution in &mut query {
        restitution.coefficient = restitution.coefficient.clamp(0.0, 1.0);
    }
}
