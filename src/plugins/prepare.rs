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
                update_mass_properties,
                clamp_restitution,
            )
                .chain()
                .in_set(PhysicsSet::Prepare),
        );
    }
}

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
            Option<&Rotation>,
            Option<&Parent>,
        ),
        Or<(Added<RigidBody>, Added<Collider>)>,
    >,
    parents: Query<&GlobalTransform, With<Children>>,
) {
    for (entity, mut transform, global_transform, pos, rot, parent) in &mut query {
        let mut body = commands.entity(entity);
        if let Some(pos) = pos {
            body.insert(PreviousPosition(pos.0));

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
            let translation;
            #[cfg(feature = "2d")]
            {
                translation = global_transform.as_ref().map_or(Vector::ZERO, |t| {
                    Vector::new(t.translation().x as Scalar, t.translation().y as Scalar)
                });
            }
            #[cfg(feature = "3d")]
            {
                translation = global_transform.as_ref().map_or(Vector::ZERO, |t| {
                    Vector::new(
                        t.translation().x as Scalar,
                        t.translation().y as Scalar,
                        t.translation().z as Scalar,
                    )
                });
            }

            body.insert(Position(translation));
            body.insert(PreviousPosition(translation));
        }

        if let Some(rot) = rot {
            body.insert(PreviousRotation(*rot));

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
            let rotation = global_transform.map_or(Rotation::default(), |t| {
                t.compute_transform().rotation.into()
            });
            body.insert(rotation);
            body.insert(PreviousRotation(rotation));
        }
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
        let mut body = commands.entity(entity);
        body.insert(AccumulatedTranslation(Vector::ZERO));

        if lin_vel.is_none() {
            body.insert(LinearVelocity::default());
        }
        body.insert(PreSolveLinearVelocity::default());
        if ang_vel.is_none() {
            body.insert(AngularVelocity::default());
        }
        body.insert(PreSolveAngularVelocity::default());
        if force.is_none() {
            body.insert(ExternalForce::default());
        }
        if torque.is_none() {
            body.insert(ExternalTorque::default());
        }
        if impulse.is_none() {
            body.insert(ExternalImpulse::default());
        }
        if angular_impulse.is_none() {
            body.insert(ExternalAngularImpulse::default());
        }
        if restitution.is_none() {
            body.insert(Restitution::default());
        }
        if friction.is_none() {
            body.insert(Friction::default());
        }
        if time_sleeping.is_none() {
            body.insert(TimeSleeping::default());
        }
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
        let mut body = commands.entity(entity);

        if mass.is_none() {
            body.insert(Mass(
                inverse_mass.map_or(0.0, |inverse_mass| 1.0 / inverse_mass.0),
            ));
        }
        if inverse_mass.is_none() {
            body.insert(InverseMass(mass.map_or(0.0, |mass| 1.0 / mass.0)));
        }
        if inertia.is_none() {
            body.insert(
                inverse_inertia.map_or(Inertia::ZERO, |inverse_inertia| inverse_inertia.inverse()),
            );
        }
        if inverse_inertia.is_none() {
            body.insert(inertia.map_or(InverseInertia::ZERO, |inertia| inertia.inverse()));
        }
        if center_of_mass.is_none() {
            body.insert(CenterOfMass::default());
        }
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
            Option<&CollidingEntities>,
            Option<&ColliderMassProperties>,
            Option<&PreviousColliderMassProperties>,
        ),
        Added<Collider>,
    >,
) {
    for (entity, collider, aabb, colliding_entities, mass_properties, previous_mass_properties) in
        &mut colliders
    {
        let mut entity_commands = commands.entity(entity);

        if aabb.is_none() {
            entity_commands.insert(ColliderAabb::from_shape(collider.get_shape()));
        }
        if colliding_entities.is_none() {
            entity_commands.insert(CollidingEntities::default());
        }
        if mass_properties.is_none() {
            entity_commands.insert(ColliderMassProperties::new_computed(collider, 1.0));
        }
        if previous_mass_properties.is_none() {
            entity_commands.insert(PreviousColliderMassProperties(ColliderMassProperties::ZERO));
        }
    }
}

type MassPropertiesChanged = Or<(
    Changed<Mass>,
    Changed<InverseMass>,
    Changed<Inertia>,
    Changed<InverseInertia>,
    Changed<Collider>,
    Changed<ColliderMassProperties>,
)>;

/// Updates each body's mass properties whenever their dependant mass properties or the body's [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
fn update_mass_properties(
    mut bodies: Query<
        (
            Entity,
            Option<&RigidBody>,
            MassPropertiesQuery,
            Option<&Collider>,
            Option<&mut ColliderMassProperties>,
            Option<&mut PreviousColliderMassProperties>,
        ),
        MassPropertiesChanged,
    >,
) {
    for (
        entity,
        rb,
        mut mass_properties,
        collider,
        collider_mass_properties,
        previous_collider_mass_properties,
    ) in &mut bodies
    {
        if mass_properties.mass.is_changed() && mass_properties.mass.0 >= Scalar::EPSILON {
            mass_properties.inverse_mass.0 = 1.0 / mass_properties.mass.0;
        }

        if let Some(collider) = collider {
            let Some(mut collider_mass_properties) = collider_mass_properties else {
                continue;
            };
            let Some(mut previous_collider_mass_properties) = previous_collider_mass_properties
            else {
                continue;
            };

            // Subtract previous collider mass props from the body's mass props
            mass_properties -= previous_collider_mass_properties.0;

            // Update previous and current collider mass props
            previous_collider_mass_properties.0 = *collider_mass_properties;
            *collider_mass_properties =
                ColliderMassProperties::new_computed(collider, collider_mass_properties.density);

            // Add new collider mass props to the body's mass props
            mass_properties += *collider_mass_properties;
        }

        // Warn about dynamic bodies with no mass or inertia
        if let Some(rb) = rb {
            let is_mass_valid =
                mass_properties.mass.is_finite() && mass_properties.mass.0 >= Scalar::EPSILON;
            #[cfg(feature = "2d")]
            let is_inertia_valid =
                mass_properties.inertia.is_finite() && mass_properties.inertia.0 >= Scalar::EPSILON;
            #[cfg(feature = "3d")]
            let is_inertia_valid =
                mass_properties.inertia.is_finite() && *mass_properties.inertia != Inertia::ZERO;
            if rb.is_dynamic() && !(is_mass_valid && is_inertia_valid) {
                warn!(
                    "Dynamic rigid body {:?} has no mass or inertia. This can cause NaN values. Consider adding a `MassPropertiesBundle` or a `Collider` with mass.",
                    entity
                );
            }
        }
    }
}

/// Clamps coefficients of [restitution](Restitution) to be between 0.0 and 1.0.
fn clamp_restitution(mut query: Query<&mut Restitution, Changed<Restitution>>) {
    for mut restitution in &mut query {
        restitution.coefficient = restitution.coefficient.clamp(0.0, 1.0);
    }
}
