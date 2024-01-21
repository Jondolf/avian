//! Runs systems at the start of each physics frame. Initializes [rigid bodies](RigidBody)
//! and updates components.
//!
//! See [`PreparePlugin`].

#![allow(clippy::type_complexity)]

use crate::prelude::*;
use bevy::{prelude::*, utils::intern::Interned};

/// Runs systems at the start of each physics frame. Initializes [rigid bodies](RigidBody)
/// and updates components.
///
/// - Adds missing rigid body components for entities with a [`RigidBody`] component
/// - Adds missing mass properties for entities with a [`RigidBody`] component
/// - Updates mass properties
/// - Clamps restitution coefficients between 0 and 1
///
/// The [`Transform`] component will be initialized based on [`Position`] or [`Rotation`]
/// and vice versa. You can configure this synchronization using the [`PrepareConfig`] resource.
///
/// The plugin takes a collider type. This should be [`Collider`] for
/// the vast majority of applications, but for custom collisión backends
/// you may use any collider that implements the [`AnyCollider`] trait.
///
/// The systems run in [`PhysicsSet::Prepare`].
pub struct PreparePlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PreparePlugin {
    /// Creates a [`PreparePlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PreparePlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

/// Systems sets for initializing and syncing missing components.
/// You can use these to schedule your own initialization systems
/// without having to worry about implementation details.
///
/// 1. `PreInit`: Used for systems that must run before initialization.
/// 2. `PropagateTransforms`: Responsible for propagating transforms.
/// 3. `InitRigidBodies`: Responsible for initializing missing [`RigidBody`] components.
/// 4. `InitMassProperties`: Responsible for initializing missing mass properties for [`RigidBody`] components.
/// 5. `InitColliders`: Responsible for initializing missing [`Collider`] components.
/// 6. `InitTransforms`: Responsible for initializing [`Transform`] based on [`Position`] and [`Rotation`]
/// or vice versa.
/// 7. `Finalize`: Responsible for performing final updates after everything is initialized.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PrepareSet {
    /// Used for systems that must run before initialization.
    PreInit,
    /// Responsible for propagating transforms.
    PropagateTransforms,
    /// Responsible for initializing missing [`RigidBody`] components.
    InitRigidBodies,
    /// Responsible for initializing missing mass properties for [`RigidBody`] components.
    InitMassProperties,
    /// Responsible for initializing missing [`Collider`] components.
    InitColliders,
    /// Responsible for initializing [`Transform`] based on [`Position`] and [`Rotation`]
    /// or vice versa. Parts of this system can be disabled with [`PrepareConfig`].
    /// Schedule your system with this to implement custom behavior for initializing transforms.
    InitTransforms,
    /// Responsible for performing final updates after everything is initialized.
    /// Updates mass properties and clamps collider density and restitution.
    Finalize,
}

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(
            self.schedule,
            (
                PrepareSet::PreInit,
                PrepareSet::PropagateTransforms,
                PrepareSet::InitRigidBodies,
                PrepareSet::InitMassProperties,
                PrepareSet::InitColliders,
                PrepareSet::InitTransforms,
                PrepareSet::Finalize,
            )
                .chain()
                .in_set(PhysicsSet::Prepare),
        );

        app.init_resource::<PrepareConfig>()
            .register_type::<PrepareConfig>();

        // Note: Collider logic is handled by the `ColliderBackendPlugin`
        app.add_systems(
            self.schedule,
            (
                apply_deferred,
                // Run transform propagation if new bodies have been added
                (
                    bevy::transform::systems::sync_simple_transforms,
                    bevy::transform::systems::propagate_transforms,
                )
                    .chain()
                    .run_if(any_new::<RigidBody>),
            )
                .chain()
                .in_set(PrepareSet::PropagateTransforms),
        )
        .add_systems(
            self.schedule,
            init_rigid_bodies.in_set(PrepareSet::InitRigidBodies),
        )
        .add_systems(
            self.schedule,
            init_mass_properties.in_set(PrepareSet::InitMassProperties),
        )
        .add_systems(
            self.schedule,
            init_transforms::<RigidBody>.in_set(PrepareSet::InitTransforms),
        )
        .add_systems(
            self.schedule,
            (
                update_mass_properties,
                clamp_collider_density,
                clamp_restitution,
                // All the components we added above must exist before we can simulate the bodies.
                apply_deferred,
            )
                .chain()
                .in_set(PrepareSet::Finalize),
        );
    }
}

/// Configures what is initialized by the [`PreparePlugin`] and how.
#[derive(Resource, Reflect, Clone, Debug, PartialEq, Eq)]
#[reflect(Resource)]
pub struct PrepareConfig {
    /// Initializes [`Transform`] based on [`Position`] and [`Rotation`].
    /// Defaults to true.
    pub position_to_transform: bool,
    /// Initializes [`Position`] and [`Rotation`] based on [`Transform`].
    /// Defaults to true.
    pub transform_to_position: bool,
}

impl Default for PrepareConfig {
    fn default() -> Self {
        PrepareConfig {
            position_to_transform: true,
            transform_to_position: true,
        }
    }
}

/// A run condition that returns `true` if a component of the given type `C` has been added to any entity.
pub fn any_new<C: Component>(query: Query<(), Added<C>>) -> bool {
    !query.is_empty()
}

// TODO: This system feels very overengineered. Try to clean it up?
/// Initializes [`Transform`] based on [`Position`] and [`Rotation`] or vice versa
/// when a component of the given type is inserted.
pub fn init_transforms<C: Component>(
    mut commands: Commands,
    config: Res<PrepareConfig>,
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
            Has<C>,
        ),
        Added<C>,
    >,
    parents: Query<
        (
            Option<&Position>,
            Option<&Rotation>,
            Option<&GlobalTransform>,
        ),
        With<Children>,
    >,
) {
    for (
        entity,
        mut transform,
        global_transform,
        pos,
        previous_pos,
        rot,
        previous_rot,
        parent,
        is_rb,
    ) in &mut query
    {
        let parent_position = parent.map(|parent| parents.get(parent.get()));

        // Compute Transform based on Position or vice versa
        let new_position = if let Some(pos) = pos {
            if config.position_to_transform {
                if let Some(ref mut transform) = transform {
                    // Initialize new translation as global position
                    #[cfg(feature = "2d")]
                    let mut new_translation = pos.as_f32().extend(transform.translation.z);
                    #[cfg(feature = "3d")]
                    let mut new_translation = pos.as_f32();

                    // If the body is a child, subtract the parent's global translation
                    // to get the local translation
                    if let Some(Ok((parent_pos, _, parent_transform))) = parent_position {
                        if let Some(parent_pos) = parent_pos {
                            #[cfg(feature = "2d")]
                            {
                                new_translation -= parent_pos.as_f32().extend(new_translation.z);
                            }
                            #[cfg(feature = "3d")]
                            {
                                new_translation -= parent_pos.as_f32();
                            }
                        } else if let Some(parent_transform) = parent_transform {
                            new_translation -= parent_transform.translation();
                        }
                    }
                    transform.translation = new_translation;
                }
            }
            pos.0
        } else if config.transform_to_position {
            let mut new_position = Vector::ZERO;

            if let Some(Ok((parent_pos, _, parent_transform))) = parent_position {
                if let Some(parent_pos) = parent_pos {
                    let translation = transform.as_ref().map_or(default(), |t| t.translation);
                    #[cfg(feature = "2d")]
                    {
                        new_position = parent_pos.0 + translation.adjust_precision().truncate();
                    }
                    #[cfg(feature = "3d")]
                    {
                        new_position = parent_pos.0 + translation.adjust_precision();
                    }
                } else if let Some(parent_transform) = parent_transform {
                    let new_pos = parent_transform
                        .transform_point(transform.as_ref().map_or(default(), |t| t.translation));
                    #[cfg(feature = "2d")]
                    {
                        new_position = new_pos.truncate().adjust_precision();
                    }
                    #[cfg(feature = "3d")]
                    {
                        new_position = new_pos.adjust_precision();
                    }
                }
            } else {
                #[cfg(feature = "2d")]
                {
                    new_position = global_transform.as_ref().map_or(Vector::ZERO, |t| {
                        Vector::new(t.translation().x as Scalar, t.translation().y as Scalar)
                    });
                }
                #[cfg(feature = "3d")]
                {
                    new_position = global_transform
                        .as_ref()
                        .map_or(Vector::ZERO, |t| t.translation().adjust_precision())
                }
            };

            new_position
        } else {
            default()
        };

        // Compute Transform based on Rotation or vice versa
        let new_rotation = if let Some(rot) = rot {
            if config.position_to_transform {
                if let Some(ref mut transform) = transform {
                    // Initialize new rotation as global rotation
                    let mut new_rotation = Quaternion::from(*rot).as_f32();

                    // If the body is a child, subtract the parent's global rotation
                    // to get the local rotation
                    if let Some(parent) = parent {
                        if let Ok((_, parent_rot, parent_transform)) = parents.get(parent.get()) {
                            if let Some(parent_rot) = parent_rot {
                                new_rotation *= Quaternion::from(*parent_rot).as_f32().inverse();
                            } else if let Some(parent_transform) = parent_transform {
                                new_rotation *=
                                    parent_transform.compute_transform().rotation.inverse();
                            }
                        }
                    }
                    transform.rotation = new_rotation;
                }
            }
            *rot
        } else if config.transform_to_position {
            if let Some(Ok((_, parent_rot, parent_transform))) = parent_position {
                let parent_rot = parent_rot.copied().unwrap_or(Rotation::from(
                    parent_transform.map_or(default(), |t| t.compute_transform().rotation),
                ));
                let rot = Rotation::from(transform.as_ref().map_or(default(), |t| t.rotation));
                #[cfg(feature = "2d")]
                {
                    parent_rot + rot
                }
                #[cfg(feature = "3d")]
                {
                    Rotation(parent_rot.0 * rot.0)
                }
            } else {
                global_transform.map_or(Rotation::default(), |t| {
                    t.compute_transform().rotation.into()
                })
            }
        } else {
            default()
        };

        if !config.transform_to_position {
            return;
        }

        // Insert the position and rotation.
        // The values are either unchanged (Position and Rotation already exist)
        // or computed based on the GlobalTransform.
        // If the entity isn't a rigid body, adding PreviousPosition and PreviousRotation
        // is unnecessary.
        if is_rb {
            commands.entity(entity).insert((
                Position(new_position),
                *previous_pos.unwrap_or(&PreviousPosition(new_position)),
                new_rotation,
                *previous_rot.unwrap_or(&PreviousRotation(new_rotation)),
                transform.map_or(Transform::default(), |t| *t),
            ));
        } else {
            commands.entity(entity).insert((
                Position(new_position),
                new_rotation,
                transform.map_or(Transform::default(), |t| *t),
            ));
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

/// Initializes missing mass properties for [rigid bodies](RigidBody).
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
        Added<RigidBody>,
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

/// Updates each body's [`InverseMass`] and [`InverseInertia`] whenever [`Mass`] or [`Inertia`] are changed.
pub fn update_mass_properties(
    mut bodies: Query<
        (
            Entity,
            &RigidBody,
            Ref<Mass>,
            &mut InverseMass,
            Ref<Inertia>,
            &mut InverseInertia,
        ),
        Or<(Changed<Mass>, Changed<Inertia>)>,
    >,
) {
    for (entity, rb, mass, mut inv_mass, inertia, mut inv_inertia) in &mut bodies {
        let is_mass_valid = mass.is_finite() && mass.0 >= Scalar::EPSILON;
        #[cfg(feature = "2d")]
        let is_inertia_valid = inertia.is_finite() && inertia.0 >= Scalar::EPSILON;
        #[cfg(feature = "3d")]
        let is_inertia_valid = inertia.is_finite() && *inertia != Inertia::ZERO;

        if mass.is_changed() && is_mass_valid {
            inv_mass.0 = 1.0 / mass.0;
        }
        if inertia.is_changed() && is_inertia_valid {
            inv_inertia.0 = inertia.inverse().0;
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

/// Clamps [`ColliderDensity`] to be above 0.0.
fn clamp_collider_density(mut query: Query<&mut ColliderDensity, Changed<ColliderDensity>>) {
    for mut density in &mut query {
        density.0 = density.max(Scalar::EPSILON);
    }
}
