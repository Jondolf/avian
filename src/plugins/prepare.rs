//! Runs systems at the start of each physics frame. Initializes [rigid bodies](RigidBody)
//! and updates components.
//!
//! See [`PreparePlugin`].

#![allow(clippy::type_complexity)]

use crate::prelude::*;
use bevy::{ecs::query::QueryFilter, prelude::*, utils::intern::Interned};

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
                    .run_if(match_any::<Added<RigidBody>>),
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

/// A run condition that returns `true` if any entity matches the given query filter.
pub(crate) fn match_any<F: QueryFilter>(query: Query<(), F>) -> bool {
    !query.is_empty()
}

/// Initializes [`Transform`] based on [`Position`] and [`Rotation`] or vice versa
/// when a component of the given type is inserted.
pub fn init_transforms<C: Component>(
    mut commands: Commands,
    config: Res<PrepareConfig>,
    query: Query<
        (
            Entity,
            Option<&Transform>,
            Option<&GlobalTransform>,
            Option<&Position>,
            Option<&PreviousPosition>,
            Option<&Rotation>,
            Option<&PreviousRotation>,
            Option<&Parent>,
            Has<RigidBody>,
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
    if !config.position_to_transform && !config.transform_to_position {
        // Nothing to do
        return;
    }

    for (
        entity,
        transform,
        global_transform,
        pos,
        previous_pos,
        rot,
        previous_rot,
        parent,
        has_rigid_body,
    ) in &query
    {
        let parent_transforms = parent.and_then(|parent| parents.get(parent.get()).ok());
        let parent_pos = parent_transforms.and_then(|(pos, _, _)| pos);
        let parent_rot = parent_transforms.and_then(|(_, rot, _)| rot);
        let parent_global_trans = parent_transforms.and_then(|(_, _, trans)| trans);

        let mut new_transform = if config.position_to_transform {
            Some(transform.copied().unwrap_or_default())
        } else {
            None
        };

        // Compute Transform based on Position or vice versa
        let new_position = if let Some(pos) = pos {
            if let Some(transform) = &mut new_transform {
                // Initialize new translation as global position
                #[cfg(feature = "2d")]
                let mut new_translation = pos.f32().extend(transform.translation.z);
                #[cfg(feature = "3d")]
                let mut new_translation = pos.f32();

                // If the body is a child, subtract the parent's global translation
                // to get the local translation
                if parent.is_some() {
                    if let Some(parent_pos) = parent_pos {
                        #[cfg(feature = "2d")]
                        {
                            new_translation -= parent_pos.f32().extend(new_translation.z);
                        }
                        #[cfg(feature = "3d")]
                        {
                            new_translation -= parent_pos.f32();
                        }
                    } else if let Some(parent_transform) = parent_global_trans {
                        new_translation -= parent_transform.translation();
                    }
                }
                transform.translation = new_translation;
            }
            pos.0
        } else if config.transform_to_position {
            let mut new_position = Vector::ZERO;

            if parent.is_some() {
                let translation = transform.as_ref().map_or(default(), |t| t.translation);
                if let Some(parent_pos) = parent_pos {
                    #[cfg(feature = "2d")]
                    {
                        new_position = parent_pos.0 + translation.adjust_precision().truncate();
                    }
                    #[cfg(feature = "3d")]
                    {
                        new_position = parent_pos.0 + translation.adjust_precision();
                    }
                } else if let Some(parent_transform) = parent_global_trans {
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
                    new_position = transform
                        .map(|t| t.translation.truncate().adjust_precision())
                        .unwrap_or(global_transform.as_ref().map_or(Vector::ZERO, |t| {
                            Vector::new(t.translation().x as Scalar, t.translation().y as Scalar)
                        }));
                }
                #[cfg(feature = "3d")]
                {
                    new_position = transform
                        .map(|t| t.translation.adjust_precision())
                        .unwrap_or(
                            global_transform
                                .as_ref()
                                .map_or(Vector::ZERO, |t| t.translation().adjust_precision()),
                        )
                }
            };

            new_position
        } else {
            default()
        };

        // Compute Transform based on Rotation or vice versa
        let new_rotation = if let Some(rot) = rot {
            if let Some(transform) = &mut new_transform {
                // Initialize new rotation as global rotation
                let mut new_rotation = Quaternion::from(*rot).f32();

                // If the body is a child, subtract the parent's global rotation
                // to get the local rotation
                if parent.is_some() {
                    if let Some(parent_rot) = parent_rot {
                        new_rotation *= Quaternion::from(*parent_rot).f32().inverse();
                    } else if let Some(parent_transform) = parent_global_trans {
                        new_rotation *= parent_transform.compute_transform().rotation.inverse();
                    }
                }
                transform.rotation = new_rotation;
            }
            *rot
        } else if config.transform_to_position {
            if parent.is_some() {
                let parent_rot = parent_rot.copied().unwrap_or(Rotation::from(
                    parent_global_trans.map_or(default(), |t| t.compute_transform().rotation),
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
                transform.map(|t| Rotation::from(t.rotation)).unwrap_or(
                    global_transform.map_or(Rotation::default(), |t| {
                        t.compute_transform().rotation.into()
                    }),
                )
            }
        } else {
            default()
        };

        let mut cmds = commands.entity(entity);

        // Insert the position and rotation.
        // The values are either unchanged (Position and Rotation already exist)
        // or computed based on the GlobalTransform.
        // If the entity isn't a rigid body, adding PreviousPosition and PreviousRotation
        // is unnecessary.
        match (has_rigid_body, new_transform) {
            (true, None) => {
                cmds.try_insert((
                    Position(new_position),
                    new_rotation,
                    *previous_pos.unwrap_or(&PreviousPosition(new_position)),
                    *previous_rot.unwrap_or(&PreviousRotation(new_rotation)),
                ));
            }
            (true, Some(transform)) => {
                cmds.try_insert((
                    transform,
                    Position(new_position),
                    new_rotation,
                    *previous_pos.unwrap_or(&PreviousPosition(new_position)),
                    *previous_rot.unwrap_or(&PreviousRotation(new_rotation)),
                ));
            }
            (false, None) => {
                cmds.try_insert((Position(new_position), new_rotation));
            }
            (false, Some(transform)) => {
                cmds.try_insert((transform, Position(new_position), new_rotation));
            }
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
        commands.entity(entity).try_insert((
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
        commands.entity(entity).try_insert((
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_init_transforms_basics() {
        let mut app = App::new();

        // Add system under test
        app.add_systems(Update, init_transforms::<RigidBody>);

        // Test all possible config permutations
        for (position_to_transform, transform_to_position) in
            [(true, true), (true, false), (false, true), (false, false)]
        {
            let config = PrepareConfig {
                position_to_transform,
                transform_to_position,
            };
            app.insert_resource(dbg!(config.clone()));

            // Spawn entities with `Position` and `Rotation`
            let (pos_0, rot_0) = {
                #[cfg(feature = "2d")]
                {
                    (Position::from_xy(1., 2.), Rotation::from_sin_cos(0.1, 0.2))
                }
                #[cfg(feature = "3d")]
                {
                    (
                        Position::from_xyz(1., 2., 3.),
                        Rotation(Quaternion::from_axis_angle(Vector::Y, 0.5)),
                    )
                }
            };
            let e_0_with_pos_and_rot = app.world.spawn((RigidBody::Dynamic, pos_0, rot_0)).id();

            let (pos_1, rot_1) = {
                #[cfg(feature = "2d")]
                {
                    (Position::from_xy(-1., 3.), Rotation::from_sin_cos(0.2, 0.3))
                }
                #[cfg(feature = "3d")]
                {
                    (
                        Position::from_xyz(-1., 3., -3.),
                        Rotation(Quaternion::from_axis_angle(Vector::X, 0.1)),
                    )
                }
            };
            let e_1_with_pos_and_rot = app.world.spawn((RigidBody::Dynamic, pos_1, rot_1)).id();

            // Spawn an entity with only `Position`
            let pos_2 = {
                #[cfg(feature = "2d")]
                {
                    Position::from_xy(10., 1.)
                }
                #[cfg(feature = "3d")]
                {
                    Position::from_xyz(10., 1., 5.)
                }
            };
            let e_2_with_pos = app.world.spawn((RigidBody::Dynamic, pos_2)).id();

            // Spawn an entity with only `Rotation`
            let rot_3 = {
                #[cfg(feature = "2d")]
                {
                    Rotation::from_sin_cos(0.4, 0.5)
                }
                #[cfg(feature = "3d")]
                {
                    Rotation(Quaternion::from_axis_angle(Vector::Z, 0.4))
                }
            };
            let e_3_with_rot = app.world.spawn((RigidBody::Dynamic, rot_3)).id();

            // Spawn entities with `Transform`
            let trans_4 = {
                Transform {
                    translation: Vec3::new(-1.1, 6., -7.),
                    rotation: Quat::from_axis_angle(Vec3::Y, 0.1),
                    scale: Vec3::ONE,
                }
            };
            let e_4_with_trans = app.world.spawn((RigidBody::Dynamic, trans_4)).id();

            let trans_5 = {
                Transform {
                    translation: Vec3::new(8., -1., 0.),
                    rotation: Quat::from_axis_angle(Vec3::Y, -0.1),
                    scale: Vec3::ONE,
                }
            };
            let e_5_with_trans = app.world.spawn((RigidBody::Dynamic, trans_5)).id();

            // Spawn entity without any transforms
            let e_6_without_trans = app.world.spawn(RigidBody::Dynamic).id();

            // Spawn entity without a ridid body
            let e_7_without_rb = app.world.spawn(()).id();

            // Run the system
            app.update();

            // Check the results are as expected
            if config.position_to_transform {
                assert!(app.world.get::<Transform>(e_0_with_pos_and_rot).is_some());
                let transform = app.world.get::<Transform>(e_0_with_pos_and_rot).unwrap();
                let expected: Vec3 = {
                    #[cfg(feature = "2d")]
                    {
                        pos_0.f32().extend(0.)
                    }
                    #[cfg(feature = "3d")]
                    {
                        pos_0.f32()
                    }
                };
                assert_eq!(transform.translation, expected);
                let expected = Quaternion::from(rot_0).f32();
                assert_eq!(transform.rotation, expected);

                assert!(app.world.get::<Transform>(e_1_with_pos_and_rot).is_some());
                let transform = app.world.get::<Transform>(e_1_with_pos_and_rot).unwrap();
                let expected: Vec3 = {
                    #[cfg(feature = "2d")]
                    {
                        pos_1.f32().extend(0.)
                    }
                    #[cfg(feature = "3d")]
                    {
                        pos_1.f32()
                    }
                };
                assert_eq!(transform.translation, expected);
                let expected = Quaternion::from(rot_1).f32();
                assert_eq!(transform.rotation, expected);

                assert!(app.world.get::<Transform>(e_2_with_pos).is_some());
                let transform = app.world.get::<Transform>(e_2_with_pos).unwrap();
                let expected: Vec3 = {
                    #[cfg(feature = "2d")]
                    {
                        pos_2.f32().extend(0.)
                    }
                    #[cfg(feature = "3d")]
                    {
                        pos_2.f32()
                    }
                };
                assert_eq!(transform.translation, expected);
                let expected = Quat::default();
                assert_eq!(transform.rotation, expected);

                assert!(app.world.get::<Transform>(e_3_with_rot).is_some());
                let transform = app.world.get::<Transform>(e_3_with_rot).unwrap();
                let expected: Vec3 = Vec3::default();
                assert_eq!(transform.translation, expected);
                let expected = Quaternion::from(rot_3).f32();
                assert_eq!(transform.rotation, expected);

                assert!(app.world.get::<Transform>(e_4_with_trans).is_some());
                let transform = app.world.get::<Transform>(e_4_with_trans).unwrap();
                assert_eq!(transform, &trans_4);

                assert!(app.world.get::<Transform>(e_5_with_trans).is_some());
                let transform = app.world.get::<Transform>(e_5_with_trans).unwrap();
                assert_eq!(transform, &trans_5);

                assert!(app.world.get::<Transform>(e_6_without_trans).is_some());
                let transform = app.world.get::<Transform>(e_6_without_trans).unwrap();
                assert_eq!(transform, &Transform::default());

                assert!(app.world.get::<Transform>(e_7_without_rb).is_none());
            }

            if config.transform_to_position {
                assert!(app.world.get::<Position>(e_0_with_pos_and_rot).is_some());
                let pos = app.world.get::<Position>(e_0_with_pos_and_rot).unwrap();
                assert_eq!(pos, &pos_0);
                assert!(app.world.get::<Rotation>(e_0_with_pos_and_rot).is_some());
                let rot = app.world.get::<Rotation>(e_0_with_pos_and_rot).unwrap();
                assert_eq!(rot, &rot_0);

                assert!(app.world.get::<Position>(e_1_with_pos_and_rot).is_some());
                let pos = app.world.get::<Position>(e_1_with_pos_and_rot).unwrap();
                assert_eq!(pos, &pos_1);
                assert!(app.world.get::<Rotation>(e_1_with_pos_and_rot).is_some());
                let rot = app.world.get::<Rotation>(e_1_with_pos_and_rot).unwrap();
                assert_eq!(rot, &rot_1);

                assert!(app.world.get::<Position>(e_2_with_pos).is_some());
                let pos = app.world.get::<Position>(e_2_with_pos).unwrap();
                assert_eq!(pos, &pos_2);
                assert!(app.world.get::<Rotation>(e_2_with_pos).is_some());
                let rot = app.world.get::<Rotation>(e_2_with_pos).unwrap();
                assert_eq!(rot, &Rotation::default());

                assert!(app.world.get::<Position>(e_3_with_rot).is_some());
                let pos = app.world.get::<Position>(e_3_with_rot).unwrap();
                assert_eq!(pos, &Position::default());
                assert!(app.world.get::<Rotation>(e_3_with_rot).is_some());
                let rot = app.world.get::<Rotation>(e_3_with_rot).unwrap();
                assert_eq!(rot, &rot_3);

                assert!(app.world.get::<Position>(e_4_with_trans).is_some());
                let pos = app.world.get::<Position>(e_4_with_trans).unwrap();
                let expected: Position = Position::new({
                    #[cfg(feature = "2d")]
                    {
                        trans_4.translation.truncate().adjust_precision()
                    }
                    #[cfg(feature = "3d")]
                    {
                        trans_4.translation.adjust_precision()
                    }
                });
                assert_eq!(pos, &expected);
                assert!(app.world.get::<Rotation>(e_4_with_trans).is_some());
                let rot = app.world.get::<Rotation>(e_4_with_trans).unwrap();
                assert_eq!(rot, &Rotation::from(trans_4.rotation));

                assert!(app.world.get::<Position>(e_5_with_trans).is_some());
                let pos = app.world.get::<Position>(e_5_with_trans).unwrap();
                let expected: Position = Position::new({
                    #[cfg(feature = "2d")]
                    {
                        trans_5.translation.truncate().adjust_precision()
                    }
                    #[cfg(feature = "3d")]
                    {
                        trans_5.translation.adjust_precision()
                    }
                });
                assert_eq!(pos, &expected);
                assert!(app.world.get::<Rotation>(e_5_with_trans).is_some());
                let rot = app.world.get::<Rotation>(e_5_with_trans).unwrap();
                assert_eq!(rot, &Rotation::from(trans_5.rotation));

                assert!(app.world.get::<Position>(e_6_without_trans).is_some());
                let pos = app.world.get::<Position>(e_6_without_trans).unwrap();
                assert_eq!(pos, &Position::default());
                assert!(app.world.get::<Rotation>(e_6_without_trans).is_some());
                let rot = app.world.get::<Rotation>(e_6_without_trans).unwrap();
                assert_eq!(rot, &Rotation::default());

                assert!(app.world.get::<Position>(e_7_without_rb).is_none());
                assert!(app.world.get::<Rotation>(e_7_without_rb).is_none());
            }
        }
    }
}
