//! Runs systems at the start of each physics frame; initializes [rigid bodies](RigidBody)
//! and [colliders](Collider) and updates components.
//!
//! See [`PreparePlugin`].

#![allow(clippy::type_complexity)]

use crate::prelude::*;
#[cfg(all(feature = "3d", feature = "async-collider"))]
use bevy::scene::SceneInstance;
use bevy::{
    ecs::query::Has,
    prelude::*,
    utils::{intern::Interned, HashMap},
};

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

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum PrepareSet {
    Init,
}

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(self.schedule, PrepareSet::Init.in_set(PhysicsSet::Prepare));

        app.init_resource::<ColliderStorageMap2d>()
            .init_resource::<ColliderStorageMap3d>()
            .add_systems(
                self.schedule,
                (
                    apply_deferred,
                    // Run transform propagation if new bodies or colliders have been added
                    (
                        bevy::transform::systems::sync_simple_transforms,
                        bevy::transform::systems::propagate_transforms,
                    )
                        .chain()
                        .run_if(any_new_physics_entities),
                    (
                        init_rigid_bodies_2d,
                        init_rigid_bodies_3d,
                        init_mass_properties_2d,
                        init_mass_properties_3d,
                        init_colliders_2d,
                        init_colliders_3d,
                    ),
                    apply_deferred,
                    (update_collider_parents_2d, update_collider_parents_3d),
                    apply_deferred,
                    (init_transforms_2d, init_transforms_3d),
                    (
                        sync::propagate_collider_transforms,
                        (
                            sync::update_child_collider_position_2d,
                            sync::update_child_collider_position_3d,
                        ),
                    )
                        .chain()
                        .run_if(any_new_physics_entities),
                    (update_mass_properties_2d, update_mass_properties_3d),
                    clamp_collider_density,
                    clamp_restitution,
                    // all the components we added above must exist before we can simulate the bodies
                    apply_deferred,
                )
                    .chain()
                    .after(PrepareSet::Init)
                    .in_set(PhysicsSet::Prepare),
            );

        app.add_systems(
            PhysicsSchedule,
            (
                (update_collider_storage_2d, update_collider_storage_3d)
                    .before(PhysicsStepSet::BroadPhase),
                (
                    handle_collider_storage_removals_2d,
                    handle_collider_storage_removals_3d,
                )
                    .after(PhysicsStepSet::SpatialQuery),
                (handle_rigid_body_removals_2d, handle_rigid_body_removals_3d)
                    .after(PhysicsStepSet::SpatialQuery),
            ),
        );

        #[cfg(all(feature = "3d", feature = "async-collider"))]
        app.add_systems(Update, (init_async_colliders, init_async_scene_colliders));
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub(crate) struct PreviousColliderTransform(ColliderTransform);

// Todo: Does this make sense in this file? It would also be nice to find an alternative approach.
/// A hash map that stores some collider data that is needed when colliders are removed from
/// rigid bodies.
///
/// This includes the collider parent for finding the associated rigid body after collider removal,
/// and collider mass properties for updating the rigid body's mass properties when its collider is removed.
///
/// Ideally, we would just have some entity removal event or callback, but that doesn't
/// exist yet, and `RemovedComponents` only returns entities, not component data.
#[derive(Resource, Reflect, Clone, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Resource)]
pub(crate) struct ColliderStorageMap2d(
    HashMap<Entity, (ColliderParent, ColliderMassProperties2d, ColliderTransform)>,
);

#[derive(Resource, Reflect, Clone, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Resource)]
pub(crate) struct ColliderStorageMap3d(
    HashMap<Entity, (ColliderParent, ColliderMassProperties3d, ColliderTransform)>,
);

/// A run condition that returns `true` if new [rigid bodies](RigidBody) or [colliders](Collider)
/// have been added. Used for avoiding unnecessary transform propagation.
fn any_new_physics_entities(
    query: Query<
        (),
        Or<(
            Added<RigidBody2d>,
            Added<RigidBody3d>,
            Added<Collider2d>,
            Added<Collider3d>,
        )>,
    >,
) -> bool {
    !query.is_empty()
}

// TODO: This system feels very overengineered. Try to clean it up?
/// Initializes [`Transform`] based on [`Position`] and [`Rotation`] or vice versa.
fn init_transforms_2d(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            Option<&mut Transform>,
            Option<&GlobalTransform>,
            Option<&Position2d>,
            Option<&PreviousPosition2d>,
            Option<&Rotation2d>,
            Option<&PreviousRotation2d>,
            Option<&Parent>,
            Has<RigidBody2d>,
        ),
        Or<(Added<RigidBody2d>, Added<Collider2d>)>,
    >,
    parents: Query<
        (
            Option<&Position2d>,
            Option<&Rotation2d>,
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
            if let Some(ref mut transform) = transform {
                // Initialize new translation as global position
                let mut new_translation = pos.as_f32().extend(transform.translation.z);

                // If the body is a child, subtract the parent's global translation
                // to get the local translation
                if let Some(Ok((parent_pos, _, parent_transform))) = parent_position {
                    if let Some(parent_pos) = parent_pos {
                        new_translation -= parent_pos.as_f32().extend(new_translation.z);
                    } else if let Some(parent_transform) = parent_transform {
                        new_translation -= parent_transform.translation();
                    }
                }
                transform.translation = new_translation;
            }
            pos.0
        } else {
            let mut new_position = Vector2::ZERO;

            if let Some(Ok((parent_pos, _, parent_transform))) = parent_position {
                if let Some(parent_pos) = parent_pos {
                    let translation = transform.as_ref().map_or(default(), |t| t.translation);
                    new_position = parent_pos.0 + translation.adjust_precision().truncate();
                } else if let Some(parent_transform) = parent_transform {
                    let new_pos = parent_transform
                        .transform_point(transform.as_ref().map_or(default(), |t| t.translation));
                    new_position = new_pos.truncate().adjust_precision();
                }
            } else {
                new_position = global_transform
                    .as_ref()
                    .map_or(Vector2::ZERO, |t| Vector2::from_math(t.translation().xy()));
            };

            new_position
        };

        // Compute Transform based on Rotation or vice versa
        let new_rotation = if let Some(rot) = rot {
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
                            new_rotation *= parent_transform.compute_transform().rotation.inverse();
                        }
                    }
                }
                transform.rotation = new_rotation;
            }
            *rot
        } else if let Some(Ok((_, parent_rot, parent_transform))) = parent_position {
            let parent_rot = parent_rot.copied().unwrap_or(Rotation2d::from(
                parent_transform.map_or(default(), |t| t.compute_transform().rotation),
            ));
            let rot = Rotation2d::from(transform.as_ref().map_or(default(), |t| t.rotation));
            parent_rot + rot
        } else {
            global_transform.map_or(Rotation2d::default(), |t| {
                t.compute_transform().rotation.into()
            })
        };

        // Insert the position and rotation.
        // The values are either unchanged (Position and Rotation already exist)
        // or computed based on the GlobalTransform.
        // If the entity isn't a rigid body, adding PreviousPosition and PreviousRotation
        // is unnecessary.
        if is_rb {
            commands.entity(entity).insert((
                Position2d(new_position),
                *previous_pos.unwrap_or(&PreviousPosition2d(new_position)),
                new_rotation,
                *previous_rot.unwrap_or(&PreviousRotation2d(new_rotation)),
                transform.map_or(Transform::default(), |t| *t),
            ));
        } else {
            commands.entity(entity).insert((
                Position2d(new_position),
                new_rotation,
                transform.map_or(Transform::default(), |t| *t),
            ));
        }
    }
}

// TODO: This system feels very overengineered. Try to clean it up?
/// Initializes [`Transform`] based on [`Position`] and [`Rotation`] or vice versa.
fn init_transforms_3d(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            Option<&mut Transform>,
            Option<&GlobalTransform>,
            Option<&Position3d>,
            Option<&PreviousPosition3d>,
            Option<&Rotation3d>,
            Option<&PreviousRotation3d>,
            Option<&Parent>,
            Has<RigidBody3d>,
        ),
        Or<(Added<RigidBody3d>, Added<Collider3d>)>,
    >,
    parents: Query<
        (
            Option<&Position3d>,
            Option<&Rotation3d>,
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
            if let Some(ref mut transform) = transform {
                // Initialize new translation as global position
                let mut new_translation = pos.as_f32();

                // If the body is a child, subtract the parent's global translation
                // to get the local translation
                if let Some(Ok((parent_pos, _, parent_transform))) = parent_position {
                    if let Some(parent_pos) = parent_pos {
                        new_translation -= parent_pos.as_f32();
                    } else if let Some(parent_transform) = parent_transform {
                        new_translation -= parent_transform.translation();
                    }
                }
                transform.translation = new_translation;
            }
            pos.0
        } else {
            let mut new_position = Vector3::ZERO;

            if let Some(Ok((parent_pos, _, parent_transform))) = parent_position {
                if let Some(parent_pos) = parent_pos {
                    let translation = transform.as_ref().map_or(default(), |t| t.translation);
                    new_position = parent_pos.0 + translation.adjust_precision();
                } else if let Some(parent_transform) = parent_transform {
                    let new_pos = parent_transform
                        .transform_point(transform.as_ref().map_or(default(), |t| t.translation));
                    new_position = new_pos.adjust_precision();
                }
            } else {
                new_position = global_transform
                    .as_ref()
                    .map_or(Vector3::ZERO, |t| Vector3::from_math(t.translation()))
            };

            new_position
        };

        // Compute Transform based on Rotation or vice versa
        let new_rotation = if let Some(rot) = rot {
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
                            new_rotation *= parent_transform.compute_transform().rotation.inverse();
                        }
                    }
                }
                transform.rotation = new_rotation;
            }
            *rot
        } else if let Some(Ok((_, parent_rot, parent_transform))) = parent_position {
            let parent_rot = parent_rot.copied().unwrap_or(Rotation3d::from(
                parent_transform.map_or(default(), |t| t.compute_transform().rotation),
            ));
            let rot = Rotation3d::from(transform.as_ref().map_or(default(), |t| t.rotation));
            Rotation3d(parent_rot.0 * rot.0)
        } else {
            global_transform.map_or(Rotation3d::default(), |t| {
                t.compute_transform().rotation.into()
            })
        };

        // Insert the position and rotation.
        // The values are either unchanged (Position3d and Rotation3d already exist)
        // or computed based on the GlobalTransform.
        // If the entity isn't a rigid body, adding PreviousPosition3d and PreviousRotation3d
        // is unnecessary.
        if is_rb {
            commands.entity(entity).insert((
                Position3d(new_position),
                *previous_pos.unwrap_or(&PreviousPosition3d(new_position)),
                new_rotation,
                *previous_rot.unwrap_or(&PreviousRotation3d(new_rotation)),
                transform.map_or(Transform::default(), |t| *t),
            ));
        } else {
            commands.entity(entity).insert((
                Position3d(new_position),
                new_rotation,
                transform.map_or(Transform::default(), |t| *t),
            ));
        }
    }
}

/// Initializes missing components for [rigid bodies](RigidBody).
fn init_rigid_bodies_2d(
    mut commands: Commands,
    mut bodies: Query<
        (
            Entity,
            Option<&LinearVelocity2d>,
            Option<&AngularVelocity2d>,
            Option<&ExternalForce2d>,
            Option<&ExternalTorque2d>,
            Option<&ExternalImpulse2d>,
            Option<&ExternalAngularImpulse2d>,
            Option<&Restitution>,
            Option<&Friction>,
            Option<&TimeSleeping>,
        ),
        Added<RigidBody2d>,
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
            AccumulatedTranslation2d(Vector2::ZERO),
            *lin_vel.unwrap_or(&LinearVelocity2d::default()),
            *ang_vel.unwrap_or(&AngularVelocity2d::default()),
            PreSolveLinearVelocity2d::default(),
            PreSolveAngularVelocity2d::default(),
            *force.unwrap_or(&ExternalForce2d::default()),
            *torque.unwrap_or(&ExternalTorque2d::default()),
            *impulse.unwrap_or(&ExternalImpulse2d::default()),
            *angular_impulse.unwrap_or(&ExternalAngularImpulse2d::default()),
            *restitution.unwrap_or(&Restitution::default()),
            *friction.unwrap_or(&Friction::default()),
            *time_sleeping.unwrap_or(&TimeSleeping::default()),
        ));
    }
}

/// Initializes missing components for [rigid bodies](RigidBody).
fn init_rigid_bodies_3d(
    mut commands: Commands,
    mut bodies: Query<
        (
            Entity,
            Option<&LinearVelocity3d>,
            Option<&AngularVelocity3d>,
            Option<&ExternalForce3d>,
            Option<&ExternalTorque3d>,
            Option<&ExternalImpulse3d>,
            Option<&ExternalAngularImpulse3d>,
            Option<&Restitution>,
            Option<&Friction>,
            Option<&TimeSleeping>,
        ),
        Added<RigidBody3d>,
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
            AccumulatedTranslation3d(Vector3::ZERO),
            *lin_vel.unwrap_or(&LinearVelocity3d::default()),
            *ang_vel.unwrap_or(&AngularVelocity3d::default()),
            PreSolveLinearVelocity3d::default(),
            PreSolveAngularVelocity3d::default(),
            *force.unwrap_or(&ExternalForce3d::default()),
            *torque.unwrap_or(&ExternalTorque3d::default()),
            *impulse.unwrap_or(&ExternalImpulse3d::default()),
            *angular_impulse.unwrap_or(&ExternalAngularImpulse3d::default()),
            *restitution.unwrap_or(&Restitution::default()),
            *friction.unwrap_or(&Friction::default()),
            *time_sleeping.unwrap_or(&TimeSleeping::default()),
        ));
    }
}

/// Initializes missing mass properties for [rigid bodies](RigidBody).
fn init_mass_properties_2d(
    mut commands: Commands,
    mass_properties: Query<
        (
            Entity,
            Option<&Mass>,
            Option<&InverseMass>,
            Option<&Inertia2d>,
            Option<&InverseInertia2d>,
            Option<&CenterOfMass2d>,
        ),
        Added<RigidBody2d>,
    >,
) {
    for (entity, mass, inverse_mass, inertia, inverse_inertia, center_of_mass) in &mass_properties {
        commands.entity(entity).insert((
            *mass.unwrap_or(&Mass(
                inverse_mass.map_or(0.0, |inverse_mass| 1.0 / inverse_mass.0),
            )),
            *inverse_mass.unwrap_or(&InverseMass(mass.map_or(0.0, |mass| 1.0 / mass.0))),
            *inertia.unwrap_or(
                &inverse_inertia
                    .map_or(Inertia2d::ZERO, |inverse_inertia| inverse_inertia.inverse()),
            ),
            *inverse_inertia
                .unwrap_or(&inertia.map_or(InverseInertia2d::ZERO, |inertia| inertia.inverse())),
            *center_of_mass.unwrap_or(&CenterOfMass2d::default()),
        ));
    }
}

/// Initializes missing mass properties for [rigid bodies](RigidBody).
fn init_mass_properties_3d(
    mut commands: Commands,
    mass_properties: Query<
        (
            Entity,
            Option<&Mass>,
            Option<&InverseMass>,
            Option<&Inertia3d>,
            Option<&InverseInertia3d>,
            Option<&CenterOfMass3d>,
        ),
        Added<RigidBody3d>,
    >,
) {
    for (entity, mass, inverse_mass, inertia, inverse_inertia, center_of_mass) in &mass_properties {
        commands.entity(entity).insert((
            *mass.unwrap_or(&Mass(
                inverse_mass.map_or(0.0, |inverse_mass| 1.0 / inverse_mass.0),
            )),
            *inverse_mass.unwrap_or(&InverseMass(mass.map_or(0.0, |mass| 1.0 / mass.0))),
            *inertia.unwrap_or(
                &inverse_inertia
                    .map_or(Inertia3d::ZERO, |inverse_inertia| inverse_inertia.inverse()),
            ),
            *inverse_inertia
                .unwrap_or(&inertia.map_or(InverseInertia3d::ZERO, |inertia| inertia.inverse())),
            *center_of_mass.unwrap_or(&CenterOfMass3d::default()),
        ));
    }
}

/// Initializes missing components for [colliders](Collider).
fn init_colliders_2d(
    mut commands: Commands,
    mut colliders: Query<
        (
            Entity,
            &Collider2d,
            Option<&ColliderAabb2d>,
            Option<&ColliderDensity>,
            Option<&ColliderMassProperties2d>,
        ),
        Added<Collider2d>,
    >,
) {
    for (entity, collider, aabb, density, mass_properties) in &mut colliders {
        let density = *density.unwrap_or(&ColliderDensity::default());
        commands.entity(entity).insert((
            *aabb.unwrap_or(&ColliderAabb2d::from_shape(collider.shape_scaled())),
            density,
            *mass_properties.unwrap_or(&collider.mass_properties(density.0)),
            CollidingEntities::default(),
        ));
    }
}

/// Initializes missing components for [colliders](Collider).
fn init_colliders_3d(
    mut commands: Commands,
    mut colliders: Query<
        (
            Entity,
            &Collider3d,
            Option<&ColliderAabb3d>,
            Option<&ColliderDensity>,
            Option<&ColliderMassProperties3d>,
        ),
        Added<Collider3d>,
    >,
) {
    for (entity, collider, aabb, density, mass_properties) in &mut colliders {
        let density = *density.unwrap_or(&ColliderDensity::default());
        commands.entity(entity).insert((
            *aabb.unwrap_or(&ColliderAabb3d::from_shape(collider.shape_scaled())),
            density,
            *mass_properties.unwrap_or(&collider.mass_properties(density.0)),
            CollidingEntities::default(),
        ));
    }
}

/// Creates [`Collider`]s from [`AsyncCollider`]s if the meshes have become available.
#[cfg(all(feature = "3d", feature = "async-collider"))]
pub fn init_async_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    async_colliders: Query<(Entity, &Handle<Mesh>, &AsyncCollider)>,
) {
    for (entity, mesh_handle, async_collider) in async_colliders.iter() {
        if let Some(mesh) = meshes.get(mesh_handle) {
            let collider = match &async_collider.0 {
                ComputedCollider::TriMesh => Collider3d::trimesh_from_mesh(mesh),
                ComputedCollider::ConvexHull => Collider3d::convex_hull_from_mesh(mesh),
                ComputedCollider::ConvexDecomposition(params) => {
                    Collider3d::convex_decomposition_from_mesh_with_config(mesh, params)
                }
            };
            if let Some(collider) = collider {
                commands
                    .entity(entity)
                    .insert(collider)
                    .remove::<AsyncCollider>();
            } else {
                error!("Unable to generate collider from mesh {:?}", mesh);
            }
        }
    }
}

/// Creates [`Collider`]s from [`AsyncSceneCollider`]s if the scenes have become available.
#[cfg(all(feature = "3d", feature = "async-collider"))]
pub fn init_async_scene_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    scene_spawner: Res<SceneSpawner>,
    async_colliders: Query<(Entity, &SceneInstance, &AsyncSceneCollider)>,
    children: Query<&Children>,
    mesh_handles: Query<(&Name, &Handle<Mesh>)>,
) {
    for (scene_entity, scene_instance, async_scene_collider) in async_colliders.iter() {
        if scene_spawner.instance_is_ready(**scene_instance) {
            for child_entity in children.iter_descendants(scene_entity) {
                if let Ok((name, handle)) = mesh_handles.get(child_entity) {
                    let Some(collider_data) = async_scene_collider
                        .meshes_by_name
                        .get(name.as_str())
                        .cloned()
                        .unwrap_or(
                            async_scene_collider
                                .default_shape
                                .clone()
                                .map(|shape| AsyncSceneColliderData { shape, ..default() }),
                        )
                    else {
                        continue;
                    };

                    let mesh = meshes.get(handle).expect("mesh should already be loaded");

                    let collider = match collider_data.shape {
                        ComputedCollider::TriMesh => Collider3d::trimesh_from_mesh(mesh),
                        ComputedCollider::ConvexHull => Collider3d::convex_hull_from_mesh(mesh),
                        ComputedCollider::ConvexDecomposition(params) => {
                            Collider3d::convex_decomposition_from_mesh_with_config(mesh, &params)
                        }
                    };
                    if let Some(collider) = collider {
                        commands.entity(child_entity).insert((
                            collider,
                            collider_data.layers,
                            ColliderDensity(collider_data.density),
                        ));
                    } else {
                        error!(
                            "unable to generate collider from mesh {:?} with name {}",
                            mesh, name
                        );
                    }
                }
            }

            commands.entity(scene_entity).remove::<AsyncSceneCollider>();
        }
    }
}

fn update_collider_parents_2d(
    mut commands: Commands,
    mut bodies: Query<(Entity, Option<&mut ColliderParent>, Has<Collider2d>), With<RigidBody2d>>,
    children: Query<&Children>,
    mut child_colliders: Query<
        Option<&mut ColliderParent>,
        (Without<RigidBody2d>, With<Collider2d>),
    >,
) {
    for (entity, collider_parent, has_collider) in &mut bodies {
        if has_collider {
            if let Some(mut collider_parent) = collider_parent {
                collider_parent.0 = entity;
            } else {
                commands.entity(entity).insert((
                    ColliderParent(entity),
                    // Todo: This probably causes a one frame delay. Compute real value?
                    ColliderTransform::default(),
                    PreviousColliderTransform::default(),
                ));
            }
        }
        for child in children.iter_descendants(entity) {
            if let Ok(collider_parent) = child_colliders.get_mut(child) {
                if let Some(mut collider_parent) = collider_parent {
                    collider_parent.0 = entity;
                } else {
                    commands.entity(child).insert((
                        ColliderParent(entity),
                        // Todo: This probably causes a one frame delay. Compute real value?
                        ColliderTransform::default(),
                        PreviousColliderTransform::default(),
                    ));
                }
            }
        }
    }
}

fn update_collider_parents_3d(
    mut commands: Commands,
    mut bodies: Query<(Entity, Option<&mut ColliderParent>, Has<Collider3d>), With<RigidBody3d>>,
    children: Query<&Children>,
    mut child_colliders: Query<
        Option<&mut ColliderParent>,
        (Without<RigidBody3d>, With<Collider3d>),
    >,
) {
    for (entity, collider_parent, has_collider) in &mut bodies {
        if has_collider {
            if let Some(mut collider_parent) = collider_parent {
                collider_parent.0 = entity;
            } else {
                commands.entity(entity).insert((
                    ColliderParent(entity),
                    // Todo: This probably causes a one frame delay. Compute real value?
                    ColliderTransform::default(),
                    PreviousColliderTransform::default(),
                ));
            }
        }
        for child in children.iter_descendants(entity) {
            if let Ok(collider_parent) = child_colliders.get_mut(child) {
                if let Some(mut collider_parent) = collider_parent {
                    collider_parent.0 = entity;
                } else {
                    commands.entity(child).insert((
                        ColliderParent(entity),
                        // Todo: This probably causes a one frame delay. Compute real value?
                        ColliderTransform::default(),
                        PreviousColliderTransform::default(),
                    ));
                }
            }
        }
    }
}

/// Updates colliders when the rigid bodies they were attached to have been removed.
fn handle_rigid_body_removals_2d(
    mut commands: Commands,
    colliders: Query<(Entity, &ColliderParent), Without<RigidBody2d>>,
    bodies: Query<(), With<RigidBody2d>>,
    removals: RemovedComponents<RigidBody2d>,
) {
    // Return if no rigid bodies have been removed
    if removals.is_empty() {
        return;
    }

    for (collider_entity, collider_parent) in &colliders {
        // If the body associated with the collider parent entity doesn't exist,
        // remove ColliderParent and ColliderTransform.
        if !bodies.contains(collider_parent.get()) {
            commands.entity(collider_entity).remove::<(
                ColliderParent,
                ColliderTransform,
                PreviousColliderTransform,
            )>();
        }
    }
}

/// Updates colliders when the rigid bodies they were attached to have been removed.
fn handle_rigid_body_removals_3d(
    mut commands: Commands,
    colliders: Query<(Entity, &ColliderParent), Without<RigidBody3d>>,
    bodies: Query<(), With<RigidBody3d>>,
    removals: RemovedComponents<RigidBody3d>,
) {
    // Return if no rigid bodies have been removed
    if removals.is_empty() {
        return;
    }

    for (collider_entity, collider_parent) in &colliders {
        // If the body associated with the collider parent entity doesn't exist,
        // remove ColliderParent and ColliderTransform.
        if !bodies.contains(collider_parent.get()) {
            commands.entity(collider_entity).remove::<(
                ColliderParent,
                ColliderTransform,
                PreviousColliderTransform,
            )>();
        }
    }
}

/// Updates [`ColliderStorageMap`], a resource that stores some collider properties that need
/// to be handled when colliders are removed from entities.
fn update_collider_storage_2d(
    // TODO: Maybe it's enough to store only colliders that aren't on rigid body entities
    //       directly (i.e. child colliders)
    colliders: Query<
        (
            Entity,
            &ColliderParent,
            &ColliderMassProperties2d,
            &ColliderTransform,
        ),
        (
            With<Collider2d>,
            Or<(
                Changed<ColliderParent>,
                Changed<ColliderTransform>,
                Changed<ColliderMassProperties2d>,
            )>,
        ),
    >,
    mut storage: ResMut<ColliderStorageMap2d>,
) {
    for (entity, parent, collider_mass_properties, collider_transform) in &colliders {
        storage.insert(
            entity,
            (*parent, *collider_mass_properties, *collider_transform),
        );
    }
}

/// Updates [`ColliderStorageMap`], a resource that stores some collider properties that need
/// to be handled when colliders are removed from entities.
fn update_collider_storage_3d(
    // TODO: Maybe it's enough to store only colliders that aren't on rigid body entities
    //       directly (i.e. child colliders)
    colliders: Query<
        (
            Entity,
            &ColliderParent,
            &ColliderMassProperties3d,
            &ColliderTransform,
        ),
        (
            With<Collider3d>,
            Or<(
                Changed<ColliderParent>,
                Changed<ColliderTransform>,
                Changed<ColliderMassProperties3d>,
            )>,
        ),
    >,
    mut storage: ResMut<ColliderStorageMap3d>,
) {
    for (entity, parent, collider_mass_properties, collider_transform) in &colliders {
        storage.insert(
            entity,
            (*parent, *collider_mass_properties, *collider_transform),
        );
    }
}

/// Removes removed colliders from the [`ColliderStorageMap`] resource at the end of the physics frame.
fn handle_collider_storage_removals_2d(
    mut removals: RemovedComponents<Collider2d>,
    mut storage: ResMut<ColliderStorageMap2d>,
) {
    removals.read().for_each(|entity| {
        storage.remove(&entity);
    });
}

/// Removes removed colliders from the [`ColliderStorageMap`] resource at the end of the physics frame.
fn handle_collider_storage_removals_3d(
    mut removals: RemovedComponents<Collider3d>,
    mut storage: ResMut<ColliderStorageMap3d>,
) {
    removals.read().for_each(|entity| {
        storage.remove(&entity);
    });
}

/// Updates each body's mass properties whenever their dependant mass properties or the body's [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
fn update_mass_properties_2d(
    mut bodies: Query<(Entity, &RigidBody2d, MassPropertiesQuery2d)>,
    mut colliders: Query<
        (
            &ColliderTransform,
            &mut PreviousColliderTransform,
            &ColliderParent,
            Ref<Collider2d>,
            &ColliderDensity,
            &mut ColliderMassProperties2d,
        ),
        Or<(
            Changed<Collider2d>,
            Changed<ColliderTransform>,
            Changed<ColliderDensity>,
            Changed<ColliderMassProperties2d>,
        )>,
    >,
    collider_map: Res<ColliderStorageMap2d>,
    mut removed_colliders: RemovedComponents<Collider2d>,
) {
    for (
        collider_transform,
        mut previous_collider_transform,
        collider_parent,
        collider,
        density,
        mut collider_mass_properties,
    ) in &mut colliders
    {
        if let Ok((_, _, mut mass_properties)) = bodies.get_mut(collider_parent.0) {
            // Subtract previous collider mass props from the body's own mass props,
            // If the collider is new, it doesn't have previous mass props, so we shouldn't subtract anything.
            if !collider.is_added() {
                mass_properties -= ColliderMassProperties2d {
                    center_of_mass: CenterOfMass2d(Vector2::from_math(
                        previous_collider_transform.transform_point(Vector3::from_math(
                            collider_mass_properties.center_of_mass.0,
                        )),
                    )),
                    ..*collider_mass_properties
                };
            }

            previous_collider_transform.0 = *collider_transform;

            // Update collider mass props
            *collider_mass_properties = collider.mass_properties(density.max(Scalar::EPSILON));

            // Add new collider mass props to the body's mass props
            mass_properties += ColliderMassProperties2d {
                center_of_mass: CenterOfMass2d(Vector2::from_math(
                    collider_transform.transform_point(Vector3::from_math(
                        collider_mass_properties.center_of_mass.0,
                    )),
                )),
                ..*collider_mass_properties
            };
        }
    }

    // Subtract mass properties of removed colliders
    for entity in removed_colliders.read() {
        if let Some((collider_parent, collider_mass_properties, collider_transform)) =
            collider_map.get(&entity)
        {
            if let Ok((_, _, mut mass_properties)) = bodies.get_mut(collider_parent.0) {
                mass_properties -= ColliderMassProperties2d {
                    center_of_mass: CenterOfMass2d(Vector2::from_math(
                        collider_transform.transform_point(Vector3::from_math(
                            collider_mass_properties.center_of_mass.0,
                        )),
                    )),
                    ..*collider_mass_properties
                };
            }
        }
    }

    for (entity, rb, mut mass_properties) in &mut bodies {
        let is_mass_valid =
            mass_properties.mass.is_finite() && mass_properties.mass.0 >= Scalar::EPSILON;
        let is_inertia_valid =
            mass_properties.inertia.is_finite() && mass_properties.inertia.0 >= Scalar::EPSILON;

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

/// Updates each body's mass properties whenever their dependant mass properties or the body's [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
fn update_mass_properties_3d(
    mut bodies: Query<(Entity, &RigidBody2d, MassPropertiesQuery3d)>,
    mut colliders: Query<
        (
            &ColliderTransform,
            &mut PreviousColliderTransform,
            &ColliderParent,
            Ref<Collider3d>,
            &ColliderDensity,
            &mut ColliderMassProperties3d,
        ),
        Or<(
            Changed<Collider3d>,
            Changed<ColliderTransform>,
            Changed<ColliderDensity>,
            Changed<ColliderMassProperties3d>,
        )>,
    >,
    collider_map: Res<ColliderStorageMap3d>,
    mut removed_colliders: RemovedComponents<Collider3d>,
) {
    for (
        collider_transform,
        mut previous_collider_transform,
        collider_parent,
        collider,
        density,
        mut collider_mass_properties,
    ) in &mut colliders
    {
        if let Ok((_, _, mut mass_properties)) = bodies.get_mut(collider_parent.0) {
            // Subtract previous collider mass props from the body's own mass props,
            // If the collider is new, it doesn't have previous mass props, so we shouldn't subtract anything.
            if !collider.is_added() {
                mass_properties -= ColliderMassProperties3d {
                    center_of_mass: CenterOfMass3d(previous_collider_transform.transform_point(
                        Vector3::from_math(collider_mass_properties.center_of_mass.0),
                    )),
                    ..*collider_mass_properties
                };
            }

            previous_collider_transform.0 = *collider_transform;

            // Update collider mass props
            *collider_mass_properties = collider.mass_properties(density.max(Scalar::EPSILON));

            // Add new collider mass props to the body's mass props
            mass_properties += ColliderMassProperties3d {
                center_of_mass: CenterOfMass3d(collider_transform.transform_point(
                    Vector3::from_math(collider_mass_properties.center_of_mass.0),
                )),
                ..*collider_mass_properties
            };
        }
    }

    // Subtract mass properties of removed colliders
    for entity in removed_colliders.read() {
        if let Some((collider_parent, collider_mass_properties, collider_transform)) =
            collider_map.get(&entity)
        {
            if let Ok((_, _, mut mass_properties)) = bodies.get_mut(collider_parent.0) {
                mass_properties -= ColliderMassProperties3d {
                    center_of_mass: CenterOfMass3d(collider_transform.transform_point(
                        Vector3::from_math(collider_mass_properties.center_of_mass.0),
                    )),
                    ..*collider_mass_properties
                };
            }
        }
    }

    for (entity, rb, mut mass_properties) in &mut bodies {
        let is_mass_valid =
            mass_properties.mass.is_finite() && mass_properties.mass.0 >= Scalar::EPSILON;
        let is_inertia_valid =
            mass_properties.inertia.is_finite() && *mass_properties.inertia != Inertia3d::ZERO;

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

/// Clamps [`ColliderDensity`] to be above 0.0.
fn clamp_collider_density(mut query: Query<&mut ColliderDensity, Changed<ColliderDensity>>) {
    for mut density in &mut query {
        density.0 = density.max(Scalar::EPSILON);
    }
}
