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
/// The [`Transform`] component will be initialized based on [`Position`] or [`Rotation`]
/// and vice versa. You can configure this synchronization using the [`PrepareConfig`] resource.
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

        app.init_resource::<ColliderStorageMap>()
            .init_resource::<PrepareConfig>()
            .register_type::<PrepareConfig>()
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
                (
                    init_colliders,
                    apply_deferred,
                    update_collider_parents,
                    apply_deferred,
                )
                    .chain()
                    .in_set(PrepareSet::InitColliders),
            )
            .add_systems(
                self.schedule,
                init_transforms.in_set(PrepareSet::InitTransforms),
            )
            .add_systems(
                self.schedule,
                (
                    (
                        sync::propagate_collider_transforms,
                        sync::update_child_collider_position,
                    )
                        .chain()
                        .run_if(any_new_physics_entities),
                    update_mass_properties,
                    clamp_collider_density,
                    clamp_restitution,
                    // All the components we added above must exist before we can simulate the bodies.
                    apply_deferred,
                )
                    .chain()
                    .in_set(PrepareSet::Finalize),
            );

        app.add_systems(
            PhysicsSchedule,
            (
                update_collider_storage.before(PhysicsStepSet::BroadPhase),
                handle_collider_storage_removals.after(PhysicsStepSet::SpatialQuery),
                handle_rigid_body_removals.after(PhysicsStepSet::SpatialQuery),
            ),
        );

        #[cfg(all(feature = "3d", feature = "async-collider"))]
        app.add_systems(Update, (init_async_colliders, init_async_scene_colliders));
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
pub(crate) struct ColliderStorageMap(
    HashMap<Entity, (ColliderParent, ColliderMassProperties, ColliderTransform)>,
);

/// A run condition that returns `true` if new [rigid bodies](RigidBody) or [colliders](Collider)
/// have been added. Used for avoiding unnecessary transform propagation.
fn any_new_physics_entities(query: Query<(), Or<(Added<RigidBody>, Added<Collider>)>>) -> bool {
    !query.is_empty()
}

// TODO: This system feels very overengineered. Try to clean it up?
/// Initializes [`Transform`] based on [`Position`] and [`Rotation`] or vice versa.
fn init_transforms(
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
            Has<RigidBody>,
        ),
        Or<(Added<RigidBody>, Added<Collider>)>,
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

/// Initializes missing components for [colliders](Collider).
fn init_colliders(
    mut commands: Commands,
    mut colliders: Query<
        (
            Entity,
            &Collider,
            Option<&ColliderAabb>,
            Option<&ColliderDensity>,
            Option<&ColliderMassProperties>,
        ),
        Added<Collider>,
    >,
) {
    for (entity, collider, aabb, density, mass_properties) in &mut colliders {
        let density = *density.unwrap_or(&ColliderDensity::default());
        commands.entity(entity).insert((
            *aabb.unwrap_or(&ColliderAabb::from_shape(collider.shape_scaled())),
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
                ComputedCollider::TriMesh => Collider::trimesh_from_mesh(mesh),
                ComputedCollider::TriMeshWithFlags(flags) => {
                    Collider::trimesh_from_mesh_with_config(mesh, *flags)
                }
                ComputedCollider::ConvexHull => Collider::convex_hull_from_mesh(mesh),
                ComputedCollider::ConvexDecomposition(params) => {
                    Collider::convex_decomposition_from_mesh_with_config(mesh, params)
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
                        ComputedCollider::TriMesh => Collider::trimesh_from_mesh(mesh),
                        ComputedCollider::TriMeshWithFlags(flags) => {
                            Collider::trimesh_from_mesh_with_config(mesh, flags)
                        }
                        ComputedCollider::ConvexHull => Collider::convex_hull_from_mesh(mesh),
                        ComputedCollider::ConvexDecomposition(params) => {
                            Collider::convex_decomposition_from_mesh_with_config(mesh, &params)
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

fn update_collider_parents(
    mut commands: Commands,
    mut bodies: Query<(Entity, Option<&mut ColliderParent>, Has<Collider>), With<RigidBody>>,
    children: Query<&Children>,
    mut child_colliders: Query<Option<&mut ColliderParent>, (With<Collider>, Without<RigidBody>)>,
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
fn handle_rigid_body_removals(
    mut commands: Commands,
    colliders: Query<(Entity, &ColliderParent), Without<RigidBody>>,
    bodies: Query<(), With<RigidBody>>,
    removals: RemovedComponents<RigidBody>,
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
fn update_collider_storage(
    // TODO: Maybe it's enough to store only colliders that aren't on rigid body entities
    //       directly (i.e. child colliders)
    colliders: Query<
        (
            Entity,
            &ColliderParent,
            &ColliderMassProperties,
            &ColliderTransform,
        ),
        (
            With<Collider>,
            Or<(
                Changed<ColliderParent>,
                Changed<ColliderTransform>,
                Changed<ColliderMassProperties>,
            )>,
        ),
    >,
    mut storage: ResMut<ColliderStorageMap>,
) {
    for (entity, parent, collider_mass_properties, collider_transform) in &colliders {
        storage.insert(
            entity,
            (*parent, *collider_mass_properties, *collider_transform),
        );
    }
}

/// Removes removed colliders from the [`ColliderStorageMap`] resource at the end of the physics frame.
fn handle_collider_storage_removals(
    mut removals: RemovedComponents<Collider>,
    mut storage: ResMut<ColliderStorageMap>,
) {
    removals.read().for_each(|entity| {
        storage.remove(&entity);
    });
}

/// Updates each body's mass properties whenever their dependant mass properties or the body's [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
fn update_mass_properties(
    mut bodies: Query<(Entity, &RigidBody, MassPropertiesQuery)>,
    mut colliders: Query<
        (
            &ColliderTransform,
            &mut PreviousColliderTransform,
            &ColliderParent,
            Ref<Collider>,
            &ColliderDensity,
            &mut ColliderMassProperties,
        ),
        Or<(
            Changed<Collider>,
            Changed<ColliderTransform>,
            Changed<ColliderDensity>,
            Changed<ColliderMassProperties>,
        )>,
    >,
    collider_map: Res<ColliderStorageMap>,
    mut removed_colliders: RemovedComponents<Collider>,
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
                mass_properties -= ColliderMassProperties {
                    center_of_mass: CenterOfMass(
                        previous_collider_transform
                            .transform_point(collider_mass_properties.center_of_mass.0),
                    ),
                    ..*collider_mass_properties
                };
            }

            previous_collider_transform.0 = *collider_transform;

            // Update collider mass props
            *collider_mass_properties = collider.mass_properties(density.max(Scalar::EPSILON));

            // Add new collider mass props to the body's mass props
            mass_properties += ColliderMassProperties {
                center_of_mass: CenterOfMass(
                    collider_transform.transform_point(collider_mass_properties.center_of_mass.0),
                ),
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
                mass_properties -= ColliderMassProperties {
                    center_of_mass: CenterOfMass(
                        collider_transform
                            .transform_point(collider_mass_properties.center_of_mass.0),
                    ),
                    ..*collider_mass_properties
                };
            }
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

/// Clamps [`ColliderDensity`] to be above 0.0.
fn clamp_collider_density(mut query: Query<&mut ColliderDensity, Changed<ColliderDensity>>) {
    for mut density in &mut query {
        density.0 = density.max(Scalar::EPSILON);
    }
}
