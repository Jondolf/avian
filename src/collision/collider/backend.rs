//! Handles generic collider backend logic, like initializing colliders and AABBs and updating related components.
//!
//! See [`ColliderBackendPlugin`].

use std::marker::PhantomData;

use crate::{broad_phase::BroadPhaseSet, prelude::*, prepare::PrepareSet, sync::SyncConfig};
#[cfg(all(feature = "bevy_scene", feature = "default-collider"))]
use bevy::scene::SceneInstance;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel, system::SystemId},
    prelude::*,
};
use mass_properties::OnChangeColliderMassProperties;

/// A plugin for handling generic collider backend logic.
///
/// - Initializes colliders, handles [`ColliderConstructor`] and [`ColliderConstructorHierarchy`].
/// - Updates [`ColliderAabb`]s.
/// - Updates collider scale based on `Transform` scale.
/// - Updates collider mass properties, also updating rigid bodies accordingly.
///
/// This plugin should typically be used together with the [`ColliderHierarchyPlugin`].
///
/// ## Custom collision backends
///
/// By default, [`PhysicsPlugins`] adds this plugin for the [`Collider`] component.
/// You can also create custom collider backends by implementing the [`AnyCollider`]
/// and [`ScalableCollider`] traits for a type.
///
/// To use a custom collider backend, simply add the [`ColliderBackendPlugin`] with your collider type:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
/// #
/// # type MyCollider = Collider;
///
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default(),
///             // MyCollider must implement AnyCollider and ScalableCollider.
///             ColliderBackendPlugin::<MyCollider>::default(),
///             // To enable collision detection for the collider,
///             // we also need to add the NarrowPhasePlugin for it.
///             NarrowPhasePlugin::<MyCollider>::default(),
///         ))
///         // ...your other plugins, systems and resources
///         .run();
/// }
/// ```
///
/// Assuming you have implemented the required traits correctly,
/// it should now work with the rest of the engine just like normal [`Collider`]s!
///
/// **Note**: [Spatial queries](spatial_query) are not supported for custom colliders yet.

pub struct ColliderBackendPlugin<C: ScalableCollider> {
    schedule: Interned<dyn ScheduleLabel>,
    _phantom: PhantomData<C>,
}

impl<C: ScalableCollider> ColliderBackendPlugin<C> {
    /// Creates a [`ColliderBackendPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
            _phantom: PhantomData,
        }
    }
}

impl<C: ScalableCollider> Default for ColliderBackendPlugin<C> {
    fn default() -> Self {
        Self {
            schedule: FixedPostUpdate.intern(),
            _phantom: PhantomData,
        }
    }
}

impl<C: ScalableCollider> Plugin for ColliderBackendPlugin<C> {
    fn build(&self, app: &mut App) {
        // Register the one-shot system that is run for all removed colliders.
        if !app.world().contains_resource::<ColliderRemovalSystem>() {
            let collider_removed_id = app.world_mut().register_system(collider_removed);
            app.insert_resource(ColliderRemovalSystem(collider_removed_id));
        }

        // Make sure necessary resources are available.
        app.init_resource::<NarrowPhaseConfig>()
            .init_resource::<PhysicsLengthUnit>();

        let hooks = app.world_mut().register_component_hooks::<C>();

        // Initialize missing components for colliders.
        hooks.on_add(|mut world, entity, _| {
            let existing_global_transform = world
                .entity(entity)
                .get::<GlobalTransform>()
                .copied()
                .unwrap_or_default();
            let global_transform = if existing_global_transform != GlobalTransform::IDENTITY {
                // This collider was built deferred, probably via `ColliderConstructor`.
                existing_global_transform
            } else {
                // This collider *may* have been `spawn`ed directly on a new entity.
                // As such, its global transform is not yet available.
                // You may notice that this will fail if the hierarchy's scale was updated in this
                // frame. Remember that `GlobalTransform` is not updated in between fixed updates.
                // But this is fine, as `update_collider_scale` will be updated in the next fixed update anyway.
                // The reason why we care about initializing this scale here is for those users that opted out of
                // `update_collider_scale` in order to do their own interpolation, which implies that they won't touch
                // the `Transform` component before the collider is initialized, which in turn means that it will
                // always be initialized with the correct `GlobalTransform`.
                let parent_global_transform = world
                    .entity(entity)
                    .get::<Parent>()
                    .and_then(|parent| world.entity(parent.get()).get::<GlobalTransform>().copied())
                    .unwrap_or_default();
                let transform = world
                    .entity(entity)
                    .get::<Transform>()
                    .copied()
                    .unwrap_or_default();
                parent_global_transform * transform
            };

            let scale = global_transform.compute_transform().scale;
            #[cfg(feature = "2d")]
            let scale = scale.xy();

            // Make sure the collider is initialized with the correct scale.
            // This overwrites the scale set by the constructor, but that one is
            // meant to be only changed after initialization.
            world
                .entity_mut(entity)
                .get_mut::<C>()
                .unwrap()
                .set_scale(scale.adjust_precision(), 10);

            let entity_ref = world.entity(entity);
            let collider = entity_ref.get::<C>().unwrap();

            let aabb = entity_ref
                .get::<ColliderAabb>()
                .copied()
                .unwrap_or(collider.aabb(Vector::ZERO, Rotation::default()));
            let density = entity_ref
                .get::<ColliderDensity>()
                .copied()
                .unwrap_or_default();

            let mass_properties = if entity_ref.get::<Sensor>().is_some() {
                ColliderMassProperties::ZERO
            } else {
                collider.mass_properties(density.0)
            };

            world.commands().entity(entity).try_insert((
                aabb,
                density,
                mass_properties,
                CollidingEntities::default(),
                ColliderMarker,
            ));
        });

        // Register a component hook that updates mass properties of rigid bodies
        // when the colliders attached to them are removed.
        // Also removes `ColliderMarker` components.
        app.world_mut()
            .register_component_hooks::<C>()
            .on_remove(|mut world, entity, _| {
                // Remove the `ColliderMarker` associated with the collider.
                // TODO: If the same entity had multiple *different* types of colliders, this would
                //       get removed even if just one collider was removed. This is a very niche edge case though.
                world.commands().entity(entity).remove::<ColliderMarker>();

                let entity_ref = world.entity_mut(entity);

                // Get the needed collider components.
                // TODO: Is there an efficient way to do this with QueryState?
                let (Some(parent), Some(collider_mass_properties)) = (
                    entity_ref.get::<ColliderParent>().copied(),
                    entity_ref.get::<ColliderMassProperties>().copied(),
                ) else {
                    return;
                };

                // Get the ID of the one-shot system run for collider removals.
                let ColliderRemovalSystem(system_id) =
                    world.resource::<ColliderRemovalSystem>().to_owned();
                let system_id = *system_id;

                // Handle collider removal with the collider data passed as input.
                world
                    .commands()
                    .run_system_with_input(system_id, (entity, parent, collider_mass_properties));
            });

        // When the `Sensor` component is added to a collider, trigger an event.
        // The `MassPropertyPlugin` responds to the event and updates the body's mass properties accordingly.
        app.observe(
            |trigger: Trigger<OnAdd, Sensor>,
             mut commands: Commands,
             query: Query<&ColliderMassProperties>| {
                if let Ok(collider_mass_properties) = query.get(trigger.entity()) {
                    // If the collider mass properties are zero, there is nothing to subtract.
                    if *collider_mass_properties == ColliderMassProperties::ZERO {
                        return;
                    }

                    // Trigger an event to update the body's mass propertiess.
                    commands.trigger_targets(
                        OnChangeColliderMassProperties {
                            previous: Some(*collider_mass_properties),
                            current: None,
                        },
                        trigger.entity(),
                    );
                }
            },
        );

        // When the `Sensor` component is removed from a collider, update its mass properties and trigger an event.
        // The `MassPropertyPlugin` responds to the event and updates the body's mass properties accordingly.
        app.observe(
            |trigger: Trigger<OnRemove, Sensor>,
             mut commands: Commands,
             mut collider_query: Query<(
                Ref<C>,
                &ColliderDensity,
                &mut ColliderMassProperties
            )>| {
                if let Ok((collider, density, mut collider_mass_properties)) =
                    collider_query.get_mut(trigger.entity())
                {
                    // Update collider mass props.
                    *collider_mass_properties =
                        collider.mass_properties(density.max(Scalar::EPSILON));

                    // If the collider mass properties are zero, there is nothing to add.
                    if *collider_mass_properties == ColliderMassProperties::ZERO {
                        return;
                    }

                    // Trigger an event to update the body's mass properties.
                    commands.trigger_targets(
                        OnChangeColliderMassProperties {
                            previous: None,
                            current: Some(*collider_mass_properties),
                        },
                        trigger.entity(),
                    );
                }
            },
        );

        app.add_systems(
            self.schedule,
            (
                init_transforms::<C>
                    .in_set(PrepareSet::InitTransforms)
                    .after(init_transforms::<RigidBody>),
                (
                    update_collider_scale::<C>,
                    update_collider_mass_properties::<C>,
                    update_previous_collider_transforms,
                )
                    .chain()
                    .in_set(PrepareSet::Finalize)
                    .before(dynamics::rigid_body::mass_properties::warn_missing_mass),
            ),
        );

        // Update collider parents for colliders that are on the same entity as the rigid body.
        app.add_systems(
            self.schedule,
            update_root_collider_parents::<C>.before(PrepareSet::Finalize),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        // Allowing ambiguities is required so that it's possible
        // to have multiple collision backends at the same time.
        physics_schedule.add_systems(
            update_aabb::<C>
                .in_set(PhysicsStepSet::BroadPhase)
                .after(BroadPhaseSet::First)
                .before(BroadPhaseSet::UpdateStructures)
                .ambiguous_with_all(),
        );

        #[cfg(feature = "default-collider")]
        app.add_systems(
            Update,
            (
                init_collider_constructors,
                init_collider_constructor_hierarchies,
            ),
        );
    }
}

/// A marker component for colliders. Inserted and removed automatically.
///
/// This is useful for filtering collider entities regardless of the [collider backend](ColliderBackendPlugin).
#[derive(Reflect, Component, Clone, Copy, Debug)]
pub struct ColliderMarker;

/// Updates [`ColliderParent`] for colliders that are on the same entity as the [`RigidBody`].
///
/// The [`ColliderHierarchyPlugin`] should be used to handle hierarchies.
fn update_root_collider_parents<C: AnyCollider>(
    mut commands: Commands,
    mut bodies: Query<
        (Entity, Option<&mut ColliderParent>),
        (With<RigidBody>, With<C>, Or<(Added<RigidBody>, Added<C>)>),
    >,
) {
    for (entity, collider_parent) in &mut bodies {
        if let Some(mut collider_parent) = collider_parent {
            collider_parent.0 = entity;
        } else {
            commands.entity(entity).try_insert((
                ColliderParent(entity),
                // TODO: This probably causes a one frame delay. Compute real value?
                ColliderTransform::default(),
                PreviousColliderTransform::default(),
            ));
        }
    }
}

/// Generates [`Collider`]s based on [`ColliderConstructor`]s.
///
/// If a [`ColliderConstructor`] requires a mesh, the system keeps running
/// until the mesh associated with the mesh handle is available.
///
/// # Panics
///
/// Panics if the [`ColliderConstructor`] requires a mesh but no mesh handle is found.
#[cfg(feature = "default-collider")]
fn init_collider_constructors(
    mut commands: Commands,
    #[cfg(feature = "collider-from-mesh")] meshes: Res<Assets<Mesh>>,
    #[cfg(feature = "collider-from-mesh")] mesh_handles: Query<&Handle<Mesh>>,
    constructors: Query<(
        Entity,
        Option<&Collider>,
        Option<&Name>,
        &ColliderConstructor,
    )>,
) {
    for (entity, existing_collider, name, constructor) in constructors.iter() {
        let name = pretty_name(name, entity);
        if existing_collider.is_some() {
            warn!(
                "Tried to add a collider to entity {name} via {constructor:#?}, \
                but that entity already holds a collider. Skipping.",
            );
            commands.entity(entity).remove::<ColliderConstructor>();
            continue;
        }
        #[cfg(feature = "collider-from-mesh")]
        let mesh = if constructor.requires_mesh() {
            let mesh_handle = mesh_handles.get(entity).unwrap_or_else(|_| panic!(
                "Tried to add a collider to entity {name} via {constructor:#?} that requires a mesh, \
                but no mesh handle was found"));
            let mesh = meshes.get(mesh_handle);
            if mesh.is_none() {
                // Mesh required, but not loaded yet
                continue;
            }
            mesh
        } else {
            None
        };

        #[cfg(feature = "collider-from-mesh")]
        let collider = Collider::try_from_constructor(constructor.clone(), mesh);
        #[cfg(not(feature = "collider-from-mesh"))]
        let collider = Collider::try_from_constructor(constructor.clone());

        if let Some(collider) = collider {
            commands.entity(entity).insert(collider);
        } else {
            error!(
                "Tried to add a collider to entity {name} via {constructor:#?}, \
                but the collider could not be generated. Skipping.",
            );
        }
        commands.entity(entity).remove::<ColliderConstructor>();
    }
}

/// Generates [`Collider`]s for descendants of entities with the [`ColliderConstructorHierarchy`] component.
///
/// If an entity has a `SceneInstance`, its collider hierarchy is only generated once the scene is ready.
#[cfg(feature = "default-collider")]
fn init_collider_constructor_hierarchies(
    mut commands: Commands,
    #[cfg(feature = "collider-from-mesh")] meshes: Res<Assets<Mesh>>,
    #[cfg(feature = "collider-from-mesh")] mesh_handles: Query<&Handle<Mesh>>,
    #[cfg(feature = "bevy_scene")] scene_spawner: Res<SceneSpawner>,
    #[cfg(feature = "bevy_scene")] scenes: Query<&Handle<Scene>>,
    #[cfg(feature = "bevy_scene")] scene_instances: Query<&SceneInstance>,
    collider_constructors: Query<(Entity, &ColliderConstructorHierarchy)>,
    children: Query<&Children>,
    child_query: Query<(Option<&Name>, Option<&Collider>)>,
) {
    for (scene_entity, collider_constructor_hierarchy) in collider_constructors.iter() {
        #[cfg(feature = "bevy_scene")]
        {
            if scenes.contains(scene_entity) {
                if let Ok(scene_instance) = scene_instances.get(scene_entity) {
                    if !scene_spawner.instance_is_ready(**scene_instance) {
                        // Wait for the scene to be ready
                        continue;
                    }
                } else {
                    // SceneInstance is added in the SpawnScene schedule, so it might not be available yet
                    continue;
                }
            }
        }

        for child_entity in children.iter_descendants(scene_entity) {
            let Ok((name, existing_collider)) = child_query.get(child_entity) else {
                continue;
            };

            let pretty_name = pretty_name(name, child_entity);

            let default_collider = || {
                Some(ColliderConstructorHierarchyConfig {
                    constructor: collider_constructor_hierarchy.default_constructor.clone(),
                    ..default()
                })
            };

            let collider_data = if let Some(name) = name {
                collider_constructor_hierarchy
                    .config
                    .get(name.as_str())
                    .cloned()
                    .unwrap_or_else(default_collider)
            } else if existing_collider.is_some() {
                warn!("Tried to add a collider to entity {pretty_name} via {collider_constructor_hierarchy:#?}, \
                        but that entity already holds a collider. Skipping. \
                        If this was intentional, add the name of the collider to overwrite to `ColliderConstructorHierarchy.config`.");
                continue;
            } else {
                default_collider()
            };

            // If the configuration is explicitly set to `None`, skip this entity.
            let Some(collider_data) = collider_data else {
                continue;
            };

            // Use the configured constructor if specified, otherwise use the default constructor.
            // If both are `None`, skip this entity.
            let Some(constructor) = collider_data
                .constructor
                .or_else(|| collider_constructor_hierarchy.default_constructor.clone())
            else {
                continue;
            };

            #[cfg(feature = "collider-from-mesh")]
            let mesh = if constructor.requires_mesh() {
                if let Ok(handle) = mesh_handles.get(child_entity) {
                    meshes.get(handle)
                } else {
                    continue;
                }
            } else {
                None
            };

            #[cfg(feature = "collider-from-mesh")]
            let collider = Collider::try_from_constructor(constructor, mesh);
            #[cfg(not(feature = "collider-from-mesh"))]
            let collider = Collider::try_from_constructor(constructor);

            if let Some(collider) = collider {
                commands.entity(child_entity).insert((
                    collider,
                    collider_data
                        .layers
                        .unwrap_or(collider_constructor_hierarchy.default_layers),
                    collider_data
                        .density
                        .unwrap_or(collider_constructor_hierarchy.default_density),
                ));
            } else {
                error!(
                        "Tried to add a collider to entity {pretty_name} via {collider_constructor_hierarchy:#?}, \
                        but the collider could not be generated. Skipping.",
                    );
            }
        }

        commands
            .entity(scene_entity)
            .remove::<ColliderConstructorHierarchy>();
    }
}

#[cfg(feature = "default-collider")]
fn pretty_name(name: Option<&Name>, entity: Entity) -> String {
    name.map(|n| n.to_string())
        .unwrap_or_else(|| format!("<unnamed entity {}>", entity.index()))
}

/// Updates the Axis-Aligned Bounding Boxes of all colliders.
#[allow(clippy::type_complexity)]
fn update_aabb<C: AnyCollider>(
    mut colliders: Query<
        (
            &C,
            &mut ColliderAabb,
            &Position,
            &Rotation,
            Option<&ColliderParent>,
            Option<&CollisionMargin>,
            Option<&SpeculativeMargin>,
            Has<SweptCcd>,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        Or<(
            Changed<Position>,
            Changed<Rotation>,
            Changed<LinearVelocity>,
            Changed<AngularVelocity>,
            Changed<C>,
        )>,
    >,
    parent_velocity: Query<
        (
            &Position,
            &CenterOfMass,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        With<Children>,
    >,
    narrow_phase_config: Res<NarrowPhaseConfig>,
    length_unit: Res<PhysicsLengthUnit>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();
    let default_speculative_margin = length_unit.0 * narrow_phase_config.default_speculative_margin;
    let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;

    for (
        collider,
        mut aabb,
        pos,
        rot,
        collider_parent,
        collision_margin,
        speculative_margin,
        has_swept_ccd,
        lin_vel,
        ang_vel,
    ) in &mut colliders
    {
        let collision_margin = collision_margin.map_or(0.0, |margin| margin.0);
        let speculative_margin = if has_swept_ccd {
            Scalar::MAX
        } else {
            speculative_margin.map_or(default_speculative_margin, |margin| margin.0)
        };

        if speculative_margin <= 0.0 {
            *aabb = collider
                .aabb(pos.0, *rot)
                .grow(Vector::splat(contact_tolerance + collision_margin));
            continue;
        }

        // Expand the AABB based on the body's velocity and CCD speculative margin.
        let (lin_vel, ang_vel) = if let (Some(lin_vel), Some(ang_vel)) = (lin_vel, ang_vel) {
            (*lin_vel, *ang_vel)
        } else if let Some(Ok((parent_pos, center_of_mass, Some(lin_vel), Some(ang_vel)))) =
            collider_parent.map(|p| parent_velocity.get(p.get()))
        {
            // If the rigid body is rotating, off-center colliders will orbit around it,
            // which affects their linear velocities. We need to compute the linear velocity
            // at the offset position.
            // TODO: This assumes that the colliders would continue moving in the same direction,
            //       but because they are orbiting, the direction will change. We should take
            //       into account the uniform circular motion.
            let offset = pos.0 - parent_pos.0 - center_of_mass.0;
            #[cfg(feature = "2d")]
            let vel_at_offset =
                lin_vel.0 + Vector::new(-ang_vel.0 * offset.y, ang_vel.0 * offset.x) * 1.0;
            #[cfg(feature = "3d")]
            let vel_at_offset = lin_vel.0 + ang_vel.cross(offset);
            (LinearVelocity(vel_at_offset), *ang_vel)
        } else {
            (LinearVelocity::ZERO, AngularVelocity::ZERO)
        };

        // Current position and predicted position for next feame
        let (start_pos, start_rot) = (*pos, *rot);
        let (end_pos, end_rot) = {
            #[cfg(feature = "2d")]
            {
                (
                    pos.0
                        + (lin_vel.0 * delta_secs)
                            .clamp_length_max(speculative_margin.max(contact_tolerance)),
                    *rot * Rotation::radians(ang_vel.0 * delta_secs),
                )
            }
            #[cfg(feature = "3d")]
            {
                let mut end_rot =
                    Rotation(Quaternion::from_scaled_axis(ang_vel.0 * delta_secs) * rot.0);
                end_rot.renormalize();
                (
                    pos.0
                        + (lin_vel.0 * delta_secs)
                            .clamp_length_max(speculative_margin.max(contact_tolerance)),
                    end_rot,
                )
            }
        };
        // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
        // TODO: Should we expand the AABB in all directions for speculative contacts?
        *aabb = collider
            .swept_aabb(start_pos.0, start_rot, end_pos, end_rot)
            .grow(Vector::splat(collision_margin));
    }
}

/// Updates the scale of colliders based on [`Transform`] scale.
#[allow(clippy::type_complexity)]
pub fn update_collider_scale<C: ScalableCollider>(
    mut colliders: ParamSet<(
        // Root bodies
        Query<(&Transform, &mut C), (Without<Parent>, Changed<Transform>)>,
        // Child colliders
        Query<(&ColliderTransform, &mut C), (With<Parent>, Changed<ColliderTransform>)>,
    )>,
    sync_config: Res<SyncConfig>,
) {
    if sync_config.transform_to_collider_scale {
        // Update collider scale for root bodies
        for (transform, mut collider) in &mut colliders.p0() {
            #[cfg(feature = "2d")]
            let scale = transform.scale.truncate().adjust_precision();
            #[cfg(feature = "3d")]
            let scale = transform.scale.adjust_precision();
            if scale != collider.scale() {
                // TODO: Support configurable subdivision count for shapes that
                //       can't be represented without approximations after scaling.
                collider.set_scale(scale, 10);
            }
        }
    }
    // Update collider scale for child colliders
    for (collider_transform, mut collider) in &mut colliders.p1() {
        if collider_transform.scale != collider.scale() {
            // TODO: Support configurable subdivision count for shapes that
            //       can't be represented without approximations after scaling.
            collider.set_scale(collider_transform.scale, 10);
        }
    }
}

/// A resource that stores the system ID for the system that reacts to collider removals.
#[derive(Resource)]
struct ColliderRemovalSystem(SystemId<(Entity, ColliderParent, ColliderMassProperties)>);

/// Updates the mass properties of bodies and wakes bodies up when an attached collider is removed.
///
/// Takes the removed collider's entity, parent, mass properties, and transform as input.
fn collider_removed(
    In((collider_entity, parent, collider_mass_props)): In<(
        Entity,
        ColliderParent,
        ColliderMassProperties,
    )>,
    mut commands: Commands,
    mut mass_prop_query: Query<&mut TimeSleeping>,
) {
    let parent = parent.get();
    if let Ok(mut time_sleeping) = mass_prop_query.get_mut(parent) {
        // Subtract the mass properties of the collider from the mass properties of the rigid body.
        commands.trigger_targets(
            OnChangeColliderMassProperties {
                previous: Some(collider_mass_props),
                current: None,
            },
            collider_entity,
        );

        // Wake up the rigid body since removing the collider could also remove active contacts.
        commands.entity(parent).remove::<Sleeping>();
        time_sleeping.0 = 0.0;
    }
}

/// Updates the mass properties of [`Collider`]s and [collider parents](ColliderParent).
#[allow(clippy::type_complexity)]
pub(crate) fn update_collider_mass_properties<C: AnyCollider>(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            Ref<C>,
            &ColliderDensity,
            &mut ColliderMassProperties,
        ),
        (
            Or<(
                Changed<C>,
                Changed<ColliderTransform>,
                Changed<ColliderDensity>,
                Changed<ColliderMassProperties>,
            )>,
            Without<Sensor>,
        ),
    >,
) {
    for (entity, collider, density, mut collider_mass_properties) in &mut query {
        // If the collider is new, it doesn't have previous mass properties.
        let previous = (!collider.is_added()).then_some(*collider_mass_properties);

        // Update the collider's mass properties.
        *collider_mass_properties = collider.mass_properties(density.max(Scalar::EPSILON));

        let current = Some(*collider_mass_properties);

        // The `MassPropertyPlugin` will respond to this event and update the body's mass properties accordingly.
        commands.trigger_targets(OnChangeColliderMassProperties { previous, current }, entity);
    }
}

/// Updates the [`PreviousColliderTransform`] component for colliders.
pub(crate) fn update_previous_collider_transforms(
    mut query: Query<(&ColliderTransform, &mut PreviousColliderTransform)>,
) {
    for (collider_transform, mut previous_collider_transform) in &mut query {
        previous_collider_transform.0 = *collider_transform;
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "default-collider")]
    use super::*;

    #[test]
    #[cfg(feature = "default-collider")]
    fn sensor_mass_properties() {
        let mut app = App::new();

        app.init_schedule(PhysicsSchedule)
            .init_schedule(SubstepSchedule);

        app.add_plugins((
            PreparePlugin::new(FixedPostUpdate),
            MassPropertyPlugin::new(FixedPostUpdate),
            ColliderBackendPlugin::<Collider>::new(FixedPostUpdate),
            ColliderHierarchyPlugin::new(FixedPostUpdate),
            HierarchyPlugin,
        ));

        let collider = Collider::capsule(0.5, 2.0);
        let mass_properties = MassPropertiesBundle::new_computed(&collider, 1.0);

        let parent = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                mass_properties.clone(),
                TransformBundle::default(),
            ))
            .id();

        let child = app
            .world_mut()
            .spawn((
                collider,
                TransformBundle::from_transform(Transform::from_xyz(1.0, 0.0, 0.0)),
            ))
            .set_parent(parent)
            .id();

        app.world_mut().run_schedule(FixedPostUpdate);

        assert_eq!(
            app.world()
                .entity(parent)
                .get::<Mass>()
                .expect("rigid body should have mass")
                .value(),
            2.0 * mass_properties.mass.value(),
        );
        assert!(
            app.world()
                .entity(parent)
                .get::<CenterOfMass>()
                .expect("rigid body should have a center of mass")
                .x
                > 0.0,
        );

        // Mark the collider as a sensor. It should no longer contribute to the mass properties of the rigid body.
        let mut entity_mut = app.world_mut().entity_mut(child);
        entity_mut.insert(Sensor);
        entity_mut.flush();

        assert_eq!(
            app.world()
                .entity(parent)
                .get::<Mass>()
                .expect("rigid body should have mass")
                .value(),
            mass_properties.mass.value(),
        );
        assert!(
            app.world()
                .entity(parent)
                .get::<CenterOfMass>()
                .expect("rigid body should have a center of mass")
                .x
                == 0.0,
        );

        // Remove the sensor component. The collider should contribute to the mass properties of the rigid body again.
        let mut entity_mut = app.world_mut().entity_mut(child);
        entity_mut.remove::<Sensor>();
        entity_mut.flush();

        assert_eq!(
            app.world()
                .entity(parent)
                .get::<Mass>()
                .expect("rigid body should have mass")
                .value(),
            2.0 * mass_properties.mass.value(),
        );
        assert!(
            app.world()
                .entity(parent)
                .get::<CenterOfMass>()
                .expect("rigid body should have a center of mass")
                .x
                > 0.0,
        );
    }
}
