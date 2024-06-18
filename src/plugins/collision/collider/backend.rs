//! Handles generic collider backend logic, like initializing colliders and AABBs and updating related components.
//!
//! See [`ColliderBackendPlugin`].

use std::marker::PhantomData;

use crate::{broad_phase::BroadPhaseSet, prelude::*, prepare::PrepareSet};
#[cfg(all(
    feature = "3d",
    feature = "async-collider",
    feature = "default-collider"
))]
use bevy::scene::SceneInstance;
use bevy::{
    ecs::{intern::Interned, system::SystemId},
    prelude::*,
};

/// A plugin for handling generic collider backend logic.
///
/// - Initializes colliders, including [`AsyncCollider`] and [`AsyncSceneCollider`].
/// - Updates [`ColliderAabb`]s.
/// - Updates collider mass properties, also updating rigid bodies accordingly.
///
/// This plugin should typically be used together with the [`ColliderHierarchyPlugin`].
///
/// ## Custom collision backends
///
/// By default, [`PhysicsPlugins`] adds this plugin for the [`Collider`] component.
/// You can also create custom collider backends by implementing the [`AnyCollider`] trait for a type.
///
/// To use a custom collider backend, simply add the [`ColliderBackendPlugin`] with your collider type:
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
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
/// Assuming you have implemented [`AnyCollider`] correctly,
/// it should now work with the rest of the engine just like normal [`Collider`]s!
///
/// Remember to also add the [`ColliderHierarchyPlugin`] for your custom collider
/// type if you want transforms to work for them.
///
/// **Note**: [Spatial queries](spatial_query) are not supported for custom colliders yet.

pub struct ColliderBackendPlugin<C: ScalableCollider> {
    schedule: Interned<dyn ScheduleLabel>,
    _phantom: PhantomData<C>,
}

impl<C: ScalableCollider> ColliderBackendPlugin<C> {
    /// Creates a [`ColliderBackendPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
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
            schedule: PostUpdate.intern(),
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

        // Register a component hook that updates mass properties of rigid bodies
        // when the colliders attached to them are removed.
        app.world_mut()
            .register_component_hooks::<C>()
            .on_remove(|mut world, entity, _| {
                let entity_ref = world.entity(entity);

                // Get the needed collider components.
                // TODO: Is there an efficient way to do this with QueryState?
                let (Some(parent), Some(collider_mass_properties), Some(collider_transform)) = (
                    entity_ref.get::<ColliderParent>().copied(),
                    entity_ref.get::<ColliderMassProperties>().copied(),
                    entity_ref.get::<ColliderTransform>().copied(),
                ) else {
                    return;
                };

                // Get the ID of the one-shot system run for collider removals.
                let ColliderRemovalSystem(system_id) =
                    world.resource::<ColliderRemovalSystem>().to_owned();
                let system_id = *system_id;

                // Handle collider removal with the collider data passed as input.
                world.commands().run_system_with_input(
                    system_id,
                    (parent, collider_mass_properties, collider_transform),
                );
            });

        app.add_systems(
            self.schedule,
            (
                init_colliders::<C>.in_set(PrepareSet::InitColliders),
                init_transforms::<C>
                    .in_set(PrepareSet::InitTransforms)
                    .after(init_transforms::<RigidBody>),
                update_collider_mass_properties::<C>
                    .in_set(PrepareSet::Finalize)
                    .before(prepare::update_mass_properties),
            ),
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

        #[cfg(all(
            feature = "3d",
            feature = "async-collider",
            feature = "default-collider"
        ))]
        app.add_systems(Update, (init_async_colliders, init_async_scene_colliders));
    }
}

/// Initializes missing components for [colliders](Collider).
#[allow(clippy::type_complexity)]
pub(crate) fn init_colliders<C: AnyCollider>(
    mut commands: Commands,
    mut colliders: Query<
        (
            Entity,
            &C,
            Option<&ColliderAabb>,
            Option<&ColliderDensity>,
            Option<&ColliderMassProperties>,
        ),
        Added<C>,
    >,
) {
    for (entity, collider, aabb, density, mass_properties) in &mut colliders {
        let density = *density.unwrap_or(&ColliderDensity::default());
        commands.entity(entity).try_insert((
            *aabb.unwrap_or(&collider.aabb(Vector::ZERO, Rotation::default())),
            density,
            *mass_properties.unwrap_or(&collider.mass_properties(density.0)),
            CollidingEntities::default(),
        ));
    }
}

/// Creates [`Collider`]s from [`AsyncCollider`]s if the meshes have become available.
#[cfg(all(
    feature = "3d",
    feature = "async-collider",
    feature = "default-collider"
))]
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
#[cfg(all(
    feature = "3d",
    feature = "async-collider",
    feature = "default-collider"
))]
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

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
#[allow(clippy::type_complexity)]
fn update_aabb<C: AnyCollider>(
    mut colliders: Query<
        (
            &C,
            &mut ColliderAabb,
            &Position,
            &Rotation,
            Option<&ColliderParent>,
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
        (&Position, Option<&LinearVelocity>, Option<&AngularVelocity>),
        With<Children>,
    >,
    dt: Res<Time>,
    narrow_phase_config: Option<Res<NarrowPhaseConfig>>,
) {
    // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
    let safety_margin_factor = 2.0 * dt.delta_seconds_adjusted();

    for (collider, mut aabb, pos, rot, collider_parent, lin_vel, ang_vel) in &mut colliders {
        let (lin_vel, ang_vel) = if let (Some(lin_vel), Some(ang_vel)) = (lin_vel, ang_vel) {
            (*lin_vel, *ang_vel)
        } else if let Some(Ok((parent_pos, Some(lin_vel), Some(ang_vel)))) =
            collider_parent.map(|p| parent_velocity.get(p.get()))
        {
            // If the rigid body is rotating, off-center colliders will orbit around it,
            // which affects their linear velocities. We need to compute the linear velocity
            // at the offset position.
            // TODO: This assumes that the colliders would continue moving in the same direction,
            //       but because they are orbiting, the direction will change. We should take
            //       into account the uniform circular motion.
            let offset = pos.0 - parent_pos.0;
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
                    pos.0 + lin_vel.0 * safety_margin_factor,
                    *rot + Rotation::from_radians(safety_margin_factor * ang_vel.0),
                )
            }
            #[cfg(feature = "3d")]
            {
                let q = Quaternion::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
                let (x, y, z, w) = (
                    rot.x + safety_margin_factor * 0.5 * q.x,
                    rot.y + safety_margin_factor * 0.5 * q.y,
                    rot.z + safety_margin_factor * 0.5 * q.z,
                    rot.w + safety_margin_factor * 0.5 * q.w,
                );
                (
                    pos.0 + lin_vel.0 * safety_margin_factor,
                    Quaternion::from_xyzw(x, y, z, w).normalize(),
                )
            }
        };

        // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
        *aabb = collider.swept_aabb(start_pos.0, start_rot, end_pos, end_rot);

        // Add narrow phase prediction distance to AABBs to avoid missed collisions
        let prediction_distance = if let Some(ref config) = narrow_phase_config {
            config.prediction_distance
        } else {
            #[cfg(feature = "2d")]
            {
                1.0
            }
            #[cfg(feature = "3d")]
            {
                0.005
            }
        };
        aabb.max.x += prediction_distance;
        aabb.min.x -= prediction_distance;
        aabb.max.y += prediction_distance;
        aabb.min.y -= prediction_distance;
        #[cfg(feature = "3d")]
        {
            aabb.max.z += prediction_distance;
            aabb.min.z -= prediction_distance;
        }
    }
}

/// A resource that stores the system ID for the system that reacts to collider removals.
#[derive(Resource)]
struct ColliderRemovalSystem(SystemId<(ColliderParent, ColliderMassProperties, ColliderTransform)>);

/// Updates the mass properties of bodies and wakes bodies up when an attached collider is removed.
///
/// Takes the removed collider's parent, mass properties, and transform as input.
fn collider_removed(
    In((parent, collider_mass_props, collider_transform)): In<(
        ColliderParent,
        ColliderMassProperties,
        ColliderTransform,
    )>,
    mut commands: Commands,
    mut mass_prop_query: Query<(MassPropertiesQuery, &mut TimeSleeping)>,
) {
    let parent = parent.get();
    if let Ok((mut mass_properties, mut time_sleeping)) = mass_prop_query.get_mut(parent) {
        // Subtract the mass properties of the collider from the mass properties of the rigid body.
        mass_properties -= ColliderMassProperties {
            center_of_mass: CenterOfMass(
                collider_transform.transform_point(collider_mass_props.center_of_mass.0),
            ),
            ..collider_mass_props
        };

        // Wake up the rigid body since removing the collider could also remove active contacts.
        commands.entity(parent).remove::<Sleeping>();
        time_sleeping.0 = 0.0;
    }
}

/// Updates the mass properties of [`Collider`]s and [collider parents](ColliderParent).
#[allow(clippy::type_complexity)]
pub(crate) fn update_collider_mass_properties<C: AnyCollider>(
    mut mass_props: Query<(Entity, MassPropertiesQuery)>,
    mut colliders: Query<
        (
            &ColliderTransform,
            &mut PreviousColliderTransform,
            &ColliderParent,
            Ref<C>,
            &ColliderDensity,
            &mut ColliderMassProperties,
        ),
        Or<(
            Changed<C>,
            Changed<ColliderTransform>,
            Changed<ColliderDensity>,
            Changed<ColliderMassProperties>,
        )>,
    >,
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
        if let Ok((_, mut mass_properties)) = mass_props.get_mut(collider_parent.0) {
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
}
