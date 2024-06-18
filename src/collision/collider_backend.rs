//! Handles generic collider backend logic, like initializing colliders and AABBs and updating related components.
//!
//! See [`ColliderBackendPlugin`].

use std::marker::PhantomData;

use crate::{
    broad_phase::BroadPhaseSet,
    prelude::*,
    prepare::{match_any, PrepareSet},
    sync::SyncSet,
};
#[cfg(all(
    feature = "3d",
    feature = "async-collider",
    feature = "default-collider"
))]
use bevy::scene::SceneInstance;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel, system::SystemId},
    prelude::*,
};
use narrow_phase::NarrowPhaseSet;

/// A plugin for handling generic collider backend logic.
///
/// - Initializes colliders.
/// - Updates [`ColliderAabb`]s.
/// - Updates [`ColliderParent`]s.
/// - Updates collider mass properties.
/// - Updates collider scale based on `Transform` scale.
/// - Propagates child collider positions.
///
/// ## Custom collision backends
///
/// By default, [`PhysicsPlugins`] adds this plugin for the [`Collider`] component.
/// You can also create custom collider backends by implementing the [`AnyCollider`] and [`ScalableCollider`] traits.
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
/// Assuming you have implemented [`AnyCollider`] and [`ScalableCollider`] correctly,
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

        // Run transform propagation if new colliders without rigid bodies have been added.
        // The `PreparePlugin` should handle transform propagation for new rigid bodies.
        app.add_systems(
            self.schedule,
            ((
                bevy::transform::systems::sync_simple_transforms,
                bevy::transform::systems::propagate_transforms,
            )
                .chain()
                .run_if(match_any::<(Added<C>, Without<RigidBody>)>),)
                .chain()
                .in_set(PrepareSet::PropagateTransforms)
                // Allowing ambiguities is required so that it's possible
                // to have multiple collision backends at the same time.
                .ambiguous_with_all(),
        );

        app.add_systems(
            self.schedule,
            (
                (
                    init_colliders::<C>,
                    apply_deferred,
                    update_collider_parents::<C>,
                    apply_deferred,
                )
                    .chain()
                    .in_set(PrepareSet::InitColliders),
                init_transforms::<C>
                    .in_set(PrepareSet::InitTransforms)
                    .after(init_transforms::<RigidBody>),
                (
                    (
                        propagate_collider_transforms,
                        update_child_collider_position,
                        update_collider_scale::<C>,
                    )
                        .chain()
                        .run_if(match_any::<Added<C>>),
                    update_collider_mass_properties::<C>,
                )
                    .chain()
                    .in_set(PrepareSet::Finalize)
                    .before(crate::prepare::update_mass_properties),
                update_collider_scale::<C>
                    .after(SyncSet::Update)
                    .before(SyncSet::Last),
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

        physics_schedule
            .add_systems(handle_rigid_body_removals.after(PhysicsStepSet::SpatialQuery));

        // Update child colliders before the narrow phase.
        physics_schedule.add_systems(
            (
                propagate_collider_transforms,
                update_child_collider_position,
            )
                .chain()
                .in_set(NarrowPhaseSet::First)
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

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub(crate) struct PreviousColliderTransform(ColliderTransform);

/// Initializes missing components for [colliders](Collider).
#[allow(clippy::type_complexity)]
fn init_colliders<C: AnyCollider>(
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
            Option<&SpeculativeMargin>,
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
    narow_phase_config: Res<NarrowPhaseConfig>,
    length_unit: Res<PhysicsLengthUnit>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_seconds_adjusted();
    let default_speculative_margin = length_unit.0 * narow_phase_config.default_speculative_margin;
    let contact_tolerance = length_unit.0 * narow_phase_config.contact_tolerance;

    for (collider, mut aabb, pos, rot, collider_parent, speculative_margin, lin_vel, ang_vel) in
        &mut colliders
    {
        let speculative_margin =
            speculative_margin.map_or(default_speculative_margin, |margin| margin.0);

        if speculative_margin <= 0.0 {
            *aabb = collider
                .aabb(pos.0, *rot)
                .grow(Vector::splat(contact_tolerance));
            continue;
        }

        // Expand the AABB based on the body's velocity and CCD speculative margin.
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
                    pos.0
                        + (lin_vel.0 * delta_secs)
                            .clamp_length_max(speculative_margin.max(contact_tolerance)),
                    *rot * Rotation::radians(ang_vel.0 * delta_secs),
                )
            }
            #[cfg(feature = "3d")]
            {
                let q = Quaternion::from_vec4(ang_vel.0.extend(0.0)) * rot.0;
                let (x, y, z, w) = (
                    rot.x + delta_secs * 0.5 * q.x,
                    rot.y + delta_secs * 0.5 * q.y,
                    rot.z + delta_secs * 0.5 * q.z,
                    rot.w + delta_secs * 0.5 * q.w,
                );
                (
                    pos.0
                        + (lin_vel.0 * delta_secs)
                            .clamp_length_max(speculative_margin.max(contact_tolerance)),
                    Quaternion::from_xyzw(x, y, z, w).normalize(),
                )
            }
        };
        // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
        // TODO: Should we expand the AABB in all directions for speculative contacts?
        *aabb = collider.swept_aabb(start_pos.0, start_rot, end_pos, end_rot);
    }
}

#[allow(clippy::type_complexity)]
fn update_collider_parents<C: AnyCollider>(
    mut commands: Commands,
    mut bodies: Query<(Entity, Option<&mut ColliderParent>, Has<C>), With<RigidBody>>,
    children: Query<&Children>,
    mut child_colliders: Query<Option<&mut ColliderParent>, (With<C>, Without<RigidBody>)>,
) {
    for (entity, collider_parent, has_collider) in &mut bodies {
        if has_collider {
            if let Some(mut collider_parent) = collider_parent {
                collider_parent.0 = entity;
            } else {
                commands.entity(entity).try_insert((
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

#[allow(clippy::type_complexity)]
pub(crate) fn update_child_collider_position(
    mut colliders: Query<
        (
            &ColliderTransform,
            &mut Position,
            &mut Rotation,
            &ColliderParent,
        ),
        Without<RigidBody>,
    >,
    parents: Query<(&Position, &Rotation), (With<RigidBody>, With<Children>)>,
) {
    for (collider_transform, mut position, mut rotation, parent) in &mut colliders {
        let Ok((parent_pos, parent_rot)) = parents.get(parent.get()) else {
            continue;
        };

        position.0 = parent_pos.0 + parent_rot * collider_transform.translation;
        #[cfg(feature = "2d")]
        {
            *rotation = *parent_rot * collider_transform.rotation;
        }
        #[cfg(feature = "3d")]
        {
            *rotation = (parent_rot.0 * collider_transform.rotation.0)
                .normalize()
                .into();
        }
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
) {
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

    // Update collider scale for child colliders
    for (collider_transform, mut collider) in &mut colliders.p1() {
        if collider_transform.scale != collider.scale() {
            // TODO: Support configurable subdivision count for shapes that
            //       can't be represented without approximations after scaling.
            collider.set_scale(collider_transform.scale, 10);
        }
    }
}

/// Updates [`ColliderTransform`]s based on entity hierarchies. Each transform is computed by recursively
/// traversing the children of each rigid body and adding their transforms together to form
/// the total transform relative to the body.
///
/// This is largely a clone of `propagate_transforms` in `bevy_transform`.
#[allow(clippy::type_complexity)]
pub(crate) fn propagate_collider_transforms(
    mut root_query: Query<(Entity, Ref<Transform>, &Children), Without<Parent>>,
    collider_query: Query<
        (
            Ref<Transform>,
            Option<&mut ColliderTransform>,
            Option<&Children>,
        ),
        With<Parent>,
    >,
    parent_query: Query<(Entity, Ref<Transform>, Has<RigidBody>, Ref<Parent>)>,
) {
    root_query.par_iter_mut().for_each(
        |(entity, transform,children)| {
            for (child, child_transform, is_child_rb, parent) in parent_query.iter_many(children) {
                assert_eq!(
                    parent.get(), entity,
                    "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
                );
                
                let changed = transform.is_changed() || parent.is_changed();
                let parent_transform = ColliderTransform::from(*transform);
                let child_transform = ColliderTransform::from(*child_transform);
                
                // SAFETY:
                // - `child` must have consistent parentage, or the above assertion would panic.
                // Since `child` is parented to a root entity, the entire hierarchy leading to it is consistent.
                // - We may operate as if all descendants are consistent, since `propagate_collider_transform_recursive` will panic before 
                //   continuing to propagate if it encounters an entity with inconsistent parentage.
                // - Since each root entity is unique and the hierarchy is consistent and forest-like,
                //   other root entities' `propagate_collider_transform_recursive` calls will not conflict with this one.
                // - Since this is the only place where `transform_query` gets used, there will be no conflicting fetches elsewhere.
                let scale = (parent_transform.scale * child_transform.scale).max(Vector::splat(Scalar::EPSILON));
                unsafe {
                    propagate_collider_transforms_recursive(
                        if is_child_rb {
                            ColliderTransform {
                                scale,
                                ..default()
                            }
                        } else {
                            ColliderTransform {
                                translation: parent_transform.scale * child_transform.translation,
                                rotation: child_transform.rotation,
                                scale,
                            }
                        },
                        &collider_query,
                        &parent_query,
                        child,
                        changed,
                    );
                }
            }
        },
    );
}

/// Recursively computes the [`ColliderTransform`] for `entity` and all of its descendants
/// by propagating transforms.
///
/// This is largely a clone of `propagate_recursive` in `bevy_transform`.
///
/// # Panics
///
/// If `entity`'s descendants have a malformed hierarchy, this function will panic occur before propagating
/// the transforms of any malformed entities and their descendants.
///
/// # Safety
///
/// - While this function is running, `transform_query` must not have any fetches for `entity`,
/// nor any of its descendants.
/// - The caller must ensure that the hierarchy leading to `entity`
/// is well-formed and must remain as a tree or a forest. Each entity must have at most one parent.
#[allow(clippy::type_complexity)]
unsafe fn propagate_collider_transforms_recursive(
    transform: ColliderTransform,
    collider_query: &Query<
        (
            Ref<Transform>,
            Option<&mut ColliderTransform>,
            Option<&Children>,
        ),
        With<Parent>,
    >,
    parent_query: &Query<(Entity, Ref<Transform>, Has<RigidBody>, Ref<Parent>)>,
    entity: Entity,
    mut changed: bool,
) {
    let children = {
        // SAFETY: This call cannot create aliased mutable references.
        //   - The top level iteration parallelizes on the roots of the hierarchy.
        //   - The caller ensures that each child has one and only one unique parent throughout the entire
        //     hierarchy.
        //
        // For example, consider the following malformed hierarchy:
        //
        //     A
        //   /   \
        //  B     C
        //   \   /
        //     D
        //
        // D has two parents, B and C. If the propagation passes through C, but the Parent component on D points to B,
        // the above check will panic as the origin parent does match the recorded parent.
        //
        // Also consider the following case, where A and B are roots:
        //
        //  A       B
        //   \     /
        //    C   D
        //     \ /
        //      E
        //
        // Even if these A and B start two separate tasks running in parallel, one of them will panic before attempting
        // to mutably access E.
        let Ok((transform_ref, collider_transform, children)) =
            (unsafe { collider_query.get_unchecked(entity) })
        else {
            return;
        };

        changed |= transform_ref.is_changed();
        if changed {
            if let Some(mut collider_transform) = collider_transform {
                if *collider_transform != transform {
                    *collider_transform = transform;
                }
            }
        }

        children
    };

    let Some(children) = children else { return };
    for (child, child_transform, is_rb, parent) in parent_query.iter_many(children) {
        assert_eq!(
            parent.get(), entity,
            "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
        );

        let child_transform = ColliderTransform::from(*child_transform);

        // SAFETY: The caller guarantees that `transform_query` will not be fetched
        // for any descendants of `entity`, so it is safe to call `propagate_collider_transforms_recursive` for each child.
        //
        // The above assertion ensures that each child has one and only one unique parent throughout the
        // entire hierarchy.
        unsafe {
            propagate_collider_transforms_recursive(
                if is_rb {
                    ColliderTransform {
                        scale: child_transform.scale,
                        ..default()
                    }
                } else {
                    ColliderTransform {
                        translation: transform.transform_point(child_transform.translation),
                        #[cfg(feature = "2d")]
                        rotation: transform.rotation * child_transform.rotation,
                        #[cfg(feature = "3d")]
                        rotation: Rotation(transform.rotation.0 * child_transform.rotation.0),
                        scale: (transform.scale * child_transform.scale)
                            .max(Vector::splat(Scalar::EPSILON)),
                    }
                },
                collider_query,
                parent_query,
                child,
                changed || parent.is_changed(),
            );
        }
    }
}

/// Updates the mass properties of [`Collider`]s and [collider parents](ColliderParent).
#[allow(clippy::type_complexity)]
fn update_collider_mass_properties<C: AnyCollider>(
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
