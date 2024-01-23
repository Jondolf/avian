//! Handles generic collider backend logic, like initializing colliders and AABBs and updating related components.
//!
//! See [`ColliderBackendPlugin`].

use std::marker::PhantomData;

use crate::{
    broad_phase::BroadPhaseSet,
    prelude::*,
    prepare::{any_new, PrepareSet},
    sync::SyncSet,
};
#[cfg(all(feature = "3d", feature = "async-collider"))]
use bevy::scene::SceneInstance;
use bevy::{
    prelude::*,
    utils::{intern::Interned, HashMap},
};

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
        app.init_resource::<ColliderStorageMap<C>>();

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
                    )
                        .chain()
                        .run_if(any_new::<C>),
                    update_collider_mass_properties::<C>,
                )
                    .chain()
                    .in_set(PrepareSet::Finalize)
                    .before(prepare::update_mass_properties),
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

        physics_schedule.add_systems((
            update_collider_storage::<C>.before(PhysicsStepSet::BroadPhase),
            handle_collider_storage_removals::<C>.after(PhysicsStepSet::SpatialQuery),
            handle_rigid_body_removals.after(PhysicsStepSet::SpatialQuery),
        ));

        physics_schedule.add_systems(
            wake_on_collider_removed::<C>
                .in_set(PhysicsStepSet::Sleeping)
                .after(sleeping::mark_sleeping_bodies)
                .before(sleeping::wake_on_changed)
                // Allowing ambiguities is required so that it's possible
                // to have multiple collision backends at the same time.
                .ambiguous_with_all(),
        );

        #[cfg(all(feature = "3d", feature = "async-collider"))]
        app.add_systems(Update, (init_async_colliders, init_async_scene_colliders));

        // Update child colliders before narrow phase in substepping loop
        let substep_schedule = app
            .get_schedule_mut(SubstepSchedule)
            .expect("add SubstepSchedule first");
        substep_schedule.add_systems(
            (
                propagate_collider_transforms,
                update_child_collider_position,
            )
                .chain()
                .after(SubstepSet::Integrate)
                .before(SubstepSet::NarrowPhase)
                .ambiguous_with_all(),
        );
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
#[reflect(Component)]
pub(crate) struct PreviousColliderTransform(ColliderTransform);

// TODO: Remove this once component removal hooks exist
/// A hash map that stores some collider data that is needed when colliders are removed from
/// rigid bodies.
///
/// This includes the collider parent for finding the associated rigid body after collider removal,
/// and collider mass properties for updating the rigid body's mass properties when its collider is removed.
///
/// Ideally, we would just have some entity removal event or callback, but that doesn't
/// exist yet, and `RemovedComponents` only returns entities, not component data.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[reflect(Resource)]
pub(crate) struct ColliderStorageMap<C: AnyCollider> {
    pub(crate) map: HashMap<Entity, (ColliderParent, ColliderMassProperties, ColliderTransform)>,
    _phantom: PhantomData<C>,
}

impl<C: AnyCollider> Default for ColliderStorageMap<C> {
    fn default() -> Self {
        Self {
            map: default(),
            _phantom: PhantomData,
        }
    }
}

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
        commands.entity(entity).insert((
            *aabb.unwrap_or(&collider.aabb(Vector::ZERO, Rotation::default())),
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
        aabb.0 = *collider.swept_aabb(start_pos.0, start_rot, end_pos, end_rot);

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
        aabb.maxs.x += prediction_distance;
        aabb.mins.x -= prediction_distance;
        aabb.maxs.y += prediction_distance;
        aabb.mins.y -= prediction_distance;
        #[cfg(feature = "3d")]
        {
            aabb.maxs.z += prediction_distance;
            aabb.mins.z -= prediction_distance;
        }
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
#[allow(clippy::type_complexity)]
fn update_collider_storage<C: AnyCollider>(
    // TODO: Maybe it's enough to store only colliders that aren't on rigid body entities
    //       directly (i.e. child colliders)
    colliders: Query<
        (
            Entity,
            &ColliderParent,
            &ColliderMassProperties,
            &ColliderTransform,
        ),
        Or<(
            Changed<ColliderParent>,
            Changed<ColliderTransform>,
            Changed<ColliderMassProperties>,
        )>,
    >,
    mut storage: ResMut<ColliderStorageMap<C>>,
) {
    for (entity, parent, collider_mass_properties, collider_transform) in &colliders {
        storage.map.insert(
            entity,
            (*parent, *collider_mass_properties, *collider_transform),
        );
    }
}

/// Removes removed colliders from the [`ColliderStorageMap`] resource at the end of the physics frame.
fn handle_collider_storage_removals<C: AnyCollider>(
    mut removals: RemovedComponents<C>,
    mut storage: ResMut<ColliderStorageMap<C>>,
) {
    removals.read().for_each(|entity| {
        storage.map.remove(&entity);
    });
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

        position.0 = parent_pos.0 + parent_rot.rotate(collider_transform.translation);
        #[cfg(feature = "2d")]
        {
            *rotation = *parent_rot + collider_transform.rotation;
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
        Query<(&Transform, &mut C), Without<Parent>>,
        // Child colliders
        Query<(&ColliderTransform, &mut C), With<Parent>>,
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
                let child_transform = ColliderTransform::from(*child_transform);

                // SAFETY:
                // - `child` must have consistent parentage, or the above assertion would panic.
                // Since `child` is parented to a root entity, the entire hierarchy leading to it is consistent.
                // - We may operate as if all descendants are consistent, since `propagate_collider_transform_recursive` will panic before 
                //   continuing to propagate if it encounters an entity with inconsistent parentage.
                // - Since each root entity is unique and the hierarchy is consistent and forest-like,
                //   other root entities' `propagate_collider_transform_recursive` calls will not conflict with this one.
                // - Since this is the only place where `transform_query` gets used, there will be no conflicting fetches elsewhere.
                unsafe {
                    propagate_collider_transforms_recursive(
                        if is_child_rb {
                            ColliderTransform {
                                scale: child_transform.scale,
                                ..default()
                            }
                        } else {
                            let transform = ColliderTransform::from(*transform);

                            ColliderTransform {
                                translation: transform.scale * child_transform.translation,
                                rotation: child_transform.rotation,
                                scale: (transform.scale * child_transform.scale).max(Vector::splat(Scalar::EPSILON)),
                            }
                        },
                        &collider_query,
                        &parent_query,
                        child,
                        transform.is_changed() || parent.is_changed()
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
                        rotation: transform.rotation + child_transform.rotation,
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
    collider_map: Res<ColliderStorageMap<C>>,
    mut removed_colliders: RemovedComponents<C>,
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

    // Subtract mass properties of removed colliders
    for entity in removed_colliders.read() {
        if let Some((collider_parent, collider_mass_properties, collider_transform)) =
            collider_map.map.get(&entity)
        {
            if let Ok((_, mut mass_properties)) = mass_props.get_mut(collider_parent.0) {
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
}

/// Removes the [`Sleeping`] component from sleeping bodies when any of their
/// colliders have been removed.
#[allow(clippy::type_complexity)]
fn wake_on_collider_removed<C: AnyCollider>(
    mut commands: Commands,
    mut bodies: Query<(Entity, &mut TimeSleeping), With<RigidBody>>,
    all_colliders: Query<&ColliderParent>,
    child_colliders: Query<
        &ColliderParent,
        (
            Without<RigidBody>,
            Or<(Changed<C>, Changed<Transform>, Changed<ColliderTransform>)>,
        ),
    >,
    mut removed_colliders: RemovedComponents<C>,
    // This stores some collider data so that we can access it even though the entity has been removed
    collider_storage: Res<ColliderStorageMap<C>>,
) {
    let removed_colliders_iter =
        all_colliders.iter_many(removed_colliders.read().filter_map(|entity| {
            collider_storage
                .map
                .get(&entity)
                .map(|(rb_entity, _, _)| rb_entity.get())
        }));
    for collider_parent in child_colliders.iter().chain(removed_colliders_iter) {
        if let Ok((entity, mut time_sleeping)) = bodies.get_mut(collider_parent.get()) {
            commands.entity(entity).remove::<Sleeping>();
            time_sleeping.0 = 0.0;
        }
    }
}
