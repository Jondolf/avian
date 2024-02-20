use crate::prelude::*;
#[cfg(all(feature = "3d", feature = "async-collider"))]
use bevy::utils::HashMap;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
    utils::HashSet,
};

/// The default [`Collider`] that uses Parry.
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
mod parry;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub use parry::*;

/// A trait for creating colliders from other types.
pub trait IntoCollider<C: AnyCollider> {
    /// Creates a collider from `self`.
    fn collider(&self) -> C;
}

/// A trait that generalizes over colliders. Implementing this trait
/// allows colliders to be used with the physics engine.
pub trait AnyCollider: Component {
    /// Computes the [Axis-Aligned Bounding Box](ColliderAabb) of the collider
    /// with the given position and rotation.
    #[cfg_attr(
        feature = "2d",
        doc = "\n\nThe rotation is counterclockwise and in radians."
    )]
    fn aabb(&self, position: Vector, rotation: impl Into<Rotation>) -> ColliderAabb;

    /// Computes the swept [Axis-Aligned Bounding Box](ColliderAabb) of the collider.
    /// This corresponds to the space the shape would occupy if it moved from the given
    /// start position to the given end position.
    #[cfg_attr(
        feature = "2d",
        doc = "\n\nThe rotation is counterclockwise and in radians."
    )]
    fn swept_aabb(
        &self,
        start_position: Vector,
        start_rotation: impl Into<Rotation>,
        end_position: Vector,
        end_rotation: impl Into<Rotation>,
    ) -> ColliderAabb {
        self.aabb(start_position, start_rotation)
            .merged(self.aabb(end_position, end_rotation))
    }

    /// Computes the collider's mass properties based on its shape and a given density.
    fn mass_properties(&self, density: Scalar) -> ColliderMassProperties;

    /// Computes all [`ContactManifold`]s between two colliders.
    ///
    /// Returns an empty vector if the colliders are separated by a distance greater than `prediction_distance`
    /// or if the given shapes are invalid.
    fn contact_manifolds(
        &self,
        other: &Self,
        position1: Vector,
        rotation1: impl Into<Rotation>,
        position2: Vector,
        rotation2: impl Into<Rotation>,
        prediction_distance: Scalar,
    ) -> Vec<ContactManifold>;
}

/// A trait for colliders that support scaling.
pub trait ScalableCollider: AnyCollider {
    /// Returns the global scaling factor of the collider.
    fn scale(&self) -> Vector;

    /// Sets the global scaling factor of the collider.
    ///
    /// If the scaling factor is not uniform and the resulting scaled shape
    /// can not be represented exactly, the given `detail` is used for an approximation.
    fn set_scale(&mut self, scale: Vector, detail: u32);

    /// Scales the collider by the given scaling factor.
    ///
    /// If the scaling factor is not uniform and the resulting scaled shape
    /// can not be represented exactly, the given `detail` is used for an approximation.
    fn scale_by(&mut self, factor: Vector, detail: u32) {
        self.set_scale(factor * self.scale(), detail)
    }
}

/// A component that will automatically generate a [`Collider`] based on the entity's `Mesh`.
/// The type of the generated collider can be specified using [`ComputedCollider`].
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// use bevy_xpbd_3d::prelude::*;
///
/// fn setup(mut commands: Commands, mut assets: ResMut<AssetServer>, mut meshes: Assets<Mesh>) {
///     // Spawn a cube with a convex hull collider generated from the mesh
///     commands.spawn((
///         AsyncCollider(ComputedCollider::ConvexHull),
///         PbrBundle {
///             mesh: meshes.add(Mesh::from(Cuboid::default())),
///             ..default()
///         },
///     ));
/// }
/// ```
#[cfg(all(feature = "3d", feature = "async-collider"))]
#[derive(Component, Clone, Debug, Default, Deref, DerefMut)]
pub struct AsyncCollider(pub ComputedCollider);

/// A component that will automatically generate colliders for the meshes in a scene
/// once the scene has been loaded. The type of the generated collider can be specified
/// using [`ComputedCollider`].
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// use bevy_xpbd_3d::prelude::*;
///
/// fn setup(mut commands: Commands, mut assets: ResMut<AssetServer>) {
///     let scene = assets.load("my_model.gltf#Scene0");
///
///     // Spawn the scene and automatically generate triangle mesh colliders
///     commands.spawn((
///         SceneBundle { scene: scene.clone(), ..default() },
///         AsyncSceneCollider::new(Some(ComputedCollider::TriMesh)),
///     ));
///
///     // Specify configuration for specific meshes by name
///     commands.spawn((
///         SceneBundle { scene: scene.clone(), ..default() },
///         AsyncSceneCollider::new(Some(ComputedCollider::TriMesh))
///             .with_shape_for_name("Tree", ComputedCollider::ConvexHull)
///             .with_layers_for_name("Tree", CollisionLayers::from_bits(0b0010, 0b1111))
///             .with_density_for_name("Tree", 2.5),
///     ));
///
///     // Only generate colliders for specific meshes by name
///     commands.spawn((
///         SceneBundle { scene: scene.clone(), ..default() },
///         AsyncSceneCollider::new(None)
///             .with_shape_for_name("Tree", ComputedCollider::ConvexHull),
///     ));
///
///     // Generate colliders for everything except specific meshes by name
///     commands.spawn((
///         SceneBundle { scene, ..default() },
///         AsyncSceneCollider::new(Some(ComputedCollider::TriMeshWithFlags(
///             TriMeshFlags::MERGE_DUPLICATE_VERTICES
///         )))
///         .without_shape_with_name("Tree"),
///     ));
/// }
/// ```
#[cfg(all(feature = "3d", feature = "async-collider"))]
#[derive(Component, Clone, Debug, Default, PartialEq)]
pub struct AsyncSceneCollider {
    /// The default collider type used for each mesh that isn't included in [`meshes_by_name`](#structfield.meshes_by_name).
    /// If `None`, all meshes except the ones in [`meshes_by_name`](#structfield.meshes_by_name) will be skipped.
    pub default_shape: Option<ComputedCollider>,
    /// Specifies data like the collider type and [`CollisionLayers`] for meshes by name.
    /// Entries with a `None` value will be skipped.
    /// For the meshes not found in this `HashMap`, [`default_shape`](#structfield.default_shape)
    /// and all collision layers will be used instead.
    pub meshes_by_name: HashMap<String, Option<AsyncSceneColliderData>>,
}

#[cfg(all(feature = "3d", feature = "async-collider"))]
impl AsyncSceneCollider {
    /// Creates a new [`AsyncSceneCollider`] with the default collider type used for
    /// meshes set to the given `default_shape`.
    ///
    /// If the given collider type is `None`, all meshes except the ones in
    /// [`meshes_by_name`](#structfield.meshes_by_name) will be skipped.
    /// You can add named shapes using [`with_shape_for_name`](Self::with_shape_for_name).
    pub fn new(default_shape: Option<ComputedCollider>) -> Self {
        Self {
            default_shape,
            meshes_by_name: default(),
        }
    }

    /// Specifies the collider type used for a mesh with the given `name`.
    pub fn with_shape_for_name(mut self, name: &str, shape: ComputedCollider) -> Self {
        if let Some(Some(data)) = self.meshes_by_name.get_mut(name) {
            data.shape = shape;
        } else {
            self.meshes_by_name.insert(
                name.to_string(),
                Some(AsyncSceneColliderData { shape, ..default() }),
            );
        }
        self
    }

    /// Specifies the [`CollisionLayers`] used for a mesh with the given `name`.
    pub fn with_layers_for_name(mut self, name: &str, layers: CollisionLayers) -> Self {
        if let Some(Some(data)) = self.meshes_by_name.get_mut(name) {
            data.layers = layers;
        } else {
            self.meshes_by_name.insert(
                name.to_string(),
                Some(AsyncSceneColliderData {
                    layers,
                    ..default()
                }),
            );
        }
        self
    }

    /// Specifies the [`ColliderDensity`] used for a mesh with the given `name`.
    pub fn with_density_for_name(mut self, name: &str, density: Scalar) -> Self {
        if let Some(Some(data)) = self.meshes_by_name.get_mut(name) {
            data.density = density;
        } else {
            self.meshes_by_name.insert(
                name.to_string(),
                Some(AsyncSceneColliderData {
                    density,
                    ..default()
                }),
            );
        }
        self
    }

    /// Sets collider for the mesh associated with the given `name` to `None`, skipping
    /// collider generation for it.
    pub fn without_shape_with_name(mut self, name: &str) -> Self {
        self.meshes_by_name.insert(name.to_string(), None);
        self
    }
}

/// Configuration for a specific collider generated from a scene using [`AsyncSceneCollider`].
#[cfg(all(feature = "3d", feature = "async-collider"))]
#[derive(Clone, Debug, PartialEq)]
pub struct AsyncSceneColliderData {
    /// The type of collider generated for the mesh.
    pub shape: ComputedCollider,
    /// The [`CollisionLayers`] used for this collider.
    pub layers: CollisionLayers,
    /// The [`ColliderDensity`] used for this collider.
    pub density: Scalar,
}

#[cfg(all(feature = "3d", feature = "async-collider"))]
impl Default for AsyncSceneColliderData {
    fn default() -> Self {
        Self {
            shape: ComputedCollider::TriMesh,
            layers: CollisionLayers::default(),
            density: 1.0,
        }
    }
}

/// Determines how a [`Collider`] is generated from a `Mesh`.
///
/// Colliders can be created from meshes with the following components and methods:
///
/// - [`AsyncCollider`] (requires `async-collider` features)
/// - [`AsyncSceneCollider`] (requires `async-collider` features)
/// - [`Collider::trimesh_from_mesh`]
/// - [`Collider::convex_hull_from_mesh`]
/// - [`Collider::convex_decomposition_from_mesh`]
#[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
#[derive(Component, Clone, Debug, Default, PartialEq)]
pub enum ComputedCollider {
    /// A triangle mesh.
    #[default]
    TriMesh,
    /// A triangle mesh with a custom configuration.
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    TriMeshWithFlags(TriMeshFlags),
    /// A convex hull.
    ConvexHull,
    /// A compound shape obtained from a decomposition into convex parts using the specified
    /// [`VHACDParameters`].
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    ConvexDecomposition(VHACDParameters),
}

/// A component that stores the `Entity` ID of the [`RigidBody`] that a [`Collider`] is attached to.
///
/// If the collider is a child of a rigid body, this points to the body's `Entity` ID.
/// If the [`Collider`] component is instead on the same entity as the [`RigidBody`] component,
/// this points to the collider's own `Entity` ID.
///
/// This component is added and updated automatically based on entity hierarchies and should not
/// be modified directly.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     // Spawn a rigid body with one collider on the same entity and two as children.
///     // Each entity will have a ColliderParent component that has the same rigid body entity.
///     commands
///         .spawn((RigidBody::Dynamic, Collider::ball(0.5)))
///         .with_children(|children| {
///             children.spawn((Collider::ball(0.5), Transform::from_xyz(2.0, 0.0, 0.0)));
///             children.spawn((Collider::ball(0.5), Transform::from_xyz(-2.0, 0.0, 0.0)));
///         });
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ColliderParent(pub(crate) Entity);

impl ColliderParent {
    /// Gets the `Entity` ID of the [`RigidBody`] that this [`Collider`] is attached to.
    pub const fn get(&self) -> Entity {
        self.0
    }
}

impl MapEntities for ColliderParent {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.0 = entity_mapper.map_entity(self.0)
    }
}

/// The transform of a collider relative to the rigid body it's attached to.
/// This is in the local space of the body, not the collider itself.
///
/// This is used for computing things like contact positions and a body's center of mass
/// without having to traverse deeply nested hierarchies. It's updated automatically,
/// so you shouldn't modify it manually.
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ColliderTransform {
    /// The translation of a collider in a rigid body's frame of reference.
    pub translation: Vector,
    /// The rotation of a collider in a rigid body's frame of reference.
    pub rotation: Rotation,
    /// The global scale of a collider. Equivalent to the `GlobalTransform` scale.
    pub scale: Vector,
}

impl ColliderTransform {
    /// Transforms a given point by applying the translation, rotation and scale of
    /// this [`ColliderTransform`].
    pub fn transform_point(&self, mut point: Vector) -> Vector {
        point *= self.scale;
        point = self.rotation.rotate(point);
        point += self.translation;
        point
    }
}

impl Default for ColliderTransform {
    fn default() -> Self {
        Self {
            translation: Vector::ZERO,
            rotation: Rotation::default(),
            scale: Vector::ONE,
        }
    }
}

impl From<Transform> for ColliderTransform {
    fn from(value: Transform) -> Self {
        Self {
            #[cfg(feature = "2d")]
            translation: value.translation.truncate().adjust_precision(),
            #[cfg(feature = "3d")]
            translation: value.translation.adjust_precision(),
            rotation: Rotation::from(value.rotation.adjust_precision()),
            #[cfg(feature = "2d")]
            scale: value.scale.truncate().adjust_precision(),
            #[cfg(feature = "3d")]
            scale: value.scale.adjust_precision(),
        }
    }
}

/// A component that marks a [`Collider`] as a sensor, also known as a trigger.
///
/// Sensor colliders send [collision events](ContactReportingPlugin#collision-events) and register intersections,
/// but allow other bodies to pass through them. This is often used to detect when something enters
/// or leaves an area or is intersecting some shape.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands) {
///     // Spawn a static body with a sensor collider.
///     // Other bodies will pass through, but it will still send collision events.
///     commands.spawn((RigidBody::Static, Collider::ball(0.5), Sensor));
/// }
/// ```
#[doc(alias = "Trigger")]
#[derive(Reflect, Clone, Component, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct Sensor;

/// The Axis-Aligned Bounding Box of a [collider](Collider).
#[derive(Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ColliderAabb {
    /// The minimum point of the AABB.
    pub min: Vector,
    /// The maximum point of thr AABB.
    pub max: Vector,
}

impl ColliderAabb {
    /// Creates a new [`ColliderAabb`] from the given `center` and `half_size`.
    pub fn new(center: Vector, half_size: Vector) -> Self {
        Self {
            min: center - half_size,
            max: center + half_size,
        }
    }

    /// Creates a new [`ColliderAabb`] from its minimum and maximum points.
    pub fn from_min_max(min: Vector, max: Vector) -> Self {
        Self { min, max }
    }

    /// Creates a new [`ColliderAabb`] from a given [`SharedShape`].
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    pub fn from_shape(shape: &crate::parry::shape::SharedShape) -> Self {
        let aabb = shape.compute_local_aabb();
        Self {
            min: aabb.mins.into(),
            max: aabb.maxs.into(),
        }
    }

    /// Computes the center of the AABB,
    pub fn center(self) -> Vector {
        (self.min + self.max) / 2.0
    }

    /// Computes the size of the AABB.
    pub fn size(self) -> Vector {
        self.max - self.min
    }

    /// Merges this AABB with another one.
    pub fn merged(self, other: Self) -> Self {
        ColliderAabb {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Checks if `self` intersects with `other`.
    #[inline(always)]
    #[cfg(feature = "2d")]
    pub fn intersects(&self, other: &Self) -> bool {
        let x_overlaps = self.min.x <= other.max.x && self.max.x >= other.min.x;
        let y_overlaps = self.min.y <= other.max.y && self.max.y >= other.min.y;
        x_overlaps && y_overlaps
    }

    /// Checks if `self` intersects with `other`.
    #[inline(always)]
    #[cfg(feature = "3d")]
    pub fn intersects(&self, other: &Self) -> bool {
        let x_overlaps = self.min.x <= other.max.x && self.max.x >= other.min.x;
        let y_overlaps = self.min.y <= other.max.y && self.max.y >= other.min.y;
        let z_overlaps = self.min.z <= other.max.z && self.max.z >= other.min.z;
        x_overlaps && y_overlaps && z_overlaps
    }
}

impl Default for ColliderAabb {
    fn default() -> Self {
        ColliderAabb {
            min: Vector::INFINITY,
            max: Vector::NEG_INFINITY,
        }
    }
}

/// A component that stores the entities that are colliding with an entity.
///
/// This component is automatically added for all entities with a [`Collider`],
/// but it will only be filled if the [`ContactReportingPlugin`] is enabled (by default, it is).
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn my_system(query: Query<(Entity, &CollidingEntities)>) {
///     for (entity, colliding_entities) in &query {
///         println!(
///             "{:?} is colliding with the following entities: {:?}",
///             entity,
///             colliding_entities
///         );
///     }
/// }
/// ```
#[derive(Reflect, Clone, Component, Debug, Default, Deref, DerefMut, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct CollidingEntities(pub HashSet<Entity>);

impl MapEntities for CollidingEntities {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.0 = self
            .0
            .clone()
            .into_iter()
            .map(|e| entity_mapper.map_entity(e))
            .collect()
    }
}
