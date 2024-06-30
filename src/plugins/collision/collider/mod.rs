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

mod constructor;
pub use constructor::{
    ColliderConstructor, ColliderConstructorHierarchy, ColliderConstructorHierarchyConfig,
};

use crate::prelude::*;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
    utils::HashSet,
};

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
