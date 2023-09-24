use std::fmt;

use crate::prelude::*;
#[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
use bevy::render::mesh::{Indices, VertexAttributeValues};
use bevy::{prelude::*, utils::HashSet};
use derive_more::From;
use parry::{
    bounding_volume::Aabb,
    shape::{SharedShape, TypedShape},
};

/// Flags used for the preprocessing of a triangle mesh collider.
pub type TriMeshFlags = parry::shape::TriMeshFlags;

/// A collider used for detecting collisions and generating contacts.
///
/// ## Creation
///
/// `Collider` has tons of methods for creating colliders of various shapes:
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Create a ball collider with a given radius
/// commands.spawn(Collider::ball(0.5));
/// // Create a capsule collider with a given height and radius
/// commands.spawn(Collider::capsule(2.0, 0.5));
/// # }
/// ```
///
/// Colliders on their own only detect contacts and generate [collision events](#collision-events).
/// To make colliders apply contact forces, they have to be attached to [rigid bodies](RigidBody):
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// #
/// // Spawn a dynamic body that falls onto a static platform
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::ball(0.5),
///         Transform::from_xyz(0.0, 2.0, 0.0),
///     ));
#[cfg_attr(
    feature = "2d",
    doc = "    commands.spawn(((RigidBody::Static, Collider::cuboid(5.0, 0.5));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    commands.spawn(((RigidBody::Static, Collider::cuboid(5.0, 0.5, 5.0));"
)]
/// }
/// ```
///
/// Colliders can be further configured using various components like [`Friction`], [`Restitution`],
/// [`Sensor`], and [`CollisionLayers`].
///
/// In addition, Bevy XPBD automatically adds some other components for colliders, like the following:
///
/// - [`ColliderAabb`]
/// - [`CollidingEntities`]
/// - [`ColliderMassProperties`]
///
/// ## Multiple colliders
///
/// It can often be useful to attach multiple colliders to the same rigid body.
///
/// This can be done in two ways. Either use [`Collider::compound`] to have one collider that consists of many
/// shapes, or for more control, spawn several collider entities as the children of a rigid body:
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// #
/// fn setup(mut commands: Commands) {
///     // Spawn a rigid body with one collider on the same entity and two as children
///     commands
///         .spawn((RigidBody::Dynamic, Collider::ball(0.5)))
///         .with_children(|children| {
///             // Spawn the child colliders positioned relative to the rigid body
///             children.spawn((Collider::ball(0.5), Transform::from_xyz(2.0, 0.0, 0.0)));
///             children.spawn((Collider::ball(0.5), Transform::from_xyz(-2.0, 0.0, 0.0)));
///         });
/// }
/// ```
///
/// Colliders can be arbitrarily nested and transformed relative to the parent.
/// The rigid body that a collider is attached to can be accessed using the [`ColliderParent`] component.
///
/// The benefit of using separate entities for the colliders is that each collider can have its own
/// [friction](Friction), [restitution](Restitution), [collision layers](CollisionLayers),
/// and other configuration options, and they send separate [collision events](#collision-events).
///
/// ## Sensors
///
/// If you want a collider to be attached to a rigid body but don't want it to apply forces on
/// contact, you can add a [`Sensor`] component to make the collider only send
/// [collision events](#collision-events):
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // This body will pass through objects but still generate collision events
/// commands.spawn((
///     RigidBody::Dynamic,
///     Collider::ball(0.5),
///     Sensor,
/// ));
/// # }
/// ```
///
/// ## Collision layers
///
/// Collision layers can be used to configure which entities can collide with each other.
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// #
/// #[derive(PhysicsLayer)]
/// enum Layer {
///     Player,
///     Enemy,
///     Ground,
/// }
///
/// fn spawn(mut commands: Commands) {
///     commands.spawn((
///         Collider::ball(0.5),
///         // Player collides with enemies and the ground, but not with other players
///         CollisionLayers::new([Layer::Player], [Layer::Enemy, Layer::Ground])
///     ));
/// }
/// ```
///
/// See [`CollisionLayers`] for more information and examples.
///
/// ## Collision events
///
/// There are currently three different collision events: [`Collision`], [`CollisionStarted`] and [`CollisionEnded`].
/// You can listen to these events as you normally would.
///
/// For example, you could read [contacts](Contacts) between entities like this:
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// #
/// fn my_system(mut collision_event_reader: EventReader<Collision>) {
///     for Collision(contact_pair) in collision_event_reader.iter() {
///         println!(
///             "{:?} and {:?} are colliding",
///             contact_pair.entity1, contact_pair.entity2
///         );
///     }
/// }
/// ```
///
/// The entities that are colliding with a given entity can also be accessed using
/// the [`CollidingEntities`] component.
///
/// For even more control, you can use the [`Collisions`] resource to get, iterate, filter and modify
/// collisions.
///
/// ## Advanced usage
///
/// Internally, `Collider` uses the shapes provided by `parry`. If you want to create a collider
/// using these shapes, you can simply use `Collider::from(SharedShape::some_method())`.
///
/// To get a reference to the internal [`SharedShape`], you can use the [`get_shape`](#method.get_shape) method.
#[derive(Clone, Component, Deref, DerefMut, From)]
pub struct Collider(SharedShape);

impl Default for Collider {
    fn default() -> Self {
        #[cfg(feature = "2d")]
        {
            Self(SharedShape::cuboid(0.5, 0.5))
        }
        #[cfg(feature = "3d")]
        {
            Self(SharedShape::cuboid(0.5, 0.5, 0.5))
        }
    }
}

impl fmt::Debug for Collider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.as_typed_shape() {
            TypedShape::Ball(shape) => write!(f, "{:?}", shape),
            TypedShape::Cuboid(shape) => write!(f, "{:?}", shape),
            TypedShape::RoundCuboid(shape) => write!(f, "{:?}", shape),
            TypedShape::Capsule(shape) => write!(f, "{:?}", shape),
            TypedShape::Segment(shape) => write!(f, "{:?}", shape),
            TypedShape::Triangle(shape) => write!(f, "{:?}", shape),
            TypedShape::RoundTriangle(shape) => write!(f, "{:?}", shape),
            TypedShape::TriMesh(_) => write!(f, "Trimesh (not representable)"),
            TypedShape::Polyline(_) => write!(f, "Polyline (not representable)"),
            TypedShape::HalfSpace(shape) => write!(f, "{:?}", shape),
            TypedShape::HeightField(shape) => write!(f, "{:?}", shape),
            TypedShape::Compound(_) => write!(f, "Compound (not representable)"),
            TypedShape::Custom(shape) => write!(f, "{:?}", shape),
            #[cfg(feature = "3d")]
            TypedShape::ConvexPolyhedron(shape) => write!(f, "{:?}", shape),
            #[cfg(feature = "3d")]
            TypedShape::Cylinder(shape) => write!(f, "{:?}", shape),
            #[cfg(feature = "3d")]
            TypedShape::Cone(shape) => write!(f, "{:?}", shape),
            #[cfg(feature = "3d")]
            TypedShape::RoundCylinder(shape) => write!(f, "{:?}", shape),
            #[cfg(feature = "3d")]
            TypedShape::RoundCone(shape) => write!(f, "{:?}", shape),
            #[cfg(feature = "3d")]
            TypedShape::RoundConvexPolyhedron(shape) => write!(f, "{:?}", shape),
            #[cfg(feature = "2d")]
            TypedShape::ConvexPolygon(shape) => write!(f, "{:?}", shape),
            #[cfg(feature = "2d")]
            TypedShape::RoundConvexPolygon(shape) => write!(f, "{:?}", shape),
        }
    }
}

impl Collider {
    /// Returns the raw shape of the collider. The shapes are provided by [`parry`].
    pub fn get_shape(&self) -> &SharedShape {
        &self.0
    }

    /// Returns a mutable reference to the raw shape of the collider. The shapes are provided by [`parry`].
    pub fn get_shape_mut(&mut self) -> &mut SharedShape {
        &mut self.0
    }

    /// Computes the [Axis-Aligned Bounding Box](ColliderAabb) of the collider.
    #[cfg(feature = "2d")]
    pub fn compute_aabb(&self, position: Vector, rotation: Scalar) -> ColliderAabb {
        ColliderAabb(self.get_shape().compute_aabb(&utils::make_isometry(
            position,
            &Rotation::from_radians(rotation),
        )))
    }

    /// Computes the [Axis-Aligned Bounding Box](ColliderAabb) of the collider.
    #[cfg(feature = "3d")]
    pub fn compute_aabb(&self, position: Vector, rotation: Quaternion) -> ColliderAabb {
        ColliderAabb(
            self.get_shape()
                .compute_aabb(&utils::make_isometry(position, &Rotation(rotation))),
        )
    }

    /// Creates a collider with a compound shape defined by a given vector of colliders with a position and a rotation.
    ///
    /// Especially for dynamic rigid bodies, compound shape colliders should be preferred over triangle meshes and polylines,
    /// because convex shapes typically provide more reliable results.
    ///
    /// If you want to create a compound shape from a 3D triangle mesh or 2D polyline, consider using the
    /// [`Collider::convex_decomposition`](#method.convex_decomposition) method.
    pub fn compound(
        shapes: Vec<(
            impl Into<Position>,
            impl Into<Rotation>,
            impl Into<Collider>,
        )>,
    ) -> Self {
        let shapes = shapes
            .into_iter()
            .map(|(p, r, c)| {
                (
                    utils::make_isometry(*p.into(), &r.into()),
                    c.into().get_shape().clone(),
                )
            })
            .collect::<Vec<_>>();
        SharedShape::compound(shapes).into()
    }

    /// Creates a collider with a ball shape defined by its radius.
    pub fn ball(radius: Scalar) -> Self {
        SharedShape::ball(radius).into()
    }

    /// Creates a collider with a cuboid shape defined by its extents.
    #[cfg(feature = "2d")]
    pub fn cuboid(x_length: Scalar, y_length: Scalar) -> Self {
        SharedShape::cuboid(x_length * 0.5, y_length * 0.5).into()
    }

    /// Creates a collider with a cuboid shape defined by its extents.
    #[cfg(feature = "3d")]
    pub fn cuboid(x_length: Scalar, y_length: Scalar, z_length: Scalar) -> Self {
        SharedShape::cuboid(x_length * 0.5, y_length * 0.5, z_length * 0.5).into()
    }

    /// Creates a collider with a cylinder shape defined by its height along the `Y` axis and its radius on the `XZ` plane.
    #[cfg(feature = "3d")]
    pub fn cylinder(height: Scalar, radius: Scalar) -> Self {
        SharedShape::cylinder(height * 0.5, radius).into()
    }

    /// Creates a collider with a cone shape defined by its height along the `Y` axis and the radius of its base on the `XZ` plane.
    #[cfg(feature = "3d")]
    pub fn cone(height: Scalar, radius: Scalar) -> Self {
        SharedShape::cone(height * 0.5, radius).into()
    }

    /// Creates a collider with a capsule shape defined by its height along the `Y` axis and its radius.
    pub fn capsule(height: Scalar, radius: Scalar) -> Self {
        SharedShape::capsule(
            (Vector::Y * height * 0.5).into(),
            (Vector::NEG_Y * height * 0.5).into(),
            radius,
        )
        .into()
    }

    /// Creates a collider with a capsule shape defined by its end points `a` and `b` and its radius.
    pub fn capsule_endpoints(a: Vector, b: Vector, radius: Scalar) -> Self {
        SharedShape::capsule(a.into(), b.into(), radius).into()
    }

    /// Creates a collider with a [half-space](https://en.wikipedia.org/wiki/Half-space_(geometry)) shape defined by the outward normal of its planar boundary.
    pub fn halfspace(outward_normal: Vector) -> Self {
        SharedShape::halfspace(nalgebra::Unit::new_normalize(outward_normal.into())).into()
    }

    /// Creates a collider with a segment shape defined by its endpoints `a` and `b`.
    pub fn segment(a: Vector, b: Vector) -> Self {
        SharedShape::segment(a.into(), b.into()).into()
    }

    /// Creates a collider with a triangle shape defined by its points `a`, `b` and `c`.
    pub fn triangle(a: Vector, b: Vector, c: Vector) -> Self {
        SharedShape::triangle(a.into(), b.into(), c.into()).into()
    }

    /// Creates a collider with a polyline shape defined by its vertices and optionally an index buffer.
    pub fn polyline(vertices: Vec<Vector>, indices: Option<Vec<[u32; 2]>>) -> Self {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        SharedShape::polyline(vertices, indices).into()
    }

    /// Creates a collider with a triangle mesh shape defined by its vertex and index buffers.
    pub fn trimesh(vertices: Vec<Vector>, indices: Vec<[u32; 3]>) -> Self {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        SharedShape::trimesh(vertices, indices).into()
    }

    /// Creates a collider with a triangle mesh shape defined by its vertex and index buffers
    /// and flags controlling the preprocessing.
    pub fn trimesh_with_flags(
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
        flags: TriMeshFlags,
    ) -> Self {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        SharedShape::trimesh_with_flags(vertices, indices, flags).into()
    }

    /// Creates a collider with a triangle mesh shape built from a given Bevy `Mesh`.
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn trimesh_from_bevy_mesh(mesh: &Mesh) -> Option<Self> {
        use parry::shape::TriMeshFlags;

        let vertices_indices = extract_mesh_vertices_indices(mesh);
        vertices_indices.map(|(v, i)| {
            SharedShape::trimesh_with_flags(v, i, TriMeshFlags::MERGE_DUPLICATE_VERTICES).into()
        })
    }

    /// Creates a collider with a triangle mesh shape built from a given Bevy `Mesh` and flags
    /// controlling its preprocessing.
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn trimesh_from_bevy_mesh_with_flags(mesh: &Mesh, flags: TriMeshFlags) -> Option<Self> {
        let vertices_indices = extract_mesh_vertices_indices(mesh);
        vertices_indices.map(|(v, i)| SharedShape::trimesh_with_flags(v, i, flags).into())
    }

    /// Creates a collider with a compound shape obtained from the decomposition of a triangle mesh
    /// built from a given Bevy `Mesh`.
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn convex_decomposition_from_bevy_mesh(mesh: &Mesh) -> Option<Self> {
        let vertices_indices = extract_mesh_vertices_indices(mesh);
        vertices_indices.map(|(v, i)| SharedShape::convex_decomposition(&v, &i).into())
    }

    /// Creates a collider shape with a compound shape obtained from the decomposition of a given polyline
    /// defined by its vertex and index buffers.
    #[cfg(feature = "2d")]
    pub fn convex_decomposition(vertices: Vec<Vector>, indices: Vec<[u32; 2]>) -> Self {
        let vertices = vertices.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        SharedShape::convex_decomposition(&vertices, &indices).into()
    }

    /// Creates a collider shape with a compound shape obtained from the decomposition of a given trimesh
    /// defined by its vertex and index buffers.
    #[cfg(feature = "3d")]
    pub fn convex_decomposition(vertices: Vec<Vector>, indices: Vec<[u32; 3]>) -> Self {
        let vertices = vertices.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        SharedShape::convex_decomposition(&vertices, &indices).into()
    }

    /// Creates a collider with a [convex polygon](https://en.wikipedia.org/wiki/Convex_polygon) shape obtained after computing
    /// the [convex hull](https://en.wikipedia.org/wiki/Convex_hull) of the given points.
    #[cfg(feature = "2d")]
    pub fn convex_hull(points: Vec<Vector>) -> Option<Self> {
        let points = points.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        SharedShape::convex_hull(&points).map(Into::into)
    }

    /// Creates a collider with a [convex polyhedron](https://en.wikipedia.org/wiki/Convex_polytope) shape obtained after computing
    /// the [convex hull](https://en.wikipedia.org/wiki/Convex_hull) of the given points.
    #[cfg(feature = "3d")]
    pub fn convex_hull(points: Vec<Vector>) -> Option<Self> {
        let points = points.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        SharedShape::convex_hull(&points).map(Into::into)
    }

    /// Creates a collider with a heightfield shape.
    ///
    /// A 2D heightfield is a segment along the `X` axis, subdivided at regular intervals.
    ///
    /// `heights` is a vector indicating the altitude of each subdivision point, and `scale` is a scalar value
    /// indicating the length of each subdivided segment along the `X` axis.
    #[cfg(feature = "2d")]
    pub fn heightfield(heights: Vec<Scalar>, scale: Scalar) -> Self {
        SharedShape::heightfield(heights.into(), Vector::splat(scale).into()).into()
    }

    /// Creates a collider with a heightfield shape.
    ///
    /// A 3D heightfield is a rectangle on the `XZ` plane, subdivided in a grid pattern at regular intervals.
    ///
    /// `heights` is a matrix indicating the altitude of each subdivision point. The number of rows indicates
    /// the number of subdivisions along the `X` axis, while the number of columns indicates the number of
    /// subdivisions along the `Z` axis.
    ///
    /// `scale` indicates the size of each rectangle on the `XZ` plane.
    #[cfg(feature = "3d")]
    pub fn heightfield(heights: Vec<Vec<Scalar>>, scale: Vector) -> Self {
        let row_count = heights.len();
        let column_count = heights[0].len();
        let data: Vec<Scalar> = heights.into_iter().flatten().collect();

        assert_eq!(
            data.len(),
            row_count * column_count,
            "Each row in `heights` must have the same amount of points"
        );

        let heights = nalgebra::DMatrix::from_vec(row_count, column_count, data);
        SharedShape::heightfield(heights, scale.into()).into()
    }
}

#[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
type VerticesIndices = (Vec<nalgebra::Point3<Scalar>>, Vec<[u32; 3]>);

#[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
fn extract_mesh_vertices_indices(mesh: &Mesh) -> Option<VerticesIndices> {
    let vertices = mesh.attribute(Mesh::ATTRIBUTE_POSITION)?;
    let indices = mesh.indices()?;

    let vtx: Vec<_> = match vertices {
        VertexAttributeValues::Float32(vtx) => Some(
            vtx.chunks(3)
                .map(|v| [v[0] as Scalar, v[1] as Scalar, v[2] as Scalar].into())
                .collect(),
        ),
        VertexAttributeValues::Float32x3(vtx) => Some(
            vtx.iter()
                .map(|v| [v[0] as Scalar, v[1] as Scalar, v[2] as Scalar].into())
                .collect(),
        ),
        _ => None,
    }?;

    let idx = match indices {
        Indices::U16(idx) => idx
            .chunks_exact(3)
            .map(|i| [i[0] as u32, i[1] as u32, i[2] as u32])
            .collect(),
        Indices::U32(idx) => idx.chunks_exact(3).map(|i| [i[0], i[1], i[2]]).collect(),
    };

    Some((vtx, idx))
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
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn rigid body with its own collider
///     let body_id = commands.spawn((RigidBody::Dynamic, Collider::ball(0.5))).id();
///     
///     // Spawn another collider and add it as a child of the rigid body.
///     // The ColliderParent component will automatically be added with the value of body_id.
///     commands
///         .spawn((
///             Collider::ball(0.5),
///             TransformBundle::from_transform(Transform::from_translation(Vec3::X * 2.0)),
///         ))
///         .set_parent(body_id);
/// }
/// ```
#[derive(Reflect, Clone, Component, Debug, PartialEq, Eq)]
pub struct ColliderParent(pub(crate) Entity);

impl ColliderParent {
    /// Gets the `Entity` ID of the [`RigidBody`] that this [`Collider`] is attached to.
    pub const fn get(&self) -> Entity {
        self.0
    }
}

/// The positional offset of a collider relative to the rigid body it's attached to.
/// This is in the local space of the body, not the collider itself.
///
/// The offset is used for computing things like contact positions and a body's center of mass
/// without having to traverse deeply nested hierarchies.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq)]
pub(crate) struct ColliderOffset(pub(crate) Vector);

/// A component that marks a [`Collider`] as a sensor collider.
///
/// Sensor colliders send [collision events](Collider#collision-events) but don't cause a collision response.
/// This is often used to detect when something enters or leaves an area.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a static ball that generates collision events but doesn't cause a collision response
///     commands.spawn((RigidBody::Static, Collider::ball(0.5), Sensor));
/// }
/// ```
#[derive(Reflect, Clone, Component, Debug, Default, PartialEq, Eq)]
#[reflect(Component)]
pub struct Sensor;

/// The Axis-Aligned Bounding Box of a collider.
#[derive(Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
pub struct ColliderAabb(pub Aabb);

impl ColliderAabb {
    /// Creates a new collider from a given [`SharedShape`] with a default density of 1.0.
    pub fn from_shape(shape: &SharedShape) -> Self {
        Self(shape.compute_local_aabb())
    }
}

impl Default for ColliderAabb {
    fn default() -> Self {
        ColliderAabb(Aabb::new_invalid())
    }
}

/// Contains the entities that are colliding with an entity.
///
/// This component is automatically added for all entities with a [`Collider`].
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
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
#[reflect(Component)]
pub struct CollidingEntities(pub HashSet<Entity>);
