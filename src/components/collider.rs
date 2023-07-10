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

/// A collider used for collision detection.
///
/// By default, colliders generate [collision events](#collision-events) and cause a collision response for
/// [rigid bodies](RigidBody). If you only want collision events, you can add a [`Sensor`] component.
///
/// ## Creation
///
/// `Collider` has tons of methods for creating colliders of various shapes.
/// For example, to add a ball collider to a [rigid body](RigidBody), use [`Collider::ball`](#method.ball):
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((RigidBody::Dynamic, Collider::ball(0.5)));
/// }
/// ```
///
/// In addition, Bevy XPBD will automatically add some other components, like the following:
///
/// - [`ColliderAabb`]
/// - [`CollidingEntities`]
/// - [`ColliderMassProperties`]
///
/// ## Collision layers
///
/// You can use collsion layers to configure which entities can collide with each other.
///
/// See [`CollisionLayers`] for more information and examples.
///
/// ## Collision events
///
/// There are currently three different collision events: [`Collision`], [`CollisionStarted`] and [`CollisionEnded`].
/// You can listen to these events as you normally would.
///
/// For example, you could read [contacts](Contact) like this:
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn my_system(mut collision_event_reader: EventReader<Collision>) {
///     for Collision(contact) in collision_event_reader.iter() {
///         println!("{:?} and {:?} are colliding", contact.entity1, contact.entity2);
///     }
/// }
/// ```
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
    /// Gets the raw shape of the collider. The shapes are provided by [`parry`].
    pub fn get_shape(&self) -> &SharedShape {
        &self.0
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

    /// Creates a collider with a triangle mesh shape built from a given Bevy `Mesh`.
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn trimesh_from_bevy_mesh(mesh: &Mesh) -> Option<Self> {
        use parry::shape::TriMeshFlags;

        let vertices_indices = extract_mesh_vertices_indices(mesh);
        vertices_indices.map(|(v, i)| {
            SharedShape::trimesh_with_flags(v, i, TriMeshFlags::MERGE_DUPLICATE_VERTICES).into()
        })
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
pub struct Sensor;

/// The Axis-Aligned Bounding Box of a collider.
#[derive(Clone, Copy, Component, Deref, DerefMut, PartialEq)]
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
