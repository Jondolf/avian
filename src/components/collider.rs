use std::fmt;

use crate::{prelude::*, utils::make_isometry};
use bevy::{log, prelude::*, utils::HashSet};
#[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
use bevy::{
    render::mesh::{Indices, VertexAttributeValues},
    utils::HashMap,
};
use collision::contact_query::UnsupportedShape;
use itertools::Either;
use parry::{
    bounding_volume::Aabb,
    shape::{RoundShape, SharedShape, TypedShape},
};

#[cfg(feature = "collider-from-mesh")]
pub use parry::transformation::vhacd::VHACDParameters;

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
    doc = "    commands.spawn((RigidBody::Static, Collider::cuboid(5.0, 0.5)));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    commands.spawn((RigidBody::Static, Collider::cuboid(5.0, 0.5, 5.0)));"
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
/// ## Querying and modifying contacts
///
/// The [`Collisions`] resource grants access to all collisions.
/// It can be used to get, modify, filter and add collisions.
///
/// See [`Collisions`] for more details.
///
/// ## Advanced usage
///
/// Internally, `Collider` uses the shapes provided by `parry`. If you want to create a collider
/// using these shapes, you can simply use `Collider::from(SharedShape::some_method())`.
///
/// To get a reference to the internal [`SharedShape`], you can use the [`get_shape`](#method.get_shape) method.
#[derive(Clone, Component)]
pub struct Collider {
    /// The raw unscaled collider shape.
    shape: SharedShape,
    /// The scaled version of the collider shape.
    ///
    /// If the scale is `Vector::ONE`, this will be `None` and `unscaled_shape`
    /// will be used instead.
    scaled_shape: SharedShape,
    /// The scale used for the collider shape.
    scale: Vector,
}

impl From<SharedShape> for Collider {
    fn from(value: SharedShape) -> Self {
        Self {
            shape: value.clone(),
            scaled_shape: value,
            scale: Vector::ONE,
        }
    }
}

impl Default for Collider {
    fn default() -> Self {
        #[cfg(feature = "2d")]
        {
            Self::cuboid(0.5, 0.5)
        }
        #[cfg(feature = "3d")]
        {
            Self::cuboid(0.5, 0.5, 0.5)
        }
    }
}

impl fmt::Debug for Collider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.shape_scaled().as_typed_shape() {
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
    /// Returns the raw unscaled shape of the collider.
    pub fn shape(&self) -> &SharedShape {
        &self.shape
    }

    /// Returns the shape of the collider with the scale from its `GlobalTransform` applied.
    pub fn shape_scaled(&self) -> &SharedShape {
        &self.scaled_shape
    }

    /// Returns the global scale of the collider.
    pub fn scale(&self) -> Vector {
        self.scale
    }

    /// Sets the unscaled shape of the collider. The collider's scale will be applied to this shape.
    pub fn set_shape(&mut self, shape: SharedShape) {
        if self.scale != Vector::ONE {
            self.scaled_shape = shape.clone();
        }
        self.shape = shape;
    }

    /// Set the scaling factor of this shape.
    ///
    /// If the scaling factor is non-uniform, and the scaled shape can’t be
    /// represented as a supported smooth shape (for example scalling a Ball
    /// with a non-uniform scale results in an ellipse which isn’t supported),
    /// the shape is approximated by a convex polygon/convex polyhedron using
    /// `num_subdivisions` subdivisions.
    pub fn set_scale(&mut self, scale: Vector, num_subdivisions: u32) {
        if scale == self.scale {
            return;
        }

        if scale == Vector::ONE {
            // Trivial case.
            self.scaled_shape = self.shape.clone();
            self.scale = Vector::ONE;
            return;
        }

        if let Ok(scaled) = scale_shape(&self.shape, scale, num_subdivisions) {
            self.scaled_shape = scaled;
            self.scale = scale;
        } else {
            log::error!("Failed to create convex hull for scaled collider.");
        }
    }

    /// Computes the [Axis-Aligned Bounding Box](ColliderAabb) of the collider.
    #[cfg(feature = "2d")]
    pub fn compute_aabb(&self, position: Vector, rotation: Scalar) -> ColliderAabb {
        ColliderAabb(self.shape_scaled().compute_aabb(&utils::make_isometry(
            position,
            Rotation::from_radians(rotation),
        )))
    }

    /// Computes the [Axis-Aligned Bounding Box](ColliderAabb) of the collider.
    #[cfg(feature = "3d")]
    pub fn compute_aabb(&self, position: Vector, rotation: Quaternion) -> ColliderAabb {
        ColliderAabb(
            self.shape_scaled()
                .compute_aabb(&utils::make_isometry(position, Rotation(rotation))),
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
                    utils::make_isometry(*p.into(), r.into()),
                    c.into().shape_scaled().clone(),
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

    /// Creates a collider shape with a compound shape obtained from the decomposition of a given polyline
    /// defined by its vertex and index buffers. The given [`VHACDParameters`] are used for configuring
    /// the decomposition process.
    #[cfg(feature = "2d")]
    pub fn convex_decomposition_with_params(
        vertices: Vec<Vector>,
        indices: Vec<[u32; 2]>,
        params: &VHACDParameters,
    ) -> Self {
        let vertices = vertices.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        SharedShape::convex_decomposition_with_params(&vertices, &indices, params).into()
    }

    /// Creates a collider shape with a compound shape obtained from the decomposition of a given trimesh
    /// defined by its vertex and index buffers. The given [`VHACDParameters`] are used for configuring
    /// the decomposition process.
    #[cfg(feature = "3d")]
    pub fn convex_decomposition_with_params(
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
        params: &VHACDParameters,
    ) -> Self {
        let vertices = vertices.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        SharedShape::convex_decomposition_with_params(&vertices, &indices, params).into()
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

    /// Creates a collider with a given `collider_type` from a `mesh`.
    ///
    /// ## Example
    ///
    /// ```rust
    /// use bevy::prelude::*;
    /// use bevy_xpbd_3d::prelude::*;
    ///
    /// // Spawn a cube with a convex hull collider generated from the mesh
    /// fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    ///     let mesh = Mesh::from(shape::Cube { size: 1.0 });
    ///     commands.spawn((
    ///         Collider::from_mesh(&mesh, &ComputedCollider::ConvexHull),
    ///         PbrBundle {
    ///             mesh: meshes.add(mesh),
    ///             ..default(),
    ///         },
    ///     ));
    /// }
    /// ```
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn from_mesh(mesh: &Mesh, collider_type: &ComputedCollider) -> Option<Self> {
        let Some((vertices, indices)) = extract_mesh_vertices_indices(mesh) else {
            return None;
        };

        match collider_type {
            ComputedCollider::TriMesh => Some(
                SharedShape::trimesh_with_flags(
                    vertices,
                    indices,
                    TriMeshFlags::MERGE_DUPLICATE_VERTICES,
                )
                .into(),
            ),
            ComputedCollider::ConvexHull => {
                SharedShape::convex_hull(&vertices).map(|shape| shape.into())
            }
            ComputedCollider::ConvexDecomposition(params) => Some(
                SharedShape::convex_decomposition_with_params(&vertices, &indices, params).into(),
            ),
        }
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

fn scale_shape(
    shape: &SharedShape,
    scale: Vector,
    num_subdivisions: u32,
) -> Result<SharedShape, UnsupportedShape> {
    match shape.as_typed_shape() {
        TypedShape::Cuboid(s) => Ok(SharedShape::new(s.scaled(&scale.into()))),
        TypedShape::RoundCuboid(s) => Ok(SharedShape::new(RoundShape {
            border_radius: s.border_radius,
            inner_shape: s.inner_shape.scaled(&scale.into()),
        })),
        TypedShape::Capsule(c) => match c.scaled(&scale.into(), num_subdivisions) {
            None => {
                log::error!("Failed to apply scale {} to Capsule shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(Either::Left(b)) => Ok(SharedShape::new(b)),
            Some(Either::Right(b)) => Ok(SharedShape::new(b)),
        },
        TypedShape::Ball(b) => match b.scaled(&scale.into(), num_subdivisions) {
            None => {
                log::error!("Failed to apply scale {} to Ball shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(Either::Left(b)) => Ok(SharedShape::new(b)),
            Some(Either::Right(b)) => Ok(SharedShape::new(b)),
        },
        TypedShape::Segment(s) => Ok(SharedShape::new(s.scaled(&scale.into()))),
        TypedShape::Triangle(t) => Ok(SharedShape::new(t.scaled(&scale.into()))),
        TypedShape::RoundTriangle(t) => Ok(SharedShape::new(RoundShape {
            border_radius: t.border_radius,
            inner_shape: t.inner_shape.scaled(&scale.into()),
        })),
        TypedShape::TriMesh(t) => Ok(SharedShape::new(t.clone().scaled(&scale.into()))),
        TypedShape::Polyline(p) => Ok(SharedShape::new(p.clone().scaled(&scale.into()))),
        TypedShape::HalfSpace(h) => match h.scaled(&scale.into()) {
            None => {
                log::error!("Failed to apply scale {} to HalfSpace shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(scaled) => Ok(SharedShape::new(scaled)),
        },
        TypedShape::HeightField(h) => Ok(SharedShape::new(h.clone().scaled(&scale.into()))),
        #[cfg(feature = "dim2")]
        TypedShape::ConvexPolygon(cp) => match cp.clone().scaled(&scale.into()) {
            None => {
                log::error!("Failed to apply scale {} to ConvexPolygon shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(scaled) => Ok(SharedShape::new(scaled)),
        },
        #[cfg(feature = "dim2")]
        TypedShape::RoundConvexPolygon(cp) => match cp.inner_shape.clone().scaled(&scale.into()) {
            None => {
                log::error!(
                    "Failed to apply scale {} to RoundConvexPolygon shape.",
                    scale
                );
                Ok(SharedShape::ball(0.0))
            }
            Some(scaled) => Ok(SharedShape::new(RoundShape {
                border_radius: cp.border_radius,
                inner_shape: scaled,
            })),
        },
        #[cfg(feature = "dim3")]
        TypedShape::ConvexPolyhedron(cp) => match cp.clone().scaled(&scale.into()) {
            None => {
                log::error!("Failed to apply scale {} to ConvexPolyhedron shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(scaled) => Ok(SharedShape::new(scaled)),
        },
        #[cfg(feature = "dim3")]
        TypedShape::RoundConvexPolyhedron(cp) => {
            match cp.clone().inner_shape.scaled(&scale.into()) {
                None => {
                    log::error!(
                        "Failed to apply scale {} to RoundConvexPolyhedron shape.",
                        scale
                    );
                    Ok(SharedShape::ball(0.0))
                }
                Some(scaled) => Ok(SharedShape::new(RoundShape {
                    border_radius: cp.border_radius,
                    inner_shape: scaled,
                })),
            }
        }
        #[cfg(feature = "dim3")]
        TypedShape::Cylinder(c) => match c.scaled(&scale.into(), num_subdivisions) {
            None => {
                log::error!("Failed to apply scale {} to Cylinder shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(Either::Left(b)) => Ok(SharedShape::new(b)),
            Some(Either::Right(b)) => Ok(SharedShape::new(b)),
        },
        #[cfg(feature = "dim3")]
        TypedShape::RoundCylinder(c) => {
            match c.inner_shape.scaled(&scale.into(), num_subdivisions) {
                None => {
                    log::error!("Failed to apply scale {} to RoundCylinder shape.", scale);
                    Ok(SharedShape::ball(0.0))
                }
                Some(Either::Left(scaled)) => Ok(SharedShape::new(RoundShape {
                    border_radius: c.border_radius,
                    inner_shape: scaled,
                })),
                Some(Either::Right(scaled)) => Ok(SharedShape::new(RoundShape {
                    border_radius: c.border_radius,
                    inner_shape: scaled,
                })),
            }
        }
        #[cfg(feature = "dim3")]
        TypedShape::Cone(c) => match c.scaled(&scale.into(), num_subdivisions) {
            None => {
                log::error!("Failed to apply scale {} to Cone shape.", scale);
                SharedShape::ball(0.0)
            }
            Some(Either::Left(b)) => SharedShape::new(b),
            Some(Either::Right(b)) => SharedShape::new(b),
        },
        #[cfg(feature = "dim3")]
        TypedShape::RoundCone(c) => match c.inner_shape.scaled(&scale.into(), num_subdivisions) {
            None => {
                log::error!("Failed to apply scale {} to RoundCone shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(Either::Left(scaled)) => Ok(SharedShape::new(RoundShape {
                border_radius: c.border_radius,
                inner_shape: scaled,
            })),
            Some(Either::Right(scaled)) => Ok(SharedShape::new(RoundShape {
                border_radius: c.border_radius,
                inner_shape: scaled,
            })),
        },
        TypedShape::Compound(c) => {
            let mut scaled = Vec::with_capacity(c.shapes().len());

            for (iso, shape) in c.shapes() {
                scaled.push((
                    #[cfg(feature = "2d")]
                    make_isometry(
                        Vector::from(iso.translation) * scale,
                        Rotation::from_radians(iso.rotation.angle()),
                    ),
                    #[cfg(feature = "3d")]
                    make_isometry(
                        Vector::from(iso.translation) * scale,
                        Quaternion::from(iso.rotation),
                    ),
                    scale_shape(shape, scale, num_subdivisions)?,
                ));
            }
            Ok(SharedShape::compound(scaled))
        }
        _ => Err(parry::query::Unsupported),
    }
}

/// A component that will generate colliders for children with meshes
/// once the scene has been loaded.
///
/// The type of the generated collider can be specified using [`ComputedCollider`].
///
/// ## Example
///
/// ```rust
/// use bevy::prelude::*;
/// use bevy_xpbd_3d::prelude::*;
///
/// fn setup(mut commands: Commands, mut assets: ResMut<AssetServer>) {
///     let scene = SceneBundle {
///         scene: assets.load("my_model.gltf#Scene0"),
///         ..default()
///     };
///
///     // Spawn the scene and automatically generate triangle mesh colliders.
///     commands.spawn((
///         scene.clone(),
///         AsyncSceneCollider::new(Some(ComputedCollider::TriMesh)),
///     ));
///
///     // In addition to the default collider type, you can specify collider types
///     // for specific meshes by name.
///     commands.spawn((
///         scene.clone(),
///         AsyncSceneCollider::new(Some(ComputedCollider::TriMesh))
///             .with_named_shape("Tree".to_string(), Some(ComputedCollider::ConvexHull)),
///     ));
///
///     // Using `None` skips collider generation for meshes.
///     commands.spawn((
///         scene.clone(),
///         // Only create collider for "Tree" mesh
///         AsyncSceneCollider::new(None)
///             .with_named_shape("Tree".to_string(), Some(ComputedCollider::ConvexHull)),
///     ));
/// }
/// ```
#[cfg(all(feature = "3d", feature = "async-collider"))]
#[derive(Component, Debug, Default, Clone)]
pub struct AsyncSceneCollider {
    /// The default collider type used for each mesh that isn't included in [`named_shapes`].
    /// If `None`, all meshes except the ones in [`named_shapes`] will be skipped.
    pub default_shape: Option<ComputedCollider>,
    /// Specifies collider types for meshes by name. If a shape is `None`, it will be skipped.
    /// For the meshes not found in this `HashMap`, [`default_shape`] is used instead.
    pub named_shapes: HashMap<String, Option<ComputedCollider>>,
}

#[cfg(all(feature = "3d", feature = "async-collider"))]
impl AsyncSceneCollider {
    /// Creates a new [`AsyncSceneCollider`] with the default collider type used for
    /// meshes set to the given `default_shape`.
    ///
    /// If the given collider type is `None`, all meshes except the ones in [`named_shapes`]
    /// will be skipped. You can add named shapes using [`with_named_shape`](#method.with_named_shape).
    pub fn new(default_shape: Option<ComputedCollider>) -> Self {
        Self {
            default_shape,
            named_shapes: default(),
        }
    }

    /// Specifies the collider type used for a mesh with the given `name`.
    /// If it is `None`, the mesh won't have a collider.
    pub fn with_named_shape(mut self, name: String, shape: Option<ComputedCollider>) -> Self {
        self.named_shapes.insert(name, shape);
        self
    }
}

/// Determines how a [`Collider`] is generated from a `Mesh`.
///
/// Colliders can be created from meshes with the following components and methods:
///
/// - [`AsyncSceneCollider`] (requires `3d` and `async-collider` features)
/// - [`Collider::from_mesh`]
#[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
#[derive(Component, Clone, Debug, Default, PartialEq)]
pub enum ComputedCollider {
    /// A triangle mesh.
    #[default]
    TriMesh,
    /// A convex hull.
    ConvexHull,
    /// A compound shape obtained from a decomposition into convex parts using the specified
    /// [`VHACDParameters`].
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
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// # use bevy_xpbd_3d::prelude::*;
/// #
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
pub struct ColliderParent(pub(crate) Entity);

impl ColliderParent {
    /// Gets the `Entity` ID of the [`RigidBody`] that this [`Collider`] is attached to.
    pub const fn get(&self) -> Entity {
        self.0
    }
}

/// The transform of a collider relative to the rigid body it's attached to.
/// This is in the local space of the body, not the collider itself.
///
/// This is used for computing things like contact positions and a body's center of mass
/// without having to traverse deeply nested hierarchies.
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
pub(crate) struct ColliderTransform {
    pub translation: Vector,
    pub rotation: Rotation,
    pub scale: Vector,
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
/// Sensor colliders send [collision events](Collider#collision-events) and register intersections,
/// but allow other bodies to pass through them. This is often used to detect when something enters
/// or leaves an area or is intersecting some shape.
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
///     // Spawn a static body with a sensor collider.
///     // Other bodies will pass through, but it will still send collision events.
///     commands.spawn((RigidBody::Static, Collider::ball(0.5), Sensor));
/// }
/// ```
#[doc(alias = "Trigger")]
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
