use std::fmt;

use crate::{prelude::*, utils::make_isometry};
#[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
use bevy::render::mesh::{Indices, VertexAttributeValues};
#[cfg(all(feature = "3d", feature = "async-collider"))]
use bevy::utils::HashMap;
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    log,
    prelude::*,
    utils::HashSet,
};
use collision::contact_query::UnsupportedShape;
use itertools::Either;
use parry::{
    bounding_volume::Aabb,
    shape::{RoundShape, SharedShape, TypedShape},
};

/// Parameters controlling the VHACD convex decomposition algorithm.
///
/// See <https://github.com/Unity-Technologies/VHACD#parameters> for details.
pub type VHACDParameters = parry::transformation::vhacd::VHACDParameters;

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
#[cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
/// #
/// # fn setup(mut commands: Commands) {
/// // Create a ball collider with a given radius
/// commands.spawn(Collider::ball(0.5));
/// // Create a capsule collider with a given height and radius
/// commands.spawn(Collider::capsule(2.0, 0.5));
/// # }
/// ```
///
/// Colliders on their own only detect contacts and generate
/// [collision events](ContactReportingPlugin#collision-events).
/// To make colliders apply contact forces, they have to be attached
/// to [rigid bodies](RigidBody):
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
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
/// - [`ColliderParent`]
/// - [`ColliderAabb`]
/// - [`CollidingEntities`]
/// - [`ColliderDensity`]
/// - [`ColliderMassProperties`]
///
#[cfg_attr(
    feature = "3d",
    doc = "Colliders can also be generated automatically from meshes and scenes. See [`AsyncCollider`] and [`AsyncSceneCollider`]."
)]
///
/// ### Multiple colliders
///
/// It can often be useful to attach multiple colliders to the same rigid body.
///
/// This can be done in two ways. Either use [`Collider::compound`] to have one collider that consists of many
/// shapes, or for more control, spawn several collider entities as the children of a rigid body:
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
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
/// and other configuration options, and they send separate [collision events](ContactReportingPlugin#collision-events).
///
/// ## See more
///
/// - [Rigid bodies](RigidBody)
/// - [Density](ColliderDensity)
/// - [Friction] and [restitution](Restitution) (bounciness)
/// - [Collision layers](CollisionLayers)
/// - [Sensors](Sensor)
#[cfg_attr(
    feature = "3d",
    doc = "- Creating colliders from meshes with [`AsyncCollider`] and [`AsyncSceneCollider`]"
)]
/// - [Get colliding entities](CollidingEntities)
/// - [Collision events](ContactReportingPlugin#collision-events)
/// - [Accessing, filtering and modifying collisions](Collisions)
/// - [Manual contact queries](contact_query)
///
/// ## Advanced usage
///
/// Internally, `Collider` uses the shapes provided by `parry`. If you want to create a collider
/// using these shapes, you can simply use `Collider::from(SharedShape::some_method())`.
///
/// To get a reference to the internal [`SharedShape`], you can use the [`Collider::shape()`]
/// or [`Collider::shape_scaled()`] methods.
#[derive(Clone, Component)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Collider {
    /// The raw unscaled collider shape.
    shape: SharedShape,
    /// The scaled version of the collider shape.
    ///
    /// If the scale is `Vector::ONE`, this will be `None` and `unscaled_shape`
    /// will be used instead.
    scaled_shape: SharedShape,
    /// The global scale used for the collider shape.
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
        self.shape = shape;

        // TODO: The number of subdivisions probably shouldn't be hard-coded
        if let Ok(scaled) = scale_shape(&self.shape, self.scale, 10) {
            self.scaled_shape = scaled;
        } else {
            log::error!("Failed to create convex hull for scaled collider.");
        }
    }

    /// Set the global scaling factor of this shape.
    ///
    /// If the scaling factor is not uniform, and the scaled shape can’t be
    /// represented as a supported shape, the shape is approximated as
    /// a convex polygon or polyhedron using `num_subdivisions`.
    ///
    /// For example, if a ball was scaled to an ellipse, the new shape would be approximated.
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

    /// Computes the collider's mass properties based on its shape and a given density.
    pub fn mass_properties(&self, density: Scalar) -> ColliderMassProperties {
        ColliderMassProperties::new(self, density)
    }

    /// Projects the given `point` onto `self` transformed by `translation` and `rotation`.
    /// The returned tuple contains the projected point and whether it is inside the collider.
    ///
    /// If `solid` is true and the given `point` is inside of the collider, the projection will be at the point.
    /// Otherwise, the collider will be treated as hollow, and the projection will be at the collider's boundary.
    pub fn project_point(
        &self,
        translation: impl Into<Position>,
        rotation: impl Into<Rotation>,
        point: Vector,
        solid: bool,
    ) -> (Vector, bool) {
        let projection = self.shape_scaled().project_point(
            &utils::make_isometry(translation, rotation),
            &point.into(),
            solid,
        );
        (projection.point.into(), projection.is_inside)
    }

    /// Computes the minimum distance between the given `point` and `self` transformed by `translation` and `rotation`.
    ///
    /// If `solid` is true and the given `point` is inside of the collider, the returned distance will be `0.0`.
    /// Otherwise, the collider will be treated as hollow, and the distance will be the distance
    /// to the collider's boundary.
    pub fn distance_to_point(
        &self,
        translation: impl Into<Position>,
        rotation: impl Into<Rotation>,
        point: Vector,
        solid: bool,
    ) -> Scalar {
        self.shape_scaled().distance_to_point(
            &utils::make_isometry(translation, rotation),
            &point.into(),
            solid,
        )
    }

    /// Tests whether the given `point` is inside of `self` transformed by `translation` and `rotation`.
    pub fn contains_point(
        &self,
        translation: impl Into<Position>,
        rotation: impl Into<Rotation>,
        point: Vector,
    ) -> bool {
        self.shape_scaled()
            .contains_point(&utils::make_isometry(translation, rotation), &point.into())
    }

    /// Computes the time of impact and normal between the given ray and `self`
    /// transformed by `translation` and `rotation`.
    ///
    /// The returned tuple is in the format `(time_of_impact, normal)`.
    ///
    /// ## Arguments
    ///
    /// - `ray_origin`: Where the ray is cast from.
    /// - `ray_direction`: What direction the ray is cast in.
    /// - `max_time_of_impact`: The maximum distance that the ray can travel.
    /// - `solid`: If true and the ray origin is inside of a collider, the hit point will be the ray origin itself.
    /// Otherwise, the collider will be treated as hollow, and the hit point will be at the collider's boundary.
    pub fn cast_ray(
        &self,
        translation: impl Into<Position>,
        rotation: impl Into<Rotation>,
        ray_origin: Vector,
        ray_direction: Vector,
        max_time_of_impact: Scalar,
        solid: bool,
    ) -> Option<(Scalar, Vector)> {
        let hit = self.shape_scaled().cast_ray_and_get_normal(
            &utils::make_isometry(translation, rotation),
            &parry::query::Ray::new(ray_origin.into(), ray_direction.into()),
            max_time_of_impact,
            solid,
        );
        hit.map(|hit| (hit.toi, hit.normal.into()))
    }

    /// Tests whether the given ray intersects `self` transformed by `translation` and `rotation`.
    ///
    /// ## Arguments
    ///
    /// - `ray_origin`: Where the ray is cast from.
    /// - `ray_direction`: What direction the ray is cast in.
    /// - `max_time_of_impact`: The maximum distance that the ray can travel.
    pub fn intersects_ray(
        &self,
        translation: impl Into<Position>,
        rotation: impl Into<Rotation>,
        ray_origin: Vector,
        ray_direction: Vector,
        max_time_of_impact: Scalar,
    ) -> bool {
        self.shape_scaled().intersects_ray(
            &utils::make_isometry(translation, rotation),
            &parry::query::Ray::new(ray_origin.into(), ray_direction.into()),
            max_time_of_impact,
        )
    }

    /// Creates a collider with a compound shape defined by a given vector of colliders with a position and a rotation.
    ///
    /// Especially for dynamic rigid bodies, compound shape colliders should be preferred over triangle meshes and polylines,
    /// because convex shapes typically provide more reliable results.
    ///
    /// If you want to create a compound shape from a 3D triangle mesh or 2D polyline, consider using the
    /// [`Collider::convex_decomposition`] method.
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

    /// Creates a collider with a cuboid shape defined by its extents and rounded corners.
    #[cfg(feature = "2d")]
    pub fn round_cuboid(x_length: Scalar, y_length: Scalar, border_radius: Scalar) -> Self {
        SharedShape::round_cuboid(x_length * 0.5, y_length * 0.5, border_radius).into()
    }

    /// Creates a collider with a cuboid shape defined by its extents and rounded corners.
    #[cfg(feature = "3d")]
    pub fn round_cuboid(
        x_length: Scalar,
        y_length: Scalar,
        z_length: Scalar,
        border_radius: Scalar,
    ) -> Self {
        SharedShape::round_cuboid(
            x_length * 0.5,
            y_length * 0.5,
            z_length * 0.5,
            border_radius,
        )
        .into()
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
    pub fn trimesh_with_config(
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
    pub fn convex_decomposition_with_config(
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
    pub fn convex_decomposition_with_config(
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

    /// Creates a collider with a triangle mesh shape from a `Mesh`.
    ///
    /// ## Example
    ///
    /// ```
    /// use bevy::prelude::*;
    /// use bevy_xpbd_3d::prelude::*;
    ///
    /// fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    ///     let mesh = Mesh::from(shape::Cube { size: 1.0 });
    ///     commands.spawn((
    ///         Collider::trimesh_from_mesh(&mesh).unwrap(),
    ///         PbrBundle {
    ///             mesh: meshes.add(mesh),
    ///             ..default()
    ///         },
    ///     ));
    /// }
    /// ```
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn trimesh_from_mesh(mesh: &Mesh) -> Option<Self> {
        extract_mesh_vertices_indices(mesh).map(|(vertices, indices)| {
            SharedShape::trimesh_with_flags(
                vertices,
                indices,
                TriMeshFlags::MERGE_DUPLICATE_VERTICES,
            )
            .into()
        })
    }

    /// Creates a collider with a triangle mesh shape from a `Mesh` using the given [`TriMeshFlags`]
    /// for controlling the preprocessing.
    ///
    /// ## Example
    ///
    /// ```
    /// use bevy::prelude::*;
    /// use bevy_xpbd_3d::prelude::*;
    ///
    /// fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    ///     let mesh = Mesh::from(shape::Cube { size: 1.0 });
    ///     commands.spawn((
    ///         Collider::trimesh_from_mesh_with_config(&mesh, TriMeshFlags::all()).unwrap(),
    ///         PbrBundle {
    ///             mesh: meshes.add(mesh),
    ///             ..default()
    ///         },
    ///     ));
    /// }
    /// ```
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn trimesh_from_mesh_with_config(mesh: &Mesh, flags: TriMeshFlags) -> Option<Self> {
        extract_mesh_vertices_indices(mesh).map(|(vertices, indices)| {
            SharedShape::trimesh_with_flags(vertices, indices, flags).into()
        })
    }

    /// Creates a collider with a convex polygon shape obtained from the convex hull of a `Mesh`.
    ///
    /// ## Example
    ///
    /// ```
    /// use bevy::prelude::*;
    /// use bevy_xpbd_3d::prelude::*;
    ///
    /// fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    ///     let mesh = Mesh::from(shape::Cube { size: 1.0 });
    ///     commands.spawn((
    ///         Collider::convex_hull_from_mesh(&mesh).unwrap(),
    ///         PbrBundle {
    ///             mesh: meshes.add(mesh),
    ///             ..default()
    ///         },
    ///     ));
    /// }
    /// ```
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn convex_hull_from_mesh(mesh: &Mesh) -> Option<Self> {
        extract_mesh_vertices_indices(mesh)
            .and_then(|(vertices, _)| SharedShape::convex_hull(&vertices).map(|shape| shape.into()))
    }

    /// Creates a compound shape obtained from the decomposition of a `Mesh`.
    ///
    /// ## Example
    ///
    /// ```
    /// use bevy::prelude::*;
    /// use bevy_xpbd_3d::prelude::*;
    ///
    /// fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    ///     let mesh = Mesh::from(shape::Cube { size: 1.0 });
    ///     commands.spawn((
    ///         Collider::convex_decomposition_from_mesh(&mesh).unwrap(),
    ///         PbrBundle {
    ///             mesh: meshes.add(mesh),
    ///             ..default()
    ///         },
    ///     ));
    /// }
    /// ```
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn convex_decomposition_from_mesh(mesh: &Mesh) -> Option<Self> {
        extract_mesh_vertices_indices(mesh).map(|(vertices, indices)| {
            SharedShape::convex_decomposition(&vertices, &indices).into()
        })
    }

    /// Creates a compound shape obtained from the decomposition of a `Mesh`
    /// with the given [`VHACDParameters`] passed to the decomposition algorithm.
    ///
    /// ## Example
    ///
    /// ```
    /// use bevy::prelude::*;
    /// use bevy_xpbd_3d::prelude::*;
    ///
    /// fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    ///     let mesh = Mesh::from(shape::Cube { size: 1.0 });
    ///     let config = VHACDParameters {
    ///         convex_hull_approximation: false,
    ///         ..default()
    ///     };
    ///     commands.spawn((
    ///         Collider::convex_decomposition_from_mesh_with_config(&mesh, &config).unwrap(),
    ///         PbrBundle {
    ///             mesh: meshes.add(mesh),
    ///             ..default()
    ///         },
    ///     ));
    /// }
    /// ```
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    pub fn convex_decomposition_from_mesh_with_config(
        mesh: &Mesh,
        parameters: &VHACDParameters,
    ) -> Option<Self> {
        extract_mesh_vertices_indices(mesh).map(|(vertices, indices)| {
            SharedShape::convex_decomposition_with_params(&vertices, &indices, parameters).into()
        })
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
        #[cfg(feature = "2d")]
        TypedShape::ConvexPolygon(cp) => match cp.clone().scaled(&scale.into()) {
            None => {
                log::error!("Failed to apply scale {} to ConvexPolygon shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(scaled) => Ok(SharedShape::new(scaled)),
        },
        #[cfg(feature = "2d")]
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
        #[cfg(feature = "3d")]
        TypedShape::ConvexPolyhedron(cp) => match cp.clone().scaled(&scale.into()) {
            None => {
                log::error!("Failed to apply scale {} to ConvexPolyhedron shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(scaled) => Ok(SharedShape::new(scaled)),
        },
        #[cfg(feature = "3d")]
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
        #[cfg(feature = "3d")]
        TypedShape::Cylinder(c) => match c.scaled(&scale.into(), num_subdivisions) {
            None => {
                log::error!("Failed to apply scale {} to Cylinder shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(Either::Left(b)) => Ok(SharedShape::new(b)),
            Some(Either::Right(b)) => Ok(SharedShape::new(b)),
        },
        #[cfg(feature = "3d")]
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
        #[cfg(feature = "3d")]
        TypedShape::Cone(c) => match c.scaled(&scale.into(), num_subdivisions) {
            None => {
                log::error!("Failed to apply scale {} to Cone shape.", scale);
                Ok(SharedShape::ball(0.0))
            }
            Some(Either::Left(b)) => Ok(SharedShape::new(b)),
            Some(Either::Right(b)) => Ok(SharedShape::new(b)),
        },
        #[cfg(feature = "3d")]
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
///             mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
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
    TriMeshWithFlags(TriMeshFlags),
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
#[derive(Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
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
