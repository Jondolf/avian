#![allow(clippy::unnecessary_cast)]

use crate::{prelude::*, utils::make_isometry};
#[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
use bevy::render::mesh::{Indices, VertexAttributeValues};
use bevy::{log, prelude::*};
use collision::contact_query::UnsupportedShape;
use itertools::Either;
use parry::shape::{RoundShape, SharedShape, TypedShape};

#[cfg(feature = "2d")]
mod primitives2d;
#[cfg(feature = "3d")]
mod primitives3d;

#[cfg(feature = "2d")]
pub(crate) use primitives2d::{EllipseWrapper, RegularPolygonWrapper};

impl<T: IntoCollider<Collider>> From<T> for Collider {
    fn from(value: T) -> Self {
        value.collider()
    }
}

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
#[cfg_attr(feature = "2d", doc = "commands.spawn(Collider::circle(0.5));")]
#[cfg_attr(feature = "3d", doc = "commands.spawn(Collider::sphere(0.5));")]
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
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.5),")]
///         TransformBundle::from_transform(Transform::from_xyz(0.0, 2.0, 0.0)),
///     ));
#[cfg_attr(
    feature = "2d",
    doc = "    commands.spawn((RigidBody::Static, Collider::rectangle(5.0, 0.5)));"
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
#[cfg_attr(
    feature = "2d",
    doc = "        .spawn((RigidBody::Dynamic, Collider::circle(0.5)))"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        .spawn((RigidBody::Dynamic, Collider::sphere(0.5)))"
)]
///         .with_children(|children| {
///             // Spawn the child colliders positioned relative to the rigid body
#[cfg_attr(
    feature = "2d",
    doc = "            children.spawn((
                Collider::circle(0.5),
                TransformBundle::from_transform(Transform::from_xyz(2.0, 0.0, 0.0)),
            ));
            children.spawn((
                Collider::circle(0.5),
                TransformBundle::from_transform(Transform::from_xyz(-2.0, 0.0, 0.0)),
            ));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "            children.spawn((
                Collider::sphere(0.5),
                TransformBundle::from_transform(Transform::from_xyz(2.0, 0.0, 0.0)),
            ));
            children.spawn((
                Collider::sphere(0.5),
                TransformBundle::from_transform(Transform::from_xyz(-2.0, 0.0, 0.0)),
            ));"
)]
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
            Self::rectangle(0.5, 0.5)
        }
        #[cfg(feature = "3d")]
        {
            Self::cuboid(0.5, 0.5, 0.5)
        }
    }
}

impl std::fmt::Debug for Collider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
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

impl AnyCollider for Collider {
    fn aabb(&self, position: Vector, rotation: impl Into<Rotation>) -> ColliderAabb {
        let aabb = self
            .shape_scaled()
            .compute_aabb(&utils::make_isometry(position, rotation));
        ColliderAabb {
            min: aabb.mins.into(),
            max: aabb.maxs.into(),
        }
    }

    fn mass_properties(&self, density: Scalar) -> ColliderMassProperties {
        let props = self.shape_scaled().mass_properties(density);

        ColliderMassProperties {
            mass: Mass(props.mass()),
            inverse_mass: InverseMass(props.inv_mass),

            #[cfg(feature = "2d")]
            inertia: Inertia(props.principal_inertia()),
            #[cfg(feature = "3d")]
            inertia: Inertia(props.reconstruct_inertia_matrix().into()),

            #[cfg(feature = "2d")]
            inverse_inertia: InverseInertia(1.0 / props.principal_inertia()),
            #[cfg(feature = "3d")]
            inverse_inertia: InverseInertia(props.reconstruct_inverse_inertia_matrix().into()),

            center_of_mass: CenterOfMass(props.local_com.into()),
        }
    }

    fn contact_manifolds(
        &self,
        other: &Self,
        position1: Vector,
        rotation1: impl Into<Rotation>,
        position2: Vector,
        rotation2: impl Into<Rotation>,
        prediction_distance: Scalar,
    ) -> Vec<ContactManifold> {
        contact_query::contact_manifolds(
            self,
            position1,
            rotation1,
            other,
            position2,
            rotation2,
            prediction_distance,
        )
    }
}

impl ScalableCollider for Collider {
    fn scale(&self) -> Vector {
        self.scale()
    }

    fn set_scale(&mut self, scale: Vector, detail: u32) {
        self.set_scale(scale, detail)
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

    /// Returns the global scale of the collider.
    pub fn scale(&self) -> Vector {
        self.scale
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

    /// Creates a collider with a circle shape defined by its radius.
    #[cfg(feature = "2d")]
    pub fn circle(radius: Scalar) -> Self {
        SharedShape::ball(radius).into()
    }

    /// Creates a collider with a sphere shape defined by its radius.
    #[cfg(feature = "3d")]
    pub fn sphere(radius: Scalar) -> Self {
        SharedShape::ball(radius).into()
    }

    /// Creates a collider with a ball shape defined by its radius.
    #[cfg_attr(
        feature = "2d",
        deprecated(since = "0.4.0", note = "please use `Collider::circle` instead")
    )]
    #[cfg_attr(
        feature = "3d",
        deprecated(since = "0.4.0", note = "please use `Collider::sphere` instead")
    )]
    pub fn ball(radius: Scalar) -> Self {
        SharedShape::ball(radius).into()
    }

    /// Creates a collider with an ellipse shape defined by a half-width and half-height.
    #[cfg(feature = "2d")]
    pub fn ellipse(half_width: Scalar, half_height: Scalar) -> Self {
        SharedShape::new(EllipseWrapper(Ellipse::new(
            half_width as f32,
            half_height as f32,
        )))
        .into()
    }

    /// Creates a collider with a rectangle shape defined by its extents.
    #[cfg(feature = "2d")]
    pub fn rectangle(x_length: Scalar, y_length: Scalar) -> Self {
        SharedShape::cuboid(x_length * 0.5, y_length * 0.5).into()
    }

    /// Creates a collider with a ball shape defined by its radius.
    #[cfg(feature = "2d")]
    #[deprecated(since = "0.4.0", note = "please use `Collider::rectangle` instead")]
    pub fn cuboid(x_length: Scalar, y_length: Scalar) -> Self {
        SharedShape::cuboid(x_length * 0.5, y_length * 0.5).into()
    }

    /// Creates a collider with a cuboid shape defined by its extents.
    #[cfg(feature = "3d")]
    pub fn cuboid(x_length: Scalar, y_length: Scalar, z_length: Scalar) -> Self {
        SharedShape::cuboid(x_length * 0.5, y_length * 0.5, z_length * 0.5).into()
    }

    /// Creates a collider with a rectangle shape defined by its extents and rounded corners.
    #[cfg(feature = "2d")]
    pub fn round_rectangle(x_length: Scalar, y_length: Scalar, border_radius: Scalar) -> Self {
        SharedShape::round_cuboid(x_length * 0.5, y_length * 0.5, border_radius).into()
    }

    /// Creates a collider with a ball shape defined by its radius.
    #[cfg(feature = "2d")]
    #[deprecated(
        since = "0.4.0",
        note = "please use `Collider::round_rectangle` instead"
    )]
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

    /// Creates a collider with a regular polygon shape defined by the circumradius and the number of sides.
    #[cfg(feature = "2d")]
    pub fn regular_polygon(circumradius: f32, sides: usize) -> Self {
        RegularPolygon::new(circumradius, sides).collider()
    }

    /// Creates a collider with a polyline shape defined by its vertices and optionally an index buffer.
    pub fn polyline(vertices: Vec<Vector>, indices: Option<Vec<[u32; 2]>>) -> Self {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        SharedShape::polyline(vertices, indices).into()
    }

    /// Creates a collider with a triangle mesh shape defined by its vertex and index buffers.
    ///
    /// Note that the resulting collider will be hollow and have no interior. This makes it more prone to tunneling and other collision issues.
    pub fn trimesh(vertices: Vec<Vector>, indices: Vec<[u32; 3]>) -> Self {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        SharedShape::trimesh(vertices, indices).into()
    }

    /// Creates a collider with a triangle mesh shape defined by its vertex and index buffers
    /// and flags controlling the preprocessing.
    ///
    /// Note that the resulting collider will be hollow and have no interior. This makes it more prone to tunneling and other collision issues.
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
    /// `heights` is a list indicating the altitude of each subdivision point, and `scale` controls
    /// the scaling factor along each axis.
    #[cfg(feature = "2d")]
    pub fn heightfield(heights: Vec<Scalar>, scale: Vector) -> Self {
        SharedShape::heightfield(heights.into(), scale.into()).into()
    }

    /// Creates a collider with a heightfield shape.
    ///
    /// A 3D heightfield is a rectangle on the `XZ` plane, subdivided in a grid pattern at regular intervals.
    ///
    /// `heights` is a matrix indicating the altitude of each subdivision point. The number of rows indicates
    /// the number of subdivisions along the `X` axis, while the number of columns indicates the number of
    /// subdivisions along the `Z` axis.
    ///
    /// `scale` controls the scaling factor along each axis.
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
    ///     let mesh = Mesh::from(Cuboid::default());
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
    ///     let mesh = Mesh::from(Cuboid::default());
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
    ///     let mesh = Mesh::from(Cuboid::default());
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
    ///     let mesh = Mesh::from(Cuboid::default());
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
    ///     let mesh = Mesh::from(Cuboid::default());
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
        TypedShape::Ball(b) => {
            #[cfg(feature = "2d")]
            {
                if scale.x == scale.y {
                    Ok(SharedShape::ball(b.radius * scale.x))
                } else {
                    // A 2D circle becomes an ellipse when scaled non-uniformly.
                    Ok(SharedShape::new(EllipseWrapper(Ellipse {
                        half_size: Vec2::splat(b.radius as f32) * scale.f32(),
                    })))
                }
            }
            #[cfg(feature = "3d")]
            match b.scaled(&scale.into(), num_subdivisions) {
                None => {
                    log::error!("Failed to apply scale {} to Ball shape.", scale);
                    Ok(SharedShape::ball(0.0))
                }
                Some(Either::Left(b)) => Ok(SharedShape::new(b)),
                Some(Either::Right(b)) => Ok(SharedShape::new(b)),
            }
        }
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
        TypedShape::Custom(_id) => {
            #[cfg(feature = "2d")]
            if _id == 1 {
                if let Some(ellipse) = shape.as_shape::<EllipseWrapper>() {
                    return Ok(SharedShape::new(EllipseWrapper(Ellipse {
                        half_size: ellipse.half_size * scale.f32(),
                    })));
                }
            } else if _id == 2 {
                if let Some(polygon) = shape.as_shape::<RegularPolygonWrapper>() {
                    if scale.x == scale.y {
                        return Ok(SharedShape::new(RegularPolygonWrapper(
                            RegularPolygon::new(
                                polygon.circumradius() * scale.x as f32,
                                polygon.sides,
                            ),
                        )));
                    } else {
                        let vertices = polygon
                            .vertices(0.0)
                            .into_iter()
                            .map(|v| v.adjust_precision().into())
                            .collect::<Vec<_>>();

                        return scale_shape(
                            &SharedShape::convex_hull(&vertices).unwrap(),
                            scale,
                            num_subdivisions,
                        );
                    }
                }
            }
            Err(parry::query::Unsupported)
        }
    }
}
