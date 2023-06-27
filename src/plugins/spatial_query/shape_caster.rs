use crate::prelude::*;
use bevy::{prelude::*, utils::HashMap};
use parry::{query::details::TOICompositeShapeShapeBestFirstVisitor, shape::Shape};

/// A component used for shape casting.
///
/// **Shape casting** is a type of [spatial query](spatial_query) where a shape travels along a straight
/// line and computes intersections with colliders. This is often used to determine how far an object can move
/// in a direction before it hits something.
///
/// Each shape cast is defined by a `shape` (a [`Collider`]), its local `shape_rotation`, a local `origin` and
/// a local `direction`. The [`ShapeCaster`] will find the closest intersection as a [`ShapeIntersection`].
///
/// The [`ShapeCaster`] is the easiest way to handle shape casting, but **it can only retrieve one intersection**.
/// If you want more control and don't want to perform shape casts on every frame, consider using
/// the [`SpatialQuery`] system parameter.
///
/// ## Example
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn setup(mut commands: Commands) {
///     // Spawn a shape caster with a ball shape moving right starting from the origin
///     commands.spawn(ShapeCaster::new(Collider::ball(0.5), Vec3::ZERO, Vec3::X));
///     // ...spawn colliders and other things
/// }
///
/// fn print_intersections(query: Query<(&ShapeCaster, &ShapeIntersection)>) {
///     for (shape_caster, intersection) in &query {
///         println!("Hit entity {:?}", intersection.entity);
///     }
/// }
/// ```
#[derive(Component)]
pub struct ShapeCaster {
    /// Controls if the shape caster is enabled.
    pub enabled: bool,
    /// The shape being cast represented as a [`Collider`].
    pub shape: Collider,
    /// The local origin of the shape relative to the [`Position`] and [`Rotation`]
    /// of the shape caster entity or its parent.
    ///
    /// To get the global origin, use the `global_origin` method.
    pub origin: Vector,
    /// The global origin of the shape.
    global_origin: Vector,
    /// The local rotation of the shape being cast relative to the [`Rotation`]
    /// of the shape caster entity or its parent. Expressed in radians.
    ///
    /// To get the global shape rotation, use the `global_shape_rotation` method.
    #[cfg(feature = "2d")]
    pub shape_rotation: Scalar,
    /// The local rotation of the shape being cast relative to the [`Rotation`]
    /// of the shape caster entity or its parent.
    ///
    /// To get the global shape rotation, use the `global_shape_rotation` method.
    #[cfg(feature = "3d")]
    pub shape_rotation: Quat,
    /// The global rotation of the shape.
    #[cfg(feature = "2d")]
    global_shape_rotation: Scalar,
    /// The global rotation of the shape.
    #[cfg(feature = "3d")]
    global_shape_rotation: Quat,
    /// The local direction of the shape cast relative to the [`Rotation`] of the shape caster entity or its parent.
    ///
    /// To get the global direction, use the `global_direction` method.
    pub direction: Vector,
    /// The global direction of the shape cast.
    global_direction: Vector,
    /// The maximum distance the shape can travel. By default this is infinite, so the shape will travel
    /// until an intersection is found.
    pub max_time_of_impact: Scalar,
    /// Controls how the shape cast behaves when the shape is already penetrating a [collider](Collider)
    /// at the shape origin.
    ///
    /// If set to true **and** the shape is being cast in a direction where it will eventually stop penetrating,
    /// the shape cast will not stop immediately, and will instead continue to find another intersection.\
    /// If set to false, the shape cast will stop immediately and return the intersection. This is the default.
    pub ignore_origin_penetration: bool,
}

impl Default for ShapeCaster {
    fn default() -> Self {
        Self {
            enabled: true,
            shape: Collider::ball(0.0),
            origin: Vector::ZERO,
            global_origin: Vector::ZERO,
            #[cfg(feature = "2d")]
            shape_rotation: 0.0,
            #[cfg(feature = "3d")]
            shape_rotation: Quat::IDENTITY,
            #[cfg(feature = "2d")]
            global_shape_rotation: 0.0,
            #[cfg(feature = "3d")]
            global_shape_rotation: Quat::IDENTITY,
            direction: Vector::ZERO,
            global_direction: Vector::ZERO,
            max_time_of_impact: Scalar::MAX,
            ignore_origin_penetration: false,
        }
    }
}

impl ShapeCaster {
    /// Creates a new [`ShapeCaster`] with a given shape, origin, shape rotation and direction.
    #[cfg(feature = "2d")]
    pub fn new(shape: Collider, origin: Vector, shape_rotation: Scalar, direction: Vector) -> Self {
        Self {
            shape,
            origin,
            shape_rotation,
            direction,
            ..default()
        }
    }
    #[cfg(feature = "3d")]
    /// Creates a new [`ShapeCaster`] with a given shape, origin, shape rotation and direction.
    pub fn new(shape: Collider, origin: Vector, shape_rotation: Quat, direction: Vector) -> Self {
        Self {
            shape,
            origin,
            shape_rotation,
            direction,
            ..default()
        }
    }

    /// Controls how the shape cast behaves when the shape is already penetrating a [collider](Collider)
    /// at the shape origin.
    ///
    /// If set to true **and** the shape is being cast in a direction where it will eventually stop penetrating,
    /// the shape cast will not stop immediately, and will instead continue to find another intersection.\
    /// If set to false, the shape cast will stop immediately and return the intersection. This is the default.
    pub fn with_ignore_origin_penetration(mut self, ignore: bool) -> Self {
        self.ignore_origin_penetration = ignore;
        self
    }

    /// Sets the maximum time of impact, i.e. the maximum distance that the ray is allowed to travel.
    pub fn with_max_time_of_impact(mut self, max_time_of_impact: Scalar) -> Self {
        self.max_time_of_impact = max_time_of_impact;
        self
    }

    /// Enables the [`ShapeCaster`].
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disables the [`ShapeCaster`].
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Returns the global origin of the ray.
    pub fn global_origin(&self) -> Vector {
        self.global_origin
    }

    /// Returns the global rotation of the shape.
    #[cfg(feature = "2d")]
    pub fn global_shape_rotation(&self) -> Scalar {
        self.global_shape_rotation
    }

    /// Returns the global rotation of the shape.
    #[cfg(feature = "3d")]
    pub fn global_shape_rotation(&self) -> Quat {
        self.global_shape_rotation
    }

    /// Returns the global direction of the ray.
    pub fn global_direction(&self) -> Vector {
        self.global_direction
    }

    /// Sets the global origin of the ray.
    pub(crate) fn set_global_origin(&mut self, global_origin: Vector) {
        self.global_origin = global_origin;
    }

    /// Sets the global rotation of the shape.
    #[cfg(feature = "2d")]
    pub fn set_global_shape_rotation(&mut self, global_rotation: Scalar) {
        self.global_shape_rotation = global_rotation;
    }

    /// Sets the global rotation of the shape.
    #[cfg(feature = "3d")]
    pub fn set_global_shape_rotation(&mut self, global_rotation: Quat) {
        self.global_shape_rotation = global_rotation;
    }

    /// Sets the global direction of the ray.
    pub(crate) fn set_global_direction(&mut self, global_direction: Vector) {
        self.global_direction = global_direction;
    }

    pub(crate) fn cast(
        &self,
        colliders: &HashMap<Entity, (Isometry<Scalar>, &dyn Shape)>,
        query_pipeline: &SpatialQueryPipeline,
    ) -> Option<ShapeIntersection> {
        let shape_rotation: Rotation;

        #[cfg(feature = "2d")]
        {
            shape_rotation = Rotation::from_radians(self.global_shape_rotation());
        }
        #[cfg(feature = "3d")]
        {
            shape_rotation = Rotation::from(self.global_shape_rotation());
        }
        let shape_isometry = utils::make_isometry(self.global_origin(), &shape_rotation);
        let shape_direction = self.global_direction().into();

        let pipeline_shape = query_pipeline.as_composite_shape(colliders);
        let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
            &*query_pipeline.dispatcher,
            &shape_isometry,
            &shape_direction,
            &pipeline_shape,
            &**self.shape.get_shape(),
            self.max_time_of_impact,
            self.ignore_origin_penetration,
        );

        query_pipeline.qbvh.traverse_best_first(&mut visitor).map(
            |(_, (entity_bits, intersection))| ShapeIntersection {
                entity: Entity::from_bits(entity_bits),
                time_of_impact: intersection.toi,
                point1: intersection.witness1.into(),
                point2: intersection.witness2.into(),
                normal1: intersection.normal1.into(),
                normal2: intersection.normal2.into(),
            },
        )
    }
}

/// A component for the closest intersection of a shape cast by a [`ShapeCaster`].
///
/// When there are no intersections, the value is `None`.
///
/// ## Example
///
/// ```
/// # use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// fn print_intersections(query: Query<&ShapeCasterIntersection>) {
///     for intersection in &query {
///         if let Some(intersection) = intersection {
///             println!(
///                 "Hit entity {:?} with time of impact {}",
///                 intersection.entity,
///                 intersection.time_of_impact,
///             );
///         }
///     }
/// }
/// ```
#[derive(Component, Clone, Copy, Debug)]
pub struct ShapeCasterIntersection(pub Option<ShapeIntersection>);

/// Data related to an intersection between a shape and a [collider](Collider) in a [shape cast](ShapeCaster).
#[derive(Component, Clone, Copy, Debug)]
pub struct ShapeIntersection {
    /// The entity of the collider that was hit by the ray.
    pub entity: Entity,
    /// How long the shape travelled before the intersection started,
    /// i.e. the distance between the origin and the point of intersection.
    pub time_of_impact: Scalar,
    /// The closest point on the cast shape, at the time of impact,
    /// expressed in the local space of the cast shape.
    pub point1: Vector,
    /// The closest point on the collider that was hit by the shape cast, at the time of impact,
    /// expressed in the local space of the collider shape.
    pub point2: Vector,
    /// The outward normal on the cast shape, at the time of impact,
    /// expressed in the local space of the cast shape.
    pub normal1: Vector,
    /// The outward normal on the collider that was hit by the shape cast, at the time of impact,
    /// expressed in the local space of the collider shape.
    pub normal2: Vector,
}
