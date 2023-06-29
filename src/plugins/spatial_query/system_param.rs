use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*, utils::HashMap};
use parry::query::{
    details::{
        RayCompositeShapeToiAndNormalBestFirstVisitor, TOICompositeShapeShapeBestFirstVisitor,
    },
    visitors::RayIntersectionsVisitor,
};

#[cfg(feature = "2d")]
type ShapeRotation = Scalar;
#[cfg(feature = "3d")]
type ShapeRotation = Quat;

/// A system parameter for spatial queries that require more precise control.
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
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// fn print_intersections(spatial_query: SpatialQuery) {
///     let intersections = spatial_query.ray_intersections(
///         Vec3::ZERO, // Origin
///         Vec3::X,    // Direction
///         100.0,      // Maximum time of impact (travel distance)
///         20,         // Maximum number of intersections
///         true,       // Is the ray solid
///     );
///
///     println!("{:?}", intersections);
/// }
/// # #[cfg(not(all(feature = "3d", feature = "f32")))]
/// # fn print_intersections() {}
/// ```
#[derive(SystemParam)]
pub struct SpatialQuery<'w, 's> {
    colliders: Query<
        'w,
        's,
        (
            Entity,
            &'static Position,
            &'static Rotation,
            &'static Collider,
        ),
    >,
    query_pipeline: ResMut<'w, SpatialQueryPipeline>,
}

impl<'w, 's> SpatialQuery<'w, 's> {
    /// Casts a [ray](RayCaster) from `origin` in a given `direction` and computes the closest [intersection](RayIntersection)
    /// with a collider. If there are no intersections, `None` is returned.
    ///
    /// This should be used when you don't need frequent ray casts and want the result instantly.
    /// Otherwise, using [`RayCaster`] is recommended.
    pub fn cast_ray(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = self
            .colliders
            .iter()
            .map(|(entity, position, rotation, collider)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        &**collider.get_shape(),
                    ),
                )
            })
            .collect();

        let pipeline_shape = self.query_pipeline.as_composite_shape(&colliders);
        let ray = parry::query::Ray::new(origin.into(), direction.into());
        let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
            &pipeline_shape,
            &ray,
            max_time_of_impact,
            solid,
        );

        self.query_pipeline
            .qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (entity_bits, intersection))| RayIntersection {
                entity: Entity::from_bits(entity_bits),
                time_of_impact: intersection.toi,
                normal: intersection.normal.into(),
            })
    }

    /// Casts a [ray](RayCaster) from `origin` in a given `direction` and computes all [intersections](RayIntersection)
    /// until `max_hits` is reached.
    ///
    /// Note that the order of the results is not guaranteed, and if there are more intersections than `max_hits`, some intersections will be missed.
    ///
    /// This should be used when you don't need frequent ray casts and want the result instantly.
    /// Otherwise, using [`RayCaster`] is recommended.
    pub fn ray_intersections(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: f32,
        max_hits: u32,
        solid: bool,
    ) -> Vec<RayIntersection> {
        let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = self
            .colliders
            .iter()
            .map(|(entity, position, rotation, collider)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        &**collider.get_shape(),
                    ),
                )
            })
            .collect();

        let mut intersections = Vec::with_capacity(max_hits.min(100) as usize);

        let ray = parry::query::Ray::new(origin.into(), direction.into());

        let mut leaf_callback = &mut |entity_bits: &u64| {
            let entity = Entity::from_bits(*entity_bits);
            if let Some((iso, shape)) = colliders.get(&entity) {
                if let Some(intersection) =
                    shape.cast_ray_and_get_normal(iso, &ray, max_time_of_impact, solid)
                {
                    intersections.push(RayIntersection {
                        entity,
                        time_of_impact: intersection.toi,
                        normal: intersection.normal.into(),
                    });

                    return (intersections.len() as u32) < max_hits;
                }
            }
            true
        };

        let mut visitor =
            RayIntersectionsVisitor::new(&ray, max_time_of_impact, &mut leaf_callback);
        self.query_pipeline.qbvh.traverse_depth_first(&mut visitor);

        intersections
    }

    /// Casts a [ray](RayCaster) from `origin` in a given `direction` and computes all [intersections](RayIntersection)
    /// until the given `callback` returns false or all intersections have been computed.
    ///
    /// Note that the order of the results is not guaranteed, and if there are more intersections than `max_hits`, some intersections will be missed.
    ///
    /// This should be used when you don't need frequent ray casts and want the result instantly.
    /// Otherwise, using [`RayCaster`] is recommended.
    pub fn ray_intersections_callback(
        &self,
        origin: Vector,
        direction: Vector,
        max_time_of_impact: f32,
        solid: bool,
        mut callback: impl FnMut(Entity, RayIntersection) -> bool,
    ) -> Vec<RayIntersection> {
        let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = self
            .colliders
            .iter()
            .map(|(entity, position, rotation, collider)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        &**collider.get_shape(),
                    ),
                )
            })
            .collect();

        let mut intersections = Vec::with_capacity(10);

        let ray = parry::query::Ray::new(origin.into(), direction.into());

        let mut leaf_callback = &mut |entity_bits: &u64| {
            let entity = Entity::from_bits(*entity_bits);
            if let Some((iso, shape)) = colliders.get(&entity) {
                if let Some(intersection) =
                    shape.cast_ray_and_get_normal(iso, &ray, max_time_of_impact, solid)
                {
                    let intersection = RayIntersection {
                        entity,
                        time_of_impact: intersection.toi,
                        normal: intersection.normal.into(),
                    };
                    intersections.push(intersection);

                    return callback(entity, intersection);
                }
            }
            true
        };

        let mut visitor =
            RayIntersectionsVisitor::new(&ray, max_time_of_impact, &mut leaf_callback);
        self.query_pipeline.qbvh.traverse_depth_first(&mut visitor);

        intersections
    }

    /// Casts a [shape](ShapeCaster) with a given rotation from `origin` in a given `direction` and computes the closest [intersection](ShapeIntersection).
    /// with a collider. If there are no intersections, `None` is returned.
    ///
    /// This should be used when you don't need to shape cast on every frame and want the result instantly.
    /// Otherwise, using [`ShapeCaster`] can be more convenient.
    pub fn cast_shape(
        &self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: ShapeRotation,
        direction: Vector,
        max_time_of_impact: f32,
        ignore_origin_penetration: bool,
    ) -> Option<ShapeIntersection> {
        let colliders: HashMap<Entity, (Isometry<Scalar>, &dyn parry::shape::Shape)> = self
            .colliders
            .iter()
            .map(|(entity, position, rotation, collider)| {
                (
                    entity,
                    (
                        utils::make_isometry(position.0, rotation),
                        &**collider.get_shape(),
                    ),
                )
            })
            .collect();

        let rotation: Rotation;
        #[cfg(feature = "2d")]
        {
            rotation = Rotation::from_radians(shape_rotation);
        }
        #[cfg(feature = "3d")]
        {
            rotation = Rotation::from(shape_rotation);
        }

        let shape_isometry = utils::make_isometry(origin, &rotation);
        let shape_direction = direction.into();
        let pipeline_shape = self.query_pipeline.as_composite_shape(&colliders);
        let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
            &*self.query_pipeline.dispatcher,
            &shape_isometry,
            &shape_direction,
            &pipeline_shape,
            &**shape.get_shape(),
            max_time_of_impact,
            ignore_origin_penetration,
        );

        self.query_pipeline
            .qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (entity_bits, intersection))| ShapeIntersection {
                entity: Entity::from_bits(entity_bits),
                time_of_impact: intersection.toi,
                point1: intersection.witness1.into(),
                point2: intersection.witness2.into(),
                normal1: intersection.normal1.into(),
                normal2: intersection.normal2.into(),
            })
    }
}
