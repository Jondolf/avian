use crate::prelude::*;
use bevy::{prelude::*, utils::HashMap};
use parry::{
    query::{
        details::RayCompositeShapeToiAndNormalBestFirstVisitor, visitors::RayIntersectionsVisitor,
    },
    shape::Shape,
};

#[derive(Component)]
pub struct RayCaster {
    pub enabled: bool,
    pub origin: Vector,
    pub direction: Vector,
    pub max_hits: u32,
}

impl Default for RayCaster {
    fn default() -> Self {
        Self {
            enabled: true,
            origin: Vector::ZERO,
            direction: Vector::ZERO,
            max_hits: u32::MAX,
        }
    }
}

impl RayCaster {
    pub fn new(origin: Vector, direction: Vector) -> Self {
        Self {
            origin,
            direction,
            ..default()
        }
    }

    pub fn with_max_hits(mut self, max_hits: u32) -> Self {
        self.max_hits = max_hits;
        self
    }

    pub fn enable(&mut self) {
        self.enabled = true;
    }

    pub fn disable(&mut self) {
        self.enabled = false;
    }

    pub fn cast(
        &self,
        intersections: &mut RayIntersections,
        colliders: &HashMap<Entity, (Isometry<Scalar>, &dyn Shape)>,
        query_pipeline: &SpatialQueryPipeline,
        max_time_of_impact: Scalar,
        max_hit_count: u32,
        solid: bool,
    ) {
        intersections.count = 0;
        if max_hit_count == 1 {
            let pipeline_shape = query_pipeline.as_composite_shape(colliders);
            let ray = parry::query::Ray::new(self.origin.into(), self.direction.into());
            let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
                &pipeline_shape,
                &ray,
                max_time_of_impact,
                solid,
            );

            if let Some(intersection) = query_pipeline.qbvh.traverse_best_first(&mut visitor).map(
                |(_, (entity_bits, intersection))| RayIntersection {
                    entity: Entity::from_bits(entity_bits),
                    time_of_impact: intersection.toi,
                    normal: intersection.normal.into(),
                },
            ) {
                if (intersections.vector.len() as u32) < intersections.count + 1 {
                    intersections.vector.push(intersection);
                } else {
                    intersections.vector[0] = intersection;
                }
                intersections.count = 1;
            }
        } else {
            let mut leaf_callback = &mut |entity_bits: &u64| {
                let entity = Entity::from_bits(*entity_bits);
                if let Some((iso, shape)) = colliders.get(&entity) {
                    let ray = parry::query::Ray::new(self.origin.into(), self.direction.into());
                    if let Some(intersection) =
                        shape.cast_ray_and_get_normal(iso, &ray, max_time_of_impact, solid)
                    {
                        if (intersections.vector.len() as u32) < intersections.count + 1 {
                            intersections.vector.push(RayIntersection {
                                entity,
                                time_of_impact: intersection.toi,
                                normal: intersection.normal.into(),
                            });
                        } else {
                            intersections.vector[intersections.count as usize] = RayIntersection {
                                entity,
                                time_of_impact: intersection.toi,
                                normal: intersection.normal.into(),
                            };
                        }

                        intersections.count += 1;

                        return intersections.count < max_hit_count;
                    }
                }
                true
            };

            let ray = parry::query::Ray::new(self.origin.into(), self.direction.into());
            let mut visitor =
                RayIntersectionsVisitor::new(&ray, max_time_of_impact, &mut leaf_callback);
            query_pipeline.qbvh.traverse_depth_first(&mut visitor);
        }
    }
}

#[derive(Component, Default)]
pub struct RayIntersections {
    pub(crate) vector: Vec<RayIntersection>,
    pub count: u32,
}

impl RayIntersections {
    pub fn iter(&self) -> &[RayIntersection] {
        &self.vector[0..self.count as usize]
    }
    pub fn iter_sorted(&self) -> std::vec::IntoIter<RayIntersection> {
        let mut vector = self.vector[0..self.count as usize].to_vec();
        vector.sort_by(|a, b| a.time_of_impact.partial_cmp(&b.time_of_impact).unwrap());
        vector.into_iter()
    }
}

#[derive(Clone, Copy)]
pub struct RayIntersection {
    pub entity: Entity,
    pub time_of_impact: Scalar,
    pub normal: Vector,
}
