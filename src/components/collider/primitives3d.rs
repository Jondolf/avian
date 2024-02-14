use bevy_math::primitives::{
    BoxedPolyline3d, Capsule3d, Cone, Cuboid, Cylinder, Line3d, Plane3d, Polyline3d, Segment3d,
    Sphere, Torus,
};
use nalgebra::{Isometry, Isometry3, Point3};
use parry::{
    bounding_volume::{Aabb, BoundingSphere},
    mass_properties::MassProperties,
    shape::{Compound, Shape, ShapeType, SharedShape, SimdCompositeShape, TypedShape},
};
use parry3d::query::{PointQuery, RayCast};

use crate::{AdjustPrecision, Collider, IntoCollider, Quaternion, Scalar, Vector};

impl IntoCollider for Sphere {
    fn collider(&self) -> Collider {
        Collider::ball(self.radius.adjust_precision())
    }
}

impl IntoCollider for Plane3d {
    fn collider(&self) -> Collider {
        let half_size = Scalar::MAX / 2.0;
        let rotation = Quaternion::from_rotation_arc(Vector::Y, self.normal.adjust_precision());
        let vertices = vec![
            rotation * Vector::new(half_size, 0.0, -half_size),
            rotation * Vector::new(-half_size, 0.0, -half_size),
            rotation * Vector::new(-half_size, 0.0, half_size),
            rotation * Vector::new(half_size, 0.0, half_size),
        ];

        Collider::trimesh(vertices, vec![[0, 1, 2], [1, 2, 0]])
    }
}

impl IntoCollider for Line3d {
    fn collider(&self) -> Collider {
        let vec = self.direction.adjust_precision() * Scalar::MAX / 2.0;
        Collider::segment(-vec, vec)
    }
}

impl IntoCollider for Segment3d {
    fn collider(&self) -> Collider {
        let (point1, point2) = (self.point1(), self.point2());
        Collider::segment(point1.adjust_precision(), point2.adjust_precision())
    }
}

impl<const N: usize> IntoCollider for Polyline3d<N> {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.map(|v| v.adjust_precision());
        Collider::polyline(vertices.to_vec(), None)
    }
}

impl IntoCollider for BoxedPolyline3d {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.iter().map(|v| v.adjust_precision());
        Collider::polyline(vertices.collect(), None)
    }
}

impl IntoCollider for Cuboid {
    fn collider(&self) -> Collider {
        let [hx, hy, hz] = self.half_size.adjust_precision().to_array();
        Collider::from(SharedShape::cuboid(hx, hy, hz))
    }
}

impl IntoCollider for Cylinder {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::cylinder(
            self.half_height.adjust_precision(),
            self.radius.adjust_precision(),
        ))
    }
}

impl IntoCollider for Capsule3d {
    fn collider(&self) -> Collider {
        Collider::capsule(
            2.0 * self.half_length.adjust_precision(),
            self.radius.adjust_precision(),
        )
    }
}

impl IntoCollider for Cone {
    fn collider(&self) -> Collider {
        Collider::cone(
            self.height.adjust_precision(),
            self.radius.adjust_precision(),
        )
    }
}

impl IntoCollider for Torus {
    fn collider(&self) -> Collider {
        // TODO: Use angled cylinders instead of capsules.
        // TODO: The resolution should maybe be configurable.
        let major_resolution = 52;

        // Points sampled along the major ring
        let mut points: Vec<Point3<Scalar>> = Vec::with_capacity(major_resolution as usize);

        let start_angle = std::f32::consts::FRAC_PI_2;
        let step = std::f32::consts::TAU / major_resolution as f32;

        for i in 0..major_resolution {
            // Compute vertex position at angle theta
            let theta = start_angle + i as f32 * step;
            let (sin, cos) = theta.sin_cos();
            let x = cos * self.major_radius;
            let y = sin * self.major_radius;

            points.push(Point3::new(x, 0.0, y));
        }

        let mut parts = vec![];
        for p in points.windows(2) {
            parts.push((
                Isometry::identity(),
                SharedShape::capsule(p[0], p[1], self.minor_radius),
            ));
        }

        Collider::from(SharedShape::new(TorusWrapper(*self, Compound::new(parts))))
    }
}

#[derive(Clone)]
pub(crate) struct TorusWrapper(pub(crate) Torus, pub(crate) Compound);

impl Shape for TorusWrapper {
    fn clone_box(&self) -> Box<dyn Shape> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb {
        self.1.compute_local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere {
        self.1.compute_local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry3<Scalar>) -> Aabb {
        self.1.compute_aabb(position)
    }

    fn mass_properties(&self, density: Scalar) -> MassProperties {
        MassProperties::from_compound(density, self.1.shapes())
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Compound
    }

    fn as_typed_shape(&self) -> TypedShape {
        TypedShape::Custom(3)
    }

    fn ccd_thickness(&self) -> Scalar {
        self.1
            .shapes()
            .iter()
            .fold(Scalar::MAX, |curr, (_, s)| curr.min(s.ccd_thickness()))
    }

    fn ccd_angular_thickness(&self) -> Scalar {
        self.1.shapes().iter().fold(Scalar::MAX, |curr, (_, s)| {
            curr.max(s.ccd_angular_thickness())
        })
    }

    fn as_composite_shape(&self) -> Option<&dyn SimdCompositeShape> {
        Some(self as &dyn SimdCompositeShape)
    }
}

impl SimdCompositeShape for TorusWrapper {
    #[inline]
    fn map_part_at(
        &self,
        shape_id: u32,
        f: &mut dyn FnMut(Option<&parry3d::math::Isometry<parry3d::math::Real>>, &dyn Shape),
    ) {
        self.1.map_part_at(shape_id, f)
    }

    #[inline]
    fn qbvh(&self) -> &parry3d::partitioning::Qbvh<u32> {
        self.1.qbvh()
    }
}

impl RayCast for TorusWrapper {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &parry::query::Ray,
        max_toi: parry::math::Real,
        solid: bool,
    ) -> Option<parry::query::RayIntersection> {
        self.1.cast_local_ray_and_get_normal(ray, max_toi, solid)
    }
}

impl PointQuery for TorusWrapper {
    fn project_local_point(
        &self,
        pt: &parry::math::Point<parry::math::Real>,
        solid: bool,
    ) -> parry::query::PointProjection {
        self.1.project_local_point(pt, solid)
    }

    fn project_local_point_and_get_feature(
        &self,
        pt: &parry::math::Point<parry::math::Real>,
    ) -> (parry::query::PointProjection, parry::shape::FeatureId) {
        self.1.project_local_point_and_get_feature(pt)
    }
}

/* TODO
impl SupportMap for ConicalFrustumWrapper {
    #[inline]
    fn local_support_point(&self, direction: &Vector3<Scalar>) -> Point3<Scalar> {
        if self.radius_top == self.radius_bottom {
            return Cylinder::new(self.height / 2.0, self.radius_top)
                .local_support_point(direction);
        }

        let frustum = self.0;
        let half_height = frustum.height / 2.0;

        let mut result = *direction;

        result.y = 0.0;

        if result.normalize_mut() == 0.0 {
            result = zero();
            result.y = half_height.copysign(direction.y);
        } else {
            result *= if direction.y >= 0.0 {
                frustum.radius_top
            } else {
                frustum.radius_bottom
            };
        }

        result.y = half_height.copysign(direction.y);

        Point3::from(result)
    }
}

impl PolygonalFeatureMap for ConicalFrustumWrapper {
    fn local_support_feature(
        &self,
        dir: &Unit<Vector3<Scalar>>,
        out_features: &mut PolygonalFeature,
    ) {
        let dir2 = Vector2::new(dir.x, dir.z)
            .try_normalize(Scalar::EPSILON)
            .unwrap_or(Vector2::x());

        let radius = if dir2.y >= 0.0 {
            self.0.radius_top
        } else {
            self.0.radius_bottom
        };
        let half_height = self.height / 2.0;

        if dir.y.abs() < 0.5 {
            // We return a segment lying on the curved part.
            out_features.vertices[0] = Point3::new(dir2.x * radius, -half_height, dir2.y * radius);
            out_features.vertices[1] = Point3::new(dir2.x * radius, half_height, dir2.y * radius);
            out_features.eids = [
                PackedFeatureId::edge(0),
                PackedFeatureId::edge(0),
                PackedFeatureId::edge(0),
                PackedFeatureId::edge(0),
            ];
            out_features.fid = PackedFeatureId::face(0);
            out_features.num_vertices = 2;
            out_features.vids = [
                PackedFeatureId::vertex(1),
                PackedFeatureId::vertex(11),
                PackedFeatureId::vertex(11),
                PackedFeatureId::vertex(11),
            ];
        } else {
            // We return a square approximation of the cap.
            let y = half_height.copysign(dir.y);

            out_features.vertices[0] = Point3::new(dir2.x * radius, y, dir2.y * radius);
            out_features.vertices[1] = Point3::new(-dir2.y * radius, y, dir2.x * radius);
            out_features.vertices[2] = Point3::new(-dir2.x * radius, y, -dir2.y * radius);
            out_features.vertices[3] = Point3::new(dir2.y * radius, y, -dir2.x * radius);

            if dir.y < 0.0 {
                out_features.eids = [
                    PackedFeatureId::edge(2),
                    PackedFeatureId::edge(4),
                    PackedFeatureId::edge(6),
                    PackedFeatureId::edge(8),
                ];
                out_features.fid = PackedFeatureId::face(9);
                out_features.num_vertices = 4;
                out_features.vids = [
                    PackedFeatureId::vertex(1),
                    PackedFeatureId::vertex(3),
                    PackedFeatureId::vertex(5),
                    PackedFeatureId::vertex(7),
                ];
            } else {
                out_features.eids = [
                    PackedFeatureId::edge(12),
                    PackedFeatureId::edge(14),
                    PackedFeatureId::edge(16),
                    PackedFeatureId::edge(18),
                ];
                out_features.fid = PackedFeatureId::face(19);
                out_features.num_vertices = 4;
                out_features.vids = [
                    PackedFeatureId::vertex(11),
                    PackedFeatureId::vertex(13),
                    PackedFeatureId::vertex(15),
                    PackedFeatureId::vertex(17),
                ];
            }
        }
    }
}

impl Shape for ConicalFrustumWrapper {
    fn compute_local_aabb(&self) -> parry3d::bounding_volume::Aabb {
        let aabb = self.0.aabb_3d(Vec3::ZERO, Quat::IDENTITY);
        parry3d::bounding_volume::Aabb::new(aabb.min.into(), aabb.max.into())
    }

    fn compute_local_bounding_sphere(&self) -> parry3d::bounding_volume::BoundingSphere {
        let sphere = self.0.bounding_sphere(Vec3::ZERO, Quat::IDENTITY);
        parry3d::bounding_volume::BoundingSphere::new(
            sphere.center.into(),
            sphere.radius().adjust_precision(),
        )
    }

    fn clone_box(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn mass_properties(&self, density: parry3d::math::Real) -> MassProperties {
        if self.radius_top == self.radius_bottom {
            MassProperties::from_cylinder(density, self.height / 2.0, self.radius_top)
        } else {
            let (min_radius, max_radius) = if self.radius_top < self.radius_bottom {
                (self.radius_top, self.radius_bottom)
            } else {
                (self.radius_bottom, self.radius_top)
            };
            let cone_height =
                max_radius * (self.height / (self.radius_top - self.radius_bottom).abs());
            let cone_half_height = cone_height / 2.0;
            let cone2_height = cone_height - self.height;
            let (com1, com2) = if self.radius_top < self.radius_bottom {
                (-cone_height / 4.0, self.height / 2.0 + cone2_height / 4.0)
            } else {
                (
                    cone_height / 4.0,
                    (cone_height - self.height) / 4.0 - self.height / 2.0,
                )
            };
            let radii_squared = self.radius_top.powi(2) + self.radius_bottom.powi(2);
            let volume1 = crate::math::PI / 3.0 * max_radius.powi(2) * cone_height;
            let volume2 = crate::math::PI / 3.0 * min_radius.powi(2) * (cone_height - self.height);
            let volume = volume1 - volume2;
            let mass = density * volume;
            let com = (com1 * volume1 - com2 * volume2) / (volume1 - volume2);

            let off_principal = max_radius.powi(2) * 3.0 / 20.0 + cone_height.powi(2) * 3.0 / 80.0;
            let principal = 1.0 / 12.0 * mass * (self.height.powi(2) + 3.0 * radii_squared);
            MassProperties::with_principal_inertia_frame(
                Point3::new(0.0, com, 0.0),
                mass,
                Vector3::new(off_principal, principal, off_principal) * mass,
                UnitQuaternion::identity(),
            )
        }
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> parry3d::shape::ShapeType {
        parry3d::shape::ShapeType::Custom
    }

    fn as_typed_shape(&self) -> parry3d::shape::TypedShape {
        parry3d::shape::TypedShape::Custom(2)
    }

    fn ccd_thickness(&self) -> parry3d::math::Real {
        self.radius_bottom.max(self.radius_top)
    }

    fn ccd_angular_thickness(&self) -> parry3d::math::Real {
        crate::math::PI / 2.0
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap> {
        Some(self as &dyn SupportMap)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, parry3d::math::Real)> {
        Some((self as &dyn PolygonalFeatureMap, 0.0))
    }
}

impl RayCast for ConicalFrustumWrapper {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &parry3d::query::Ray,
        max_toi: parry3d::math::Real,
        solid: bool,
    ) -> Option<parry3d::query::RayIntersection> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            ray,
            max_toi,
            solid,
        )
    }
}

impl PointQuery for ConicalFrustumWrapper {
    fn project_local_point(
        &self,
        pt: &parry3d::math::Point<parry3d::math::Real>,
        solid: bool,
    ) -> parry3d::query::PointProjection {
        local_point_projection_on_support_map(self, &mut VoronoiSimplex::new(), pt, solid)
    }

    fn project_local_point_and_get_feature(
        &self,
        pt: &parry3d::math::Point<parry3d::math::Real>,
    ) -> (parry3d::query::PointProjection, parry3d::shape::FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Unknown)
    }
}

#[derive(Clone, Copy, Debug, Deref)]
pub(super) struct ConicalFrustumWrapper(pub(super) ConicalFrustum);

#[cfg(test)]
mod tests {
    use bevy_math::primitives::ConicalFrustum;
    use nalgebra::{Point3, Vector3};
    use parry3d::shape::SupportMap;

    use super::ConicalFrustumWrapper;

    #[test]
    fn conical_frustum_support() {
        let frustum = ConicalFrustumWrapper(ConicalFrustum {
            radius_top: 0.5,
            radius_bottom: 1.0,
            height: 2.0,
        });

        assert_eq!(
            frustum.local_support_point(&Vector3::new(1.0, 1.0, 0.0)),
            Point3::new(0.5, 1.0, 0.0)
        );
        assert_eq!(
            frustum.local_support_point(&Vector3::new(-1.0, -1.0, 0.0)),
            Point3::new(-1.0, -1.0, 0.0)
        );
    }
}
*/
