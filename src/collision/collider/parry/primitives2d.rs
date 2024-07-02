use crate::{AdjustPrecision, AsF32, Scalar, Vector, FRAC_PI_2, PI, TAU};

use super::{Collider, IntoCollider};
use bevy::prelude::Deref;
use bevy_math::{bounding::Bounded2d, prelude::*};
use nalgebra::{Point2, UnitVector2, Vector2};
use parry::{
    mass_properties::MassProperties,
    math::Isometry,
    query::{
        details::local_ray_intersection_with_support_map_with_params, gjk::VoronoiSimplex,
        point::local_point_projection_on_support_map, PointQuery, RayCast,
    },
    shape::{
        FeatureId, PackedFeatureId, PolygonalFeature, PolygonalFeatureMap, Shape, SharedShape,
        SupportMap,
    },
};

impl IntoCollider<Collider> for Circle {
    fn collider(&self) -> Collider {
        Collider::circle(self.radius.adjust_precision())
    }
}

impl IntoCollider<Collider> for Ellipse {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::new(EllipseWrapper(*self)))
    }
}

#[derive(Clone, Copy, Debug, Deref)]
pub(crate) struct EllipseWrapper(pub(crate) Ellipse);

impl SupportMap for EllipseWrapper {
    #[inline]
    fn local_support_point(&self, direction: &Vector2<Scalar>) -> Point2<Scalar> {
        let [a, b] = self.half_size.adjust_precision().to_array();
        let denom = (direction.x.powi(2) * a * a + direction.y.powi(2) * b * b).sqrt();
        Point2::new(a * a * direction.x / denom, b * b * direction.y / denom)
    }
}

impl Shape for EllipseWrapper {
    fn compute_local_aabb(&self) -> parry::bounding_volume::Aabb {
        let aabb = self.aabb_2d(Vec2::ZERO, 0.0);
        parry::bounding_volume::Aabb::new(
            aabb.min.adjust_precision().into(),
            aabb.max.adjust_precision().into(),
        )
    }

    fn compute_aabb(&self, position: &Isometry<Scalar>) -> parry::bounding_volume::Aabb {
        let aabb = self.aabb_2d(
            Vector::from(position.translation).f32(),
            position.rotation.angle() as f32,
        );
        parry::bounding_volume::Aabb::new(
            aabb.min.adjust_precision().into(),
            aabb.max.adjust_precision().into(),
        )
    }

    fn compute_local_bounding_sphere(&self) -> parry::bounding_volume::BoundingSphere {
        let sphere = self.bounding_circle(Vec2::ZERO, 0.0);
        parry::bounding_volume::BoundingSphere::new(
            sphere.center.adjust_precision().into(),
            sphere.radius().adjust_precision(),
        )
    }

    fn compute_bounding_sphere(
        &self,
        position: &Isometry<Scalar>,
    ) -> parry::bounding_volume::BoundingSphere {
        let sphere = self.bounding_circle(
            Vector::from(position.translation).f32(),
            position.rotation.angle() as f32,
        );
        parry::bounding_volume::BoundingSphere::new(
            sphere.center.adjust_precision().into(),
            sphere.radius().adjust_precision(),
        )
    }

    fn clone_box(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn mass_properties(&self, density: Scalar) -> MassProperties {
        let volume = self.area().adjust_precision();
        let mass = volume * density;
        let inertia = mass * self.half_size.length_squared().adjust_precision() / 4.0;
        MassProperties::new(Point2::origin(), mass, inertia)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> parry::shape::ShapeType {
        parry::shape::ShapeType::Custom
    }

    fn as_typed_shape(&self) -> parry::shape::TypedShape {
        parry::shape::TypedShape::Custom(1)
    }

    fn ccd_thickness(&self) -> Scalar {
        self.half_size.max_element().adjust_precision()
    }

    fn ccd_angular_thickness(&self) -> Scalar {
        crate::math::PI
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap> {
        Some(self as &dyn SupportMap)
    }
}

impl RayCast for EllipseWrapper {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &parry::query::Ray,
        max_toi: Scalar,
        solid: bool,
    ) -> Option<parry::query::RayIntersection> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            ray,
            max_toi,
            solid,
        )
    }
}

impl PointQuery for EllipseWrapper {
    fn project_local_point(
        &self,
        pt: &parry::math::Point<Scalar>,
        solid: bool,
    ) -> parry::query::PointProjection {
        local_point_projection_on_support_map(self, &mut VoronoiSimplex::new(), pt, solid)
    }

    fn project_local_point_and_get_feature(
        &self,
        pt: &parry::math::Point<Scalar>,
    ) -> (parry::query::PointProjection, parry::shape::FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Unknown)
    }
}

impl IntoCollider<Collider> for Plane2d {
    fn collider(&self) -> Collider {
        let vec = self.normal.perp().adjust_precision() * 100_000.0 / 2.0;
        Collider::segment(-vec, vec)
    }
}

impl IntoCollider<Collider> for Line2d {
    fn collider(&self) -> Collider {
        let vec = self.direction.adjust_precision() * 100_000.0 / 2.0;
        Collider::segment(-vec, vec)
    }
}

impl IntoCollider<Collider> for Segment2d {
    fn collider(&self) -> Collider {
        let (point1, point2) = (self.point1(), self.point2());
        Collider::segment(point1.adjust_precision(), point2.adjust_precision())
    }
}

impl<const N: usize> IntoCollider<Collider> for Polyline2d<N> {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.map(|v| v.adjust_precision());
        Collider::polyline(vertices.to_vec(), None)
    }
}

impl IntoCollider<Collider> for BoxedPolyline2d {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.iter().map(|v| v.adjust_precision());
        Collider::polyline(vertices.collect(), None)
    }
}

impl IntoCollider<Collider> for Triangle2d {
    fn collider(&self) -> Collider {
        Collider::triangle(
            self.vertices[0].adjust_precision(),
            self.vertices[1].adjust_precision(),
            self.vertices[2].adjust_precision(),
        )
    }
}

impl IntoCollider<Collider> for Rectangle {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::cuboid(
            self.half_size.x.adjust_precision(),
            self.half_size.y.adjust_precision(),
        ))
    }
}

impl<const N: usize> IntoCollider<Collider> for Polygon<N> {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.map(|v| v.adjust_precision());
        let indices = (0..N as u32 - 1).map(|i| [i, i + 1]).collect();
        Collider::convex_decomposition(vertices.to_vec(), indices)
    }
}

impl IntoCollider<Collider> for BoxedPolygon {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.iter().map(|v| v.adjust_precision());
        let indices = (0..self.vertices.len() as u32 - 1)
            .map(|i| [i, i + 1])
            .collect();
        Collider::convex_decomposition(vertices.collect(), indices)
    }
}

impl IntoCollider<Collider> for RegularPolygon {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::new(RegularPolygonWrapper(*self)))
    }
}

#[derive(Clone, Copy, Debug, Deref)]
pub(crate) struct RegularPolygonWrapper(pub(crate) RegularPolygon);

impl SupportMap for RegularPolygonWrapper {
    #[inline]
    fn local_support_point(&self, direction: &Vector2<Scalar>) -> Point2<Scalar> {
        // TODO: For polygons with a small number of sides, maybe just iterating
        //       through the vertices and comparing dot products is faster?

        let external_angle = self.external_angle_radians().adjust_precision();
        let circumradius = self.circumradius().adjust_precision();

        // Counterclockwise
        let angle_from_top = if direction.x < 0.0 {
            -Vector::from(*direction).angle_between(Vector::Y)
        } else {
            TAU - Vector::from(*direction).angle_between(Vector::Y)
        };

        // How many rotations of `external_angle` correspond to the vertex closest to the support direction.
        let n = (angle_from_top / external_angle).round() % self.sides as Scalar;

        // Rotate by an additional 90 degrees so that the first vertex is always at the top.
        let target_angle = n * external_angle + FRAC_PI_2;

        // Compute the vertex corresponding to the target angle on the unit circle.
        Point2::from(circumradius * Vector::from_angle(target_angle))
    }
}

impl PolygonalFeatureMap for RegularPolygonWrapper {
    #[inline]
    fn local_support_feature(
        &self,
        direction: &UnitVector2<Scalar>,
        out_feature: &mut PolygonalFeature,
    ) {
        let external_angle = self.external_angle_radians().adjust_precision();
        let circumradius = self.circumradius().adjust_precision();

        // Counterclockwise
        let angle_from_top = if direction.x < 0.0 {
            -Vector::from(*direction).angle_between(Vector::Y)
        } else {
            TAU - Vector::from(*direction).angle_between(Vector::Y)
        };

        // How many rotations of `external_angle` correspond to the vertices.
        let n_unnormalized = angle_from_top / external_angle;
        let n1 = n_unnormalized.floor() % self.sides as Scalar;
        let n2 = n_unnormalized.ceil() % self.sides as Scalar;

        // Rotate by an additional 90 degrees so that the first vertex is always at the top.
        let target_angle1 = n1 * external_angle + FRAC_PI_2;
        let target_angle2 = n2 * external_angle + FRAC_PI_2;

        // Compute the vertices corresponding to the target angle on the unit circle.
        let vertex1 = Point2::from(circumradius * Vector::from_angle(target_angle1));
        let vertex2 = Point2::from(circumradius * Vector::from_angle(target_angle2));

        *out_feature = PolygonalFeature {
            vertices: [vertex1, vertex2],
            vids: [
                PackedFeatureId::vertex(n1 as u32),
                PackedFeatureId::vertex(n2 as u32),
            ],
            fid: PackedFeatureId::face(n1 as u32),
            num_vertices: 2,
        };
    }
}

impl Shape for RegularPolygonWrapper {
    fn compute_local_aabb(&self) -> parry::bounding_volume::Aabb {
        let aabb = self.aabb_2d(Vec2::ZERO, 0.0);
        parry::bounding_volume::Aabb::new(
            aabb.min.adjust_precision().into(),
            aabb.max.adjust_precision().into(),
        )
    }

    fn compute_aabb(&self, position: &Isometry<Scalar>) -> parry::bounding_volume::Aabb {
        let aabb = self.aabb_2d(
            Vector::from(position.translation).f32(),
            position.rotation.angle() as f32,
        );
        parry::bounding_volume::Aabb::new(
            aabb.min.adjust_precision().into(),
            aabb.max.adjust_precision().into(),
        )
    }

    fn compute_local_bounding_sphere(&self) -> parry::bounding_volume::BoundingSphere {
        let sphere = self.bounding_circle(Vec2::ZERO, 0.0);
        parry::bounding_volume::BoundingSphere::new(
            sphere.center.adjust_precision().into(),
            sphere.radius().adjust_precision(),
        )
    }

    fn compute_bounding_sphere(
        &self,
        position: &Isometry<Scalar>,
    ) -> parry::bounding_volume::BoundingSphere {
        let sphere = self.bounding_circle(
            Vector::from(position.translation).f32(),
            position.rotation.angle() as f32,
        );
        parry::bounding_volume::BoundingSphere::new(
            sphere.center.adjust_precision().into(),
            sphere.radius().adjust_precision(),
        )
    }

    fn clone_box(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn mass_properties(&self, density: Scalar) -> MassProperties {
        let volume = self.area().adjust_precision();
        let mass = volume * density;

        let half_external_angle = PI / self.sides as Scalar;
        let angular_inertia = mass * self.circumradius().adjust_precision().powi(2) / 6.0
            * (1.0 + 2.0 * half_external_angle.cos().powi(2));

        MassProperties::new(Point2::origin(), mass, angular_inertia)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> parry::shape::ShapeType {
        parry::shape::ShapeType::Custom
    }

    fn as_typed_shape(&self) -> parry::shape::TypedShape {
        parry::shape::TypedShape::Custom(2)
    }

    fn ccd_thickness(&self) -> Scalar {
        self.circumradius().adjust_precision()
    }

    fn ccd_angular_thickness(&self) -> Scalar {
        crate::math::PI - self.internal_angle_radians().adjust_precision()
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap> {
        Some(self as &dyn SupportMap)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, Scalar)> {
        Some((self as &dyn PolygonalFeatureMap, 0.0))
    }

    fn feature_normal_at_point(
        &self,
        feature: FeatureId,
        _point: &Point2<Scalar>,
    ) -> Option<UnitVector2<Scalar>> {
        match feature {
            FeatureId::Face(id) => {
                let external_angle = self.external_angle_radians().adjust_precision();
                let normal_angle = id as Scalar * external_angle - external_angle * 0.5 + FRAC_PI_2;
                Some(UnitVector2::new_unchecked(
                    Vector::from_angle(normal_angle).into(),
                ))
            }
            FeatureId::Vertex(id) => {
                let external_angle = self.external_angle_radians().adjust_precision();
                let normal_angle = id as Scalar * external_angle + FRAC_PI_2;
                Some(UnitVector2::new_unchecked(
                    Vector::from_angle(normal_angle).into(),
                ))
            }
            _ => None,
        }
    }
}

impl RayCast for RegularPolygonWrapper {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &parry::query::Ray,
        max_toi: Scalar,
        solid: bool,
    ) -> Option<parry::query::RayIntersection> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            ray,
            max_toi,
            solid,
        )
    }
}

impl PointQuery for RegularPolygonWrapper {
    fn project_local_point(
        &self,
        pt: &parry::math::Point<Scalar>,
        solid: bool,
    ) -> parry::query::PointProjection {
        local_point_projection_on_support_map(self, &mut VoronoiSimplex::new(), pt, solid)
    }

    fn project_local_point_and_get_feature(
        &self,
        pt: &parry::math::Point<Scalar>,
    ) -> (parry::query::PointProjection, parry::shape::FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Unknown)
    }
}

impl IntoCollider<Collider> for Capsule2d {
    fn collider(&self) -> Collider {
        Collider::capsule(
            self.radius.adjust_precision(),
            2.0 * self.half_length.adjust_precision(),
        )
    }
}
