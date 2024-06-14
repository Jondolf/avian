use bevy_math::primitives::{
    BoxedPolyline3d, Capsule3d, Cone, Cuboid, Cylinder, Line3d, Plane3d, Polyline3d, Segment3d,
    Sphere,
};
use parry::shape::SharedShape;

use crate::{AdjustPrecision, Collider, IntoCollider, Quaternion, Vector};

impl IntoCollider<Collider> for Sphere {
    fn collider(&self) -> Collider {
        Collider::sphere(self.radius.adjust_precision())
    }
}

impl IntoCollider<Collider> for Plane3d {
    fn collider(&self) -> Collider {
        let half_size = 10_000.0;
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

impl IntoCollider<Collider> for Line3d {
    fn collider(&self) -> Collider {
        let vec = self.direction.adjust_precision() * 10_000.0;
        Collider::segment(-vec, vec)
    }
}

impl IntoCollider<Collider> for Segment3d {
    fn collider(&self) -> Collider {
        let (point1, point2) = (self.point1(), self.point2());
        Collider::segment(point1.adjust_precision(), point2.adjust_precision())
    }
}

impl<const N: usize> IntoCollider<Collider> for Polyline3d<N> {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.map(|v| v.adjust_precision());
        Collider::polyline(vertices.to_vec(), None)
    }
}

impl IntoCollider<Collider> for BoxedPolyline3d {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.iter().map(|v| v.adjust_precision());
        Collider::polyline(vertices.collect(), None)
    }
}

impl IntoCollider<Collider> for Cuboid {
    fn collider(&self) -> Collider {
        let [hx, hy, hz] = self.half_size.adjust_precision().to_array();
        Collider::from(SharedShape::cuboid(hx, hy, hz))
    }
}

impl IntoCollider<Collider> for Cylinder {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::cylinder(
            self.half_height.adjust_precision(),
            self.radius.adjust_precision(),
        ))
    }
}

impl IntoCollider<Collider> for Capsule3d {
    fn collider(&self) -> Collider {
        Collider::capsule(
            2.0 * self.half_length.adjust_precision(),
            self.radius.adjust_precision(),
        )
    }
}

impl IntoCollider<Collider> for Cone {
    fn collider(&self) -> Collider {
        Collider::cone(
            self.height.adjust_precision(),
            self.radius.adjust_precision(),
        )
    }
}

// TODO: ConicalFrustum
// TODO: Torus
