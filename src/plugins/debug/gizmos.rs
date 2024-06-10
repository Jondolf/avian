use crate::prelude::*;
use bevy::prelude::*;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use parry::shape::{SharedShape, TypedShape};

/// An extension trait for `Gizmos<PhysicsGizmo>`.
pub trait PhysicsGizmoExt {
    /// Draws a line from `a` to `b`.
    fn draw_line(&mut self, a: Vector, b: Vector, color: Color);

    /// Draws lines between a list of points.
    fn draw_line_strip(
        &mut self,
        points: Vec<Vector>,
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        closed: bool,
        color: Color,
    );

    /// Draws a polyline based on the given vertex and index buffers.
    fn draw_polyline(
        &mut self,
        vertices: &[Vector],
        indices: &[[u32; 2]],
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        color: Color,
    );

    /// Draws an arrow from `a` to `b` with an arrowhead that has a length of `head_length`.
    fn draw_arrow(&mut self, a: Vector, b: Vector, head_length: Scalar, color: Color);

    /// Draws a [`Collider`] shape.
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    fn draw_collider(
        &mut self,
        collider: &Collider,
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        color: Color,
    );

    /// Draws the results of a [raycast](SpatialQuery#raycasting).
    #[allow(clippy::too_many_arguments)]
    fn draw_raycast(
        &mut self,
        origin: Vector,
        direction: Dir,
        max_time_of_impact: Scalar,
        hits: &[RayHitData],
        ray_color: Color,
        point_color: Color,
        normal_color: Color,
    );

    /// Draws the results of a [shapecast](SpatialQuery#shapecasting).
    #[allow(clippy::too_many_arguments)]
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    fn draw_shapecast(
        &mut self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: impl Into<Rotation>,
        direction: Dir,
        max_time_of_impact: Scalar,
        hits: &[ShapeHitData],
        ray_color: Color,
        shape_color: Color,
        point_color: Color,
        normal_color: Color,
    );
}

impl<'w, 's> PhysicsGizmoExt for Gizmos<'w, 's, PhysicsGizmos> {
    /// Draws a line from `a` to `b`.
    fn draw_line(&mut self, a: Vector, b: Vector, color: Color) {
        #[cfg(feature = "2d")]
        self.line_2d(a.f32(), b.f32(), color);
        #[cfg(feature = "3d")]
        self.line(a.f32(), b.f32(), color);
    }

    /// Draws lines between a list of points.
    fn draw_line_strip(
        &mut self,
        points: Vec<Vector>,
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        closed: bool,
        color: Color,
    ) {
        let position: Position = position.into();
        let rotation: Rotation = rotation.into();

        let pos = position.f32();
        #[cfg(feature = "2d")]
        self.linestrip_2d(
            points.iter().map(|p| pos + rotation.rotate(*p).f32()),
            color,
        );
        #[cfg(feature = "3d")]
        self.linestrip(
            points.iter().map(|p| pos + rotation.rotate(*p).f32()),
            color,
        );

        if closed && points.len() > 2 {
            let a = position.0 + rotation.rotate(points[0]);
            let b = position.0 + rotation.rotate(*points.last().unwrap());
            self.draw_line(a, b, color);
        }
    }

    /// Draws a polyline based on the given vertex and index buffers.
    fn draw_polyline(
        &mut self,
        vertices: &[Vector],
        indices: &[[u32; 2]],
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        color: Color,
    ) {
        let position: Position = position.into();
        let rotation: Rotation = rotation.into();

        for [i1, i2] in indices {
            let a = position.0 + rotation.rotate(vertices[*i1 as usize]);
            let b = position.0 + rotation.rotate(vertices[*i2 as usize]);
            self.draw_line(a, b, color);
        }
    }

    /// Draws an arrow from `a` to `b` with an arrowhead that has a length of `head_length`
    /// and a width of `head_width`.
    fn draw_arrow(&mut self, a: Vector, b: Vector, head_length: Scalar, color: Color) {
        #[cfg(feature = "2d")]
        {
            self.arrow_2d(a.f32(), b.f32(), color)
                .with_tip_length(head_length as f32);
        }

        #[cfg(feature = "3d")]
        {
            self.arrow(a.f32(), b.f32(), color)
                .with_tip_length(head_length as f32);
        }
    }

    /// Draws a collider shape with a given position and rotation.
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    #[allow(clippy::unnecessary_cast)]
    fn draw_collider(
        &mut self,
        collider: &Collider,
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        color: Color,
    ) {
        let position: Position = position.into();
        let rotation: Rotation = rotation.into();

        let nalgebra_to_glam =
            |points: &[_]| points.iter().map(|p| Vector::from(*p)).collect::<Vec<_>>();
        match collider.shape_scaled().as_typed_shape() {
            #[cfg(feature = "2d")]
            TypedShape::Ball(s) => {
                self.circle(
                    position.extend(0.0).f32(),
                    Direction3d::Z,
                    s.radius as f32,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Ball(s) => {
                self.sphere(position.f32(), rotation.f32(), s.radius as f32, color);
            }
            #[cfg(feature = "2d")]
            TypedShape::Cuboid(s) => {
                self.cuboid(
                    Transform::from_scale(Vector::from(s.half_extents).extend(0.0).f32() * 2.0)
                        .with_translation(position.extend(0.0).f32())
                        .with_rotation(Quaternion::from(rotation).f32()),
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Cuboid(s) => {
                self.cuboid(
                    Transform::from_scale(Vector::from(s.half_extents).f32() * 2.0)
                        .with_translation(position.f32())
                        .with_rotation(rotation.f32()),
                    color,
                );
            }
            #[cfg(feature = "2d")]
            TypedShape::Capsule(s) => {
                self.draw_line_strip(
                    nalgebra_to_glam(&s.to_polyline(32)),
                    position,
                    rotation,
                    true,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Capsule(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(
                    &nalgebra_to_glam(&vertices),
                    &indices,
                    position,
                    rotation,
                    color,
                );
            }
            TypedShape::Segment(s) => self.draw_line_strip(
                vec![s.a.into(), s.b.into()],
                position,
                rotation,
                false,
                color,
            ),
            TypedShape::Triangle(s) => self.draw_line_strip(
                vec![s.a.into(), s.b.into(), s.c.into()],
                position,
                rotation,
                true,
                color,
            ),
            TypedShape::TriMesh(s) => {
                for tri in s.triangles() {
                    self.draw_collider(
                        &Collider::from(SharedShape::new(tri)),
                        position,
                        rotation,
                        color,
                    );
                }
            }
            TypedShape::Polyline(s) => self.draw_polyline(
                &nalgebra_to_glam(s.vertices()),
                s.indices(),
                position,
                rotation,
                color,
            ),
            #[cfg(feature = "2d")]
            TypedShape::HalfSpace(s) => {
                let basis = Vector::new(-s.normal.y, s.normal.x);
                let a = basis * 10_000.0;
                let b = basis * -10_000.0;
                self.draw_line_strip(vec![a, b], position, rotation, false, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::HalfSpace(s) => {
                let n = s.normal;
                let sign = n.z.signum();
                let a = -1.0 / (sign + n.z);
                let b = n.x * n.y * a;
                let basis1 = Vector::new(1.0 + sign * n.x * n.x * a, sign * b, -sign * n.x);
                let basis2 = Vector::new(b, sign + n.y * n.y * a, -n.y);
                let a = basis1 * 10_000.0;
                let b = basis1 * -10_000.0;
                let c = basis2 * 10_000.0;
                let d = basis2 * -10_000.0;
                self.draw_polyline(&[a, b, c, d], &[[0, 1], [2, 3]], position, rotation, color);
            }
            TypedShape::HeightField(s) => {
                #[cfg(feature = "2d")]
                for segment in s.segments() {
                    self.draw_collider(
                        &Collider::from(SharedShape::new(segment)),
                        position,
                        rotation,
                        color,
                    );
                }
                #[cfg(feature = "3d")]
                for triangle in s.triangles() {
                    self.draw_collider(
                        &Collider::from(SharedShape::new(triangle)),
                        position,
                        rotation,
                        color,
                    );
                }
            }
            TypedShape::Compound(s) => {
                for (sub_pos, shape) in s.shapes() {
                    let pos = Position(position.0 + rotation.rotate(sub_pos.translation.into()));
                    #[cfg(feature = "2d")]
                    let rot = rotation + Rotation::from_radians(sub_pos.rotation.angle());
                    #[cfg(feature = "3d")]
                    let rot = Rotation((rotation.mul_quat(sub_pos.rotation.into())).normalize());
                    self.draw_collider(&Collider::from(shape.to_owned()), pos, rot, color);
                }
            }
            #[cfg(feature = "2d")]
            TypedShape::ConvexPolygon(s) => {
                self.draw_line_strip(
                    nalgebra_to_glam(s.points()),
                    position,
                    rotation,
                    true,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::ConvexPolyhedron(s) => {
                let indices = s
                    .edges()
                    .iter()
                    .map(|e| [e.vertices.x, e.vertices.y])
                    .collect::<Vec<_>>();
                self.draw_polyline(
                    &nalgebra_to_glam(s.points()),
                    &indices,
                    position,
                    rotation,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Cylinder(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(
                    &nalgebra_to_glam(&vertices),
                    &indices,
                    position,
                    rotation,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Cone(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(
                    &nalgebra_to_glam(&vertices),
                    &indices,
                    position,
                    rotation,
                    color,
                );
            }
            // ------------
            // Round shapes
            // ------------
            #[cfg(feature = "2d")]
            TypedShape::RoundCuboid(s) => {
                self.draw_line_strip(
                    nalgebra_to_glam(&s.to_polyline(32)),
                    position,
                    rotation,
                    true,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::RoundCuboid(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(
                    &nalgebra_to_glam(&vertices),
                    &indices,
                    position,
                    rotation,
                    color,
                );
            }
            TypedShape::RoundTriangle(s) => {
                // Parry doesn't have a method for the rounded outline, so we have to just use a normal triangle
                // (or compute the outline manually based on the border radius)
                self.draw_collider(
                    &Collider::from(SharedShape::new(s.inner_shape)),
                    position,
                    rotation,
                    color,
                );
            }
            #[cfg(feature = "2d")]
            TypedShape::RoundConvexPolygon(s) => {
                self.draw_line_strip(
                    nalgebra_to_glam(&s.to_polyline(32)),
                    position,
                    rotation,
                    true,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::RoundConvexPolyhedron(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(
                    &nalgebra_to_glam(&vertices),
                    &indices,
                    position,
                    rotation,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::RoundCylinder(s) => {
                let (vertices, indices) = s.to_outline(32, 32);
                self.draw_polyline(
                    &nalgebra_to_glam(&vertices),
                    &indices,
                    position,
                    rotation,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::RoundCone(s) => {
                let (vertices, indices) = s.to_outline(32, 32);
                self.draw_polyline(
                    &nalgebra_to_glam(&vertices),
                    &indices,
                    position,
                    rotation,
                    color,
                );
            }
            TypedShape::Custom(_id) =>
            {
                #[cfg(feature = "2d")]
                if _id == 1 {
                    if let Some(ellipse) = collider.shape_scaled().as_shape::<EllipseWrapper>() {
                        self.primitive_2d(
                            ellipse.0,
                            position.f32(),
                            rotation.as_radians() as f32,
                            color,
                        );
                    }
                } else if _id == 2 {
                    if let Some(polygon) =
                        collider.shape_scaled().as_shape::<RegularPolygonWrapper>()
                    {
                        self.primitive_2d(
                            polygon.0,
                            position.f32(),
                            rotation.as_radians() as f32,
                            color,
                        );
                    }
                }
            }
        }
    }

    /// Draws the results of a [raycast](SpatialQuery#raycasting).
    #[allow(clippy::too_many_arguments)]
    fn draw_raycast(
        &mut self,
        origin: Vector,
        direction: Dir,
        max_time_of_impact: Scalar,
        hits: &[RayHitData],
        ray_color: Color,
        point_color: Color,
        normal_color: Color,
    ) {
        let max_toi = hits
            .iter()
            .max_by(|a, b| a.time_of_impact.total_cmp(&b.time_of_impact))
            .map_or(max_time_of_impact, |hit| hit.time_of_impact);

        // Draw ray as arrow
        #[cfg(feature = "2d")]
        self.draw_arrow(
            origin,
            origin + direction.adjust_precision() * max_toi,
            8.0,
            ray_color,
        );
        #[cfg(feature = "3d")]
        self.draw_arrow(
            origin,
            origin + direction.adjust_precision() * max_toi,
            0.1,
            ray_color,
        );

        // Draw all hit points and normals
        for hit in hits {
            let point = origin + direction.adjust_precision() * hit.time_of_impact;

            // Draw hit point
            #[cfg(feature = "2d")]
            self.circle_2d(point.f32(), 3.0, point_color);
            #[cfg(feature = "3d")]
            self.sphere(point.f32(), default(), 0.025, point_color);

            // Draw hit normal as arrow
            #[cfg(feature = "2d")]
            self.draw_arrow(point, point + hit.normal * 30.0, 8.0, normal_color);
            #[cfg(feature = "3d")]
            self.draw_arrow(point, point + hit.normal * 0.5, 0.1, normal_color);
        }
    }

    /// Draws the results of a [shapecast](SpatialQuery#shapecasting).
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    #[allow(clippy::too_many_arguments)]
    fn draw_shapecast(
        &mut self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: impl Into<Rotation>,
        direction: Dir,
        max_time_of_impact: Scalar,
        hits: &[ShapeHitData],
        ray_color: Color,
        shape_color: Color,
        point_color: Color,
        normal_color: Color,
    ) {
        let shape_rotation = shape_rotation.into();
        #[cfg(feature = "3d")]
        let shape_rotation = Rotation(shape_rotation.normalize());

        let max_toi = hits
            .iter()
            .max_by(|a, b| a.time_of_impact.total_cmp(&b.time_of_impact))
            .map_or(max_time_of_impact, |hit| hit.time_of_impact);

        // Draw collider at origin
        self.draw_collider(shape, origin, shape_rotation, shape_color);

        // Draw arrow from origin to position of shape at final hit
        // TODO: We could render the swept collider outline instead
        #[cfg(feature = "2d")]
        self.draw_arrow(
            origin,
            origin + max_toi * direction.adjust_precision(),
            8.0,
            ray_color,
        );
        #[cfg(feature = "3d")]
        self.draw_arrow(
            origin,
            origin + max_toi * direction.adjust_precision(),
            0.1,
            ray_color,
        );

        // Draw all hit points, normals and the shape at the hit points
        for hit in hits {
            // Draw hit point
            #[cfg(feature = "2d")]
            self.circle_2d(hit.point1.f32(), 3.0, point_color);
            #[cfg(feature = "3d")]
            self.sphere(hit.point1.f32(), default(), 0.025, point_color);

            // Draw hit normal as arrow
            #[cfg(feature = "2d")]
            self.draw_arrow(
                hit.point1,
                hit.point1 + hit.normal1 * 30.0,
                8.0,
                normal_color,
            );
            #[cfg(feature = "3d")]
            self.draw_arrow(
                hit.point1,
                hit.point1 + hit.normal1 * 0.5,
                0.1,
                normal_color,
            );

            // Draw collider at hit point
            self.draw_collider(
                shape,
                origin + hit.time_of_impact * direction.adjust_precision(),
                shape_rotation,
                Color::rgba(shape_color.r(), shape_color.g(), shape_color.b(), 0.3),
            );
        }
    }
}
