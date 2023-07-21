use std::marker::PhantomData;

use crate::prelude::*;
use bevy::{ecs::system::SystemParam, prelude::*};
use parry::shape::{SharedShape, TypedShape};

// Todo: Allow custom rendering backends through generics
/// A `SystemParam` for physics debug rendering.
#[derive(SystemParam)]
pub struct PhysicsDebugRenderer<'w, 's> {
    /// A `SystemParam` for drawing lines and shapes using bevy_gizmos.
    pub gizmos: Gizmos<'s>,
    phantom_data: PhantomData<&'w ()>,
}

impl<'w, 's> PhysicsDebugRenderer<'w, 's> {
    /// Draws a line from `a` to `b`.
    pub fn draw_line(&mut self, a: Vector, b: Vector, color: Color) {
        #[cfg(feature = "2d")]
        self.gizmos.line_2d(a.as_f32(), b.as_f32(), color);
        #[cfg(feature = "3d")]
        self.gizmos.line(a.as_f32(), b.as_f32(), color);
    }

    /// Draws lines between a list of points.
    pub fn draw_line_strip(
        &mut self,
        points: Vec<Vector>,
        position: &Position,
        rotation: &Rotation,
        closed: bool,
        color: Color,
    ) {
        let pos = position.as_f32();
        #[cfg(feature = "2d")]
        self.gizmos.linestrip_2d(
            points.iter().map(|p| pos + rotation.rotate(*p).as_f32()),
            color,
        );
        #[cfg(feature = "3d")]
        self.gizmos.linestrip(
            points.iter().map(|p| pos + rotation.rotate(*p).as_f32()),
            color,
        );

        if closed && points.len() > 2 {
            let a = position.0 + rotation.rotate(points[0]);
            let b = position.0 + rotation.rotate(*points.last().unwrap());
            self.draw_line(a, b, color);
        }
    }

    /// Draws a polyline based on the given vertex and index buffers.
    pub fn draw_polyline(
        &mut self,
        vertices: &[Vector],
        indices: &[[u32; 2]],
        position: &Position,
        rotation: &Rotation,
        color: Color,
    ) {
        for [i1, i2] in indices {
            let a = position.0 + rotation.rotate(vertices[*i1 as usize]);
            let b = position.0 + rotation.rotate(vertices[*i2 as usize]);
            self.draw_line(a, b, color);
        }
    }

    /// Draws a collider shape with a given position and rotation.
    #[allow(clippy::unnecessary_cast)]
    pub fn draw_collider(
        &mut self,
        collider: &Collider,
        position: &Position,
        rotation: &Rotation,
        color: Color,
    ) {
        let nalgebra_to_glam =
            |points: &[_]| points.iter().map(|p| Vector::from(*p)).collect::<Vec<_>>();
        match collider.as_typed_shape() {
            #[cfg(feature = "2d")]
            TypedShape::Ball(s) => {
                self.gizmos.circle(
                    position.extend(0.0).as_f32(),
                    Vec3::Z,
                    s.radius as f32,
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Ball(s) => {
                self.gizmos
                    .sphere(position.as_f32(), rotation.as_f32(), s.radius as f32, color);
            }
            #[cfg(feature = "2d")]
            TypedShape::Cuboid(s) => {
                self.gizmos.cuboid(
                    Transform::from_scale(Vector::from(s.half_extents).extend(0.0).as_f32() * 2.0)
                        .with_translation(position.extend(0.0).as_f32())
                        .with_rotation(Quaternion::from(*rotation).as_f32()),
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Cuboid(s) => {
                self.gizmos.cuboid(
                    Transform::from_scale(Vector::from(s.half_extents).as_f32() * 2.0)
                        .with_translation(position.as_f32())
                        .with_rotation(rotation.as_f32()),
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
                    let pos = Position(position.0 + Vector::from(sub_pos.translation));
                    #[cfg(feature = "2d")]
                    let rot = *rotation + Rotation::from_radians(sub_pos.rotation.angle());
                    #[cfg(feature = "3d")]
                    let rot = Rotation((rotation.0 + sub_pos.rotation.into()).normalize());
                    self.draw_collider(&Collider::from(shape.to_owned()), &pos, &rot, color);
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
            TypedShape::Custom(_) => (),
        }
    }
}
