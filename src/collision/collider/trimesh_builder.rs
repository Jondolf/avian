use bevy::prelude::*;
use parry::shape::{SharedShape, TypedShape};
use thiserror::Error;

use crate::prelude::*;

#[derive(Debug, Clone)]
pub struct TrimeshBuilder {
    shape: SharedShape,
    position: Position,
    rotation: Rotation,
    fallback_subdivs: u32,
    ball_subdivs: Option<(u32, u32)>,
    capsule_subdivs: Option<(u32, u32)>,
    cylinder_subdivs: Option<u32>,
    cone_subdivs: Option<u32>,
}

#[derive(Debug, Clone, PartialEq, Reflect, Default)]
pub struct Trimesh {
    vertices: Vec<Vector>,
    indices: Vec<[u32; 3]>,
}

impl Trimesh {
    /// Extends the trimesh with the vertices and indices of another trimesh.
    /// The indices of `other` will be offset by the number of vertices in `self`.
    pub fn extend(&mut self, other: Trimesh) {
        let next_vertex_index = self.vertices.len() as u32;
        self.vertices.extend(other.vertices);
        self.indices.extend(
            other
                .indices
                .iter()
                .map(|is| is.map(|i| i + next_vertex_index)),
        );
    }
}

#[derive(Debug, Error)]
pub enum TrimeshBuilderError {
    #[error("Unsupported shape type: {0}")]
    UnsupportedShape(String),
}

impl TrimeshBuilder {
    pub fn new(shape: SharedShape) -> Self {
        TrimeshBuilder {
            shape,
            position: default(),
            rotation: default(),
            fallback_subdivs: 16,
            ball_subdivs: None,
            capsule_subdivs: None,
            cylinder_subdivs: None,
            cone_subdivs: None,
        }
    }

    pub fn translated(&mut self, position: impl Into<Position>) -> &mut Self {
        self.position = position.into();
        self
    }

    pub fn rotated(&mut self, rotation: impl Into<Rotation>) -> &mut Self {
        self.rotation = rotation.into();
        self
    }

    pub fn fallback_subdivs(&mut self, fallback_subdivs: u32) -> &mut Self {
        self.fallback_subdivs = fallback_subdivs;
        self
    }

    pub fn ball_subdivs(&mut self, theta: u32, phi: u32) -> &mut Self {
        self.ball_subdivs = Some((theta, phi));
        self
    }

    pub fn capsule_subdivs(&mut self, theta: u32, phi: u32) -> &mut Self {
        self.capsule_subdivs = Some((theta, phi));
        self
    }

    pub fn cylinder_subdivs(&mut self, cylinder_subdivs: u32) -> &mut Self {
        self.cylinder_subdivs = Some(cylinder_subdivs);
        self
    }

    pub fn cone_subdivs(&mut self, cone_subdivs: u32) -> &mut Self {
        self.cone_subdivs = Some(cone_subdivs);
        self
    }

    fn subdivs(&self, get: impl Fn(&Self) -> Option<u32>) -> u32 {
        get(self).unwrap_or(self.fallback_subdivs)
    }

    pub fn build(&self) -> Result<Trimesh, TrimeshBuilderError> {
        let (vertices, indices) = match self.shape.as_typed_shape() {
            // Simple cases
            TypedShape::Cuboid(cuboid) => cuboid.to_trimesh(),
            TypedShape::Voxels(voxels) => voxels.to_trimesh(),
            TypedShape::ConvexPolyhedron(convex_polyhedron) => convex_polyhedron.to_trimesh(),
            TypedShape::HeightField(height_field) => height_field.to_trimesh(),
            // Triangles
            TypedShape::Triangle(triangle) => {
                (vec![triangle.a, triangle.b, triangle.c], vec![[0, 1, 2]])
            }
            TypedShape::TriMesh(tri_mesh) => {
                (tri_mesh.vertices().to_vec(), tri_mesh.indices().to_vec())
            }
            // Need subdivisions
            TypedShape::Ball(ball) => ball.to_trimesh(
                self.subdivs(|t| t.ball_subdivs?.0.into()),
                self.subdivs(|t| t.ball_subdivs?.1.into()),
            ),
            TypedShape::Capsule(capsule) => capsule.to_trimesh(
                self.subdivs(|t| t.capsule_subdivs?.0.into()),
                self.subdivs(|t| t.capsule_subdivs?.1.into()),
            ),
            TypedShape::Cylinder(cylinder) => {
                cylinder.to_trimesh(self.subdivs(|t| t.cylinder_subdivs))
            }
            TypedShape::Cone(cone) => cone.to_trimesh(self.subdivs(|t| t.cone_subdivs)),
            // Compounds need to be unpacked
            TypedShape::Compound(compound) => {
                let mut sub_builder = self.clone();
                return Ok(compound.shapes().iter().fold(
                    Trimesh::default(),
                    move |mut compound_trimesh, (sub_pos, shape)| {
                        sub_builder.shape = shape.clone();
                        sub_builder.position = Position(
                            self.position.0 + self.rotation * Vector::from(sub_pos.translation),
                        );
                        sub_builder.rotation = self
                            .rotation
                            .mul_quat(sub_pos.rotation.into())
                            .normalize()
                            .into();
                        let trimesh = match sub_builder.build() {
                            Ok(trimesh) => trimesh,
                            Err(error) => {
                                warn!("Failed to convert compound sub-shape to trimesh: {error}");
                                return compound_trimesh;
                            }
                        };

                        compound_trimesh.extend(trimesh);

                        // No need to track recursive compounds because parry panics on nested compounds anyways lol
                        compound_trimesh
                    },
                ));
            }
            // Rounded shapes ignore the rounding and use the inner shape
            TypedShape::RoundCuboid(round_shape) => round_shape.inner_shape.to_trimesh(),
            TypedShape::RoundTriangle(round_shape) => (
                vec![
                    round_shape.inner_shape.a,
                    round_shape.inner_shape.b,
                    round_shape.inner_shape.c,
                ],
                vec![[0, 1, 2]],
            ),
            TypedShape::RoundConvexPolyhedron(round_shape) => round_shape.inner_shape.to_trimesh(),
            TypedShape::RoundCylinder(round_shape) => round_shape
                .inner_shape
                .to_trimesh(self.subdivs(|t| t.cylinder_subdivs)),
            TypedShape::RoundCone(round_shape) => round_shape
                .inner_shape
                .to_trimesh(self.subdivs(|t| t.cone_subdivs)),
            // Not supported
            TypedShape::Segment(segment) => {
                return Err(TrimeshBuilderError::UnsupportedShape(format!(
                    "{segment:?}",
                )));
            }
            TypedShape::Polyline(polyline) => {
                return Err(TrimeshBuilderError::UnsupportedShape(format!(
                    "{polyline:?}",
                )));
            }
            TypedShape::HalfSpace(half_space) => {
                return Err(TrimeshBuilderError::UnsupportedShape(format!(
                    "{half_space:?}",
                )));
            }
            TypedShape::Custom(_shape) => {
                return Err(TrimeshBuilderError::UnsupportedShape("Custom".to_string()));
            }
        };
        let pos = self.position;
        Ok(Trimesh {
            vertices: vertices
                .into_iter()
                .map(|v| pos.0 + Vector::from(self.rotation * Vector::from(v)))
                .collect(),
            indices: indices.into_iter().map(|i| i.into()).collect(),
        })
    }
}

impl Collider {
    pub fn trimesh_builder(&self) -> TrimeshBuilder {
        TrimeshBuilder::new(self.shape_scaled().clone())
    }
}

#[cfg(feature = "collider-from-mesh")]
impl From<Trimesh> for Mesh {
    fn from(trimesh: Trimesh) -> Self {
        use bevy::asset::RenderAssetUsages;
        use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues, prelude::*};

        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        mesh.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            VertexAttributeValues::Float32x3(
                trimesh
                    .vertices
                    .into_iter()
                    .map(|v| v.f32().to_array())
                    .collect(),
            ),
        );
        mesh.insert_indices(Indices::U32(
            trimesh
                .indices
                .into_iter()
                .flat_map(|i| i.map(|i| i as u32))
                .collect(),
        ));
        mesh.compute_normals();
        if let Err(err) = mesh.generate_tangents() {
            warn!("Failed to generate tangents for mesh: {err}");
        }

        mesh
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rasterizes_cuboid() {
        let collider = Collider::cuboid(1.0, 2.0, 3.0);
        let trimesh = collider.trimesh_builder().build().unwrap();
        assert_eq!(trimesh.vertices.len(), 8);
        assert_eq!(trimesh.indices.len(), 12);
    }
}
