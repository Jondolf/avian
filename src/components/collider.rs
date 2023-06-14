use crate::prelude::*;
use bevy::prelude::*;
#[cfg(feature = "3d")]
use bevy::render::mesh::{Indices, VertexAttributeValues};
use derive_more::From;
use parry::{bounding_volume::Aabb, shape::SharedShape};

/// A physics shape used for things like colliders.
pub type Shape = SharedShape;

/// A collider used for collision detection.
#[derive(Clone, Component, Deref, DerefMut, From)]
pub struct Collider(Shape);

impl Default for Collider {
    fn default() -> Self {
        #[cfg(feature = "2d")]
        {
            Self(Shape::cuboid(0.5, 0.5))
        }
        #[cfg(feature = "3d")]
        {
            Self(Shape::cuboid(0.5, 0.5, 0.5))
        }
    }
}

impl Collider {
    pub fn get_shape(&self) -> &Shape {
        &self.0
    }

    pub fn compound(shapes: Vec<(impl Into<Pos>, impl Into<Rot>, impl Into<Collider>)>) -> Self {
        let shapes = shapes
            .into_iter()
            .map(|(p, r, c)| {
                (
                    utils::make_isometry(*p.into(), &r.into()),
                    c.into().get_shape().clone(),
                )
            })
            .collect::<Vec<_>>();
        Shape::compound(shapes).into()
    }

    pub fn ball(radius: Scalar) -> Self {
        Shape::ball(radius).into()
    }

    #[cfg(feature = "2d")]
    pub fn cuboid(x_length: Scalar, y_length: Scalar) -> Self {
        Shape::cuboid(x_length * 0.5, y_length * 0.5).into()
    }

    #[cfg(feature = "3d")]
    pub fn cuboid(x_length: Scalar, y_length: Scalar, z_length: Scalar) -> Self {
        Shape::cuboid(x_length * 0.5, y_length * 0.5, z_length * 0.5).into()
    }

    #[cfg(feature = "3d")]
    pub fn cylinder(height: Scalar, radius: Scalar) -> Self {
        Shape::cylinder(height * 0.5, radius).into()
    }

    #[cfg(feature = "3d")]
    pub fn cone(height: Scalar, radius: Scalar) -> Self {
        Shape::cone(height * 0.5, radius).into()
    }

    pub fn capsule(height: Scalar, radius: Scalar) -> Self {
        Shape::capsule(
            (Vector::Y * height * 0.5).into(),
            (Vector::NEG_Y * height * 0.5).into(),
            radius,
        )
        .into()
    }

    pub fn capsule_endpoints(a: Vector, b: Vector, radius: Scalar) -> Self {
        Shape::capsule(a.into(), b.into(), radius).into()
    }

    pub fn halfspace(outward_normal: Vector) -> Self {
        SharedShape::halfspace(nalgebra::Unit::new_normalize(outward_normal.into())).into()
    }

    pub fn segment(a: Vector, b: Vector) -> Self {
        Shape::segment(a.into(), b.into()).into()
    }

    pub fn triangle(a: Vector, b: Vector, c: Vector) -> Self {
        Shape::triangle(a.into(), b.into(), c.into()).into()
    }

    pub fn polyline(vertices: Vec<Vector>, indices: Option<Vec<[u32; 2]>>) -> Self {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        Shape::polyline(vertices, indices).into()
    }

    pub fn trimesh(vertices: Vec<Vector>, indices: Vec<[u32; 3]>) -> Self {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        Shape::trimesh(vertices, indices).into()
    }

    #[cfg(feature = "3d")]
    pub fn trimesh_from_bevy_mesh(mesh: &Mesh) -> Option<Self> {
        use parry::shape::TriMeshFlags;

        let vertices_indices = extract_mesh_vertices_indices(mesh);
        vertices_indices.map(|(v, i)| {
            Shape::trimesh_with_flags(v, i, TriMeshFlags::MERGE_DUPLICATE_VERTICES).into()
        })
    }

    #[cfg(feature = "3d")]
    pub fn convex_decomposition_from_bevy_mesh(mesh: &Mesh) -> Option<Self> {
        let vertices_indices = extract_mesh_vertices_indices(mesh);
        vertices_indices.map(|(v, i)| Shape::convex_decomposition(&v, &i).into())
    }

    #[cfg(feature = "2d")]
    pub fn convex_decomposition(vertices: Vec<Vector>, indices: Vec<[u32; 2]>) -> Self {
        let vertices = vertices.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        Shape::convex_decomposition(&vertices, &indices).into()
    }

    #[cfg(feature = "3d")]
    pub fn convex_decomposition(vertices: Vec<Vector>, indices: Vec<[u32; 3]>) -> Self {
        let vertices = vertices.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        Shape::convex_decomposition(&vertices, &indices).into()
    }

    pub fn convex_hull(points: Vec<Vector>) -> Option<Self> {
        let points = points.iter().map(|v| (*v).into()).collect::<Vec<_>>();
        Shape::convex_hull(&points).map(Into::into)
    }

    #[cfg(feature = "2d")]
    pub fn heightfield(heights: Vec<Scalar>, scale: Vector) -> Self {
        Shape::heightfield(heights.into(), scale.into()).into()
    }

    #[cfg(feature = "3d")]
    pub fn heightfield(
        heights: Vec<Scalar>,
        row_count: usize,
        column_count: usize,
        scale: Vector,
    ) -> Self {
        assert_eq!(
            heights.len(),
            row_count * column_count,
            "Number of heights is not equal to row_count * column_count"
        );
        let heights = nalgebra::DMatrix::from_vec(row_count, column_count, heights);
        Shape::heightfield(heights, scale.into()).into()
    }
}

#[cfg(feature = "3d")]
type VerticesIndices = (Vec<nalgebra::Point3<Scalar>>, Vec<[u32; 3]>);

#[cfg(feature = "3d")]
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

/// The Axis-Aligned Bounding Box of a collider.
#[derive(Clone, Copy, Component, Deref, DerefMut, PartialEq)]
pub struct ColliderAabb(pub Aabb);

impl ColliderAabb {
    /// Creates a new collider from a given [`Shape`] with a default density of 1.0.
    pub fn from_shape(shape: &Shape) -> Self {
        Self(shape.compute_local_aabb())
    }
}

impl Default for ColliderAabb {
    fn default() -> Self {
        ColliderAabb(Aabb::new_invalid())
    }
}

/// The mass properties derived from a given collider shape and density.
///
/// These will be added to the body's actual [`Mass`], [`InvMass`], [`Inertia`], [`InvInertia`] and [`LocalCom`] components.
///
/// You should generally not create or modify this directly. Instead, you can generate this automatically using a given collider shape and density with the associated `from_shape_and_density` method.
#[derive(Reflect, Clone, Copy, Component, PartialEq)]
#[reflect(Component)]
pub struct ColliderMassProperties {
    /// Mass given by collider.
    pub mass: Mass,
    /// Inverse mass given by collider.
    pub inv_mass: InvMass,
    /// Inertia given by collider.
    pub inertia: Inertia,
    /// Inverse inertia given by collider.
    pub inv_inertia: InvInertia,
    /// Local center of mass given by collider.
    pub local_center_of_mass: LocalCom,
    /// Density used for calculating other mass properties.
    pub density: Scalar,
}

impl ColliderMassProperties {
    pub const ZERO: Self = Self {
        mass: Mass::ZERO,
        inv_mass: InvMass(Scalar::INFINITY),
        inertia: Inertia::ZERO,
        inv_inertia: InvInertia::ZERO,
        local_center_of_mass: LocalCom::ZERO,
        density: 0.0,
    };
}

impl ColliderMassProperties {
    /// Computes mass properties for a given shape and density.
    pub fn from_shape_and_density(shape: &SharedShape, density: Scalar) -> Self {
        let props = shape.mass_properties(density);

        Self {
            mass: Mass(props.mass()),
            inv_mass: InvMass(props.inv_mass),

            #[cfg(feature = "2d")]
            inertia: Inertia(props.principal_inertia()),
            #[cfg(feature = "3d")]
            inertia: Inertia(props.reconstruct_inertia_matrix().into()),

            #[cfg(feature = "2d")]
            inv_inertia: InvInertia(1.0 / props.principal_inertia()),
            #[cfg(feature = "3d")]
            inv_inertia: InvInertia(props.reconstruct_inverse_inertia_matrix().into()),

            local_center_of_mass: LocalCom(props.local_com.into()),

            density,
        }
    }
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        Self::ZERO
    }
}

/// The previous [`ColliderMassProperties`].
#[derive(Clone, Copy, Component, Default, Deref, DerefMut, PartialEq)]
pub(crate) struct PrevColliderMassProperties(pub ColliderMassProperties);
