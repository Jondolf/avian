use crate::components::{AngularVelocity, CenterOfMass};
use crate::prelude::{Collider, LinearVelocity, Mass, RigidBody, Vec3};
use bevy::gltf::{GltfMesh, WeakGltf};
use bevy::prelude::{Assets, Commands, Entity, Mesh, Res};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct OmiPhysicsShape {
    pub shapes: Vec<Shape>,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type")]
pub enum Shape {
    #[serde(rename = "box")]
    Box(BoxShape),
    #[serde(rename = "sphere")]
    Sphere(SphereShape),
    #[serde(rename = "capsule")]
    Capsule(CapsuleShape),
    #[serde(rename = "cylinder")]
    Cylinder(CylinderShape),
    #[serde(rename = "convex")]
    Convex(ConvexShape),
    #[serde(rename = "trimesh")]
    Trimesh(TrimeshShape),
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BoxShape {
    #[serde(rename = "box")]
    pub details: BoxDetails,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BoxDetails {
    #[serde(default = "size_default")]
    pub size: [f32; 3],
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SphereShape {
    #[serde(default = "radius_default")]
    pub radius: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CapsuleShape {
    #[serde(default = "radius_default")]
    pub radius: f32,
    #[serde(default = "height_default")]
    pub height: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CylinderShape {
    #[serde(default = "radius_default")]
    pub radius: f32,
    #[serde(default = "height_default")]
    pub height: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ConvexShape {
    #[serde(default = "mesh_default")]
    pub mesh: i32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct TrimeshShape {
    #[serde(default = "mesh_default")]
    pub mesh: i32,
}

#[derive(Serialize, Deserialize)]
pub struct PhysicsBodyExtension {
    #[serde(rename = "OMI_physics_body")]
    pub omi_physics_body: Option<OmiPhysicsBody>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct OmiPhysicsBody {
    pub motion: Option<Motion>,
    pub collider: Option<OmiCollider>,
    pub trigger: Option<Trigger>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Motion {
    #[serde(rename = "type")]
    pub motion_type: MotionType,
    #[serde(default = "default_mass")]
    pub mass: f32,
    #[serde(rename = "linearVelocity", default)]
    pub linear_velocity: [f32; 3],
    #[serde(rename = "angularVelocity", default)]
    pub angular_velocity: [f32; 3],
    #[serde(rename = "centerOfMass", default)]
    pub center_of_mass: [f32; 3],
    #[serde(rename = "inertiaDiagonal", default)]
    pub inertia_diagonal: [f32; 3],
    #[serde(rename = "inertiaOrientation", default)]
    pub inertia_orientation: [f32; 4],
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "lowercase")]
pub enum MotionType {
    Static,
    Kinematic,
    Dynamic,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct OmiCollider {
    pub shape: u32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Trigger {
    #[serde(default = "shape_default")]
    pub shape: i32,
}
fn default_mass() -> f32 {
    1.0
}
fn shape_default() -> i32 {
    -1
}

fn radius_default() -> f32 {
    0.5
}

fn height_default() -> f32 {
    2.0
}

fn size_default() -> [f32; 3] {
    [1.0, 1.0, 1.0]
}

fn mesh_default() -> i32 {
    -1
}

impl OmiPhysicsShape {
    pub fn into_colliders(
        self,
        gltf: &WeakGltf,
        meshes: &Res<Assets<Mesh>>,
        gltf_meshes: &Res<Assets<GltfMesh>>,
    ) -> Vec<Collider> {
        let mut colliders = vec![];
        for shape in self.shapes {
            if let Some(collider) = shape.try_into_collider(gltf, meshes, gltf_meshes) {
                colliders.push(collider)
            }
        }
        colliders
    }
}

impl Shape {
    pub fn try_into_collider(
        self,
        gltf: &WeakGltf,
        meshes: &Res<Assets<Mesh>>,
        gltf_meshes: &Res<Assets<GltfMesh>>,
    ) -> Option<Collider> {
        Some(match self {
            Shape::Box(r#box) => Collider::cuboid(
                r#box.details.size[0],
                r#box.details.size[1],
                r#box.details.size[2],
            ),
            Shape::Sphere(sphere) => Collider::sphere(sphere.radius),
            Shape::Capsule(capsule) => Collider::capsule(capsule.height, capsule.radius),
            Shape::Cylinder(cylinder) => Collider::cylinder(cylinder.height, cylinder.radius),
            Shape::Convex(convex) => {
                if convex.mesh == -1 {
                    return None;
                }
                let mesh = gltf.meshes.get(convex.mesh as usize)?;
                let mesh = gltf_meshes.get(mesh)?;
                let mesh = meshes.get(&mesh.primitives.first().as_ref()?.mesh)?;
                Collider::convex_hull_from_mesh(mesh)?
            }
            Shape::Trimesh(trimesh) => {
                if trimesh.mesh == -1 {
                    return None;
                }
                let mesh = gltf.meshes.get(trimesh.mesh as usize)?;
                let mesh = gltf_meshes.get(mesh)?;
                let mesh = meshes.get(&mesh.primitives.first().as_ref()?.mesh)?;
                Collider::trimesh_from_mesh(mesh)?
            }
        })
    }
}

impl OmiPhysicsBody {
    pub fn try_spawn(
        self,
        entity: Entity,
        commands: &mut Commands,
        physics_shapes: &[Collider],
    ) -> Option<()> {
        if let Some(collider) = self.collider {
            commands
                .entity(entity)
                .insert(physics_shapes.get(collider.shape as usize)?.clone());
        }
        if let Some(motion) = self.motion {
            match motion.motion_type {
                MotionType::Static => commands.entity(entity).insert(RigidBody::Static),
                MotionType::Kinematic => commands.entity(entity).insert(RigidBody::Kinematic),
                MotionType::Dynamic => commands.entity(entity).insert(RigidBody::Dynamic),
            };
            commands.entity(entity).insert((
                Mass(motion.mass),
                LinearVelocity(Vec3::from(motion.linear_velocity)),
                AngularVelocity(Vec3::from(motion.angular_velocity)),
                CenterOfMass(Vec3::from(motion.center_of_mass)),
                //TODO inertia
            ));
        }
        Some(())
    }
}
