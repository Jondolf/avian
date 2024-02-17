//! An example demonstrating how to make a custom collider and use it for collision detection.

use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_xpbd_2d::{math::*, prelude::*, PhysicsSchedule, PhysicsStepSet};
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            XpbdExamplePlugin,
            // Add collider backend for our custom collider.
            // This handles things like initializing and updating required components
            // and managing collider hierarchies.
            ColliderBackendPlugin::<CircleCollider>::default(),
            // Enable collision detection for our custom collider.
            NarrowPhasePlugin::<CircleCollider>::default(),
        ))
        .insert_resource(ClearColor(Color::rgb(0.01, 0.01, 0.025)))
        .insert_resource(Gravity::ZERO)
        .add_systems(Startup, setup)
        .add_systems(
            PhysicsSchedule,
            (center_gravity, rotate).before(PhysicsStepSet::BroadPhase),
        )
        .run();
}

/// A basic collider with a circle shape. Only supports uniform scaling.
#[derive(Component)]
struct CircleCollider {
    /// The radius of the circle collider. This may be scaled by the `Transform` scale.
    radius: Scalar,
    /// The radius of the circle collider without `Transform` scale applied.
    unscaled_radius: Scalar,
    /// The scaling factor, determined by `Transform` scale.
    scale: Scalar,
}

impl CircleCollider {
    fn new(radius: Scalar) -> Self {
        Self {
            radius,
            unscaled_radius: radius,
            scale: 1.0,
        }
    }
}

impl AnyCollider for CircleCollider {
    fn aabb(&self, position: Vector, _rotation: impl Into<Rotation>) -> ColliderAabb {
        ColliderAabb::new(position, Vector::splat(self.radius))
    }

    fn mass_properties(&self, density: Scalar) -> ColliderMassProperties {
        // In 2D, the Z length is assumed to be 1.0, so volume = area
        let volume = bevy_xpbd_2d::math::PI * self.radius.powi(2);
        let mass = density * volume;
        let inertia = self.radius.powi(2) / 2.0;

        ColliderMassProperties {
            mass: Mass(mass),
            inverse_mass: InverseMass(mass.recip()),
            inertia: Inertia(inertia),
            inverse_inertia: InverseInertia(inertia.recip()),
            center_of_mass: CenterOfMass::default(),
        }
    }

    // This is the actual collision detection part.
    // It compute all contacts between two colliders at the given positions.
    fn contact_manifolds(
        &self,
        other: &Self,
        position1: Vector,
        rotation1: impl Into<Rotation>,
        position2: Vector,
        rotation2: impl Into<Rotation>,
        prediction_distance: Scalar,
    ) -> Vec<ContactManifold> {
        let rotation1: Rotation = rotation1.into();
        let rotation2: Rotation = rotation2.into();

        let inv_rotation1 = rotation1.inverse();
        let delta_pos = inv_rotation1.rotate(position2 - position1);
        let delta_rot = inv_rotation1.mul(rotation2);

        let distance_squared = delta_pos.length_squared();
        let sum_radius = self.radius + other.radius;

        if distance_squared < (sum_radius + prediction_distance).powi(2) {
            let normal1 = if distance_squared != 0.0 {
                delta_pos.normalize_or_zero()
            } else {
                Vector::X
            };
            let normal2 = delta_rot.inverse().rotate(-normal1);
            let point1 = normal1 * self.radius;
            let point2 = normal2 * other.radius;

            vec![ContactManifold {
                index: 0,
                normal1,
                normal2,
                contacts: vec![ContactData {
                    index: 0,
                    point1,
                    point2,
                    normal1,
                    normal2,
                    penetration: sum_radius - distance_squared.sqrt(),
                    // Impulses are computed by the constraint solver
                    normal_impulse: 0.0,
                    tangent_impulse: 0.0,
                }],
            }]
        } else {
            vec![]
        }
    }
}

// Note: This circle collider only supports uniform scaling.
impl ScalableCollider for CircleCollider {
    fn scale(&self) -> Vector {
        Vector::splat(self.scale)
    }

    fn set_scale(&mut self, scale: Vector, _detail: u32) {
        // For non-unifprm scaling, this would need to be converted to an ellipse collider or a convex hull.
        self.scale = scale.max_element();
        self.radius = self.unscaled_radius * scale.max_element();
    }
}

/// A marker component for the rotating body at the center.
#[derive(Component)]
struct CenterBody;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2dBundle::default());

    let center_radius = 200.0;
    let particle_radius = 5.0;

    let red = materials.add(Color::rgb(0.9, 0.3, 0.3).into());
    let blue = materials.add(Color::rgb(0.1, 0.6, 1.0).into());
    let particle_mesh = meshes.add(shape::Circle::new(particle_radius).into());

    // Spawn rotating body at the center.
    commands
        .spawn((
            MaterialMesh2dBundle {
                mesh: meshes.add(shape::Circle::new(center_radius).into()).into(),
                material: materials.add(Color::rgb(0.7, 0.7, 0.8).into()).clone(),
                ..default()
            },
            RigidBody::Kinematic,
            CircleCollider::new(center_radius.adjust_precision()),
            CenterBody,
        ))
        .with_children(|c| {
            // Spawn obstacles along the perimeter of the rotating body, like the teeth of a cog.
            let count = 8;
            let angle_step = std::f32::consts::TAU / count as f32;
            for i in 0..count {
                let pos = Quat::from_rotation_z(i as f32 * angle_step) * Vec3::Y * center_radius;
                c.spawn((
                    MaterialMesh2dBundle {
                        mesh: particle_mesh.clone().into(),
                        material: red.clone(),
                        transform: Transform::from_translation(pos).with_scale(Vec3::ONE * 5.0),
                        ..default()
                    },
                    CircleCollider::new(particle_radius.adjust_precision()),
                ));
            }
        });

    let x_count = 10;
    let y_count = 30;

    // Spawm grid of particles. These will be pulled towards the rotating body.
    for x in -x_count / 2..x_count / 2 {
        for y in -y_count / 2..y_count / 2 {
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh: particle_mesh.clone().into(),
                    material: blue.clone(),
                    transform: Transform::from_xyz(
                        x as f32 * 3.0 * particle_radius - 350.0,
                        y as f32 * 3.0 * particle_radius,
                        0.0,
                    ),
                    ..default()
                },
                RigidBody::Dynamic,
                CircleCollider::new(particle_radius.adjust_precision()),
                LinearDamping(0.4),
            ));
        }
    }
}

/// Pulls all particles towards the center.
fn center_gravity(
    mut particles: Query<(&Transform, &mut LinearVelocity), Without<CenterBody>>,
    time: Res<Time>,
) {
    let delta_seconds = time.delta_seconds_f64().adjust_precision();
    for (transform, mut lin_vel) in &mut particles {
        let pos_delta = transform.translation.truncate().adjust_precision();
        let dir = -pos_delta.normalize_or_zero();
        lin_vel.0 += 800.0 * delta_seconds * dir;
    }
}

/// Rotates the center body periodically clockwise and counterclockwise.
fn rotate(mut query: Query<&mut AngularVelocity, With<CenterBody>>, time: Res<Time>) {
    let sin = 3.0 * time.elapsed_seconds_f64().adjust_precision().sin();
    for mut ang_vel in &mut query {
        ang_vel.0 = sin;
    }
}
